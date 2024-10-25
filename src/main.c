#include <stdio.h>
#include <math.h>
#include "driver/gpio.h"
#include "esp_pm.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "config.h"
#include "buffer.h"
#include "mpu6050.h"

static buf_handle_t buffer;
static int step_count = 0;
static SemaphoreHandle_t xSemaphore = NULL;

static void sampling_task(void *arg) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    int16_t x, y, z;
    
    while (1) {
        getMPU6050Accel(&x, &y, &z);
        uint32_t magnitude = (uint32_t)sqrt((int32_t)x * x + 
                                          (int32_t)y * y + 
                                          (int32_t)z * z) / 100;
        addToBuffer(buffer, magnitude);
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(SAMPLING_PERIOD));
    }
}

static void algo_task(void *arg) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    
    while (1) {
        int size = getBufferCount(buffer);
        
        if (size > 0) {
            // Calculate mean
            double mean = 0;
            for (int i = 0; i < size; i++) {
                mean += readFromBuffer(buffer, i);
            }
            mean /= size;
            
            // Calculate standard deviation
            double variance = 0;
            for (int i = 0; i < size; i++) {
                double diff = readFromBuffer(buffer, i) - mean;
                variance += diff * diff;
            }
            variance /= size;
            double sd = sqrt(variance);
            
            if (sd < MIN_SD) {
                sd = MIN_SD;
            }
            
            int lastStepTS = -MIN_INTRA_STEP_TIME;
            double threshold = mean + K * sd;
            
            for (int i = 0; i < size; i++) {
                uint32_t sample = popFromBuffer(buffer);
                int currentTS = i * SAMPLING_PERIOD;
                printf("Sample: %d, Threshold: %f\n", (int)sample, threshold);
                if (sample > threshold && 
                    (currentTS - lastStepTS) >= MIN_INTRA_STEP_TIME) {
                    step_count++;
                    lastStepTS = currentTS;
                    printf("Step detected! Count: %d\n", step_count);
                }
            }
        }
        
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(ALGO_PERIOD));
    }
}

static void IRAM_ATTR button_isr_handler(void *arg) {
    xSemaphoreGiveFromISR(xSemaphore, NULL);
}

static void led_task(void *arg) {
    while (1) {
        if (xSemaphoreTake(xSemaphore, portMAX_DELAY) == pdTRUE) {
            if (step_count >= STEPS_GOAL) {
                // Goal reached: Quick triple flash
                for (int i = 0; i < 3; i++) {
                    gpio_set_level(LED_PIN, 1);
                    vTaskDelay(pdMS_TO_TICKS(100));
                    gpio_set_level(LED_PIN, 0);
                    vTaskDelay(pdMS_TO_TICKS(100));
                }
            } else {
                // Goal not reached: Slow single flash
                gpio_set_level(LED_PIN, 1);
                vTaskDelay(pdMS_TO_TICKS(500));
                gpio_set_level(LED_PIN, 0);
            }
        }
    }
}

void app_main() {
    // Configure power management
    esp_pm_config_esp32_t pm_config = {
        .max_freq_mhz = 240,
        .min_freq_mhz = 80,
        .light_sleep_enable = true
    };
  //  ESP_ERROR_CHECK(esp_pm_configure(&pm_config));
    
    // Initialize I2C and MPU6050
    initI2C(33, 32);  // SDA = GPIO33, SCL = GPIO32
    initMPU6050();
    
    // Create circular buffer
    buffer = createBuffer(BUFF_SIZE);
    
    // Configure LED
    gpio_reset_pin(LED_PIN);
    gpio_set_direction(LED_PIN, GPIO_MODE_OUTPUT);
    
    // Configure button with interrupt
    gpio_reset_pin(BUTTON_PIN);
    gpio_set_direction(BUTTON_PIN, GPIO_MODE_INPUT);
    gpio_set_intr_type(BUTTON_PIN, GPIO_INTR_NEGEDGE);
    gpio_install_isr_service(0);
    gpio_isr_handler_add(BUTTON_PIN, button_isr_handler, NULL);
    
    // Create semaphore
    xSemaphore = xSemaphoreCreateBinary();
    
    // Create tasks
    xTaskCreate(sampling_task, "sampling", 2048, NULL, 5, NULL);
    xTaskCreate(algo_task, "algo", 2048, NULL, 4, NULL);
    xTaskCreate(led_task, "led", 2048, NULL, 3, NULL);
}