/*
#include "mpu6050.h"
#define SDA_PIN 33
#define SCL_PIN 32
void app_main() {
    MPU6050_init(SDA_PIN, SCL_PIN);

    while(1) {
        AccelData accel = MPU6050_readAccel();
        printf("Acceleration - X: %d, Y: %d, Z: %d\n", accel.x, accel.y, accel.z);
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}
*/