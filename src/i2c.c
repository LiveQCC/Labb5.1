#include <freertos/FreeRTOS.h>
#include <freertos/task.h> // Include the necessary header file

#include <stdint.h> 

#include <driver/i2c.h>
//#include "esp_log.h"
#include <esp_err.h>

#define MPU6050_ADDR 0x68
#define MPU6050_PWR_MGMT_1 0x6B
#define MPU6050_SMPLRT_DIV 0x19
#define MPU6050_ACCEL_XOUT_H 0x3B
#define MPU6050_ACCEL_YOUT_H 0x3D
#define MPU6050_ACCEL_ZOUT_H 0x3F

// Initialize I2C for ESP32
void initI2C(int sdapin, int sclpin) {
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = sdapin;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_io_num = sclpin;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = 100000; // Set the clock speed to 100 kHz;
    conf.clk_flags = 0;

    i2c_param_config(I2C_NUM_0, &conf);
    i2c_driver_install(I2C_NUM_0, conf.mode, 0, 0, 0);
}

// Write to a register
void writeI2C(uint8_t address, uint8_t reg, uint8_t data){
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (address << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd,MPU6050_PWR_MGMT_1,1);
    i2c_master_write_byte(cmd, 0x00, 1);
    i2c_master_stop(cmd);
    ESP_ERROR_CHECK(i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_PERIOD_MS));
    i2c_cmd_link_delete(cmd);
}

void readI2C(uint8_t address, uint8_t reg, uint8_t *buffer, int len){
    
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        esp_err_t res = i2c_master_start(cmd);
        ESP_ERROR_CHECK(res);
        res = i2c_master_write_byte(cmd, address << 1 | I2C_MASTER_WRITE, 1); // WRITE bit set!
        ESP_ERROR_CHECK(res);
        res = i2c_master_write_byte(cmd, reg, 1); // read low first
        ESP_ERROR_CHECK(res);
        res = i2c_master_stop(cmd);
        ESP_ERROR_CHECK(res);
        res = i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_PERIOD_MS);
        ESP_ERROR_CHECK(res);
        i2c_cmd_link_delete(cmd);

        // wait a little
        vTaskDelay(pdMS_TO_TICKS(10));

        // now read the answer
        cmd = i2c_cmd_link_create();
        res = i2c_master_start(cmd);
        ESP_ERROR_CHECK(res);
        res = i2c_master_write_byte(cmd, MPU6050_ADDR << 1 | I2C_MASTER_READ, 1); // READ bit set!
        ESP_ERROR_CHECK(res);
        res = i2c_master_read(cmd, buffer, 1, I2C_MASTER_NACK);
        ESP_ERROR_CHECK(res);
        res = i2c_master_stop(cmd);
        ESP_ERROR_CHECK(res);
        res = i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_PERIOD_MS);
        ESP_ERROR_CHECK(res);
        i2c_cmd_link_delete(cmd);
    
}

