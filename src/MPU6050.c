#include "driver/i2c.h"
#include "mpu6050.h"

void initI2C(int sdapin, int sclpin) {
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = sdapin,
        .scl_io_num = sclpin,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 400000
    };
    
    ESP_ERROR_CHECK(i2c_param_config(I2C_NUM_0, &conf));
    ESP_ERROR_CHECK(i2c_driver_install(I2C_NUM_0, conf.mode, 0, 0, 0));
}

static void writeI2C(uint8_t address, uint8_t reg, uint8_t data) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (address << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_write_byte(cmd, data, true);
    i2c_master_stop(cmd);
   ESP_ERROR_CHECK(i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_PERIOD_MS));
    i2c_cmd_link_delete(cmd);
}

static void readI2C(uint8_t address, uint8_t reg, uint8_t *buffer, int len) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (address << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (address << 1) | I2C_MASTER_READ, true);
    if (len > 1) {
        i2c_master_read(cmd, buffer, len - 1, I2C_MASTER_ACK);
    }
    i2c_master_read_byte(cmd, buffer + len - 1, I2C_MASTER_NACK);
    i2c_master_stop(cmd);
    ESP_ERROR_CHECK(i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_PERIOD_MS));
    i2c_cmd_link_delete(cmd);
}

void initMPU6050() {
    writeI2C(MPU6050_ADDR, MPU6050_PWR_MGMT_1, 0x00);
}

void getMPU6050Accel(int16_t *x, int16_t *y, int16_t *z) {
    uint8_t data[6];
    readI2C(MPU6050_ADDR, MPU6050_ACCEL_XOUT_H, data, 6);
    
    *x = (data[0] << 8) | data[1];
    *y = (data[2] << 8) | data[3];
    *z = (data[4] << 8) | data[5];
}