#include <freertos/FreeRTOS.h>
#include <freertos/task.h> // Include the necessary header file
#include "math.h"
#include <driver/i2c.h>
#include <stdint.h> 
#include <stdio.h>
#include "../../../../../.platformio/packages/toolchain-xtensa-esp-elf/xtensa-esp-elf/include/sys/types.h"

#define MPU6050_ADDR 0x68
#define MPU6050_PWR_MGMT_1 0x6B
#define MPU6050_SMPLRT_DIV 0x19
#define MPU6050_ACCEL_XOUT_H 0x3B
#define MPU6050_ACCEL_YOUT_H 0x3D
#define MPU6050_ACCEL_ZOUT_H 0x3F
#define SDA_PIN 33
#define SCL_PIN 32
#define Xhigh 0x3B
#define Xlow  0x3C

#define Yhigh 0x3D
#define Ylow  0x3E

#define Zhigh 0x3F
#define Zlow  0x40


int16_t readaccel(uint16_t registerHigh, uint16_t registerLow) {
    int16_t tempRaw = 0;
    uint8_t buffer;
    
    // read low register
    // send just the register number with no other data

        i2c_cmd_handle_t cmd = i2c_cmd_link_create();

    i2c_master_write_byte(cmd, MPU6050_ADDR << 1 | I2C_MASTER_WRITE,1);  // WRITE bit set!
    
    i2c_master_write_byte(cmd, registerLow,1);  // read low first
    
    i2c_master_stop(cmd);
    
    i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_PERIOD_MS);
    
    i2c_cmd_link_delete(cmd);
    // wait a little
    vTaskDelay(10 / portTICK_PERIOD_MS);
    // now read the answer
    cmd = i2c_cmd_link_create();
   
    i2c_master_write_byte(cmd, MPU6050_ADDR << 1 | I2C_MASTER_READ,1);  // READ bit set!
    
    i2c_master_read(cmd, &buffer, 1, I2C_MASTER_NACK);
    
    i2c_master_stop(cmd);
    
    i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_PERIOD_MS);
    
    i2c_cmd_link_delete(cmd);
    tempRaw = buffer;
    // read high register
    cmd = i2c_cmd_link_create();
    
    
    i2c_master_write_byte(cmd, MPU6050_ADDR << 1 | I2C_MASTER_WRITE,1);  // WRITE bit set!
    
    
    i2c_master_write_byte(cmd, registerHigh, 1);  // read high
    
    i2c_master_stop(cmd);
    
    i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_PERIOD_MS);
    
    
    i2c_cmd_link_delete(cmd);
    vTaskDelay(10 / portTICK_PERIOD_MS);
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);

    i2c_master_write_byte(cmd, MPU6050_ADDR << 1 | I2C_MASTER_READ,1);  // READ bit set!
    
    i2c_master_read(cmd, &buffer, 1, I2C_MASTER_NACK);
    

    
    i2c_master_stop(cmd);
    
     i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_PERIOD_MS);
    
    
    i2c_cmd_link_delete(cmd);
    // combine high and low registers into a signed integer
    tempRaw |= ((int16_t)buffer) << 8;

    return tempRaw;
}


u_int32_t getaccelmagnitude() {
    double Xaccel = readaccel(Xhigh, Xlow);
    double Yaccel = readaccel(Yhigh, Ylow);
    double Zaccel = readaccel(Zhigh, Zlow);

    return (u_int32_t)sqrt(pow(Xaccel, 2) + pow(Yaccel, 2) + pow(Zaccel, 2));
}