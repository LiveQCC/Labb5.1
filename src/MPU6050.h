#ifndef MPU6050_H
#define MPU6050_H

#include <stdint.h>

// MPU6050 I2C address and registers
#define MPU6050_ADDR 0x68
#define MPU6050_PWR_MGMT_1 0x6B
#define MPU6050_ACCEL_XOUT_H 0x3B

void initI2C(int sdapin, int sclpin);
void initMPU6050(void);
void getMPU6050Accel(int16_t *x, int16_t *y, int16_t *z);

#endif