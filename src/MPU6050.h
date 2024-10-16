/*s
#ifndef MPU6050_H
#define MPU6050_H

#include <stdint.h>

typedef struct {
    int16_t x;
    int16_t y;
    int16_t z;
} AccelData;

void MPU6050_init(int sdapin, int sclpin);
AccelData MPU6050_readAccel(void);

#endif
*/