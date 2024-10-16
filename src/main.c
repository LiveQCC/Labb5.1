#include <freertos/FreeRTOS.h>
#include <freertos/task.h> // Include the necessary header file
#include "readAccel.h"
#include <stdint.h> 
#include <stdio.h>
#include "i2c.h"
#include "math.h"

#define MPU6050_ADDR 0x68
#define MPU6050_PWR_MGMT_1 0x6B
#define MPU6050_SMPLRT_DIV 0x19
#define MPU6050_ACCEL_XOUT_H 0x3B
#define MPU6050_ACCEL_XOUT_L 0x3C 
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
void app_main()
{
    
   initI2C(SDA_PIN, SCL_PIN);        
   writeI2C(MPU6050_ADDR, MPU6050_SMPLRT_DIV, 250);
    uint8_t buffer; // Buffer to hold acceleration data
    int16_t tempRaw = 0;
    uint16_t Values[2];
   // writeI2C(MPU6050_ADDR,MPU6050_ACCEL_XOUT_H,buffer);
    while(1){
        
        // Extract acceleration values from buffer
      //  int acellMag = getaccelmagnitude();
        readI2C(MPU6050_ADDR,Xlow,&buffer, 1);
        tempRaw = buffer;
        readI2C(MPU6050_ADDR,Xhigh, &buffer, 1);
        tempRaw |= ((int16_t)buffer) << 8;
        //printf("XValue %d" , tempRaw);
        Values[0] = tempRaw;
         readI2C(MPU6050_ADDR,Ylow,&buffer, 1);
         tempRaw = buffer;
         readI2C(MPU6050_ADDR,Yhigh,&buffer, 1);
        tempRaw |= ((int16_t)buffer) << 8;

        Values[1] = tempRaw; 
         //        printf("YValue %d" , tempRaw);

         readI2C(MPU6050_ADDR,Zlow,&buffer, 1);
         tempRaw = buffer;
         readI2C(MPU6050_ADDR,Zhigh,&buffer, 1);
         tempRaw |= ((int16_t)buffer) << 8;
        Values[2] = tempRaw;

        int16_t ax = Values[0];
        int16_t ay = Values[1];
        int16_t az = Values[2];

        uint32_t sum = sqrt(Values[0] + Values[1] + Values[2]);
        
        printf("Acceleration - X: %lu,\n ", (unsigned long)sum);
        
   // double Xaccel = readaccel(Xhigh, Xlow);
    //double Yaccel = readaccel(Yhigh, Ylow);
    //double Zaccel = readaccel(Zhigh, Zlow);

   // return (u_int32_t)sqrt(pow(Xaccel, 2) + pow(Yaccel, 2) + pow(Zaccel, 2));
       /*
        int8_t ax = (int8_t)(buffer[0] << 8) | buffer[2];
        int8_t ay = (int8_t)(buffer[2] << 8) | buffer[4];
        int8_t az = (int8_t)(buffer[4] << 8) | buffer[5];
        */
        // Print acceleration
        //printf("Acceleration - X: %d, Y: %d, Z: %d\n ", ax, ay, az);
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}