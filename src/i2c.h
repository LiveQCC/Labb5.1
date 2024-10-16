#include <stdio.h>
#include <stdint.h> // Add this line to include the header file that defines 'uint8_t'

/**
 * Initialises I2C bus, uses controller 0
 */
void initI2C(int sdapin, int sclpin);

/**
 * Writes one byte into a register
 */
void writeI2C(uint8_t address, uint8_t reg, uint8_t data);

/**
 * Reads len bytes and places them into a buffer, buffer must be pre-allocated
 */
void readI2C(uint8_t address, uint8_t register, uint8_t *buffer, int len);