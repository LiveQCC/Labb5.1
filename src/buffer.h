#ifndef BUFFER_H
#define BUFFER_H

#include <stdint.h>

typedef struct circular_buffer_t* buf_handle_t;

buf_handle_t createBuffer(int size);
void addToBuffer(buf_handle_t buffer, uint32_t value);
uint32_t readFromBuffer(buf_handle_t buffer, int position);
uint32_t popFromBuffer(buf_handle_t buffer);
int getBufferCount(buf_handle_t buffer);
void printBuffer(buf_handle_t buffer);

#endif