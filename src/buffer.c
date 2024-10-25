#include <stdio.h>
#include <stdlib.h>
#include "buffer.h"

typedef struct circular_buffer_t {
    uint32_t *data;
    int head;
    int tail;
    int size;
    int count;
} circular_buffer_t;

buf_handle_t createBuffer(int size) {
    buf_handle_t buffer = (buf_handle_t)malloc(sizeof(circular_buffer_t));
    buffer->data = (uint32_t*)malloc(size * sizeof(uint32_t));
    buffer->head = 0;
    buffer->tail = 0;
    buffer->size = size;
    buffer->count = 0;
    return buffer;
}

void addToBuffer(buf_handle_t buffer, uint32_t value) {
    buffer->data[buffer->head] = value;
    buffer->head = (buffer->head + 1) % buffer->size;
    if (buffer->count < buffer->size) {
        buffer->count++;
    } else {
        buffer->tail = (buffer->tail + 1) % buffer->size;
    }
}

uint32_t readFromBuffer(buf_handle_t buffer, int position) {
    if (position >= buffer->count) return 0;
    int index = (buffer->tail + position) % buffer->size;
    return buffer->data[index];
}

uint32_t popFromBuffer(buf_handle_t buffer) {
    if (buffer->count == 0) return 0;
    uint32_t value = buffer->data[buffer->tail];
    buffer->tail = (buffer->tail + 1) % buffer->size;
    buffer->count--;
    return value;
}

int getBufferCount(buf_handle_t buffer) {
    return buffer->count;
}

void printBuffer(buf_handle_t buffer) {
    printf("Buffer contents: ");
    for (int i = 0; i < buffer->count; i++) {
        printf("%lu ", readFromBuffer(buffer, i));
    }
    printf("\n");
}