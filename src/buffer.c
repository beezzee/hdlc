#include <stdio.h>
#include "buffer.h"

int buffer_append_byte(buffer_t * buffer, uint8_t c) {
  if(buffer->fill==buffer->size) {
    return 0;
  }

  buffer->data[buffer->fill++] = c;
  return 1;
}

uint8_t buffer_at_index(const buffer_t *buffer, const uint16_t index) {
  return buffer->data[index];
}

void buffer_print(const buffer_t *buffer) {
    int i;
    printf("[");
    for(i=0;i<buffer->fill;i++) {
        printf("0x%02x ",buffer->data[i]);
    }
    printf("]");
}

int buffer_copy(buffer_t * dst,const buffer_t *src) {
    unsigned int i;

    dst->fill = 0;
    for(i=0;(i<src->fill) && (i<dst->size);i++) {
        dst->data[i] = src->data[i];
        dst->fill++;
    }

    return i;
}
