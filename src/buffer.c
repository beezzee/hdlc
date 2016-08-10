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

int buffer_copy(buffer_t * dst,const buffer_t *src) {
    unsigned int i;

    dst->fill = 0;
    for(i=0;(i<src->fill) && (i<dst->size);i++) {
        dst->data[i] = src->data[i];
        dst->fill++;
    }

    return i;
}
