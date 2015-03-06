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
