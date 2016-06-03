#ifndef _buffer_h
#define _buffer_h


#include <stdlib.h>
#include <inttypes.h>

typedef struct buffer {
  uint16_t size;
  uint16_t fill;
  uint8_t *data;
} buffer_t;

/**
   Appends one byte c to buffer.

   The function returns the number of appended bytes.  If the buffer
   is not full, the byte is appended, ie. the byte c is copied to
   buffer->data[buffer->fill] and buffer->fill is
   incremented. Otherwise 0 is returned because no byte has been
   appended.

  */

int buffer_append_byte(buffer_t * buffer, uint8_t c);


uint8_t buffer_at_index(const buffer_t *buffer, const uint16_t index);

#endif // _buffer_h
