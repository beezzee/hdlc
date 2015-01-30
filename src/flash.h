#ifndef _flash_h
#define _flash_h

#include "buffer.h"

/**
   The size of a single flash segment
*/
#define flash_segment_size 128


/**
   Start adresses of flash segment Info A
 */
#define flash_info_a_addr   (0x1980)


/**
   Start adresses of flash segment Info B
 */
#define flash_info_b_addr   (0x1900)

/**
   Start adresses of flash segment Info C
 */
#define flash_info_c_addr   (0x1880)

/**
   Start adresses of flash segment Info D
 */
#define flash_info_d_addr   (0x1800)


int flash_write_buffer(const uint16_t* addr, buffer_t buffer);

int flash_update_word(const uint16_t* addr, uint16_t value);

#endif
