#include "driverlib.h"
#include "flash.h"

int flash_write_buffer(const uint16_t* addr, buffer_t buffer) {

}

/*
writing to segment A currently not supported because unlocking is
required. Segment A contains calibration information of device and
should not be overwritten.
 */
int flash_update_word(const uint16_t* addr, uint16_t value) {
  
  uint16_t tmp[flash_segment_size/2];
  const unsigned int offset = ((unsigned int) addr) % flash_segment_size;
  uint8_t *segment_start = (((uint8_t*) addr) - offset);
  int i;

  //read complete segment
  for(i=0;i<flash_segment_size/2;i++) {
    tmp[i]=((uint16_t*) segment_start)[i];
  }

  tmp[offset/2]=value;

  do {
    //erase segment, segment address is used by masking 
    //offset within segment
    FLASH_segmentErase(segment_start);

  } while (FLASH_eraseCheck((uint8_t*) segment_start,flash_segment_size) == STATUS_FAIL);

  //Flash Write
  FLASH_write16(
		tmp,
		(uint16_t*) segment_start,
		flash_segment_size/2
		);
}


