#include "hdlc.h"

/*
  dummy implementation as long as CRC is not implemented
 */ 
void hdlc_init_crc(void) {
  return;
}
/*
  dummy implementation as long as CRC is not implemented
 */ 
int hdlc_crc_error(const buffer_t *buffer) {
  return 0;
}
/*
  dummy implementation as long as CRC is not implemented
 */
void hdlc_update_crc(uint8_t in) {
  return;
}

void hdlc_init_reception(buffer_t *hdlc_buffer,int *read_index, volatile const buffer_t *in_buffer) {
  hdlc_buffer->fill = 0;
  hdlc_init_crc();

  /*do not do this as we may loose bytes that have not been processed yet*/
  //  *read_index = in_buffer->fill;
}

#define abs(X) (X > 0 ? X : -X) 

int hdlc_update_rx_buffer(buffer_t *hdlc_buffer,int *read_index,  buffer_t volatile const *in_buffer) {
  uint8_t rx_data;
  
  int volatile input_fill = in_buffer->fill;

  while(*read_index != input_fill) {

    if(HDLC_FRAME_BOUNDARY_OCTET == in_buffer->data[(*read_index)]) {
      (*read_index)++;
      *read_index = *read_index % in_buffer->size;
      /*
	rfc1662, 4.3:

	Frames which are too short (less than 4 octets 
	when using the 16-bit FCS), or which end with 
	a Control Escape octet followed immediately
	by a closing Flag Sequence, or in which 
	octet-framing is violated (by transmitting a 
	"0" stop bit where a "1" bit is expected), are
	silently discarded, and not counted as a FCS error.
      */
      if(
	 (hdlc_buffer->fill < 4)
  /* || */
  /* 	 (HDLC_ESCAPE_OCTET  */
  /* 	  == hdlc_buffer->data[hdlc_buffer->fill-1])   */
) {
	/*rewind, but do not return to be able to consume more bytes in input buffer*/
	hdlc_buffer->fill = 0;
      } else {
	if (hdlc_crc_error(hdlc_buffer)) {
	  return HDLC_STATUS_CRC_ERROR;
	} else {
	  return HDLC_STATUS_FRAME_COMPLETE;
	}
      } 
      /* if this byte is not frame boundary */
    } else {
    /*
      if we have detected an escape, we need to wait for one more
      available byte to decide what to do
    */
    if(HDLC_ESCAPE_OCTET == in_buffer->data[(*read_index)]) {
      if( (abs(*read_index-input_fill) % in_buffer->size) < 2) {
	return HDLC_STATUS_LISTEN;
      } else {
	/* skip escape character */
	(*read_index)++;
	*read_index = *read_index % in_buffer->size;

	/* flip bit 5 of next data byte */
	rx_data = in_buffer->data[(*read_index)++] ^ 0x20;
	*read_index = *read_index % in_buffer->size;
      }
    } else {
      /* consume next byte in in_buffer */
      rx_data = in_buffer->data[(*read_index)++];
      *read_index = *read_index % in_buffer->size;
    }


      /* /\* the last received byte was the escape byte ..*\/ */
      /* if ( */
      /* 	  (hdlc_buffer->fill > 0) &&  */
      /* 	  (HDLC_ESCAPE_OCTET  */
      /* 	   == hdlc_buffer->data[hdlc_buffer->fill-1])) { */

      /* 	/\* then bit-flip bit 5 *\/ */
      /* 	rx_data= rx_data ^ 0x20; */

      /* 	/\* overwrite escape byte *\/ */
      /* 	hdlc_buffer->fill--; */
      /* } else { */
      /* 	/\* update CRC only if this byte is not the escape byte ...*\/ */
      /* 	if(HDLC_ESCAPE_OCTET != rx_data) { */
      /* 	  hdlc_update_crc(hdlc_buffer->data[hdlc_buffer->fill-1]);	     */
	  
      /* 	} */
      /* }	 */
      hdlc_update_crc(rx_data);	    
      if(hdlc_buffer->fill < hdlc_buffer->size) {
	hdlc_buffer->data[hdlc_buffer->fill++]=rx_data;
      }	else {
	return HDLC_STATUS_BUFFER_OVERFLOW_ERROR;
      }
    

    }
  }

  return HDLC_STATUS_LISTEN;
  
}
