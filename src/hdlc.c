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

void hdlc_init_reception(buffer_t *hdlc_buffer) {
  hdlc_buffer->fill = 0;
  hdlc_init_crc();

  /*do not do this as we may loose bytes that have not been processed yet*/
  //  *read_index = in_buffer->fill;
}

#define abs(X) (X > 0 ? X : -X)

int hdlc_receive_frame(buffer_t *hdlc_buffer) {
  uint8_t rx_data;
  uint8_t escape_active, new_data;


  hdlc_init_reception(hdlc_buffer);
  escape_active = 0;
  new_data=0;


  while(1) {

    rx_data = hdlc_getchar();

    if(escape_active) {
      rx_data ^= 0x20;
      escape_active = 0;
      new_data=1;
    } else {
      switch (rx_data) {

      case HDLC_FRAME_BOUNDARY_OCTET:
	escape_active = 0;
	new_data=0;
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
	  hdlc_init_reception(hdlc_buffer);
	} else {
	  if (hdlc_crc_error(hdlc_buffer)) {
	    return HDLC_STATUS_CRC_ERROR;
	  } else {
	    /*
	      remove CRC bytes
	    */
	    hdlc_buffer->fill = hdlc_buffer->fill - 2;
	    return HDLC_STATUS_FRAME_COMPLETE;
	  }
	}


	break;
      case HDLC_ESCAPE_OCTET:
	/* if this byte is not frame boundary */

	/*
	  if we have detected an escape, we need to wait for one more
	  available byte to decide what to do
	*/
	escape_active = 1;
	new_data = 0;
	break;
    default:
        escape_active = 0;
        new_data=1;

        break;
      }
    }

    if(new_data) {
      hdlc_update_crc(rx_data);
      if(hdlc_buffer->fill < hdlc_buffer->size) {
        hdlc_buffer->data[hdlc_buffer->fill++]=rx_data;
      }	else {
        return HDLC_STATUS_BUFFER_OVERFLOW_ERROR;
      }
    }


  }
}



int hdlc_transmit_frame(const buffer_t *buffer){
  int i;
  int crc=0;

  /*
    assume that buffer contains already initialzed address and control
    data.
   */
  hdlc_putchar(HDLC_FRAME_BOUNDARY_OCTET);
  for (i=0;i<buffer->fill;i++) {
    hdlc_update_crc(buffer->data[i]);
    if (
	(HDLC_FRAME_BOUNDARY_OCTET == buffer->data[i] )
	||(HDLC_ESCAPE_OCTET == buffer->data[i] )
	) {
      hdlc_putchar(HDLC_ESCAPE_OCTET);
      hdlc_putchar(buffer->data[i] ^ 0x20);
    } else {
      hdlc_putchar(buffer->data[i]);
    }
  }


  /*
    currently, CRC is only dummy function. Endianess of CRC has to be
    checked.
   */
  hdlc_putchar(crc & 0xFF);
  hdlc_putchar(crc >> 8);

  hdlc_putchar(HDLC_FRAME_BOUNDARY_OCTET);
  return HDLC_STATUS_FRAME_COMPLETE;
}

buffer_t hdlc_get_payload(const buffer_t *buffer) {
    buffer_t tmp;

    /*  address, control, 2x CRC*/
    if(buffer->fill + buffer->size < 8) {
        tmp.fill = 0;
        tmp.size = buffer->size;
        tmp.data = buffer->data;
    } else {
        /*cut off 4 bytes address, control at beginning and  2x CRC at end*/
        tmp.fill = buffer->fill-4;

        /*cut off beginning of buffer*/
        tmp.size = buffer->size-2;
        tmp.data = buffer->data+2;
    }

    return tmp;
}
