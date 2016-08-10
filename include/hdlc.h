/**
   Implementation of HDLC framing.

   We use only a subset of HDLC, basically as it is defined for PPP in
   HDLC-like Framing (RFC-1662).
 */


#ifndef _hdlc_h
#define _hdlc_h

#include "buffer.h"

#define HDLC_FRAME_BOUNDARY_OCTET 0x7e
#define HDLC_ESCAPE_OCTET 0x7D

typedef struct hdlc_frame {
  uint8_t address;
  uint8_t control;
  buffer_t *inf;
  uint16_t fcs;
  uint8_t status;
} hdlc_frame_t;

#define HDLC_STATUS_CRC_ERROR 1 << 4
#define HDLC_STATUS_BUFFER_OVERFLOW_ERROR 1<<5

#define HDLC_STATUS_FRAME_COMPLETE 1 << 1
#define HDLC_STATUS_LISTEN 0

/**
    Byte offset of HDLC Address field.
    */
#define HDLC_ADDR_OFFSET 0

/**
    Byte offset of HDLC Control field
*/
#define HDLC_CTRL_OFFSET 1


extern char hdlc_getchar();
extern void hdlc_putchar(char c);

int hdlc_receive_frame(buffer_t *hdlc_buffer);
void hdlc_init_reception(buffer_t *hdlc_buffer);

/**
   Formats and transmit HDLC frame.

   Takes the payload given by buffer and transmit its as a HDLC
   frame. This includes escape octet stuffing and CRC.

   Note that this function is blocking.
 */
int hdlc_transmit_frame(const buffer_t *buffer);

buffer_t hdlc_get_payload(const buffer_t *buffer);

#endif //_hdlc_h
