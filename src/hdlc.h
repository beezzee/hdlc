/**
   Implementation of HDLC framing.

   We use only a subset of HDLC, basically as it is defined for PPP in
   HDLC-like Framing (RFC-1662).
 */


#ifndef _hdlc_h
#define _hdlc_h

#include "usart.h"
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
#define HDLC_STATUS_BUFFER_OVERFLOW_ERROR 1<<5;

#define HDLC_STATUS_FRAME_COMPLETE 1 << 1

int hdlc_update_rx_buffer(buffer_t *hdlc_buffer, int *read_index, const buffer_t *in_buffer);
void hdlc_init_reception(buffer_t *hdlc_buffer,int *read_index, const buffer_t *in_buffer);

#endif //_hdlc_h
