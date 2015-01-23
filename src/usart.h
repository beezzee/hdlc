#ifndef _usart_h
#define _usart_h

#include "buffer.h"

typedef struct usart {
  uint16_t base_address;
  uint16_t port;
  uint16_t rx_pin;
  uint16_t tx_pin;
} usart_t;

typedef struct usart_buffer {
  uint8_t status;
  uint8_t preamble;
  uint8_t remaining_bytes;
  uint8_t crc_0;
  uint8_t crc_1;
  buffer_t *payload;
} usart_buffer_t;

#define USART_STATUS_ERROR_MASK 0xF0
#define USART_STATUS_STATE_MASK 0x0F

#define USART_STATUS_BUFFER_OVERFLOW 1<<4

#define USART_STATUS_WAIT_PREAMBLE 0
#define USART_STATUS_WAIT_BYTE_COUNT 1
#define USART_STATUS_WAIT_PAYLOAD 2
#define USART_STATUS_FRAME_COMPLETE  3
#define USART_STATUS_TRANSMISSION_IDLE 4;
#define USART_STATUS_TRANSMISSION_ONGOING 5;

#define USART_PREAMBLE 0x00
#define USART_FRAME_INDEX_PREAMBLE 0
#define USART_FRAME_INDEX_FRAMESIZE 1

int usart_frame_complete(usart_buffer_t *buffer);
void usart_start_reception(usart_t *usart);
void usart_rx_interrupt_handler(usart_t *usart, volatile buffer_t *buffer);
void usart_stop_reception(usart_t *usart);
void usart_init(usart_t *usart);
int usart_putchar(usart_t *usart, int s);
uint8_t usart_get_byte(usart_t *usart);

void usart_tx_interrupt_handler(usart_t *usart, volatile int * index,  volatile const buffer_t *buffer);

int usart_transmit_buffer(usart_t *usart, volatile int * index, volatile const buffer_t *buffer);

int usart_transmit_init(usart_t *usart, volatile int * index, volatile const buffer_t *buffer);

#endif //_usart_h
