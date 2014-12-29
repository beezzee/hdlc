#ifndef _usart_h
#define _usart_h


typedef struct usart {
  uint16_t base_address;
  uint16_t port;
  uint16_t rx_pin;
  uint16_t tx_pin;
} usart_t;

typedef struct buffer {
  uint16_t size;
  uint16_t fill;
  uint8_t status;
  uint8_t *data;
} usart_buffer_t;

#define USART_STATUS_BUFFER_OVERFLOW 1<<4
#define USART_STATUS_FRAME_COMPLETE  1
#define USART_PREAMBLE 0x00
#define USART_FRAME_INDEX_PREAMBLE 0
#define USART_FRAME_INDEX_FRAMESIZE 1

void usart_init_reception(usart_t *usart, usart_buffer_t *buffer);
void usart_rx_interrupt_handler(usart_t *usart, usart_buffer_t *buffer);

void usart_init(usart_t *usart);
int usart_putchar(usart_t *usart, int s);

#endif //_usart_h
