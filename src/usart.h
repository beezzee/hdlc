#ifndef _usart_h
#define _usart_h


typedef struct usart {
  uint16_t base_address;
  uint16_t port;
  uint16_t rx_pin;
  uint16_t tx_pin;
} usart_t;

void usart_init(usart_t *usart);
int usart_putchar(usart_t *usart, int s);

#endif //_usart_h
