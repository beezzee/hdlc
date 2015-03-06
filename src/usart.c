#include "driverlib.h"
#include "usart.h"

//from http://software-dl.ti.com/msp430/msp430_public_sw/mcu/msp430/MSP430BaudRateConverter/index.html
//9.6 kbaud @ 32.768 kHz
/*
#define usart_clock_prescale   3
#define usart_mod_reg_1  0
#define usart_mod_reg_2  3
#define usart_oversampling USCI_A_UART_LOW_FREQUENCY_BAUDRATE_GENERATION 
*/

//9.6 kbaud @ 4 MHz
/*
#define usart_clock_prescale   26
#define usart_mod_reg_1  1
#define usart_mod_reg_2  0
#define usart_oversampling USCI_A_UART_OVERSAMPLING_BAUDRATE_GENERATION 
*/

//115.2 kbaud @ 4 MHz
#define usart_clock_prescale   2
#define usart_mod_reg_1  2
#define usart_mod_reg_2  3
#define usart_oversampling USCI_A_UART_OVERSAMPLING_BAUDRATE_GENERATION 


void usart_init(usart_t *usart) {
  //P3.4,5 = USCI_A0 TXD/RXD
  GPIO_setAsPeripheralModuleFunctionInputPin(
					     usart->port,
					     usart->rx_pin + usart->tx_pin
					     );

  if ( STATUS_FAIL 
       == USCI_A_UART_initAdvance(usart->base_address,
				  USCI_A_UART_CLOCKSOURCE_SMCLK,
				  usart_clock_prescale,
				  usart_mod_reg_1,
				  usart_mod_reg_2,
				  USCI_A_UART_NO_PARITY,
				  USCI_A_UART_LSB_FIRST,
				  USCI_A_UART_ONE_STOP_BIT,
				  USCI_A_UART_MODE,
				  usart_oversampling)){
    return;
  }

  //Enable UART module for operation
  USCI_A_UART_enable(usart->base_address);


  USCI_A_UART_clearInterruptFlag(usart->base_address,
				 USCI_A_UART_RECEIVE_INTERRUPT);
  // USCI_A_UART_enableInterrupt(base_address,
  //			      USCI_A_UART_RECEIVE_INTERRUPT);




}


int usart_frame_complete(usart_buffer_t *buffer) {
  return ((buffer->status & USART_STATUS_STATE_MASK) == USART_STATUS_FRAME_COMPLETE);
}

void usart_start_reception(usart_t *usart) {
        //Enable Receive Interrupt
        USCI_A_UART_clearInterruptFlag(usart->base_address,
                                       USCI_A_UART_RECEIVE_INTERRUPT
                                       );
        USCI_A_UART_enableInterrupt(usart->base_address,
                                    USCI_A_UART_RECEIVE_INTERRUPT
                                    );


}

uint8_t usart_get_byte(usart_t *usart) {
  return USCI_A_UART_receiveData(usart->base_address);
}

void usart_stop_reception(usart_t *usart) {
    USCI_A_UART_disableInterrupt(usart->base_address,
  				 USCI_A_UART_RECEIVE_INTERRUPT
  				 );

}


void usart_rx_interrupt_handler(usart_t *usart, volatile buffer_t *buffer) {
  uint8_t rx_data;
  //  uint8_t next_state;
  //  uint8_t new_error=0;
  /* USCI_A_UART_clearInterruptFlag(usart->base_address, */
  /* 				 USCI_A_UART_RECEIVE_INTERRUPT); */


  rx_data = USCI_A_UART_receiveData(usart->base_address);


  buffer->data[buffer->fill] = rx_data;
  buffer->fill++;
  buffer->fill = buffer->fill % buffer->size;
  


  /* switch (buffer->status & USART_STATUS_STATE_MASK) { */
  /* case USART_STATUS_WAIT_PREAMBLE: */
  /*   if (USART_PREAMBLE == rx_data) { */
  /*     next_state = USART_STATUS_WAIT_BYTE_COUNT; */
  /*     buffer->preamble = rx_data; */
  /*   } else { */
  /*     usart_init_reception(usart,buffer); */
  /*   } */
  /*   break; */

  /* case USART_STATUS_WAIT_BYTE_COUNT: */
  /*   buffer->remaining_bytes = rx_data; */
  /*   if (buffer->remaining_bytes > 0 ) { */
  /*     next_state =  USART_STATUS_WAIT_PAYLOAD; */
  /*   } else { */
  /*     next_state = USART_STATUS_FRAME_COMPLETE; */
  /*   } */
  /*   break; */

  /* case USART_STATUS_WAIT_PAYLOAD: */
  /*   buffer->remaining_bytes --; */
  /*   /\* */
  /*     always consume remaining bytes, even if not possible to store */
  /*    *\/ */
  /*   if(buffer->payload->fill >= buffer->payload->size) { */
  /*     new_error = USART_STATUS_BUFFER_OVERFLOW; */
  /*   } else { */

  /*   } */
  /*   if(0==buffer->remaining_bytes){ */
  /*     next_state = USART_STATUS_FRAME_COMPLETE; */
  /*   } */
  /*   break; */
  /* default: */
  /*   break; */
  /* } */

  /* if(USART_STATUS_FRAME_COMPLETE ==  next_state) { */
  /*   USCI_A_UART_disableInterrupt(usart->base_address, */
  /* 				 USCI_A_UART_RECEIVE_INTERRUPT */
  /* 				 ); */
  /* } */

  /* buffer->status &= ~USART_STATUS_STATE_MASK; */
  /* buffer->status |= next_state; */
  /* buffer->status |= new_error & USART_STATUS_ERROR_MASK; */

  /* return; */

}


int usart_putchar(usart_t *usart, int s) {
  USCI_A_UART_transmitData(usart->base_address,
			   (char) s);

  while(!USCI_A_UART_getInterruptStatus(usart->base_address, 
					USCI_A_UART_TRANSMIT_INTERRUPT_FLAG )
	); 


  return(s);
}

int usart_transmit_init(usart_t *usart, volatile int * index, volatile const buffer_t *buffer) {
  USCI_A_UART_disableInterrupt(usart->base_address,
				   USCI_A_UART_TRANSMIT_INTERRUPT);
  *index = 0;
}

int usart_transmit_buffer(usart_t *usart, volatile int * index, volatile const buffer_t *buffer) {

  if (0 == *index ) {
    USCI_A_UART_clearInterruptFlag(usart->base_address,
				   USCI_A_UART_TRANSMIT_INTERRUPT);
    USCI_A_UART_enableInterrupt(usart->base_address,
				USCI_A_UART_TRANSMIT_INTERRUPT);

    *index=buffer->fill;
    return USART_STATUS_TRANSMISSION_IDLE;
  } else {
    return USART_STATUS_TRANSMISSION_ONGOING;
  }
}


void usart_tx_interrupt_handler(usart_t *usart, volatile int * index,  volatile const buffer_t *buffer) {
  uint8_t rx_data;
  //  uint8_t next_state;
  //  uint8_t new_error=0;
  USCI_A_UART_clearInterruptFlag(usart->base_address,
  				 USCI_A_UART_RECEIVE_INTERRUPT);

  if(0==*index) {
    USCI_A_UART_disableInterrupt(usart->base_address,
				 USCI_A_UART_TRANSMIT_INTERRUPT);
  } else {
    
    USCI_A_UART_transmitData(usart->base_address, (char) buffer->data[buffer->fill-*index]);
    *index--;
  }
}
