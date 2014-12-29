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

  //Enable Receive Interrupt
  USCI_A_UART_clearInterruptFlag(usart->base_address,
				 USCI_A_UART_RECEIVE_INTERRUPT);
  //  USCI_A_UART_enableInterrupt(base_address,
  //			      USCI_A_UART_RECEIVE_INTERRUPT);




}
