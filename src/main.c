/* --COPYRIGHT--,BSD
 * Copyright (c) 2014, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * --/COPYRIGHT--*/
//******************************************************************************
//!
//! ADC12_A - Sample A0 Input, AVcc Ref, Repeated Single Conversion
//!
//!  This example shows how to perform repeated conversions on a single channel
//!  using "repeat-single-channel" mode.  AVcc is used for the reference and
//!  repeated conversions are performed on Channel A0. Each conversion result
//!  is moved to an 8-element array called results[].  Test by applying a
//!  voltage to channel A0, then running. Open a watch window in debugger and
//!  view the results. Set Breakpoint1 in the index increment line to see the
//!  array value change sequentially and Breakpoint to see the entire array of
//!  conversion results in "results[]" for the specified Num_of_Results. This
//!  can run even in LPM4 mode as ADC has its own clock.
//!
//!                MSP430F552x
//!             -----------------
//!         /|\|                 |
//!          | |                 |
//!          --|RST       P6.0/A0|<- Vin
//!            |                 |
//!
//!
//! This example uses the following peripherals and I/O signals.  You must
//! review these and change as needed for your own board:
//! - ADC12_A peripheral
//! - GPIO Port peripheral
//! - A0
//!
//! This example uses the following interrupt handlers.  To use this example
//! in your own application you must add these interrupt handlers to your
//! vector table.
//! - ADC12_A_VECTOR
//!
//******************************************************************************

#include <msp430.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>


#include "driverlib.h"

#include "timer.h"
#include "usart.h"
#include "hdlc.h"

#define motor_down_pin GPIO_PIN1
#define motor_up_pin GPIO_PIN2
#define motor_port GPIO_PORT_P1

#define led_1_port GPIO_PORT_P1
#define led_1_pin GPIO_PIN0

#define led_2_port GPIO_PORT_P4
#define led_2_pin GPIO_PIN7

#define log_usart_port GPIO_PORT_P4
#define log_usart_rx_pin GPIO_PIN5 
#define log_usart_tx_pin GPIO_PIN4

#define xt2_port GPIO_PORT_P5
#define xt2_input_pin GPIO_PIN2
#define xt2_output_pin GPIO_PIN3



#define log_usart_base USCI_A1_BASE
usart_t log_usart;

#define clock_source_mclk UCS_DCOCLK_SELECT 
//#define clock_source_mclk UCS_XT2CLK_SELECT
#define clock_divider_mclk UCS_CLOCK_DIVIDER_1

#define clock_source_smclk UCS_XT2CLK_SELECT
#define clock_divider_smclk UCS_CLOCK_DIVIDER_1

#define clock_source_amclk UCS_XT1CLK_SELECT
#define clock_divider_amclk UCS_CLOCK_DIVIDER_1



#define xt1_freq UCS_REFOCLK_FREQUENCY 
#define xt2_freq 40000000
#define xt2_drive_strength UCS_XT2DRIVE_4MHZ_8MHZ
//#define xt2_drive_strength UCS_XT2DRIVE_24MHZ_32MHZ


#define adc_port GPIO_PORT_P6
#define adc_pin  GPIO_PIN0
#define adc_reference_voltage REF_VREF2_0V

#define  log_temperature_buffer_size   3
#define  temperature_buffer_size   1 << log_temperature_buffer_size   

#define UART_PRINTF

//#define time_out_value_ms 60000

/**
   0 degree celsius equals 27315E-2 degree Kelvin
 */
#define zero_degree_celsius 27315

/**
   calibration temperature in Milli Kelvin
 */
//#define calibration_temperature_mk 100E3 + zero_degree_celsius_mk
#define calibration_temperature 2030 + zero_degree_celsius

#define temp_calibration_port GPIO_PORT_P2
#define temp_calibration_pin GPIO_PIN1

#define timer_start_port GPIO_PORT_P1
#define timer_start_pin GPIO_PIN1

#define event_no_event 0
#define event_timeout_mask 1



volatile uint16_t temperature_buffer[temperature_buffer_size];
volatile int event;

uint16_t temperature_slope=100;

timer_t timer;


/**
   The size of a single flash segment
*/
#define flash_segment_size 128


/**
   Start adresses of flash segment Info A
 */
#define flash_info_a_addr   (0x1980)


/**
   Start adresses of flash segment Info B
 */
#define flash_info_b_addr   (0x1900)

/**
   Start adresses of flash segment Info C
 */
#define flash_info_c_addr   (0x1880)

/**
   Start adresses of flash segment Info D
 */
#define flash_info_d_addr   (0x1800)

/** 
    Adress of temperature calibration value
*/
#define flash_temperature_calibration_addr ((uint16_t*) flash_info_c_addr)

/**
   The largest timestamp that we can handle
 */
#define TIMEOUT_MAX 0xffffffff

/** 
    The time between two status updates.
 */
#define log_interval   500

#define brewing_time_s 150

//Needs to be global in this
//example. Otherwise, the
//compiler removes it because it
//is not used for anything.

#define HDLC_BUFFER_SIZE ((uint16_t) 256)

/**
   Size of usart ring buffer for buffering incomming data. 

   It needs to be large enough to buffer interrupt based incomming
   data when receiving a frame and the processor is blocked by another
   task.
 */
#define USART_RX_BUFFER_SIZE ((uint16_t) 16)


/**
   Size of the usart output buffer.

   It has to be large enough to hold the largest chunk of data that
   has to be transmitted, for example by print functions.
 */
#define USART_TX_BUFFER_SIZE ((uint16_t) 256)

buffer_t volatile usart_rx_buffer;

usart_t cmd_usart;

//int usart_tx_index;

buffer_t volatile usart_tx_buffer;

void memset_16(void *block, int c, size_t size){
  memset(block,c,2*size);
}

void motor_stop(void) {
  //Set all P1 pins HI
  GPIO_setOutputHighOnPin(
			  motor_port,motor_down_pin+motor_up_pin             );
}

void motor_down(void) {
  motor_stop();
  GPIO_setOutputLowOnPin(motor_port,motor_down_pin);
}

void motor_up(void) {
  motor_stop();
  GPIO_setOutputLowOnPin(motor_port,motor_up_pin);
}

void ports_init(void) {
			    
  motor_stop();
  //Set P1.x to output direction
  GPIO_setAsOutputPin(
		      motor_port,motor_down_pin+motor_up_pin
		      );
  GPIO_setAsOutputPin(
		      led_1_port,led_1_pin);

  GPIO_setAsOutputPin(
		      led_2_port,led_2_pin);

  GPIO_setAsInputPinWithPullUpResistor(
		     temp_calibration_port,
		     temp_calibration_pin);

	
}

void clocks_init(void) {

  if(
     (UCS_XT2CLK_SELECT == clock_source_smclk ) ||
     (UCS_XT2CLK_SELECT == clock_source_amclk ) ||
     (UCS_XT2CLK_SELECT == clock_source_mclk ) 
     ) 
    {
      /*
	configure XT2 pins for special function
      */
      GPIO_setAsPeripheralModuleFunctionInputPin(xt2_port,xt2_input_pin);
      GPIO_setAsPeripheralModuleFunctionOutputPin(xt2_port,xt2_output_pin);

      UCS_XT2Start(xt2_drive_strength);
    };

  UCS_clockSignalInit(UCS_SMCLK,clock_source_smclk,clock_divider_smclk);
  UCS_clockSignalInit(UCS_MCLK,clock_source_mclk,clock_divider_mclk);
  UCS_clockSignalInit(UCS_ACLK,clock_source_amclk,clock_divider_amclk);

  UCS_setExternalClockSource(xt1_freq,xt2_freq);


  /* if (clock_source_master_external) { */
    
  


  /*   if (UCS_faultFlagStatus(UCS_XT2OFFG)){ */
  /*     printf("Failed to enable XT2. Stay at internal clock.\n"); */
  /*   } else { */
  /*     UCS_clockSignalInit(UCS_MCLK,clock_source_master,clock_divider_master); */
  /*   } */
  /* } */
}

void adc_init(void) {

  //Enable A/D channel A0
  GPIO_setAsPeripheralModuleFunctionInputPin(adc_port,
					     adc_pin
					     );

  //Initialize the ADC12_A Module
  /*
   * Base address of ADC12_A Module Use internal ADC12_A bit as
   * sample/hold signal to start conversion USE SMCLK Digital
   * Oscillator as clock source Use clock divider of 32. With an
   * external frequency of 32khz, this gives a sampling frequency of
   * 1kHz.
   */
  ADC12_A_init(ADC12_A_BASE,
	       ADC12_A_SAMPLEHOLDSOURCE_SC,
	       //	       ADC12_A_CLOCKSOURCE_ADC12OSC,
	       ADC12_A_CLOCKSOURCE_ACLK,
	       //ADC12_A_CLOCKSOURCE_SMCLK,
	       ADC12_A_CLOCKDIVIDER_8
	       //ADC12_A_CLOCKDIVIDER_32
	       );

  ADC12_A_enable(ADC12_A_BASE);

  /*
   * Base address of ADC12_A Module
   * For memory buffers 0-7 sample/hold for 128 clock cycles
   * For memory buffers 8-15 sample/hold for128 clock cycles (default)
   * Enable Multiple Sampling
   */
  ADC12_A_setupSamplingTimer(ADC12_A_BASE,
			     ADC12_A_CYCLEHOLD_4_CYCLES,
			     ADC12_A_CYCLEHOLD_4_CYCLES,
			     ADC12_A_MULTIPLESAMPLESENABLE);


  REF_setReferenceVoltage(REF_BASE,adc_reference_voltage);
  REF_enableReferenceVoltage(REF_BASE);
    
  //Configure Memory Buffer
  /*
   * Base address of the ADC12_A Module
   * Configure memory buffer 0
   * Map input A0 to memory buffer 0
   * Vref+ = AVcc
   * Vref- = AVss
   * Memory buffer 0 is not the end of a sequence
   */
  ADC12_A_memoryConfigure(ADC12_A_BASE,
			  ADC12_A_MEMORY_0,
			  ADC12_A_INPUT_A0,
			  ADC12_A_VREFPOS_INT,
			  ADC12_A_VREFNEG_AVSS,
			  ADC12_A_NOTENDOFSEQUENCE);

  //Enable memory buffer 0 interrupt
  ADC12_A_clearInterrupt(ADC12_A_BASE,
			 ADC12IFG0);
  ADC12_A_enableInterrupt(ADC12_A_BASE,
			  ADC12IE0);

  //Enable/Start first sampling and conversion cycle
  /*
   * Base address of ADC12_A Module
   * Start the conversion into memory buffer 0
   * Use the repeated single-channel
   */
  ADC12_A_startConversion(ADC12_A_BASE,
			  ADC12_A_MEMORY_0,
			  ADC12_A_REPEATED_SINGLECHANNEL);

}


#define usart_printf printf 

/* void usart_printf(const char *format, ...) { */

/*   va_list arg; */
/*   int done; */
/*   uint8_t ctrl, psctrl; */



/*   /\*pass arguments through to the system printf*\/ */
/*   va_start (arg, format); */
/*   done = vprintf (format, arg); */
/*   va_end (arg); */

/*   fflush(stdout); */
				 
/*   while(USCI_A_UART_queryStatusFlags 	(log_usart_base,USCI_A_UART_BUSY )); */
				 
		
/* } */


/**
   Function that writes one character to the selected USART.

   The special character \\n is forwarded as \\r to \a stream.
*/
/* int usart_putchar(char c, FILE *s) { */
/*   // Load data onto buffer */



/*   return 0; */
/* } */

int putchar(int s)
{
  return usart_putchar(&log_usart,s);
}

/* static FILE usart_out = FDEV_SETUP_STREAM( usart_putchar,  */
/* 					   NULL,  */
/* 					   _FDEV_SETUP_WRITE ); */

void rtc_init(void) {

}


void lcd_init(void) {

}

void temperature_update(uint16_t *tmp, volatile uint16_t *tmp_buffer, int log_buffer_length) {
  int i;

  *tmp=0;

  for(i=0;i<(1<<log_buffer_length);i++) {
    *tmp+=tmp_buffer[i];
  }

  *tmp = *tmp >> log_buffer_length;
  
}

/**
writing to segment A currently not supported because unlocking is
required. Segment A contains calibration information of device and
should not be overwritten.
 */
void flash_update_word(const uint16_t* addr, uint16_t value) {
  
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



void main(void)
{
  unsigned int i;
  uint32_t temperature;
  uint32_t time;
  uint16_t voltage;
  uint16_t voltage_at_calibration=0;
  const uint16_t *calibration_voltage_flash_ptr = flash_temperature_calibration_addr;
  int brewing = 0;



#define TASK_STATUS_LOG 0
#define TASK_START_BREW 1
#define TASK_STOP_BREW 2
#define TASK_CNT 3


  uint32_t timeouts[TASK_CNT];

  uint8_t hdlc_buffer_data[HDLC_BUFFER_SIZE];
  buffer_t hdlc_buffer;
  buffer_t cmd_buffer;
  
  int hdlc_read_index;

  uint8_t usart_rx_buffer_data[USART_RX_BUFFER_SIZE];

  uint8_t usart_tx_buffer_data[USART_TX_BUFFER_SIZE];

  //  buffer_t hdlc_buffer_payload;

  log_usart.base_address = log_usart_base;
  log_usart.port = log_usart_port;
  log_usart.rx_pin = log_usart_rx_pin;
  log_usart.tx_pin = log_usart_tx_pin;

  /*
    Temporarily, use the same usart for both, logging and for command
    exchange.
   */
  cmd_usart = log_usart;

  hdlc_buffer.size = HDLC_BUFFER_SIZE;
  hdlc_buffer.data = hdlc_buffer_data;

  /*
    The command buffer will point to the payload of the HDLC
    frame. The payload starts at 3rd byte (excluding address and
    control byte).
   */
  cmd_buffer.size = hdlc_buffer.size - 2;
  cmd_buffer.data = hdlc_buffer.data + 2;

  usart_rx_buffer.size = USART_RX_BUFFER_SIZE;
  usart_rx_buffer.data = usart_rx_buffer_data;

  usart_tx_buffer.size = USART_TX_BUFFER_SIZE;
  usart_tx_buffer.data = usart_tx_buffer_data;

  //  hdlc_buffer.payload = &hdlc_buffer_payload;

  //  stdout = &usart_out;
  //Stop Watchdog Timer
  WDT_A_hold(WDT_A_BASE);



  clocks_init();

  ports_init();
	
  adc_init();

  usart_init(&log_usart);


  timer_init(&timer);


  timer_start(&timer);


  printf("\n Booting..\n");

  memset_16((void *) temperature_buffer,0,temperature_buffer_size);



  //initialize calibration voltage from flash
  voltage_at_calibration = *calibration_voltage_flash_ptr;

  printf("Loaded calibration voltage %4x\n",voltage_at_calibration);

  //Enter LPM4, Enable interrupts
  //  __bis_SR_register(LPM4_bits + GIE);
   __bis_SR_register(GIE);

        //Enter LPM0, enable interrupts
  //     __bis_SR_register(LPM0_bits + GIE);
  // __enable_interrupt();

  //For debugger
  __no_operation();

  usart_printf("Start ... \n");

  event = event_no_event;


  timeouts[TASK_STATUS_LOG]=1;
  timeouts[TASK_START_BREW]=TIMEOUT_MAX;
  timeouts[TASK_STOP_BREW]=TIMEOUT_MAX;

  i=0;
  
  
  usart_start_reception(&cmd_usart);
  //  usart_transmit_init(&cmd_usart,&usart_tx_index,&usart_tx_buffer);
  hdlc_init_reception(&hdlc_buffer,&hdlc_read_index, &usart_rx_buffer);
  while(1) {

    //logging task
    if(0){
    // if (timer_timeout(&timer,timeouts[TASK_STATUS_LOG])){
      temperature_update(&voltage,temperature_buffer,log_temperature_buffer_size);
      /* usart_printf("\rTemperature: %7u mK (%7i mC), Time: %10u ms", */
      /* 		   temperature*temperature_slope, */
      /* 		   temperature*temperature_slope-zero_degree_celsius_mk, */
      /* 		   time_out_value_ms-timer_current_time() */
      /* 		   ); */

      temperature = (((uint32_t) calibration_temperature)*((uint32_t) voltage));
      temperature/=voltage_at_calibration;
      if(brewing) {
	time = timeouts[TASK_STOP_BREW]-timer_current_time(&timer);
      } else {
	time =   (brewing_time_s * ((uint32_t) 1024));
      }
      usart_printf("\rTemperature: %3lu.%02lu K , Time: %7lu.%03lu s",
      		   temperature/100,temperature%100,
		   //		   (temperature-zero_degree_celsius_mk)/1000,
		   // (temperature-zero_degree_celsius_mk)%1000,
   		   time/((uint32_t)1024),
      		   time & ((1<<10) -1)
      		   );
      //      time = timer_current_time(&timer);
      // usart_printf("\n %lu\n",time);
      //      usart_printf("\rTemperature: %10u C",temperature);
      GPIO_toggleOutputOnPin(led_1_port,led_1_pin);
      timeouts[TASK_STATUS_LOG]+=log_interval;
    }


    if(timer_timeout(&timer,timeouts[TASK_STOP_BREW])) {
      usart_printf("\nTimeout detected\n");
      timeouts[TASK_STOP_BREW]=TIMEOUT_MAX;
      brewing = 0;
    }


    //handler for BUTTON events

    if(GPIO_INPUT_PIN_LOW == GPIO_getInputPinValue(
    		     temp_calibration_port,
    		     temp_calibration_pin)) {
      temperature_update(&voltage,temperature_buffer,log_temperature_buffer_size);
      voltage_at_calibration = voltage;
      printf("\n Temperature calibration: %4x units\n",voltage_at_calibration);

      printf("Program into flash...\n");
      flash_update_word(calibration_voltage_flash_ptr,voltage_at_calibration);
      printf("Read from flash: %4x\n",*calibration_voltage_flash_ptr);
    }

    if(GPIO_INPUT_PIN_LOW == GPIO_getInputPinValue(
    		     timer_start_port,
    		     timer_start_pin)) {
      usart_printf("\nReset and start timer at button release.\n");
      while (GPIO_INPUT_PIN_LOW == GPIO_getInputPinValue(
    		     timer_start_port,
    		     timer_start_pin));
      timeouts[TASK_STOP_BREW]= (timer_current_time(&timer) + (brewing_time_s*((uint32_t) 1024)));
      brewing = 1;
    }
    
    switch (hdlc_update_rx_buffer(&hdlc_buffer,&hdlc_read_index, &usart_rx_buffer)) {
    case HDLC_STATUS_FRAME_COMPLETE:
      //if(USART_STATUS_WAIT_PREAMBLE != hdlc_buffer.status) {
      printf("\nFrame received: ");
      for(i=0;i<hdlc_buffer.fill;i++) {
	printf("%02x ",hdlc_buffer.data[i]);
      }
      printf("\n");
      //      usart_init_reception(&cmd_usart,&hdlc_buffer);
      hdlc_init_reception(&hdlc_buffer,&hdlc_read_index, &usart_rx_buffer);
      break;
    case HDLC_STATUS_BUFFER_OVERFLOW_ERROR:
      printf("\nCommand buffer overflow\n");
      hdlc_init_reception(&hdlc_buffer,&hdlc_read_index, &usart_rx_buffer);
      break;
    case HDLC_STATUS_LISTEN:
      break;
    default:
      hdlc_init_reception(&hdlc_buffer,&hdlc_read_index, &usart_rx_buffer);
      break;
    }

  }
  
  
}

#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=ADC12_VECTOR
__interrupt
#elif defined(__GNUC__)
__attribute__((interrupt(ADC12_VECTOR)))
//__attribute__((interrupt(TIMER1_A1_VECTOR)))
#endif
void ADC12ISR(void)
{
  static uint8_t index = 0;

  switch (ADC12IV) {
  case  ADC12IV_NONE: break;         //Vector  0:  No interrupt
  case  ADC12IV_ADC12OVIFG: break;         //Vector  2:  ADC overflow
  case  ADC12IV_ADC12TOVIFG: break;         //Vector  4:  ADC timing overflow
  case  ADC12IV_ADC12IFG0:                //Vector  6:  ADC12IFG0
    //Move results
    temperature_buffer[index] =
      ADC12_A_getResults(ADC12_A_BASE,
			 ADC12_A_MEMORY_0);

    //Increment results index, modulo;
    //Set Breakpoint1 here and watch results[]
    index++;

    if (index == temperature_buffer_size)
      index = 0;
  default: break;
  }
  ADC12_A_clearInterrupt(ADC12_A_BASE,
			 ADC12IFG0);

}



#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=TIMERB1_VECTOR
__interrupt
#elif defined(__GNUC__)
__attribute__((interrupt(TIMERB1_VECTOR)))
#endif
void TIMERB1_ISR(void)
{
  timer_isr(&timer);
}


#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=USCI_A1_VECTOR
__interrupt
#elif defined(__GNUC__)
__attribute__((interrupt(USCI_A1_VECTOR)))
#endif
void USCI_A1_ISR(void)
{
  /*
    read to UCA1IV resets interrupt flag
  */

        switch (UCA1IV) {
	  //Vector 2 - RXIFG
        case 2:
	  usart_rx_interrupt_handler(&cmd_usart,&usart_rx_buffer);
	  break;
	case 4:
	  //TXIFG
	  //	  usart_tx_interrupt_handler(&cmd_usart,&usart_tx_index,&usart_tx_buffer);
	  break;
	case 6:
	  //TTIFG
	  break;
	case 8:
	  //TXCPTIFG
	  break;
        default: break;
        }
}
