#ifndef _timer_h
#define _timer_h

#include <msp430.h>

typedef struct timer_struct {
  volatile uint16_t time_most_significant;
} timer_t;

#define time_to_seconds(t) ( t >> 10 )


void timer_isr(timer_t * t);

void timer_reset(timer_t * t);

void timer_stop(timer_t * t);

void timer_init(timer_t * t);

void timer_start(timer_t * t);

uint32_t timer_current_time(timer_t * t);

int timer_timeout(timer_t *t,uint32_t to);

/**
   Returns the value the timer will have in v seconds.

   This function is useful to register a timeout and determine the
   value of the timer at the timeout.
 */
uint32_t timer_in_future(timer_t *timer,uint16_t v);

#endif //_timer_h
