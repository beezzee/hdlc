#ifndef _timer_h
#define _timer_h

#include <msp430.h>

typedef struct timer_struct {
  uint16_t time_most_significant;
} timer_t;

void timer_isr(timer_t * t);

void timer_reset(timer_t * t);

void timer_stop(timer_t * t);

void timer_init(timer_t * t);

void timer_start(timer_t * t);

uint32_t timer_current_time(timer_t * t);

int timer_timeout(timer_t *t,uint32_t to);

#endif //_timer_h
