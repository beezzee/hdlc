#include "driverlib.h"
#include "timer.h"



void timer_isr(timer_t* t) {
  //increase most significant bytes of time
  t->time_most_significant++;
  TIMER_B_clearTimerInterruptFlag(TIMER_B0_BASE);
}

void timer_reset(timer_t* t) {
  TIMER_B_clearTimerInterruptFlag(TIMER_B0_BASE);
  TIMER_B_clear(TIMER_B0_BASE);
  t->time_most_significant=0;
}

void timer_stop(timer_t* t) {
  TIMER_B_stop(TIMER_B0_BASE);
}

void timer_init(timer_t* t) {
 
  //Start timer

  /* TIMER_B_configureUpMode(   TIMER_B0_BASE, */
  /*                            TIMER_B_CLOCKSOURCE_SMCLK, */
  /*                            TIMER_B_CLOCKSOURCE_DIVIDER_32, */
  /*                            time_out_value_ms, */
  /*                            TIMER_B_TBIE_INTERRUPT_DISABLE, */
  /*                            TIMER_B_CAPTURECOMPARE_INTERRUPT_ENABLE, */
  /*                            TIMER_B_DO_CLEAR */
  /*                            ); */
  timer_stop(t);

  TIMER_B_configureContinuousMode(   TIMER_B0_BASE,
				     TIMER_B_CLOCKSOURCE_ACLK,
				     TIMER_B_CLOCKSOURCE_DIVIDER_32,
				     TIMER_B_TBIE_INTERRUPT_ENABLE,
				     TIMER_B_DO_CLEAR
				     );


  timer_reset(t);

}



void timer_start(timer_t* t) {
  TIMER_B_startCounter(
		       TIMER_B0_BASE,
		       //			 TIMER_B_UP_MODE
		       TIMER_B_CONTINUOUS_MODE
		       );
}

uint32_t timer_current_time(timer_t* t) { 
  uint32_t result;
  result = (uint32_t) t->time_most_significant;
  result = result << 16;
  result |= TIMER_B_getCounterValue(TIMER_B0_BASE);
  return result;
}

int timer_timeout(timer_t *t,uint32_t to) {
  return to<timer_current_time(t);
}

uint32_t timer_in_future(timer_t *timer,uint16_t v) {
  return (timer_current_time(timer) + (v*((uint32_t) 1024)));
}
