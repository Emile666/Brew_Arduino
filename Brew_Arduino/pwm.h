/*==================================================================
  File Name    : $Id$
  Function name: -
  Author       : E v.d. Logt
  ------------------------------------------------------------------
  Purpose : Header file for PWM routines
  ------------------------------------------------------------------
  $Log$
  ================================================================== */ 
#ifndef _PWM_H_
#define _PWM_H_
#include <avr/io.h>

//--------------------------------------------------
// Timer 1 is used for 2 purposes:
// 1) Generation of 20 kHz frequency for PWM signal
// 2) Time-measurement for task-scheduler
//--------------------------------------------------
#define USEC_PER_CLOCKTICK  (50)
#define CLOCKTICKS_PER_MSEC (1000/USEC_PER_CLOCKTICK)
#define CLOCKTICKS_E_2_MSEC (USEC_PER_CLOCKTICK/10)
#define TMR1_CNT_MAX        (100)

void  pwm_init(void);
void  pwm_write(uint8_t duty_cycle); // duty_cyle varies 0 % and 100 %

#endif /* _PWM_H_ */