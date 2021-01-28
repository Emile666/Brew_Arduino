/*==================================================================
  File Name    : pwm.h
  Author       : Emile
  ------------------------------------------------------------------
  Purpose : Header file for PWM routines
  ------------------------------------------------------------------
  $Log$
  Revision 1.5  2014/05/03 11:27:44  Emile
  - Ethernet support added for W550io module
  - No response for L, N, P, W commands anymore
  - All source files now have headers

  ================================================================== */ 
#ifndef _PWM_H_
#define _PWM_H_
#include <avr/io.h>

#define PWMB (0)
#define PWMH (1)

//--------------------------------------------------
// Timer 1 is used for 2 purposes:
// 1) Generation of 25 kHz frequency for PWM signal
// 2) Time-measurement for task-scheduler
//--------------------------------------------------
#define USEC_PER_CLOCKTICK  (50)
#define CLOCKTICKS_PER_MSEC (1000/USEC_PER_CLOCKTICK)
#define CLOCKTICKS_E_2_MSEC (USEC_PER_CLOCKTICK/10)
#define TMR1_CNT_MAX        (80)

//--------------------------------------------------------------
// TCCR1A - Timer Counter Control Register 1A
// ------------------------------------------
// BIT 7 : COM1A1 [1] Clear OC1A on compare match,
// BIT 6 : COM1A0 [0] set OC1A at bottom (non-inverting mode)
// BIT 5 : COM1B1 [1] Clear OC1B on compare match,
// BIT 4 : COM1B0 [0] set OC1B at bottom (non-inverting mode)
// BIT 3 : -
// BIT 2 : -
// BIT 1 : WGM11 [1] Mode 14 Fast PWM with TOP == ICR1
// BIT 0 : WGM10 [0] Mode 14 Fast PWM with TOP == ICR1
// 
// TCCR1B - Timer Counter Control Register 1B
// ------------------------------------------
// BIT 7 : ICNC1, not used
// BIT 6 : ICES1, not used
// BIT 5 : -
// BIT 4 : WGM13 [1] Mode 14 Fast PWM with TOP == ICR1
// BIT 3 : WGM12 [1] Mode 14 Fast PWM with TOP == ICR1
// BIT 2 : CS12  [0] Set prescaler to divide by 8
// BIT 1 : CS11  [1] Timer-clock is then 2 MHz
// BIT 0 : CS10  [0]
//--------------------------------------------------------------
#define TCCR1A_VAL (0xA2)
#define TCCR1B_VAL (0x1A)

void pwm_init(void);
void pwm_write(uint8_t pwm_ch, uint8_t duty_cycle); // duty_cyle varies 0 % and 100 %

#endif /* _PWM_H_ */