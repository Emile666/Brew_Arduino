/*==================================================================
  File Name    : $Id$
  Function name: pwm_init(), pwm_write()
  Author       : E v.d. Logt
  ------------------------------------------------------------------
  Purpose : PWM routines
  ------------------------------------------------------------------
  $Log$
  Revision 1.3  2014/05/03 11:27:44  Emile
  - Ethernet support added for W550io module
  - No response for L, N, P, W commands anymore
  - All source files now have headers

  ================================================================== */ 
#include "pwm.h"

/*************************************************************************
  Initializes the PWM function
  - Use OC1A as output pin for HLT-PWM signal
  - Use OC1B as output pin for BOIL-PWM signal
  - PWM frequency = 25 kHz
  - Prescaler: divide by 8: 16 MHz / 8 = 2 MHz
  - Use ICR1=80 as TOP value: 2 MHz / 80 = 25 kHz PWM frequency
  - Give OCR1A and OCR1B a value between 0 % and 100 %
  Input:    -
  Return:   - 
*************************************************************************/
void pwm_init(void)
{
   DDRB   |= (1<<DDB1) | (1<<DDB2); // PB1/OC1A and PB2/OC1B are now outputs
   ICR1    = TMR1_CNT_MAX;          // Set ICR1 value as max. value (100)
   TCCR1A  = TCCR1A_VAL;
   TCCR1B  = TCCR1B_VAL;
} // pwm_init()

void pwm_write(uint8_t pwm_ch, uint8_t duty_cycle) // duty_cyle varies 0 % and 100 %
{
    uint8_t temp = 100 - ((uint16_t)duty_cycle << 2) / 5;
	// A low-value pulls the base of the BC640 transistor low, so that
	// it starts to conducts. Inverted Logic!
	// Since the TOP value (in ICR1) is 80, the duty_cycle is converter from 100 to 80.
	if (pwm_ch == PWMB)
	     OCR1B = temp;
	else OCR1A = temp;
} // pwm_write()

