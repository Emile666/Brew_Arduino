/*
 * pwm.c
 *
 * Created: 20-4-2013 19:33:19
 *  Author: Emile
 */ 
#include "pwm.h"

/*************************************************************************
  Initializes the PWM function
  - Use OC1A as output pin
  - PWM frequency = 20 kHz
  - Prescaler: divide by 8: 16 MHz / 8 = 2 MHz
  - Use ICR1=100 as TOP value: 2 MHz / 100 = 20 kHz PWM frequency
  - Give OCR1A a value between 0 % and 100 %
  Input:    -
  Return:   - 
*************************************************************************/
void pwm_init(void)
{
   DDRB   |= (1<<DDB1);    // PB1/OC1A is now an output
   ICR1    = TMR1_CNT_MAX; // Set ICR1 value as max. value (100)
   TCCR1A |= (1<<COM1A1);  // Set non-inverting mode for OC1A
   TCCR1A |= (1<<WGM11);
   TCCR1A &= ~((1<<COM1A0) | (1<<WGM10));
   TCCR1B |= (1<<WGM12) | (1<<WGM13); // Set fast PWM with ICR1 as TOP
   TCCR1B |= (1<<CS11); // Set prescaler to divide by 8
   TCCR1B &= ~((1<<CS12) | (1<<CS10));
} // pwm_init()

void pwm_write(uint8_t duty_cycle) // duty_cyle varies 0 % and 100 %
{
	// A low-value pulls the base of the BC640 transistor low, so that
	// it starts to conducts. Inverted Logic!
	OCR1A = 100 - duty_cycle;
} // pwm_write()

