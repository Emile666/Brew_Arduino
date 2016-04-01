/*==================================================================
  File Name    : $Id$
  Function name: pwm_init(), pwm_write()
  Author       : E v.d. Logt
  ------------------------------------------------------------------
  Purpose : PWM routines
  ------------------------------------------------------------------
  $Log$
  Revision 1.4  2016/01/10 16:00:24  Emile
  First version (untested!) for new HW PCB 3.30 with 4 x temperature, 4 x flowsensor and 2 PWM outputs.
  - Added: owb_task(), owc_task(), tcfc_ and tboil_ variables. Removed: vhlt_ and vhlt_ variables.
  - A5..A8 commands added (flowsensors), A1..A4 commands re-arranged.
  - Wxxx command is now Hxxx command, new Bxxx command added.
  - pwm_write() and pwm_2_time() now for 2 channels (HLT and Boil): OCR1A and OCR1B timers used.
  - SPI_SS now from PB2 to PD7 (OC1B/PB2 used for 2nd PWM signal). PWM freq. now set to 25 kHz.
  - PCINT1_vect now works with 4 flowsensors: flow_cfc_out and flow4 added.
  - MCP23017 instead of MCP23008: PORTB used for HLT_NMOD, HLT_230V, BOIL_NMOD, BOIL_230V and PUMP_230V.
  - set_parameter(): parameters 7-12 removed.

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
	uint8_t temp = 100 - duty_cycle;
	//if (temp < 100) temp++; // compensate for small offset in duty-cycle
	// A low-value pulls the base of the BC640 transistor low, so that
	// it starts to conducts. Inverted Logic!
	// Since the TOP value (in ICR1) is 80, the duty_cycle is converted from 100 to 80.
    temp = (((uint16_t)temp << 3) + 5) / 10;
	if (pwm_ch == PWMB)
	     OCR1B = temp;
	else OCR1A = temp;
} // pwm_write()

