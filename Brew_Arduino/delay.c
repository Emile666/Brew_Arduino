/*==================================================================
   Created: 07/12/2011 15:17:35
   Author : Emile
   File   : delay.c
  ================================================================== */
#include "delay.h"

/******************************************************************
    Delay loop using a 16-bit counter \c __count, so up to 65536
    iterations are possible.  (The value 65536 would have to be
    passed as 0.)  The loop executes four CPU cycles per iteration,
    not including the overhead the compiler requires to setup the
    counter register pair.

    Thus, at a CPU speed of 16 MHz, delays of up to about 16
    milliseconds can be achieved.
 ******************************************************************/
void delay_loop_2(uint16_t __count)
{
	__asm__ volatile (
		"1: sbiw %0,1" "\n\t"
		"brne 1b"
		: "=w" (__count)
		: "0" (__count)
	);
} // delay_loop_2()

void delay_usec(uint16_t us)
{
	uint8_t  ticks = TICKS_PER_USEC * us;

	if (us >= 1000)
	{
		delay_msec(us / 1000);
		return;
	} // if
	delay_loop_2(ticks);
} // delay_usec()

