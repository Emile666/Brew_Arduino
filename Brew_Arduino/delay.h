#ifndef _DELAY_H_
#define _DELAY_H_

#include <inttypes.h>

#define TICKS_PER_USEC ((F_CPU) / 4e6)

void delay_loop_2(uint16_t __count);
void delay_usec(uint16_t us);
//-------------------------------------------------------------------------
// The functions delay_msec() and millis() and implementation dependent and
// normally defined in the same file as where main() is defined.
// For this project it is located in Brew_Arduino.c
//-------------------------------------------------------------------------
unsigned long millis(void);
void          delay_msec(uint16_t ms);
#endif /* _DELAY_H_ */