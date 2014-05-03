#ifndef _DELAY_H_
#define _DELAY_H_
//-------------------------------------------------------------------------
// The functions delay_msec() and millis() and implementation dependent and
// normally defined in the same file as where main() is defined.
// For this project it is located in Brew_Arduino.c
//-------------------------------------------------------------------------
unsigned long millis(void);
void          delay_msec(uint16_t ms);
#endif /* _DELAY_H_ */