#ifndef _PWM_H_
#define _PWM_H_
#include <avr/io.h>

#define USEC_PER_CLOCKTICK  (50)
#define TMR1_CNT_MAX       (100)

void  pwm_init(void);
void  pwm_write(uint8_t duty_cycle); // duty_cyle varies 0 % and 100 %

#endif /* _PWM_H_ */