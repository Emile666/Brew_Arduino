#ifndef _ADC_H_
#define _ADC_H_
#include <avr/io.h>

#define ADC_CH0 (0)
#define ADC_CH1 (1)
#define ADC_CH2 (2)
#define ADC_CH3 (3)
#define ADC_CH4 (4)
#define ADC_CH5 (5)
#define ADC_CH6 (6)
#define ADC_CH7 (7)
#define ADC_CH8 (8)

//-------------------------------------------------------------------------
// ADC Channel Definitions
//-------------------------------------------------------------------------
#define VMLT (ADC_CH0)
#define VHLT (ADC_CH1)
#define LM35 (ADC_CH2)

void     adc_init();
uint16_t adc_read(uint8_t adc_ch);

#endif /* _ADC_H_ */