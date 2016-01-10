/*==================================================================
  File Name    : $Id$
  Function name: -
  Author       : E v.d. Logt
  ------------------------------------------------------------------
  Purpose : Header file for ADC routines
  ------------------------------------------------------------------
  $Log$
  Revision 1.3  2014/05/03 11:27:44  Emile
  - Ethernet support added for W550io module
  - No response for L, N, P, W commands anymore
  - All source files now have headers

  ================================================================== */ 
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
#define LM35 (ADC_CH6)

void     adc_init();
uint16_t adc_read(uint8_t adc_ch);

#endif /* _ADC_H_ */