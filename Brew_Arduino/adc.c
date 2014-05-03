/*==================================================================
  File Name    : $Id$
  Function name: adc_init(), adc_read()
  Author       : E v.d. Logt
  ------------------------------------------------------------------
  Purpose : ADC routines
  ------------------------------------------------------------------
  $Log$
  ================================================================== */ 
#include "adc.h"

void adc_init(void)
{
	DIDR0   = (1<<ADC0D) | (1<<ADC1D) | (1<<ADC2D); // disable digital inputs
	ADMUX  |= (1<<REFS1) | (1<<REFS0); // Select Internal Vref (1.1 Volt)
	//ADMUX  |=  (1<<REFS0); // Select AVcc (5 Volt)
	ADCSRA |= ((1<<ADPS2) | (1<<ADPS1) | (1<<ADPS0) | (1<<ADEN)); // set ADC-CLK = 125 kHz
} // adc_init()

uint16_t adc_read(uint8_t adc_ch)
{
	ADMUX = (ADMUX & 0xf0) | (adc_ch & 0x0f); // select ADC channel
	ADCSRA |= (1<<ADSC); // single-conversion mode
	while (ADCSRA & (1<<ADSC)) ;
	return ADC;
} // adc_read()

