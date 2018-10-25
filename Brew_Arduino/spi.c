/*==================================================================
  File Name    : spi.c
  Author       : E v.d. Logt
  ------------------------------------------------------------------
  Purpose : SPI-bus low-level routines
  ================================================================== */ 
#include "spi.h"

// Initialize pins for SPI communication
// MCP23S08: CPOL, CPHA = (0,0) /* (0,1) does NOT work! */
void spi_init(void)
{
	PORTD |= (1<<DD_SS);               // Disable Slave-Select
	DDRD  |= (1<<DD_SS);	 	       // Set Slave-Select as output-pin
	
	SPCR = ((1 << SPE)  |               // SPI Enable
  	        (0 << SPIE) |               // SPI Interrupt Enable
	        (0 << DORD) |               // Data Order (0:MSB first / 1:LSB first)
		    (1 << MSTR) |               // Master/Slave select
	        (0 << SPR1) | (1 << SPR0)|  // SPI Clock Rate
	        (0 << CPOL) |               // Clock Polarity (0:SCK low / 1:SCK hi when idle)
	        (0 << CPHA));               // Clock Phase (0:leading / 1:trailing edge sampling)

	SPSR = (1 << SPI2X);                // Double Clock Rate; SPI-clock is now 2 MHz

	// Set direction register for SCK and MOSI pin. MISO pin automatically overrides to INPUT.
	// By doing this AFTER enabling SPI, we avoid accidentally clocking in a single bit since 
	// the lines go directly from "input" to SPI control.
	// http://code.google.com/p/arduino/issues/detail?id=888
	DDRB |= ((1<<DD_MOSI)| (1<<DD_SCK));
} // spi_init()

uint8_t spi_transfer(uint8_t _data) 
{
  SPDR = _data;
  while (!(SPSR & (1<<SPIF))) ;
  return SPDR;
} // spi_transfer()

void spi_set_ss(void)    
{ 
	PORTD &= ~(1<<DD_SS); 
} // spi_set_ss()

void spi_reset_ss(void)    
{ 
	PORTD |= (1<<DD_SS); 
} // spi_reset_ss()
 

