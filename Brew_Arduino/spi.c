/*
 * spi.c
 *
 * Created: 14-4-2013 10:27:13
 *  Author: Emile
 */ 
#include "spi.h"
#include <avr/io.h>
#include <avr/interrupt.h>

#define PORT_SPI    PORTB
#define DDR_SPI     DDRB
#define DD_MISO     DDB4
#define DD_MOSI     DDB3
#define DD_SS       DDB2
#define DD_SCK      DDB5

//-----------------------------------------
// Numbers 0..9 for a 7-segment display
//-----------------------------------------
const uint8_t D[10] = {0x7e,0x30,0x6d,0x79,0x33,0x5b,0x5f,0x70,0x7f,0x7b};

// Initialize pins for SPI communication
// MCP23S08: CPOL, CPHA = (0,0) /* (0,1) does NOT work! */
void spi_init()
{
	PORT_SPI |= (1<<DD_SS);            // Disable Slave-Select
	DDR_SPI  |= (1<<DD_SS);	 	       // Set Slave-Select as output-pin
	
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
	DDR_SPI  |= ((1<<DD_MOSI)| (1<<DD_SCK));
} // spi_init()

uint8_t spi_read (uint8_t * dataout, uint8_t len)
// Shift full array through target device
{
	uint8_t i, datain;
	
	PORT_SPI &= ~(1<<DD_SS); // Enable Slave-Select for MAX6951
	for (i = 0; i < len; i++) 
	{
		SPDR = dataout[i];
		while((SPSR & (1<<SPIF)) == 0) ;
	} // for
	datain = SPDR;
	PORT_SPI |= (1<<DD_SS); // Disable Slave-Select for MAX6951
    return datain;
} // spi_read()

void spi_transmit_sync (uint8_t * dataout, uint8_t len)
// Shift full array to target device without receiving any byte
{
	uint8_t i;
	PORT_SPI &= ~(1<<DD_SS); // Enable Slave-Select for MAX6951
	for (i = 0; i < len; i++) 
	{
		SPDR = dataout[i];
		while((SPSR & (1<<SPIF)) == 0) ;
	} // for
	PORT_SPI |= (1<<DD_SS); // Disable Slave-Select for MAX6951
} // spi_transmit_sync()

uint8_t spi_fast_shift (uint8_t data)
// Clocks only one byte to target device and returns the received one
{
	PORT_SPI &= ~(1<<DD_SS); // Enable Slave-Select for MAX6951
	SPDR = data;
	while((SPSR & (1<<SPIF)) == 0) ;
	PORT_SPI |= (1<<DD_SS); // Disable Slave-Select for MAX6951
	return SPDR;
} // spi_fast_shift()

void max6951_init(void)
{
	uint16_t spi_dout;

	// Set configuration register to clear display data, blinking enable and normal operation
	spi_dout = MAX6951_DECODE_NONE;
	spi_transmit_sync((uint8_t *)&spi_dout,2);       // decode OFF for all digits

    spi_dout = MAX6951_INTENSITY | 0x04;
	spi_transmit_sync((uint8_t *)&spi_dout,2);       // set intensity for LED displays

    spi_dout = MAX6951_SCAN_LIMIT | 0x07;
    spi_transmit_sync((uint8_t *)&spi_dout,2);       // 8 digits scanned

    spi_dout = MAX6951_CONFIG | MAX6951_CLEAR_DIGITS | MAX6951_NORMAL_OPERATION |
               MAX6951_BLINK_ENABLE | MAX6951_FAST_BLINKING;
    spi_transmit_sync((uint8_t *)&spi_dout,2);
} // max6951_init()

/*------------------------------------------------------------------
  Purpose  : This function writes a number to the MAX6951 display driver.
  Variables:
          d: the number to write to the display
         dp: the number of decimals behind the comma
         nr: [0,1], the top row (0) or the bottom row (1)
      blink: [0,255]: each bit indicates which digit to blink, 0 to
                      disable blinking of all digits
  Returns  : -
  ------------------------------------------------------------------*/
void max6951_print_nr(int16_t d, int dp, uint8_t nr, uint8_t blink)
{
   uint8_t  dd,i,j;
   uint16_t digit = (nr << 10); // ((nr * 4) << 8)
   uint16_t spi_dout;

   if ((d >= 10000) || (d <= -1000) || (dp < 0) || (dp > 3) || (nr < 0) || (nr > 1))
   {
      return; // error in input
   } // if
   
   if (d < 0)
   {
      d  = -d; // invert number
	  spi_dout = MAX6951_DIGIT3 | digit | MAX6951_MINUS_SIGN;
	  spi_transmit_sync((uint8_t *)&spi_dout,2); // set last digit to '-'
   } // if
   else
   {  // d >= 0
      dd  = d / 1000; // MSB digit
      d  -= dd * 1000;
	  spi_dout = MAX6951_DIGIT3 | digit | D[dd] | ((dp == 3) ? 0x80 : 0x00);
	  spi_transmit_sync((uint8_t *)&spi_dout,2); // set digit 3
   } // else
   
   dd  = d / 100;
   d  -= dd * 100;
   spi_dout = MAX6951_DIGIT2 | digit | D[dd] | ((dp == 2) ? 0x80 : 0x00);
   spi_transmit_sync((uint8_t *)&spi_dout,2); // set digit 6 or 2

   dd  = d / 10;
   d  -= dd * 10;
   spi_dout = MAX6951_DIGIT1 | digit | D[dd] | ((dp == 1) ? 0x80 : 0x00);
   spi_transmit_sync((uint8_t *)&spi_dout,2); // set digit 5 or 1

   spi_dout = MAX6951_DIGIT0 | digit | D[d]  | ((dp == 1) ? 0x80 : 0x00);
   spi_transmit_sync((uint8_t *)&spi_dout,2); // set digit 4 or 0

   i = 128; j = 7; // start with digit 7
   while (i > 0)
   {  // check and set every digit for blinking
      if (blink & i)
      {  
		 spi_dout = ((0x40 + j) << 8) | 0x00; 
         spi_transmit_sync((uint8_t *)&spi_dout,2); // write a 0 in digit of plane 1
      } // if
      i >>= 1; // i = i/2
      j--;    
   } // while
} // max6951_print_nr()

void mcp23s08_init(void)
{
	//--------------------------------------------------------------------------------------
	// Init. the MCP23S08 (GPIO)
	// MCP23S08 bits 7..5 contain the LEDs
	// MCP23S08 bits 3..0 contain the switches (bit 4 is not connected)
	//--------------------------------------------------------------------------------------
	mcp23s08_write(MCP23S08_IODIR, 0x1F);  // bits 7-5 output, bits 4-0 input
	mcp23s08_write(MCP23S08_IPOL,  0x0FF); // switches have inverted logic
	mcp23s08_write(MCP23S08_GPPU,  0x1F);  // enable pull-ups for bits 4..0
	mcp23s08_write(MCP23S08_GPIO,  0xAF);  // clear outputs
} // mcp23s08_init()

void mcp23s08_write(uint8_t mcp_reg, uint8_t data)
{
	uint8_t spi_dout[3];

	spi_dout[0] = MCP23S08_BASE; // write MCP23S08 base address
	spi_dout[1] = mcp_reg;       // write register to write to
	spi_dout[2] = data;          // write data-byte to write
	spi_transmit_sync((uint8_t *)&spi_dout,3);
} // mcp23s08_write()

uint8_t mcp23s08_read(uint8_t mcp_reg)
{
	uint8_t spi_dout[3];

    spi_dout[0] = MCP23S08_BASE | 0x01; // write MCP23S08 base address
	spi_dout[1] = mcp_reg;              // address of MCP23S08 register to read
	spi_dout[2] = 0xff;                 // dummy byte to generate CLK for byte to read
	return spi_read((uint8_t *)&spi_dout,3);   // read data-byte
} // mcp23s08_read()

