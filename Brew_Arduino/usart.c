/*==================================================================
   Created: 07/12/2011 15:17:35
   Author : Emile
   File   : usart.c
  ================================================================== */
#include "usart.h"
#include "ring_buffer.h"
#include "delay.h"         /* for delay_msec() */
#include <stdio.h>

// buffers for use with the ring buffer (belong to the USART)
bool ovf_buf_in; // true = input buffer overflow
uint16_t isr_cnt = 0;

struct ring_buffer ring_buffer_out;
struct ring_buffer ring_buffer_in;
uint8_t            out_buffer[TX_BUF_SIZE];
uint8_t            in_buffer[RX_BUF_SIZE];

//-----------------------------------------------------------------------------
// USART Data Register Empty Interrupt.
//
// This interrupt will be executed as long as UDRE0 is set. UDRE0 is cleared
// by writing UDR0. When the Data Register Empty Interrupt Enable (UDRIE0) bit
// in UCSR0B is set, the USART Data Register Empty Interrupt will be executed
// as long as UDRE0 is set (provided that global interrupts are enabled).
// UDRE0 is cleared by writing UDR0.
//-----------------------------------------------------------------------------
ISR(USART_UDRE_vect)
{
	if (!ring_buffer_is_empty(&ring_buffer_out))
	{   // if there is data in the ring buffer, fetch it and send it
		UDR0 = ring_buffer_get(&ring_buffer_out);
	}
	else
	{   // no more data to send, turn off interrupt
		UCSR0B &= ~(1 << UDRIE0);
	} /* else */
} /* ISR(USART_UDRE_vect) */

//-----------------------------------------------------------------------------
// USART Receive Complete Interrupt.

// When the Receive Complete Interrupt Enable (RXCIE0) in UCSR0B is set,
// the USART Receive Complete interrupt will be executed as long as the
// RXC0 Flag is set (provided that global interrupts are enabled). When
// interrupt-driven data reception is used, the receive complete routine
// must read the received data from UDR0 in order to clear the RXC0 Flag,
// otherwise a new interrupt will occur once the interrupt routine terminates.
//-----------------------------------------------------------------------------
ISR(USART_RX_vect)
{
	volatile uint8_t ch;
	
	if (!ring_buffer_is_full(&ring_buffer_in))
	{
		ring_buffer_put(&ring_buffer_in, UDR0);
		ovf_buf_in = false;
	} // if
	else
	{
		ch = UDR0; // clear RXC0 flag
		ovf_buf_in = true;
	}
	isr_cnt++;
} /* ISR(USART0_RX_vect) */

/*------------------------------------------------------------------
  Purpose  : This function initializes the USART device, which is 
             the Serial Interface
  Variables: 
       ubrr: the baud-rate for the Serial Interface
  Returns  : -
  ------------------------------------------------------------------*/
void usart_init(unsigned int ubrr)
{
   /*Set baud rate */
   UBRR0H  = (unsigned char)(ubrr>>8);
   UBRR0L  = (unsigned char)ubrr;

   // enable RX and TX and set interrupts on rx complete
   UCSR0B = (1 << RXEN0) | (1 << TXEN0) | (1 << RXCIE0);
   UCSR0A &= ~(1 << U2X0); // Disable double baud-rate (for Arduino UNO!).
   
   // 8-bit, 1 stop bit, no parity, asynchronous UART
   UCSR0C = (1 << UCSZ01) | (1 << UCSZ00) | (0 << USBS0)  |
   (0 << UPM01)  | (0 << UPM00)  | (0 << UMSEL01)| (0 << UMSEL00);

   // initialize in- and out-buffer for the UART
   ring_buffer_out = ring_buffer_init(out_buffer, TX_BUF_SIZE);
   ring_buffer_in  = ring_buffer_init(in_buffer , RX_BUF_SIZE);
} // usart_init()

/*------------------------------------------------------------------
  Purpose  : This function writes a character to the serial port
  Variables:
        ch : The character to write to serial port 0
  Returns  : -
  ------------------------------------------------------------------*/
void usart_putc(unsigned char data )
{
	// At 38400 Baud, sending 1 byte takes a max. of 0.26 msec.
	while (ring_buffer_is_full(&ring_buffer_out)) 
	{
		UCSR0B |= (1 << UDRIE0); // make sure interrupt is enabled
		delay_msec(1);           // Give interrupt time to send buffer
	};			
	cli(); // Disable interrupts to get exclusive access to ring_buffer_out
	if (ring_buffer_is_empty(&ring_buffer_out))
	{
		UCSR0B |= (1 << UDRIE0); // First data in buffer, enable data ready interrupt
	} // if
	ring_buffer_put(&ring_buffer_out, data); // Put data in buffer
	sei(); // Re-enable interrupts
} // usart_putc()

/*------------------------------------------------------------------
  Purpose  : This function writes a character to serial port 0.
             if a linefeed is sent, a carriage return is inserted
             before the linefeed.
  Variables:
        ch : The character to write to serial port 0
  Returns  : the character written
  ------------------------------------------------------------------*/
void xputc(unsigned char ch)
{                       /* Write Character to Serial Port */
  if (ch == '\n')       /* Check for LF */
  {                            
    //usart_putc('\r');  /* Output CR */
  }
  usart_putc(ch); /* Transmit Character */
} // xputc()

/*------------------------------------------------------------------
  Purpose  : This function writes a string to serial port 0, using
             the xputc() routine.
  Variables:
         s : The string to write to serial port 0
  Returns  : the number of characters written
  ------------------------------------------------------------------*/
void xputs(const char *s)
{
	while (*s) xputc(*s++);
} // xputs()

/*------------------------------------------------------------------
  Purpose  : This function checks if a character is present in the
             receive buffer.
  Variables: -
  Returns  : 1 if a character is received, 0 otherwise
  ------------------------------------------------------------------*/
uint8_t usart_kbhit(void) /* returns true if character in receive buffer */
{
	return !ring_buffer_is_empty(&ring_buffer_in);
} // usart_kbhit()

/*------------------------------------------------------------------
  Purpose  : This function reads a character from serial port 0.
  Variables: -
  Returns  : the character received
  ------------------------------------------------------------------*/
unsigned char usart_getc( void )
{
	return ring_buffer_get(&ring_buffer_in);
} // usart_getc()

