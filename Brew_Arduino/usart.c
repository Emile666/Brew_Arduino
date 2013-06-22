/*
 * usart.c
 *
 * Created: 07/12/2011 15:17:35
 *  Author: Boomber
 */ 
#include "usart.h"
#include <stdio.h>

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
   UCSR0B  = (1<<RXEN0)|(1<<TXEN0); //Enable receiver and transmitter
   UCSR0A &= ~(1 << U2X0); // Disable double baud-rate (for Arduino UNO!).
   /* Set frame format: 8data, 2stop bit */
   UCSR0C  = (1<<USBS0)|(3<<UCSZ00);
} // usart_init()

/*------------------------------------------------------------------
  Purpose  : This function writes a character to the serial port
  Variables:
        ch : The character to write to serial port 0
  Returns  : -
  ------------------------------------------------------------------*/
void usart_putc(unsigned char data )
{
   /* Wait for empty transmit buffer */
   while (!(UCSR0A & (1 << UDRE0))) ;
   /* Put data into buffer, sends the data */
   UDR0 = data;
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
    usart_putc('\r');  /* Output CR */
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
   if (UCSR0A & (1<<RXC0))
   {
      return 1;
   }
   else
   {
      return 0;
   } // else
} // usart_kbhit()

/*------------------------------------------------------------------
  Purpose  : This function reads a character from serial port 0.
  Variables: -
  Returns  : the character received
  ------------------------------------------------------------------*/
unsigned char usart_getc( void )
{
   while ( !(UCSR0A & (1<<RXC0)) ) ; /* Wait for data to be received */
   return UDR0; /* Get and return received data from buffer */
} // usart_getc()

