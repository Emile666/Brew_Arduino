/*
 * usart.h
 *
 * Created: 07/12/2011 15:16:27
 *  Author: Boomber
 */ 
#ifndef USART_H_
#define USART_H_

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <stdio.h>
#include <stdbool.h>

#define USART_BUFLEN (20)
#define BAUD         (19200)
#define MYUBRR       (((((F_CPU * 10) / (16L * BAUD)) + 5) / 10) - 1)

#define TX_BUF_SIZE (30)
#define RX_BUF_SIZE (10)

void          usart_init(unsigned int ubrr);
void          usart_putc(unsigned char data );
void          xputc(unsigned char ch);
void          xputs(const char *s);
uint8_t       usart_kbhit(void); /* returns 1 if character in receive buffer */
unsigned char usart_getc( void );

#endif /* USART_H_ */