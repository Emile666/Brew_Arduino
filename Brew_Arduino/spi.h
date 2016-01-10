/*==================================================================
  File Name    : $Id$
  Function name: -
  Author       : E v.d. Logt
  ------------------------------------------------------------------
  Purpose : SPI-bus low-level routines
  ------------------------------------------------------------------
  $Log$
  Revision 1.2  2014/05/03 11:27:44  Emile
  - Ethernet support added for W550io module
  - No response for L, N, P, W commands anymore
  - All source files now have headers

  ================================================================== */ 
#ifndef _SPI_H_
#define _SPI_H_
#include <avr/io.h>

// Defines for PORTB
#define DD_MOSI     DDB3
#define DD_MISO     DDB4
#define DD_SCK      DDB5
#define DD_SS       DDD7

void    spi_init(void);
uint8_t spi_transfer(uint8_t _data);
void    spi_set_ss(void);
void    spi_reset_ss(void);

#endif /* _SPI_H_ */