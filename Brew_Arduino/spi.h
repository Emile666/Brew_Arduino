/*==================================================================
  File Name    : $Id$
  Function name: -
  Author       : E v.d. Logt
  ------------------------------------------------------------------
  Purpose : SPI-bus low-level routines
  ------------------------------------------------------------------
  $Log$
  ================================================================== */ 
#ifndef _SPI_H_
#define _SPI_H_
#include <avr/io.h>

// Defines for PORTB
#define DD_MISO     DDB4
#define DD_MOSI     DDB3
#define DD_SS       DDB2
#define DD_SCK      DDB5

void    spi_init(void);
uint8_t spi_transfer(uint8_t _data);
void    spi_set_ss(void);
void    spi_reset_ss(void);

#endif /* _SPI_H_ */