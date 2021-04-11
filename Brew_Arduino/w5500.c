/*==================================================================
  File Name    : w5500.c
  Function name: see w5500.h
  Author       : WIZnet <support@wiznet.co.kr>
				 C-version: E v.d. Logt
  ------------------------------------------------------------------
  Purpose : w5500 low-level routines
  ================================================================== */ 
/*
 * Copyright (c) 2010 by WIZnet <support@wiznet.co.kr>
 *
 * This file is free software; you can redistribute it and/or modify
 * it under the terms of either the GNU General Public License version 2
 * or the GNU Lesser General Public License version 2.1, both as
 * published by the Free Software Foundation.
 */
#include <stdio.h>
#include <string.h>
#include "spi.h"
#include "w5500.h"
#include "delay.h"         /* for delay_msec() */

/*------------------------------------------------------------------
  Purpose  : This function reads 1, 2 of N bytes from the W5500,
             using the SPI bus.
  Variables: 
      addr : the 16-bit address of the SPI register in the W5500
        cb : the control-byte for the W5500
       buf : the receive buffer. Note that the calling program must
             make sure that the size is large enough!
       len : the number of bytes to read from the W5500. This is 
	         either 1 or > 2. Reading 2 bytes should be done by 2
			 single reads.
  Returns  : len <= 2: the actual byte/word read from the W5500
             len > 2 : the number of bytes read
  ------------------------------------------------------------------*/
uint16_t w5500_read(uint16_t addr, uint8_t cb, uint8_t *buf, uint16_t len)
{
	uint8_t  i;
	uint16_t ret = len; // return #bytes read in case of len > 2
	
	spi_set_ss();
	spi_transfer(addr >> 8);   // MSB of 16-bit address
	spi_transfer(addr & 0xFF); // LSB of 16-bit address
	spi_transfer(cb);          // control-byte
	for (i = 0; i < len; i++)
	{
		buf[i] = spi_transfer(0);
	} // for
	spi_reset_ss();
	
	if (len == 1) ret = (uint16_t)buf[0]; // return uint8_t
	else if (len == 2)
	{
		ret  = (uint16_t)buf[0] << 8;
		ret += buf[1];
	} // if
	return ret; // return byte (len <=2) or len (len > 2)
} // w5500_read()

/*------------------------------------------------------------------
  Purpose  : This function reads from any of the common registers
             of the W5500.
  Variables: 
  reg_name : the W5500 register name (see w5500.h)
       buf : the receive buffer. Note that the calling program must
             make sure that the size is large enough!
  Returns  : len <= 2: the actual byte/word read from the W5500
             len > 2 : the number of bytes read
  ------------------------------------------------------------------*/
uint16_t w5500_read_common_register(uint8_t reg_name, uint8_t *buf)
{
	uint16_t ret;      // Return value
	uint8_t  len = 1;  // Length of register to read. Default: 1

	if      ((reg_name == RTR)  || (reg_name == UPORT))  len = 2;
	else if ((reg_name == GAR)  || (reg_name == SUBR) || 
	         (reg_name == SIPR) || (reg_name == UIPR))   len = 4;
	else if  (reg_name == SHAR) 				  		 len = 6;

	// The Register Name is also defined as the Register Address
	// So use reg_name also as address here
	ret = w5500_read(reg_name, 0x00, buf, len);
	return ret;
} // w5500_read_common_register()

/*------------------------------------------------------------------
  Purpose  : This function reads from any of the socket registers
             of the W5500.
  Variables: 
         s : the socket number to read from 
  reg_name : the W5500 socket register name (see w5500.h)
       buf : the receive buffer. Note that the calling program must
             make sure that the size is large enough!
  Returns  : len <= 1: the actual byte read from the W5500
             len > 2 : the number of bytes read
  ------------------------------------------------------------------*/
uint16_t w5500_read_socket_register(SOCKET s, uint8_t reg_name, uint8_t *buf) 
{
	uint16_t ret;      // Return value
	uint8_t  len = 2;  // Length of register to read. Default: 2
    uint8_t  cb = (s<<5) + 0x08; // Control Byte
	   
	if      ((reg_name == Sn_MR) || (reg_name == Sn_CR)    || (reg_name == Sn_IR)  || 
	         (reg_name == Sn_SR) || (reg_name == Sn_PROTO) || (reg_name == Sn_TOS) ||
	         (reg_name == Sn_TTL)) len = 1;
	else if (reg_name == Sn_DIPR)  len = 4;
	else if (reg_name == Sn_DHAR)  len = 6;
	// The Register Name is also defined as the Register Address
	// So use reg_name also as address here
	ret = w5500_read(reg_name, cb, buf, len);
	return ret;
} // w5500_read_socket_register()

/*------------------------------------------------------------------
  Purpose  : This function writes 1, 2 of N bytes to the W5500,
             using the SPI bus.
  Variables: 
      addr : the 16-bit address of the SPI register in the W5500
        cb : the control-byte for the W5500
       buf : the send buffer. Note that the calling program must
             make sure that the size is large enough!
       len : the number of bytes to write to the W5500  
  Returns  : the number of bytes written
  ------------------------------------------------------------------*/
uint16_t w5500_write(uint16_t addr, uint8_t cb, const uint8_t *buf, uint16_t len)
{
	uint8_t i;
	
    spi_set_ss();
    spi_transfer(addr >> 8);   // MSB of 16-bit address
    spi_transfer(addr & 0xFF); // LSB of 16-bit address
    spi_transfer(cb);
    for (i = 0; i < len; i++)
    {
        spi_transfer(buf[i]);
    } // for
    spi_reset_ss();
    return len; // #bytes written
} // w5500_write()

/*------------------------------------------------------------------
  Purpose  : This function writes 1, 2 of N bytes to the W5500
			 common registers, using the SPI bus.
  Variables: 
  reg_name : the W5500 socket register name (see w5500.h)
       buf : the send buffer. Note that the calling program must
			 make sure that the size is large enough!
  Returns  : the number of bytes written
  ------------------------------------------------------------------*/
uint16_t w5500_write_common_register(uint8_t reg_name, uint8_t *buf)
{
	uint8_t  len = 1;  // Length of register to write to. Default: 1

	if      ((reg_name == RTR)  || (reg_name == UPORT))  len = 2;
	else if ((reg_name == GAR)  || (reg_name == SUBR) || 
	         (reg_name == SIPR) || (reg_name == UIPR))   len = 4;
	else if  (reg_name == SHAR) 				  		 len = 6;

	// The Register Name is also defined as the Register Address
	// So use reg_name also as address here.
	// 0x04 = Write Common Register
	return w5500_write(reg_name, 0x04, buf, len); // #bytes written
} // w5500_write_common_register()

/*------------------------------------------------------------------
  Purpose  : This function writes 1, 2 of N bytes to the W5500
			 socket registers, using the SPI bus.
  Variables: 
         s : the socket number to write to
  reg_name : the W5500 socket register name (see w5500.h)
       buf : the send buffer. Note that the calling program must
			 make sure that the size is large enough!
  Returns  : the number of bytes written
  ------------------------------------------------------------------*/
uint16_t w5500_write_socket_register(SOCKET s, uint8_t reg_name, uint8_t *buf) 
{
	uint8_t  len = 2;  // Length of register to write to. Default: 2
    uint8_t  cb = (s<<5) + 0x0C; // Control Byte
    
    switch (reg_name)
    {
	    case Sn_MR:  case Sn_CR:  case Sn_IR:  case Sn_SR: case Sn_PROTO: 
	    case Sn_TOS: case Sn_TTL: case Sn_RXBUF_SIZE: case Sn_TXBUF_SIZE:
	    	 	 len = 1; break;
		case Sn_DIPR: 
				 len = 4; break;
		case Sn_DHAR: 
				 len = 6; break;
		default: len = 2; break;	
	} // switch
	// The Register Name is also defined as the Register Address
	// So use reg_name also as address here
    return w5500_write(reg_name, cb, buf, len); // #bytes written
} // w5500_write_socket_register()

/*------------------------------------------------------------------
  Purpose  : This function initializes the W5500 chip.
  Variables: -
  Returns  : -
  ------------------------------------------------------------------*/
void w5500_init(void)
{
	uint8_t i;
	uint8_t buf[2];
	
	buf[0] = 2; // 2 KB Buffer Size
	delay_msec(300);
    for (i = 0; i < MAX_SOCK_NUM; i++) 
    {
        w5500_write_socket_register(i, Sn_RXBUF_SIZE, buf);
        w5500_write_socket_register(i, Sn_TXBUF_SIZE, buf);
    } // for
} // w5500_init()

/*------------------------------------------------------------------
  Purpose  : This function returns the free size in bytes of the
             TX buffer in the W5500 chip.
  Variables: 
         s : the socket number
  Returns  : The number of bytes free in the TX buffer
  ------------------------------------------------------------------*/
uint16_t w5500_getTXFreeSize(SOCKET s)
{
    uint16_t val=0, val1=0;
	uint8_t  buf[2];
	
    do {
    	val1 = w5500_read_socket_register(s, Sn_TX_FSR, buf);
        if (val1 != 0)
            val = w5500_read_socket_register(s, Sn_TX_FSR, buf);
    } 
    while (val != val1);
    return val;
} // w5500_getTXFReeSize()

/*------------------------------------------------------------------
  Purpose  : This function returns the free size in bytes of the
             RX buffer in the W5500 chip.
  Variables: 
         s : the socket number
  Returns  : The number of bytes free in the RX buffer
  ------------------------------------------------------------------*/
uint16_t w5500_getRXReceivedSize(SOCKET s)
{
    uint16_t val=0,val1=0;
	uint8_t  buf[2];
    do {
        val1 = w5500_read_socket_register(s, Sn_RX_RSR, buf);
        if (val1 != 0)
            val = w5500_read_socket_register(s, Sn_RX_RSR, buf);
    } 
    while (val != val1);
    return val;
} // w5500_getRXReceivedSize()

/*------------------------------------------------------------------
  Purpose  : 
  Variables: 
         s : the socket number
	  data : 
	   len : 	 
  Returns  : -
  ------------------------------------------------------------------*/
void w5500_send_data_processing(SOCKET s, const uint8_t *data, uint16_t len)
{
  // This is same as having no offset in a call to send_data_processing_offset
  w5500_send_data_processing_offset(s, 0, data, len);
} // w5500_send_data_processing()

/*------------------------------------------------------------------
  Purpose  : 
  Variables: 
         s : the socket number
	  data : 
    offset :
	   len : 	 
  Returns  : -
  ------------------------------------------------------------------*/
void w5500_send_data_processing_offset(SOCKET s, uint16_t offset, const uint8_t *data, uint16_t len)
{
	uint8_t  buf[2];
	uint16_t ptr;
    uint8_t  cntl_byte = (0x14 + (s<<5));
	
    ptr  = w5500_read_socket_register(s, Sn_TX_WR, buf);
    ptr += offset;
    w5500_write(ptr, cntl_byte, data, len);
    ptr += len;
	buf[0] = ptr >> 8;
	buf[1] = ptr & 0xff;
    w5500_write_socket_register(s, Sn_TX_WR, buf);  
} // w5500_send_data_processing_offset()

/*------------------------------------------------------------------
  Purpose  : 
  Variables: 
         s : the socket number
	   src : 
       dst :
	   len : 	 
  Returns  : -
  ------------------------------------------------------------------*/
void w5500_read_data(SOCKET s, volatile uint16_t src, volatile uint8_t *dst, uint16_t len)
{
    uint8_t cntl_byte = (0x18 + (s<<5));
    w5500_read(src , cntl_byte, (uint8_t *)dst, len);
} // w5500_read_data()

/*------------------------------------------------------------------
  Purpose  : 
  Variables: 
         s : the socket number
      data :
	   len : 	 
	  peek : 
  Returns  : -
  ------------------------------------------------------------------*/
void w5500_recv_data_processing(SOCKET s, uint8_t *data, uint16_t len, uint8_t peek)
{
	uint8_t  buf[2];
    uint16_t ptr;

    ptr = w5500_read_socket_register(s, Sn_RX_RD, buf);
    w5500_read_data(s, ptr, data, len);
    if (!peek)
    {
        ptr += len;
		buf[0] = ptr >> 8;
		buf[1] = ptr & 0xff;
        w5500_write_socket_register(s, Sn_RX_RD, buf);
    } // if
} // w5500_recv_data_processing()

/*------------------------------------------------------------------
  Purpose  : Write a command into the W5500 Command Register
  Variables: 
         s : the socket number
       cmd : the command number (see w5500.h)
  Returns  : -
  ------------------------------------------------------------------*/
void w5500_execCmdSn(SOCKET s, enum SockCMD cmd) 
{
	uint8_t buf[2];
	
	buf[0] = cmd;
    w5500_write_socket_register(s, Sn_CR, buf);         // Send command to socket
    while (w5500_read_socket_register(s, Sn_CR, buf)) ; // Wait for command to complete
} // w5500_execCmdSn()
