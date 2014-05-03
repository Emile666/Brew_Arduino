/*==================================================================
  File Name    : $Id$
  Function name: -
  Author       : WIZnet <support@wiznet.co.kr>
				 C-version: E v.d. Logt
  ------------------------------------------------------------------
  Purpose : w5500 low-level routines
  ------------------------------------------------------------------
  $Log$
  ================================================================== */ 
/*
* Copyright (c) 2010 by WIZnet <support@wiznet.co.kr>
 *
 * This file is free software; you can redistribute it and/or modify
 * it under the terms of either the GNU General Public License version 2
 * or the GNU Lesser General Public License version 2.1, both as
 * published by the Free Software Foundation.
 */

#ifndef	W5500_H_INCLUDED
#define	W5500_H_INCLUDED
#include <avr/io.h>
#include <stdbool.h>

typedef uint8_t SOCKET; 

#define MAX_SOCK_NUM 8
#define RST     (7)
#define SOCKETS (8)
// Max TX and RX Buffer size
#define W5500_SSIZE  (2048)
#define W5500_RSIZE  (2048)

//-----------------------
// W5500 Common Registers
//-----------------------
#define MR       (0x0000) /* 1: Mode */
#define GAR      (0x0001) /* 4: Gateway IP address */  
#define SUBR     (0x0005) /* 4: Subnet mask address */
#define SHAR     (0x0009) /* 6: Source MAC address */
#define SIPR     (0x000F) /* 4: Source IP address */
// Not implemented: INTLEVEL Interrupt Low level Timer
#define IR       (0x0015) /* 1: Interrupt */
#define IMR      (0x0016) /* 1: Interrupt Mask */
// Not implemented: SIR Socket Interrupt Timer
// Not implemented: SIMR Socket Interrupt Mask
#define RTR      (0x0019) /* 2: Timeout address */
#define RCR      (0x001B) /* 2: Retry count */
// Not implemented: PTIMER, PMAGIC  :  PPP Link Control Protocol
// Not implemented: PHAR, PSID, PMRU: PPPoE mode Registers
#define UIPR     (0x0028) /* 4: Unreachable IP address in UDP mode */
#define UPORT    (0x002C) /* 2: Unreachable Port address in UDP mode */
#define PHYCFGR  (0x002E) /* 1: PHY Configuration register, default */
#define VERSIONR (0x0039) /* 1: Version number of W5500 */

//------------------------  
// W5500 Socket Registers
//------------------------  
#define Sn_MR         (0x0000) /* 1: Mode */
#define Sn_CR         (0x0001) /* 1: Command */
#define Sn_IR         (0x0002) /* 1: Interrupt */
#define Sn_SR         (0x0003) /* 1: Status */
#define Sn_PORT       (0x0004) /* 2: Source Port */
#define Sn_DHAR       (0x0006) /* 6: Destination HW Address */
#define Sn_DIPR       (0x000C) /* 4: Destination IP Address */
#define Sn_DPORT      (0x0010) /* 2: Destination Port */
#define Sn_MSSR       (0x0012) /* 2: Max Segment Size */
#define Sn_PROTO      (0x0014) /* 1: Protocol in IP RAW Mode */
#define Sn_TOS        (0x0015) /* 1: IP TOS */
#define Sn_TTL        (0x0016) /* 1: IP TTL */
#define Sn_RXBUF_SIZE (0x001E) /* 1: RX Buffer Size */
#define Sn_TXBUF_SIZE (0x001F) /* 1: RX Buffer Size */
#define Sn_TX_FSR     (0x0020) /* 2: TX Free Size */
#define Sn_TX_RD      (0x0022) /* 2: TX Read Pointer */
#define Sn_TX_WR      (0x0024) /* 2: TX Write Pointer */
#define Sn_RX_RSR     (0x0026) /* 2: RX Free Size */
#define Sn_RX_RD      (0x0028) /* 2: RX Read Pointer */
#define Sn_RX_WR      (0x002A) /* 2: RX Write Pointer (supported?) */

// Sn_MR Register
#define SnMR_CLOSE  (0x00)
#define SnMR_TCP    (0x01)
#define SnMR_UDP    (0x02)
#define SnMR_IPRAW  (0x03)
#define SnMR_MACRAW (0x04)
#define SnMR_PPPOE  (0x05)
#define SnMR_ND     (0x20)
#define SnMR_MULTI  (0x80)

// Sn_IR Register
#define  SEND_OK (0x10)
#define  TIMEOUT (0x08)
#define  RECV    (0x04)
#define  DISCON  (0x02)
#define  CON     (0x01)

// Sn_SR Register
#define  SnSR_CLOSED      (0x00)
#define  SnSR_INIT        (0x13)
#define  SnSR_LISTEN      (0x14)
#define  SnSR_SYNSENT     (0x15)
#define  SnSR_SYNRECV     (0x16)
#define  SnSR_ESTABLISHED (0x17)
#define  SnSR_FIN_WAIT    (0x18)
#define  SnSR_CLOSING     (0x1A)
#define  SnSR_TIME_WAIT   (0x1B)
#define  SnSR_CLOSE_WAIT  (0x1C)
#define  SnSR_LAST_ACK    (0x1D)
#define  SnSR_UDP         (0x22)
#define  SnSR_IPRAW       (0x32)
#define  SnSR_MACRAW      (0x42)
#define  SnSR_PPPOE       (0x5F)

// IPPROTO Register
#define IP   (0)
#define ICMP (1)
#define IGMP (2)
#define GGP  (3)
#define TCP  (6)
#define PUP  (12)
#define UDP  (17)
#define IDP  (22)
#define ND   (77)
#define RAW  (255)

// SockCMD Defines
enum SockCMD 
{
	Sock_OPEN      = 0x01,
	Sock_LISTEN    = 0x02,
	Sock_CONNECT   = 0x04,
	Sock_DISCON    = 0x08,
	Sock_CLOSE     = 0x10,
	Sock_SEND      = 0x20,
	Sock_SEND_MAC  = 0x21,
	Sock_SEND_KEEP = 0x22,
	Sock_RECV      = 0x40
};

// ----------------------------------------------------------------------------
// W5500 Registers & Socket Register
// ----------------------------------------------------------------------------
uint16_t w5500_read(uint16_t _addr, uint8_t _cb, uint8_t *buf, uint16_t len);
uint16_t w5500_read_common_register(uint8_t reg_name, uint8_t *buf);
uint16_t w5500_read_socket_register(SOCKET s, uint8_t reg_name, uint8_t *buf);
 
uint16_t w5500_write(uint16_t _addr, uint8_t _cb, const uint8_t *buf, uint16_t len);
uint16_t w5500_write_common_register(uint8_t reg_name, uint8_t *buf);
uint16_t w5500_write_socket_register(SOCKET s, uint8_t reg_name, uint8_t *buf); 

void     w5500_init(void);
uint16_t w5500_getTXFreeSize(SOCKET s);
uint16_t w5500_getRXReceivedSize(SOCKET s);
void     w5500_send_data_processing(SOCKET s, const uint8_t *data, uint16_t len);
void     w5500_send_data_processing_offset(SOCKET s, uint16_t offset, const uint8_t *data, uint16_t len);
void     w5500_read_data(SOCKET s, volatile uint16_t  src, volatile uint8_t * dst, uint16_t len);
void     w5500_recv_data_processing(SOCKET s, uint8_t *data, uint16_t len, uint8_t peek);
void     w5500_execCmdSn(SOCKET s, enum SockCMD _cmd);
  
#endif
