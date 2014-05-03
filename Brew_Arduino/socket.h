/*==================================================================
  File Name    : $Id$
  Function name: 
  Author       : ?
				 C-version: E v.d. Logt
  ------------------------------------------------------------------
  Purpose : Socket routines for use with the WIZ550io module
  ------------------------------------------------------------------
  $Log$
  ================================================================== */ 
#ifndef	_SOCKET_H_
#define	_SOCKET_H_

#include "w5500.h"

void     ipcpy(uint8_t *dest, uint8_t *src);
bool     ipequ(uint8_t *str1, uint8_t *str2);

uint8_t  socket(SOCKET s, uint8_t protocol, uint16_t port, uint8_t flag); // Opens a socket(TCP or UDP or IP_RAW mode)
void     close(SOCKET s); // Close socket
uint8_t  listen(SOCKET s);	// Establish TCP connection (Passive connection)
uint8_t  connect(SOCKET s, uint8_t * addr, uint16_t port); // Establish TCP connection (Active connection)
void     disconnect(SOCKET s); // disconnect the connection
uint16_t send(SOCKET s, const uint8_t * buf, uint16_t len); // Send data (TCP)
int16_t  recv(SOCKET s, uint8_t * buf, int16_t len);	// Receive data (TCP)
uint16_t peek(SOCKET s, uint8_t *buf);
uint16_t sendto(SOCKET s, const uint8_t * buf, uint16_t len, uint8_t * addr, uint16_t port); // Send data (UDP/IP RAW)
uint16_t recvfrom(SOCKET s, uint8_t * buf, uint16_t len, uint8_t * addr, uint16_t *port); // Receive data (UDP/IP RAW)
void     flush(SOCKET s); // Wait for transmission to complete
uint16_t igmpsend(SOCKET s, const uint8_t * buf, uint16_t len);
uint16_t bufferData(SOCKET s, uint16_t offset, const uint8_t* buf, uint16_t len);
int      startUDP(SOCKET s, uint8_t* addr, uint16_t port);
int      sendUDP(SOCKET s);

#endif
/* _SOCKET_H_ */
