/*==================================================================
  File Name    : $Id$
  Function name: see Udp.h
  Author       : bjoern@cs.stanford.edu 12/30/2008
				 C-version: E v.d. Logt
  ------------------------------------------------------------------
  Purpose : This file implements the Udp protocol
  ------------------------------------------------------------------
  $Log$
  ================================================================== */ /*
 *  Udp.cpp: Library to send/receive UDP packets with the Arduino ethernet shield.
 *  This version only offers minimal wrapping of socket.c/socket.h
 *  Drop Udp.h/.cpp into the Ethernet library directory at hardware/libraries/Ethernet/ 
 *
 * MIT License:
 * Copyright (c) 2008 Bjoern Hartmann
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *
 * bjoern@cs.stanford.edu 12/30/2008
 */

#include "w5500.h"
#include "socket.h"
#include "Ethernet.h"
#include "Udp.h"
#include "Dns.h"
#include "usart.h"

extern    uint16_t Ethernet_server_port[]; // defined in Ethernet.c
extern    uint8_t  dnsServerAddress[];	   // defined in Ethernet.c

uint8_t   remoteIP[4];           // remote IP address for the incoming packet whilst it's being processed
uint16_t  remotePort;            // remote port for the incoming packet whilst it's being processed
uint8_t   _sock = MAX_SOCK_NUM;  // socket ID for Wiz5100
uint16_t  _port;                 // local port to listen on
uint16_t  _offset;               // offset into the packet being sent
uint16_t  _remaining;            // remaining bytes of incoming packet yet to be processed

/* Start EthernetUDP socket, listening at local port PORT */
uint8_t udp_begin(uint16_t port) 
{
  uint8_t  bufr[2]; // buffer for w5500_read_socket_register()
  uint8_t  s;       // Status register value
  
  if (_sock != MAX_SOCK_NUM)
    return 0;

  for (int i = 0; i < MAX_SOCK_NUM; i++) 
  {
    s = w5500_read_socket_register(i, Sn_SR, bufr);
    if ((s == SnSR_CLOSED) || (s == SnSR_FIN_WAIT)) 
    {
      _sock = i;
      break;
    } // if
  } // for
  if (_sock == MAX_SOCK_NUM) return 0;
  _port      = port;
  _remaining = 0;
  socket(_sock, SnMR_UDP, _port, 0);
  return 1;
} // udp_begin()

/* return number of bytes available in the current packet,
   will return zero if parsePacket hasn't been called yet */
int udp_available(void) 
{
  return _remaining;
} // udp_available()

/* Release any resources being used by this EthernetUDP instance */
void udp_stop(void)
{
  if (_sock == MAX_SOCK_NUM)
    return;
  close(_sock);
  Ethernet_server_port[_sock] = 0;
  _sock = MAX_SOCK_NUM;
} // udp_stop()

// Start building up a packet to send to the remote host specific in host and port
// Returns 1 if successful, 0 if there was a problem resolving the hostname or port
int udp_beginPacketHost(const char *host, uint16_t port)
{
  // Look up the host first
  int     ret = 0;
  uint8_t remote_addr[4];

  ipcpy(remote_addr, dnsServerAddress);
  dns_begin(remote_addr);
  ret = dns_getHostByName(host, remote_addr);
  if (ret == 1) 
       return udp_beginPacketIP(remote_addr, port);
  else return ret;
} // udp_beginPacketHost()

// Start building up a packet to send to the remote host specific in ip and port
// Returns 1 if successful, 0 if there was a problem with the supplied IP address or port
int udp_beginPacketIP(uint8_t *ip, uint16_t port)
{
  _offset = 0;
  return startUDP(_sock, ip, port);
} // udp_beginPacketIP()

int udp_endPacket(void)
{
  return sendUDP(_sock);
} // udp_endPacket()

uint16_t udp_write(const uint8_t *buffer, uint16_t size)
{
  uint16_t bytes_written = bufferData(_sock, _offset, buffer, size);
  _offset += bytes_written;
  return bytes_written;
} // udp_write()

// Start processing the next available incoming packet
// Returns the size of the packet in bytes, or 0 if no packets are available
int udp_parsePacket(void)
{
  uint8_t tmpBuf[8];
  int     ret;

  // discard any remaining bytes in the last packet
  udp_flush();
  if (w5500_getRXReceivedSize(_sock) > 0)
  {
    //HACK - hand-parse the UDP packet using TCP recv method
    ret = 0; 
    //read 8 header bytes and get IP and port from it
    ret = recv(_sock,tmpBuf,8);
    if (ret > 0)
    {
	  ipcpy(remoteIP, tmpBuf); // copy address into remoteIP
      remotePort = tmpBuf[4];
      remotePort = (remotePort << 8) + tmpBuf[5];
      _remaining = tmpBuf[6];
      _remaining = (_remaining << 8) + tmpBuf[7];
      // When we get here, any remaining bytes are the data
      ret = _remaining;
    } // if
    return ret;
  } // if
  // There aren't any packets available
  return 0;
} // udp_parsePacket()

// Read a single byte from the current packet
int udp_read1(void)
{
  uint8_t byte;

  if ((_remaining > 0) && (recv(_sock, &byte, 1) > 0))
  {
    // We read things without any problems
    _remaining--;
    return byte;
  }
  // If we get here, there's no data available
  return -1;
} // udp_read1()

// Read up to len bytes from the current packet and place them into buffer
// Returns the number of bytes read, or 0 if none are available
int udp_read(unsigned char* buffer, int16_t len)
{
  if (_remaining > 0)
  {
    int got;

    if (_remaining <= len)
    {
      // data should fit in the buffer
      got = recv(_sock, buffer, _remaining);
    }
    else
    {
      // too much data for the buffer, 
      // grab as much as will fit
      got = recv(_sock, buffer, len);
    }
    if (got > 0)
    {
      _remaining -= got;
      return got;
    } // if
  } // if
  // If we get here, there's no data available or recv failed
  return -1;
} // udp_read()

// Return the next byte from the current packet without moving on to the next byte
int udp_peek(void)
{
  uint8_t b;
  // Unlike recv, peek doesn't check to see if there's any data available, so we must.
  // If the user hasn't called parsePacket yet then return nothing otherwise they
  // may get the UDP header
  if (!_remaining)
    return -1;
  peek(_sock, &b); // peek() from socket.c
  return b;
} // udp_peek()

// Finish reading the current packet
void udp_flush(void)
{
  // could this fail (loop endlessly) if _remaining > 0 and recv in read fails?
  // should only occur if recv fails after telling us the data is there, lets
  // hope the w5500 always behaves :)
  while (_remaining)
  {
    udp_read1();
  } // while
} // udp_flush()

