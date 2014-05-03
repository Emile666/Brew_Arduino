/*==================================================================
  File Name    : $Id$
  Function name: -
  Author       : bjoern@cs.stanford.edu 12/30/2008
				 C-version: E v.d. Logt
  ------------------------------------------------------------------
  Purpose : This is the header file for the Udp routines
  ------------------------------------------------------------------
  $Log$
  ================================================================== */ 
/*
 *  Udp.cpp: Library to send/receive UDP packets with the Arduino ethernet shield.
 *  This version only offers minimal wrapping of socket.c/socket.h
 *  Drop Udp.h/.cpp into the Ethernet library directory at hardware/libraries/Ethernet/ 
 *
 * NOTE: UDP is fast, but has some important limitations (thanks to Warren Gray for mentioning these)
 * 1) UDP does not guarantee the order in which assembled UDP packets are received. This
 * might not happen often in practice, but in larger network topologies, a UDP
 * packet can be received out of sequence. 
 * 2) UDP does not guard against lost packets - so packets *can* disappear without the sender being
 * aware of it. Again, this may not be a concern in practice on small local networks.
 * For more information, see http://www.cafeaulait.org/course/week12/35.html
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
 */
#ifndef UDP_H
#define UDP_H

#define UDP_TX_PACKET_MAX_SIZE 24

uint8_t  udp_begin(uint16_t port); // init., start listening on specified port. Returns 1 if successful, 0 if no sockets available to use
int      udp_available(void);
void     udp_stop(void);
int      udp_beginPacketIP(uint8_t *ip, uint16_t port);
int      udp_beginPacketHost(const char *host, uint16_t port);
int      udp_endPacket(void);
uint16_t udp_write(const uint8_t *buffer, uint16_t size);
int      udp_parsePacket(void);
int      udp_read1(void);
int      udp_read(unsigned char* buffer, int16_t len);
int      udp_peek(void);
void     udp_flush(void);

#endif
