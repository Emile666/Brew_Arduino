/*==================================================================
  File Name    : $Id$
  Function name: -
  Author       : Soohwan Kim (suhwan@wiznet.co.kr), mod. 12 Aug 2013
				 C-version: E v.d. Logt
  ------------------------------------------------------------------
  Purpose : This is the header file for the ethernet routines
  ------------------------------------------------------------------
  $Log$
  ================================================================== */ 
#ifndef ETHERNET5500_H
#define ETHERNET5500_H

#include "Dhcp.h"
#include <string.h>
//#include "EthernetClient.h"
//#include "EthernetServer.h"

// Initialize function when use the ioShield series (included WIZ550io)
// WIZ550io has a MAC address which is written after reset.
// Default IP, Gateway and subnet address are also writen.
// so, It needs some initial time. please refer WIZ550io Datasheet in details.
int  Ethernet_begin(void);
void Ethernet_begin_ip(uint8_t *local_ip);
int  Ethernet_maintain(void);

#endif
