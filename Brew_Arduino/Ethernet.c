/*==================================================================
  File Name    : $Id$
  Function name: Ethernet_begin(), Ethernet_begin_ip(), Ethernet_maintain()
  Author       : Soohwan Kim (suhwan@wiznet.co.kr), mod. 12 Aug 2013
				 C-version: E v.d. Logt
  ------------------------------------------------------------------
  Purpose : This file contains the Ethernet routines
  ------------------------------------------------------------------
  $Log$
  ================================================================== */ 
#include "Ethernet.h"
#include "w5500.h"
#include "socket.h"
#include "usart.h"

extern uint8_t dhcpLocalIp[4];
extern uint8_t dhcpGatewayIp[4];
extern uint8_t dhcpSubnetMask[4];
extern uint8_t dhcpDnsServerIp[4];

// XXX: don't make assumptions about the value of MAX_SOCK_NUM.
uint8_t  Ethernet_state[MAX_SOCK_NUM] = { 0, };
uint16_t Ethernet_server_port[MAX_SOCK_NUM] = { 0, };

uint8_t dnsServerAddress[4];

int Ethernet_begin(void)
{
  uint8_t mac_address[6] = {0,};
  uint8_t tmp[4] = {0,0,0,0};
  int ret = 0;
  uint8_t s[25];
  	  
  // Initialize the basic info
  xputs("w5500_init():");
  w5500_init();
  w5500_write_common_register(SIPR,tmp);
  xputs("ok\nread SHAR:");
  w5500_read_common_register(SHAR ,mac_address);
  sprintf(s,"%02x:%02x:%02x:%02x:%02x:%02x ",mac_address[0],mac_address[1],mac_address[2],mac_address[3],mac_address[4],mac_address[5]);
  xputs(s);
  if (!(mac_address[0] | mac_address[1] | mac_address[2] | mac_address[3] | mac_address[4] | mac_address[5]))
  {
	   xputs(": no WIZ550IO\n");
	   return ret; // 0: error, no WIZ550IO found
  } // if
  else xputs("\ndhcp_begin:");
  
  // Now try to get our config info from a DHCP server
  ret = dhcp_begin(mac_address);
  if (ret == 1)
  {
    // We have successfully found a DHCP server and got our configuration info, 
    // so set things accordingly
	w5500_write_common_register(SIPR,dhcpLocalIp);
	w5500_write_common_register(GAR ,dhcpGatewayIp);
	w5500_write_common_register(SUBR,dhcpSubnetMask); 
	ipcpy(dnsServerAddress, dhcpDnsServerIp); // save result
	xputs("ok\n");
  } // if
  else xputs("no DHCP\n");
  return ret;
} // Ethernet_begin()

void Ethernet_begin_ip(uint8_t *local_ip)
{
  uint8_t tmp_ip[4];
  // Assume the DNS server and Gateway will be the machine on the same network 
  // as the local IP but with last octet being '1'
  ipcpy(tmp_ip, local_ip);
  tmp_ip[3] = 1;
  w5500_init(); // init. W5500 chipset
  w5500_write_common_register(SIPR,local_ip); // init. local IP address
  ipcpy(dnsServerAddress, tmp_ip);            // init. DNS-server address
  w5500_write_common_register(GAR ,tmp_ip);   // init. default Gateway address
  tmp_ip[0] = tmp_ip[1] = tmp_ip[2] = 255;
  tmp_ip[3] = 0;							  // subnet-mask = 255.255.255.0
  w5500_write_common_register(SUBR,tmp_ip);   // init. subnet-mask
} // Ethernet_begin_ip()

int Ethernet_maintain(void)
{
  int rc = DHCP_CHECK_NONE;
  rc = dhcp_checkLease();
  switch ( rc )
  {
      case DHCP_CHECK_NONE:
        //nothing done
        break;
      case DHCP_CHECK_RENEW_OK:
      case DHCP_CHECK_REBIND_OK:
        //we might have got a new IP.
		w5500_write_common_register(SIPR,dhcpLocalIp);
		w5500_write_common_register(GAR ,dhcpGatewayIp);
		w5500_write_common_register(SUBR,dhcpSubnetMask);
		ipcpy(dnsServerAddress, dhcpDnsServerIp); // save result
        break;
      default:
        //this is actually a error, it will retry though
        break;
    } // switch
  return rc;
} // Ethernet_maintain()
