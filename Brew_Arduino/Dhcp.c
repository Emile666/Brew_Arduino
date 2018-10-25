/*==================================================================
  File Name    : Dhcp.c
  Author       : Jordan Terrell - blog.jordanterrell.com
				 C-version: E v.d. Logt
  ------------------------------------------------------------------
  Purpose : DHCP Library v0.3 - April 25, 2009
  ================================================================== */ 
#include "w5500.h"
#include <string.h>
#include <stdlib.h>
#include "Dhcp.h"
#include "util.h"
#include "Udp.h"
#include "socket.h"
#include "delay.h"         /* for delay_msec() */

extern uint8_t  remoteIP[4]; // remote IP address for the incoming packet whilst it's being processed
extern uint16_t remotePort; // remote port (from udp.c) for the incoming packet whilst it's being processed

uint32_t      dhcpInitialTransactionId;
uint32_t      dhcpTransactionId;
uint8_t       dhcpMacAddr[6];
uint8_t       dhcpLocalIp[4];
uint8_t       dhcpSubnetMask[4];
uint8_t       dhcpGatewayIp[4];
uint8_t       dhcpDhcpServerIp[4];
uint8_t       dhcpDnsServerIp[4];
uint32_t      dhcpLeaseTime;
uint32_t      dhcpT1, dhcpT2;
signed long   renewInSec;
signed long   rebindInSec;
signed long   lastCheck;
unsigned long timeout;
unsigned long responseTimeout;
unsigned long secTimeout;
uint8_t       dhcp_state;

int dhcp_begin(uint8_t *mac)
{
    dhcpLeaseTime   = 0;
    dhcpT1          = 0;
    dhcpT2          = 0;
    lastCheck       = 0;
    timeout         = DHCP_TIMEOUT;
    responseTimeout = DHCP_RESPONSE_TIMEOUT;

    srand(TCNT2);
	// zero out dhcpMacAddr
    memset(dhcpMacAddr, 0, 6); 
    reset_DHCP_lease();

    memcpy((void*)dhcpMacAddr, (void*)mac, 6);
    dhcp_state = STATE_DHCP_START;
    return request_DHCP_lease();
} // dhcp_begin()

void reset_DHCP_lease(void)
{
	uint8_t i;
	
    // zero out dhcpLocalIp, dhcpSubnetMask, dhcpGatewayIp, dhcpDhcpServerIp, dhcpDnsServerIp
	for (i = 0; i < 4; i++)
	{
		dhcpLocalIp[i] = dhcpSubnetMask[i] = dhcpGatewayIp[i] = dhcpDhcpServerIp[i] = dhcpDnsServerIp[i] = 0;
	} // for i	
} // reset_DHCP_lease()

//return:0 on error, 1 if request is sent and response is received
int request_DHCP_lease(void)
{
    uint8_t messageType = 0;
	
    // Pick an initial transaction ID
    dhcpTransactionId = rand() % 2000UL + 1UL;
    dhcpInitialTransactionId = dhcpTransactionId;

    udp_stop();
    if (udp_begin(DHCP_CLIENT_PORT) == 0)
    {
      // Couldn't get a socket
      return 0;
    }
    presend_DHCP();

    int result = 0;
    unsigned long startTime = millis();
    
    while(dhcp_state != STATE_DHCP_LEASED)
    {
        if(dhcp_state == STATE_DHCP_START)
        {
            dhcpTransactionId++;
            
            send_DHCP_MESSAGE(DHCP_DISCOVER, ((millis() - startTime) / 1000));
            dhcp_state = STATE_DHCP_DISCOVER;
        }
        else if(dhcp_state == STATE_DHCP_REREQUEST)
		{
            dhcpTransactionId++;
            send_DHCP_MESSAGE(DHCP_REQUEST, ((millis() - startTime)/1000));
            dhcp_state = STATE_DHCP_REQUEST;
        }
        else if(dhcp_state == STATE_DHCP_DISCOVER)
        {
            uint32_t respId;
            messageType = parseDHCPResponse(responseTimeout, &respId);
            if(messageType == DHCP_OFFER)
            {
                // We'll use the transaction ID that the offer came with,
                // rather than the one we were up to
                dhcpTransactionId = respId;
                send_DHCP_MESSAGE(DHCP_REQUEST, ((millis() - startTime) / 1000));
                dhcp_state = STATE_DHCP_REQUEST;
            } // if
        } // else if
        else if(dhcp_state == STATE_DHCP_REQUEST)
        {
            uint32_t respId;
            messageType = parseDHCPResponse(responseTimeout, &respId);
            if (messageType == DHCP_ACK)
            {
                dhcp_state = STATE_DHCP_LEASED;
                result = 1;
                //use default lease time if we didn't get it
                if(dhcpLeaseTime == 0)
				{
                    dhcpLeaseTime = DEFAULT_LEASE;
                }
                //calculate T1 & T2 if we didn't get it
                if(dhcpT1 == 0)
				{
                    //T1 should be 50% of dhcpLeaseTime
                    dhcpT1 = dhcpLeaseTime >> 1;
                }
                if(dhcpT2 == 0)
				{
                    //T2 should be 87.5% (7/8ths) of dhcpLeaseTime
                    dhcpT2 = dhcpT1 << 1;
                }
                renewInSec = dhcpT1;
                rebindInSec = dhcpT2;
            } // if
            else if (messageType == DHCP_NAK)
                dhcp_state = STATE_DHCP_START;
        } // else if
        if (messageType == 255)
        {
            messageType = 0;
            dhcp_state = STATE_DHCP_START;
        } // if
        if(result != 1 && ((millis() - startTime) > timeout))
            break;
    }
    // We're done with the socket now
    udp_stop();
    dhcpTransactionId++;
    return result;
} // request_DHCP_lease()

void presend_DHCP(void)
{
} // presend_DHCP()

void send_DHCP_MESSAGE(uint8_t messageType, uint16_t secondsElapsed)
{
    uint8_t buffer[32];
    memset(buffer, 0, 32);
    uint8_t dest_addr[4] = {255, 255, 255, 255}; // Broadcast address

    if (-1 == udp_beginPacketIP(dest_addr, DHCP_SERVER_PORT))
    {
        // FIXME Need to return errors
        return;
    }

    buffer[0] = DHCP_BOOTREQUEST;   // op
    buffer[1] = DHCP_HTYPE10MB;     // htype
    buffer[2] = DHCP_HLENETHERNET;  // hlen
    buffer[3] = DHCP_HOPS;          // hops

    // xid
    unsigned long xid = htonl(dhcpTransactionId);
    memcpy(buffer + 4, &xid, 4);

    // 8, 9 - seconds elapsed
    buffer[8] = ((secondsElapsed & 0xff00) >> 8);
    buffer[9] = (secondsElapsed & 0x00ff);

    // flags
    unsigned short flags = htons(DHCP_FLAGSBROADCAST);
    memcpy(buffer + 10, &flags, 2);

    // ciaddr: already zeroed
    // yiaddr: already zeroed
    // siaddr: already zeroed
    // giaddr: already zeroed

    //put data in W5500 transmit buffer
    udp_write(buffer, 28);
    memset(buffer, 0, 32); // clear local buffer
    memcpy(buffer, dhcpMacAddr, 6); // chaddr

    //put data in W5500 transmit buffer
    udp_write(buffer, 16);
    memset(buffer, 0, 32); // clear local buffer

    // leave zeroed out for sname && file
    // put in W5500 transmit buffer x 6 (192 bytes)
    for (int i = 0; i < 6; i++) 
	{
        udp_write(buffer, 32);
    } // for
  
    // OPT - Magic Cookie
    buffer[0] = (uint8_t)((MAGIC_COOKIE >> 24)& 0xFF);
    buffer[1] = (uint8_t)((MAGIC_COOKIE >> 16)& 0xFF);
    buffer[2] = (uint8_t)((MAGIC_COOKIE >> 8)& 0xFF);
    buffer[3] = (uint8_t)(MAGIC_COOKIE& 0xFF);

    // OPT - message type
    buffer[4] = dhcpMessageType;
    buffer[5] = 0x01;
    buffer[6] = messageType; //DHCP_REQUEST;

    // OPT - client identifier
    buffer[7] = dhcpClientIdentifier;
    buffer[8] = 0x07;
    buffer[9] = 0x01;
    memcpy(buffer + 10, dhcpMacAddr, 6);

    // OPT - host name
    buffer[16] = hostName;
    buffer[17] = strlen(HOST_NAME) + 6; // length of hostname + last 3 bytes of mac address
    strcpy((char*)&(buffer[18]), HOST_NAME);

    dhcp_printByte((char*)&(buffer[24]), dhcpMacAddr[3]);
    dhcp_printByte((char*)&(buffer[26]), dhcpMacAddr[4]);
    dhcp_printByte((char*)&(buffer[28]), dhcpMacAddr[5]);

    //put data in W5500 transmit buffer
    udp_write(buffer, 30);

    if (messageType == DHCP_REQUEST)
    {
        buffer[0]  = dhcpRequestedIPaddr;
        buffer[1]  = 0x04;
        buffer[2]  = dhcpLocalIp[0];
        buffer[3]  = dhcpLocalIp[1];
        buffer[4]  = dhcpLocalIp[2];
        buffer[5]  = dhcpLocalIp[3];

        buffer[6]  = dhcpServerIdentifier;
        buffer[7]  = 0x04;
        buffer[8]  = dhcpDhcpServerIp[0];
        buffer[9]  = dhcpDhcpServerIp[1];
        buffer[10] = dhcpDhcpServerIp[2];
        buffer[11] = dhcpDhcpServerIp[3];

        //put data in W5500 transmit buffer
        udp_write(buffer, 12);
    } // if
    
    buffer[0] = dhcpParamRequest;
    buffer[1] = 0x06;
    buffer[2] = subnetMask;
    buffer[3] = routersOnSubnet;
    buffer[4] = dns;
    buffer[5] = domainName;
    buffer[6] = dhcpT1value;
    buffer[7] = dhcpT2value;
    buffer[8] = endOption;
    
    //put data in W5500 transmit buffer
    udp_write(buffer, 9);
    udp_endPacket();
} // send_DHCP_MESSAGE()

uint8_t parseDHCPResponse(unsigned long responseTimeout, uint32_t *transactionId)
{
    uint8_t type = 0;
    uint8_t opt_len = 0;
     
    unsigned long startTime = millis();

    while(udp_parsePacket() <= 0)
    {
        if((millis() - startTime) > responseTimeout)
        {
            return 255;
        }
        delay_msec(50);
    }
    // start reading in the packet
    RIP_MSG_FIXED fixedMsg;
    udp_read((uint8_t*)&fixedMsg, sizeof(RIP_MSG_FIXED));
  
    if ((fixedMsg.op == DHCP_BOOTREPLY) && (remotePort == DHCP_SERVER_PORT))
    {
        *transactionId = ntohl(fixedMsg.xid);
        if(memcmp(fixedMsg.chaddr, dhcpMacAddr, 6) != 0 || (*transactionId < dhcpInitialTransactionId) || (*transactionId > dhcpTransactionId))
        {
            // Need to read the rest of the packet here regardless
            udp_flush();
            return 0;
        }
        memcpy(dhcpLocalIp, fixedMsg.yiaddr, 4);

        // Skip to the option part
        // Doing this a byte at a time so we don't have to put a big buffer
        // on the stack (as we don't have lots of memory lying around)
        for (int i =0; i < (240 - (int)sizeof(RIP_MSG_FIXED)); i++)
        {
            udp_read1(); // we don't care about the returned byte
        }

        while (udp_available() > 0) 
        {
            switch (udp_read1()) 
            {
                case endOption :
                    break;
                    
                case padOption :
                    break;
                
                case dhcpMessageType :
                    opt_len = udp_read1();
                    type    = udp_read1();
                    break;
                
                case subnetMask :
                    opt_len = udp_read1();
                    udp_read(dhcpSubnetMask, 4);
                    break;
                
                case routersOnSubnet :
                    opt_len = udp_read1();
                    udp_read(dhcpGatewayIp, 4);
                    for (int i = 0; i < opt_len-4; i++)
                    {
                        udp_read1();
                    }
                    break;
                
                case dns :
                    opt_len = udp_read1();
                    udp_read(dhcpDnsServerIp, 4);
                    for (int i = 0; i < opt_len-4; i++)
                    {
                        udp_read1();
                    }
                    break;
                
                case dhcpServerIdentifier :
                    opt_len = udp_read1();
                    if( *((uint32_t*)dhcpDhcpServerIp) == 0 || ipequ(dhcpDhcpServerIp, remoteIP))
                    {
                        udp_read(dhcpDhcpServerIp, 4);
                    }
                    else
                    {
                        // Skip over the rest of this option
                        while (opt_len--)
                        {
                            udp_read1();
                        } // while
                    } // else
                    break;

                case dhcpT1value : 
                    opt_len = udp_read1();
                    udp_read((uint8_t*)&dhcpT1, sizeof(dhcpT1));
                    dhcpT1  = ntohl(dhcpT1);
                    break;

                case dhcpT2value : 
                    opt_len = udp_read1();
                    udp_read((uint8_t*)&dhcpT2, sizeof(dhcpT2));
                    dhcpT2  = ntohl(dhcpT2);
                    break;

                case dhcpIPaddrLeaseTime :
                    opt_len = udp_read1();
                    udp_read((uint8_t*)&dhcpLeaseTime, sizeof(dhcpLeaseTime));
                    dhcpLeaseTime = ntohl(dhcpLeaseTime);
                    renewInSec    = dhcpLeaseTime;
                    break;

                default :
                    opt_len = udp_read1();
                    // Skip over the rest of this option
                    while (opt_len--)
                    {
                        udp_read1();
                    } // while
                    break;
            } // switch
        } // while
    } // if
    // Need to skip to end of the packet regardless here
    udp_flush();
    return type;
} // parseDHCPResponse()

/*
    returns:
    0/DHCP_CHECK_NONE: nothing happened
    1/DHCP_CHECK_RENEW_FAIL: renew failed
    2/DHCP_CHECK_RENEW_OK: renew success
    3/DHCP_CHECK_REBIND_FAIL: rebind fail
    4/DHCP_CHECK_REBIND_OK: rebind success
*/
int dhcp_checkLease(void)
{
    //this uses a signed / unsigned trick to deal with millis overflow
    unsigned long now = millis();
    signed long snow = (long)now;
    int rc=DHCP_CHECK_NONE;
    if (lastCheck != 0)
	{
        signed long factor;
        //calc how many ms past the timeout we are
        factor = snow - (long)secTimeout;
        //if on or passed the timeout, reduce the counters
        if ( factor >= 0 )
		{
            //next timeout should be now plus 1000 ms minus parts of second in factor
            secTimeout = snow + 1000 - factor % 1000;
            //how many seconds late are we, minimum 1
            factor = factor / 1000 +1;
            
            //reduce the counters by that much
            //if we can assume that the cycle time (factor) is fairly constant
            //and if the remainder is less than cycle time * 2 
            //do it early instead of late
            if (renewInSec < factor*2 )
                 renewInSec = 0;
            else renewInSec -= factor;
            
            if (rebindInSec < factor*2 )
                 rebindInSec = 0;
            else rebindInSec -= factor;
        } // if

        //if we have a lease but should renew, do it
        if (dhcp_state == STATE_DHCP_LEASED && renewInSec <=0)
		{
            dhcp_state = STATE_DHCP_REREQUEST;
            rc = 1 + request_DHCP_lease();
        } // if

        //if we have a lease or is renewing but should bind, do it
        if ((dhcp_state == STATE_DHCP_LEASED || dhcp_state == STATE_DHCP_START) && rebindInSec <=0)
		{
            //this should basically restart completely
            dhcp_state = STATE_DHCP_START;
            reset_DHCP_lease();
            rc = 3 + request_DHCP_lease();
        } // if
    } // if
    else
	{
        secTimeout = snow + 1000;
    } // else
    lastCheck = now;
    return rc;
} // dhcp_checkLease()

void dhcp_printByte(char * buf, uint8_t n) 
{
  char *str = &buf[1];
  buf[0]='0';
  do {
    unsigned long m = n;
    n /= 16;
    char c = m - 16 * n;
    *str-- = c < 10 ? c + '0' : c + 'A' - 10;
  } while(n);
} // dhcp_printByte()
