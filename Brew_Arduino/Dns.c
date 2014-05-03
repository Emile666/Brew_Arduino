/*==================================================================
  File Name    : $Id$
  Function name: see Dns.h
  Author       : (c) Copyright 2009-2010 MCQN Ltd.
				 C-version: E v.d. Logt
  ------------------------------------------------------------------
  Purpose : Arduino DNS client for WizNet5500-based Ethernet shield
			Released under Apache License, version 2.0
  ------------------------------------------------------------------
  $Log$
  ================================================================== */ 
#include "w5500.h"
#include "Udp.h"
#include "util.h"
#include "Dns.h"
#include "socket.h"
#include <string.h>
#include "delay.h"         /* for delay_msec() */

extern uint8_t   remoteIP[4]; // remote IP address for the incoming packet whilst it's being processed
extern uint16_t  remotePort;  // remote port (from udp.c) for the incoming packet whilst it's being processed

uint8_t  iDNSServer[4];
uint16_t iRequestId;
uint8_t  INADDR_NONE[4] = {0,0,0,0};
 
void dns_begin(uint8_t *aDNSServer)
{
    ipcpy(iDNSServer, aDNSServer);
    iRequestId = 0;
} // dns_begin()

int dns_inet_aton(const char *aIPAddrString, uint8_t *aResult)
{
    // See if we've been given a valid IP address
    const char* p = aIPAddrString;
    while (*p &&
           ( (*p == '.') || (*p >= '0') || (*p <= '9') ))
    {
        p++;
    }

    if (*p == '\0')
    {
        // It's looking promising, we haven't found any invalid characters
        p = aIPAddrString;
		uint8_t segment      = 0;
		int     segmentValue = 0;
        while (*p && (segment < 4))
        {
            if (*p == '.')
            {
                // We've reached the end of a segment
                if (segmentValue > 255)
                {
                    // You can't have an IP address segment that doesn't fit in a byte
                    return 0;
                }
                else
                {
                    aResult[segment] = (uint8_t)segmentValue;
                    segment++;
                    segmentValue = 0;
                }
            }
            else
            {
                // Next digit
                segmentValue = (segmentValue*10)+(*p - '0');
            }
            p++;
        }
        // We've reached the end of an address, but there'll still be the last
        // segment to deal with
        if ((segmentValue > 255) || (segment > 3))
        {
            // You can't have IP address segments that doesn't fit in a byte,
            // or have more than four segments
            return 0;
        }
        else
        {
            aResult[segment] = (uint8_t)segmentValue;
            return 1;
        } // else
    } // if
    else
    {
        return 0;
    } // else
} // dns_inet_aton()

int dns_getHostByName(const char *aHostname, uint8_t *aResult)
{
    int ret =0;

    // See if it's a numeric IP address
    if (dns_inet_aton(aHostname, aResult))
    {
        // It is, our work here is done
        return 1;
    } // if

    // Check we've got a valid DNS server to use
    if (ipequ(iDNSServer, INADDR_NONE))
    {
        return INVALID_SERVER;
    } // if
	
    // Find a socket to use
    if (udp_begin(1024+(millis() & 0xF)) == 1)
    {
        // Try up to three times
        int retries = 0;
//        while ((retries < 3) && (ret <= 0))
        {
            // Send DNS request
            ret = udp_beginPacketIP(iDNSServer, DNS_PORT);
            if (ret != 0)
            {
                // Now output the request data
                ret = dns_BuildRequest(aHostname);
                if (ret != 0)
                {
                    // And finally send the request
                    ret = udp_endPacket();
                    if (ret != 0)
                    {
                        // Now wait for a response
                        int wait_retries = 0;
                        ret = TIMED_OUT;
                        while ((wait_retries < 3) && (ret == TIMED_OUT))
                        {
                            ret = dns_ProcessResponse(5000, aResult);
                            wait_retries++;
                        } // while
                    } // if
                } // if
            } // if
            retries++;
        } // while
        // We're done with the socket now
        udp_stop();
    } // if
    return ret;
} // dns_getHostByName()

uint16_t dns_BuildRequest(const char* aName)
{
    // Build header
    //                                    1  1  1  1  1  1
    //      0  1  2  3  4  5  6  7  8  9  0  1  2  3  4  5
    //    +--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+
    //    |                      ID                       |
    //    +--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+
    //    |QR|   Opcode  |AA|TC|RD|RA|   Z    |   RCODE   |
    //    +--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+
    //    |                    QDCOUNT                    |
    //    +--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+
    //    |                    ANCOUNT                    |
    //    +--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+
    //    |                    NSCOUNT                    |
    //    +--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+
    //    |                    ARCOUNT                    |
    //    +--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+
    // As we only support one request at a time at present, we can simplify
    // some of this header
    iRequestId = millis(); // generate a random ID
    uint16_t twoByteBuffer;

    // FIXME We should also check that there's enough space available to write to, rather
    // FIXME than assume there's enough space (as the code does at present)
    udp_write((uint8_t*)&iRequestId, sizeof(iRequestId));

    twoByteBuffer = htons(QUERY_FLAG | OPCODE_STANDARD_QUERY | RECURSION_DESIRED_FLAG);
    udp_write((uint8_t*)&twoByteBuffer, sizeof(twoByteBuffer));

    twoByteBuffer = htons(1);  // One question record
    udp_write((uint8_t*)&twoByteBuffer, sizeof(twoByteBuffer));

    twoByteBuffer = 0;  // Zero answer records
    udp_write((uint8_t*)&twoByteBuffer, sizeof(twoByteBuffer));

    udp_write((uint8_t*)&twoByteBuffer, sizeof(twoByteBuffer));
    // and zero additional records
    udp_write((uint8_t*)&twoByteBuffer, sizeof(twoByteBuffer));

    // Build question
    const char* start = aName;
    const char* end   = start;
    uint8_t len;
    // Run through the name being requested
    while (*end)
    {
        // Find out how long this section of the name is
        end = start;
        while (*end && (*end != '.') )
        {
            end++;
        }
        if (end > start)
        {
            // Write out the size of this section
            len = end - start;
            udp_write(&len, sizeof(len));
            // And then write out the section
            udp_write((uint8_t*)start, end-start);
        }
        start = end + 1;
    } // while

    // We've got to the end of the question name, so
    // terminate it with a zero-length section
    len = 0;
    udp_write(&len, sizeof(len));
    // Finally the type and class of question
    twoByteBuffer = htons(TYPE_A);
    udp_write((uint8_t*)&twoByteBuffer, sizeof(twoByteBuffer));

    twoByteBuffer = htons(CLASS_IN);  // Internet class of question
    udp_write((uint8_t*)&twoByteBuffer, sizeof(twoByteBuffer));
    // Success!  Everything buffered okay
    return 1;
} // dns_BuildRequest()

uint16_t dns_ProcessResponse(uint16_t aTimeout, uint8_t *aAddress)
{
    uint32_t startTime = millis();

    // Wait for a response packet
    while(udp_parsePacket() <= 0)
    {
        if((millis() - startTime) > aTimeout)
            return TIMED_OUT;
        delay_msec(50);
    } // while

    // We've had a reply!
    // Read the UDP header
    uint8_t header[DNS_HEADER_SIZE]; // Enough space to reuse for the DNS header
    // Check that it's a response from the right server and the right port
    if (!ipequ(iDNSServer, remoteIP) || (remotePort != DNS_PORT))
    {
        // It's not from who we expected
        return INVALID_SERVER;
    } // if
    // Read through the rest of the response
    if (udp_available() < DNS_HEADER_SIZE)
    {
        return TRUNCATED;
    } // if
    udp_read(header, DNS_HEADER_SIZE);

    uint16_t header_flags = htons(*((uint16_t*)&header[2]));
    // Check that it's a response to this request
    if ( ( iRequestId != (*((uint16_t*)&header[0])) ) ||
        ((header_flags & QUERY_RESPONSE_MASK) != (uint16_t)RESPONSE_FLAG) )
    {
        // Mark the entire packet as read
        udp_flush();
        return INVALID_RESPONSE;
    } // if
    // Check for any errors in the response (or in our request)
    // although we don't do anything to get round these
    if ( (header_flags & TRUNCATION_FLAG) || (header_flags & RESP_MASK) )
    {
        // Mark the entire packet as read
        udp_flush();
        return -5; //INVALID_RESPONSE;
    } // if

    // And make sure we've got (at least) one answer
    uint16_t answerCount = htons(*((uint16_t*)&header[6]));
    if (answerCount == 0 )
    {
        // Mark the entire packet as read
        udp_flush();
        return -6; //INVALID_RESPONSE;
    } // if

    // Skip over any questions
    for (uint16_t i =0; i < htons(*((uint16_t*)&header[4])); i++)
    {
        // Skip over the name
        uint8_t len;
        do
        {
            udp_read(&len, sizeof(len));
            if (len > 0)
            {
                // Don't need to actually read the data out for the string, just
                // advance ptr to beyond it
                while(len--)
                {
                    udp_read1(); // we don't care about the returned byte
                } // while
            } // if
        } while (len != 0);

        // Now jump over the type and class
        for (int i =0; i < 4; i++)
        {
            udp_read1(); // we don't care about the returned byte
        } // for
    } // for

    // Now we're up to the bit we're interested in, the answer
    // There might be more than one answer (although we'll just use the first
    // type A answer) and some authority and additional resource records but
    // we're going to ignore all of them.

    for (uint16_t i = 0; i < answerCount; i++)
    {
        // Skip the name
        uint8_t len;
        do
        {
            udp_read(&len, sizeof(len));
            if ((len & LABEL_COMPRESSION_MASK) == 0)
            {
                // It's just a normal label
                if (len > 0)
                {
                    // And it's got a length
                    // Don't need to actually read the data out for the string,
                    // just advance ptr to beyond it
                    while(len--)
                    {
                        udp_read1(); // we don't care about the returned byte
                    } // while
                } // if
            } // if
            else
            {
                // This is a pointer to a somewhere else in the message for the
                // rest of the name.  We don't care about the name, and RFC1035
                // says that a name is either a sequence of labels ended with a
                // 0 length octet or a pointer or a sequence of labels ending in
                // a pointer.  Either way, when we get here we're at the end of
                // the name. Skip over the pointer
                udp_read1(); // we don't care about the returned byte
                // And set len so that we drop out of the name loop
                len = 0;
            } // else
        } while (len != 0);

        // Check the type and class
        uint16_t answerType;
        uint16_t answerClass;
        udp_read((uint8_t*)&answerType, sizeof(answerType));
        udp_read((uint8_t*)&answerClass, sizeof(answerClass));

        // Ignore the Time-To-Live as we don't do any caching
        for (int i =0; i < TTL_SIZE; i++)
        {
            udp_read1(); // we don't care about the returned byte
        } // for

        // And read out the length of this answer
        // Don't need header_flags anymore, so we can reuse it here
        udp_read((uint8_t*)&header_flags, sizeof(header_flags));

        if ( (htons(answerType) == TYPE_A) && (htons(answerClass) == CLASS_IN) )
        {
            if (htons(header_flags) != 4)
            {
                // It's a weird size
                // Mark the entire packet as read
                udp_flush();
                return -9;//INVALID_RESPONSE;
            } // if
            udp_read((uint8_t *)aAddress, 4);
            return SUCCESS;
        } // if
        else
        {
            // This isn't an answer type we're after, move onto the next one
            for (uint16_t i =0; i < htons(header_flags); i++)
            {
                udp_read1(); // we don't care about the returned byte
            } // for
        } // else
    } // for

    // Mark the entire packet as read
    udp_flush();

    // If we get here then we haven't found an answer
    return -10;//INVALID_RESPONSE;
} // dns_ProcessResponse()

