/*==================================================================
  File Name    : socket.c
  Function name: see socket.h
  Author       : ?
				 C-version: E v.d. Logt
  ------------------------------------------------------------------
  Purpose : Socket routines for use with the WIZ550io module
  ================================================================== */ 
#include "socket.h"

static uint16_t local_port;

/*------------------------------------------------------------------
  Purpose  : This function copies a src IP address into a 
             dest IP address
  Variables: 
       src : the source IP address
      dest : the destination IP address 
  Returns  : -
  ------------------------------------------------------------------*/
void ipcpy(uint8_t *dest, uint8_t *src)
{
	uint8_t i;
	
	for (i = 0; i < 4; i++) dest[i] = src[i];
} // ipcpy()

/*------------------------------------------------------------------
  Purpose  : This function checks if two IP addresses are identical
  Variables: 
      str1 : IP address 1
      str2 : IP address 2
  Returns  : -
  ------------------------------------------------------------------*/
bool ipequ(uint8_t *str1, uint8_t *str2)
{
	if ((str1[0] == str2[0]) && (str1[1] == str2[1]) && (str1[2] == str2[2]) && (str1[3] == str2[3]))
	return      true;
	else return false;
} // ipequ()

/**
 * @brief	This Socket function initialize the channel in particular mode, and set the port and wait for W5500 done it.
 * @return 	1 for success else 0.
 */
uint8_t socket(SOCKET s, uint8_t protocol, uint16_t port, uint8_t flag)
{
  uint8_t  bufr[2]; // buffer for w5500_write_socket_register()
  
  if ((protocol == SnMR_TCP) || (protocol == SnMR_UDP) || (protocol == SnMR_IPRAW) || (protocol == SnMR_MACRAW) || (protocol == SnMR_PPPOE))
  {
    close(s);
	bufr[0] = protocol | flag;
	w5500_write_socket_register(s, Sn_MR, bufr); // write to Mode Register
    if (port != 0) 
	{
	  bufr[0] = port >> 8;   // MSB of port
	  bufr[1] = port & 0xff;	// LSB of port
      w5500_write_socket_register(s, Sn_PORT, bufr); // write to Source Port Register
    } // if
    else 
	{
      local_port++; // if source port is not set, set the local port number.
	  bufr[0] = local_port >> 8;   // MSB of local port
	  bufr[1] = local_port & 0xff; // LSB of local port
      w5500_write_socket_register(s, Sn_PORT, bufr); // write to Source Port Register
    } // else
    w5500_execCmdSn(s, Sock_OPEN); // Write OPEN command to W5500 Command Register
    return 1;
  } // if
  return 0;
} // socket()

/**
 * @brief	This function close the socket and parameter is "s" which represent the socket number
 */
void close(SOCKET s)
{
  uint8_t  bufr[2]; // buffer for w5500_read_socket_register()

  w5500_execCmdSn(s, Sock_CLOSE);  // Write OPEN command to W5500 Command Register
  bufr[0] = 0xff;
  w5500_write_socket_register(s, Sn_IR, bufr); // Clear all Socket interrupt flags
} // close()

/**
 * @brief	This function establishes  the connection for the channel in passive (server) mode. This function waits for the request from the peer.
 * @return	1 for success else 0.
 */
uint8_t listen(SOCKET s)
{
  uint8_t  bufr[2]; // buffer for w5500_read_socket_register()
  
  if (w5500_read_socket_register(s, Sn_SR, bufr) != SnSR_INIT)
    return 0;
  w5500_execCmdSn(s, Sock_LISTEN); // Write LISTEN command to W5500 Command Register
  return 1;
} // listen()

/**
 * @brief	This function establishes  the connection for the channel in Active (client) mode. 
 * 		This function waits until the connection is established.
 * 		
 * @return	1 for success else 0.
 */
uint8_t connect(SOCKET s, uint8_t * addr, uint16_t port)
{
  uint8_t bufr[2];
  
  if (((addr[0] == 0xFF) && (addr[1] == 0xFF) && (addr[2] == 0xFF) && (addr[3] == 0xFF)) ||
      ((addr[0] == 0x00) && (addr[1] == 0x00) && (addr[2] == 0x00) && (addr[3] == 0x00)) ||
      (port == 0x00)) 
	return 0;

  // set destination IP
  w5500_write_socket_register(s, Sn_DIPR , addr);  // Write Destination IP Address Register
  bufr[0] = port >> 8;
  bufr[1] = port & 0xff;
  w5500_write_socket_register(s, Sn_DPORT, bufr); // Write Destination IP Port Register
  w5500_execCmdSn(s, Sock_CONNECT);  // Write CONNECT command to W5500 Command Register
  return 1;
} // connect()

/**
 * @brief	This function used for disconnect the socket and parameter is "s" which represent the socket number
 * @return	1 for success else 0.
 */
void disconnect(SOCKET s)
{
  w5500_execCmdSn(s, Sock_DISCON); // Write DISCONNECT command to W5500 Command Register
} // disconnect()

/**
 * @brief	This function used to send the data in TCP mode
 * @return	1 for success else 0.
 */
uint16_t send(SOCKET s, const uint8_t * buf, uint16_t len)
{
  uint8_t  status   = 0;
  uint16_t ret      = 0;
  uint16_t freesize = 0;
  uint8_t  bufr[2]; // buffer for w5500_read_socket_register()

  if (len > W5500_SSIZE) 
       ret = W5500_SSIZE; // check that size does not exceed MAX size
  else ret = len;

  // if free space in the buf is available, start.
  do 
  {
    freesize = w5500_getTXFreeSize(s);
    status   = w5500_read_socket_register(s, Sn_SR, bufr);
    if ((status != SnSR_ESTABLISHED) && (status != SnSR_CLOSE_WAIT))
    {
      ret = 0; 
      break;
    } // if
  } 
  while (freesize < ret);

  // copy data
  w5500_send_data_processing(s, (uint8_t *)buf, ret);
  w5500_execCmdSn(s, Sock_SEND);

  while ((w5500_read_socket_register(s, Sn_IR, bufr) & SEND_OK) != SEND_OK ) 
  {
    if (w5500_read_socket_register(s, Sn_SR, bufr) == SnSR_CLOSED )
    {
      close(s);
      return 0;
    } // if
  } // while
  bufr[0] = SEND_OK;
  w5500_write_socket_register(s, Sn_IR, bufr);
  return ret;
} // send()

/**
 * @brief	This function is an application I/F function which is used to receive the data in TCP mode.
 * 		It continues to wait for data as much as the application wants to receive.
 * 		
 * @return	received data size for success else -1.
 */
int16_t recv(SOCKET s, uint8_t *buf, int16_t len)
{
  uint8_t status;
  uint8_t bufr[2]; // buffer for w5500_read_socket_register()
  int16_t ret = w5500_getRXReceivedSize(s); // Check how much data is available
  
  if (ret == 0)
  {
    // No data available.
    status = w5500_read_socket_register(s, Sn_SR, bufr);
    if ((status == SnSR_LISTEN) || (status == SnSR_CLOSED) || (status == SnSR_CLOSE_WAIT))
    {
      // The remote end has closed its side of the connection, so this is the eof state
      ret = 0;
    }
    else
    {
      // The connection is still up, but there's no data waiting to be read
      ret = -1;
    } // else
  } // if
  else if (ret > len)
  {
    ret = len;
  } // else if

  if (ret > 0)
  {
    w5500_recv_data_processing(s, buf, ret, 0); // with peek = 0
    w5500_execCmdSn(s, Sock_RECV);
  } // if
  return ret;
} // recv()

/**
 * @brief	Returns the first byte in the receive queue (no checking)
 * 		
 * @return
 */
uint16_t peek(SOCKET s, uint8_t *buf)
{
  w5500_recv_data_processing(s, buf, 1, 1);
  return 1;
} // peek()


/**
 * @brief	This function is an application I/F function which is used to send the data for other than TCP mode. 
 * 		Unlike TCP transmission, The peer's destination address and the port is needed.
 * 		
 * @return	This function returns send data size for success else -1.
 */
uint16_t sendto(SOCKET s, const uint8_t *buf, uint16_t len, uint8_t *addr, uint16_t port)
{
  uint16_t ret = 0;
  uint8_t  bufr[2]; // buffer for w5500_read_socket_register()

  if (len > W5500_SSIZE) 
       ret = W5500_SSIZE; // check that size does not exceed MAX size
  else ret = len;

  if (((addr[0] == 0x00) && (addr[1] == 0x00) && (addr[2] == 0x00) && (addr[3] == 0x00)) ||
      ((port == 0x00)) ||(ret == 0)) 
  {
    ret = 0;
  }
  else
  {
    w5500_write_socket_register(s, Sn_DIPR , addr);  // Write Destination IP Address Register
	bufr[0] = (port >> 8);
	bufr[1] = (port & 0xff);
    w5500_write_socket_register(s, Sn_DPORT, bufr); // Write Destination IP Port Register
    // copy data
    w5500_send_data_processing(s, (uint8_t *)buf, ret);
    w5500_execCmdSn(s, Sock_SEND);

    while ((w5500_read_socket_register(s, Sn_IR, bufr) & SEND_OK) != SEND_OK ) 
    {
      if (w5500_read_socket_register(s, Sn_IR, bufr) & TIMEOUT)
      {
		bufr[0] = SEND_OK | TIMEOUT;  
        w5500_write_socket_register(s, Sn_IR, bufr); /* clear SEND_OK & TIMEOUT */
        return 0;
      } // if
    } // while
	bufr[0] = SEND_OK;
    w5500_write_socket_register(s, Sn_IR, bufr);
  } // else
  return ret;
} // sendto()

/**
 * @brief	This function is an application I/F function which is used to receive the data in other then
 * 	TCP mode. This function is used to receive UDP, IP_RAW and MAC_RAW mode, and handle the header as well. 
 * 	
 * @return	This function return received data size for success else -1.
 */
uint16_t recvfrom(SOCKET s, uint8_t *buf, uint16_t len, uint8_t *addr, uint16_t *port)
{
  uint8_t  bufr[2]; // buffer for w5500_read_socket_register()
  uint8_t  head[8];
  uint16_t data_len = 0;
  uint16_t ptr      = 0;

  if (len > 0)
  {
    ptr = w5500_read_socket_register(s, Sn_RX_RD, bufr);
    switch (w5500_read_socket_register(s, Sn_MR, bufr) & 0x07)
    {
		case SnMR_UDP :
		  w5500_read_data(s, ptr, head, 0x08);
		  ptr += 8;
		  // read peer's IP address, port number.
		  ipcpy(addr, head); // copy IP address in head to addr
		  *port    = head[4];
		  *port    = (*port << 8) + head[5];
		  data_len = head[6];
		  data_len = (data_len << 8) + head[7];

		  w5500_read_data(s, ptr, buf, data_len); // data copy.
		  ptr += data_len;
		  w5500_write_socket_register(s, Sn_RX_RD, (uint8_t *)ptr);
		  break;

		case SnMR_IPRAW :
		  w5500_read_data(s, ptr, head, 0x06);
		  ptr += 6;
		  ipcpy(addr, head); // copy IP address in head to addr
		  data_len = head[4];
		  data_len = (data_len << 8) + head[5];

		  w5500_read_data(s, ptr, buf, data_len); // data copy.
		  ptr += data_len;
		  w5500_write_socket_register(s, Sn_RX_RD, (uint8_t *)ptr);
		  break;

		case SnMR_MACRAW:
		  w5500_read_data(s, ptr, head, 2);
		  ptr += 2;
		  data_len = head[0];
		  data_len = (data_len<<8) + head[1] - 2;

		  w5500_read_data(s, ptr, buf, data_len);
		  ptr += data_len;
		  w5500_write_socket_register(s, Sn_RX_RD, (uint8_t *)ptr);
		  break;

		default :
		  break;
    } // switch
    w5500_execCmdSn(s, Sock_RECV);
  } // if
  return data_len;
} // recvfrom()

/**
 * @brief	Wait for buffered transmission to complete.
 */
void flush(SOCKET s) 
{
  // TODO
} // flush()

uint16_t igmpsend(SOCKET s, const uint8_t * buf, uint16_t len)
{
  uint8_t  bufr[2]; // buffer for w5500_read_socket_register()
  uint8_t  status = 0;
  uint16_t ret    = 0;

  if (len > W5500_SSIZE)
  ret = W5500_SSIZE; // check that size does not exceed MAX size
  else ret = len;

  if (ret == 0) return 0;

  w5500_send_data_processing(s, (uint8_t *)buf, ret);
  w5500_execCmdSn(s, Sock_SEND);

  while ((w5500_read_socket_register(s, Sn_IR, bufr) & SEND_OK) != SEND_OK ) 
  {
    status = w5500_read_socket_register(s, Sn_SR, bufr);
    if (w5500_read_socket_register(s, Sn_IR, bufr) & TIMEOUT)
    {
      /* in case of igmp, if send fails, then socket closed */
      /* if you want change, remove this code. */
      close(s);
      return 0;
    } // if
  } // while

  bufr[0] = SEND_OK;
  w5500_write_socket_register(s, Sn_IR, bufr);
  return ret;
} // igmpsend()

/*
  @brief This function copies up to len bytes of data from buf into a UDP datagram to be
  sent later by sendUDP.  Allows datagrams to be built up from a series of bufferData calls.
  @return Number of bytes successfully buffered
*/
uint16_t bufferData(SOCKET s, uint16_t offset, const uint8_t* buf, uint16_t len)
{
  uint16_t ret = 0;
  if (len > w5500_getTXFreeSize(s))
  {
    ret = w5500_getTXFreeSize(s); // check size not to exceed MAX size.
  }
  else
  {
    ret = len;
  }
  w5500_send_data_processing_offset(s, offset, buf, ret);
  return ret;
} // bufferData()

// Functions to allow buffered UDP send (i.e. where the UDP datagram is built up over a
// number of calls before being sent
/*
  @brief This function sets up a UDP datagram, the data for which will be provided by one
  or more calls to bufferData and then finally sent with sendUDP.
  @return 1 if the datagram was successfully set up, or 0 if there was an error
*/int startUDP(SOCKET s, uint8_t* addr, uint16_t port)
{
  uint8_t bufr[2]; // buffer for w5500_read_socket_register()

  if(((addr[0] == 0x00) && (addr[1] == 0x00) && (addr[2] == 0x00) && (addr[3] == 0x00)) ||
     ((port == 0x00))) 
  {
    return 0;
  } // if
  else
  {
    w5500_write_socket_register(s, Sn_DIPR , addr);  // Write Destination IP Address Register
	bufr[0] = port >> 8;
	bufr[1] = port & 0xff;
    w5500_write_socket_register(s, Sn_DPORT, bufr); // Write Destination IP Port Register
    return 1;
  } // else
} // startUDP()

/*
  @brief Send a UDP datagram built up from a sequence of startUDP followed by one or more
  calls to bufferData.
  @return 1 if the datagram was successfully sent, or 0 if there was an error
*/
int sendUDP(SOCKET s)
{
  uint8_t bufr[2]; // buffer for w5500_read_socket_register()

  w5500_execCmdSn(s, Sock_SEND);
  while ((w5500_read_socket_register(s, Sn_IR, bufr) & SEND_OK) != SEND_OK ) 
  {
    if (w5500_read_socket_register(s, Sn_IR, bufr) & TIMEOUT)
    {
	  bufr[0] = SEND_OK | TIMEOUT;	
      w5500_write_socket_register(s, Sn_IR, bufr);
      return 0;
    } // if
  } // while
  bufr[0] = SEND_OK;
  w5500_write_socket_register(s, Sn_IR, bufr);
  return 1; // Sent ok
} // sendUDP()

