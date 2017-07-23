/*==================================================================
  File Name    : $Id: $
  Function name: -
  Author       : E v.d. Logt
  ------------------------------------------------------------------
  Purpose : EEPROM routines
  ------------------------------------------------------------------
  $Log:$
  ================================================================== */ 
#include "eep.h"

extern bool ethernet_WIZ550i;
extern uint8_t      localIP[4];  // local IP address
extern unsigned int localPort;   // local port to listen on

void check_and_init_eeprom(void)
{
	uint8_t x;
	
	x = eeprom_read_byte(EEPARB_INIT);
	if (x == NO_INIT)
	{
		eeprom_write_byte(EEPARB_INIT  , 0x01);    // Eeprom init. flag
		eeprom_write_byte(EEPARB_ETHUSB, USE_USB); // Default: Use USB
		eeprom_write_byte(EEPARB_IP0 ,192);
		eeprom_write_byte(EEPARB_IP1 ,168);
		eeprom_write_byte(EEPARB_IP2 ,  1);
		eeprom_write_byte(EEPARB_IP3 ,177);
		eeprom_write_word(EEPARW_PORT,8888);
	} // if
} // check_and_init_eeprom()

void read_eeprom_parameters(void)
{
	if (eeprom_read_byte(EEPARB_ETHUSB) == USE_ETH)
		 ethernet_WIZ550i = true;
	else ethernet_WIZ550i = false;
	localIP[0] = eeprom_read_byte(EEPARB_IP0);	
	localIP[1] = eeprom_read_byte(EEPARB_IP1);
	localIP[2] = eeprom_read_byte(EEPARB_IP2);
	localIP[3] = eeprom_read_byte(EEPARB_IP3);
	localPort  = eeprom_read_word(EEPARW_PORT);
} // read_eeprom_parameters()

void write_eeprom_parameters(void)
{
	if (ethernet_WIZ550i)
	     eeprom_write_byte(EEPARB_ETHUSB, USE_ETH);
	else eeprom_write_byte(EEPARB_ETHUSB, USE_USB);
	eeprom_write_byte(EEPARB_IP0 ,localIP[0]);
	eeprom_write_byte(EEPARB_IP1 ,localIP[1]);
	eeprom_write_byte(EEPARB_IP2 ,localIP[2]);
	eeprom_write_byte(EEPARB_IP3 ,localIP[3]);
	eeprom_write_word(EEPARW_PORT,localPort);
} // write_eeprom_parameters()
