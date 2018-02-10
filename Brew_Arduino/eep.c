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

void check_and_init_eeprom(void)
{
	uint8_t x;
	
	x = eeprom_read_byte(EEPARB_INIT);
	if (x == NO_INIT)
	{
		eeprom_write_byte(EEPARB_INIT  , 0x01);    // Eeprom init. flag
		eeprom_write_byte(EEPARB_ETHUSB, USE_USB); // Default: Use USB
	} // if
} // check_and_init_eeprom()

void read_eeprom_parameters(void)
{
	if (eeprom_read_byte(EEPARB_ETHUSB) == USE_ETH)
		 ethernet_WIZ550i = true;
	else ethernet_WIZ550i = false;
} // read_eeprom_parameters()

void write_eeprom_parameters(void)
{
	if (ethernet_WIZ550i)
	     eeprom_write_byte(EEPARB_ETHUSB, USE_ETH);
	else eeprom_write_byte(EEPARB_ETHUSB, USE_USB);
} // write_eeprom_parameters()
