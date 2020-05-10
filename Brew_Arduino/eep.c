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
extern bool delayed_start_enable; // true = delayed start is enabled

void check_and_init_eeprom(void)
{
	uint8_t x;
	
	x = eeprom_read_byte(EEPARB_INIT);
	if (x == NO_INIT)
	{
		eeprom_write_byte(EEPARB_INIT  , 0x01);         // Eeprom init. flag
		eeprom_write_byte(EEPARB_ETHUSB, USE_USB);      // Default: Use USB
		eeprom_write_byte(EEPARB_DEL_START_ENA ,false); // delayed-start enable
		eeprom_write_word(EEPARB_DEL_START_TMR1,0);     // delayed-start timer 1
		eeprom_write_word(EEPARB_DEL_START_TIME,0);     // delayed-start time
	} // if
} // check_and_init_eeprom()

void read_eeprom_parameters(void)
{
	if (eeprom_read_byte(EEPARB_ETHUSB) == USE_ETH)
		 ethernet_WIZ550i = true;
	else ethernet_WIZ550i = false;

	if ((delayed_start_enable = eeprom_read_byte(EEPARB_DEL_START_ENA)) != true)
	{
		eeprom_write_byte(EEPARB_DEL_START_ENA ,false); // could also be 0xff the 1st time
		eeprom_write_word(EEPARB_DEL_START_TMR1,0);     // reset timer1 in eeprom
		eeprom_write_word(EEPARB_DEL_START_TIME,0);     // delayed-start time
	} // if
} // read_eeprom_parameters()

void write_eeprom_parameters(void)
{
	if (ethernet_WIZ550i)
	     eeprom_write_byte(EEPARB_ETHUSB, USE_ETH);
	else eeprom_write_byte(EEPARB_ETHUSB, USE_USB);
} // write_eeprom_parameters()
