/*==================================================================
  File Name    : $Id: $
  Function name: -
  Author       : E v.d. Logt
  ------------------------------------------------------------------
  Purpose : EEPROM routines
  ------------------------------------------------------------------
  $Log:$
  ================================================================== */ 
#ifndef EEP_H_
#define EEP_H_
#include <avr\eeprom.h>
#include <stdbool.h>

// EEprom Parameters
#define EEPTR        (void *)
#define EEPTR1       (uint8_t *)  
#define EEPTR2       (uint16_t *)
#define EEPTRF       (float *)

#define EEPARB_INIT   EEPTR1(0x08)
#define EEPARB_ETHUSB EEPTR1(0x0A) /* 0x00 = ETH, 0xff = USB */
#define EEPARB_IP0    EEPTR1(0x0C)
#define EEPARB_IP1    EEPTR1(0x0C)
#define EEPARB_IP2    EEPTR1(0x0E)
#define EEPARB_IP3    EEPTR1(0x10)
#define EEPARW_PORT   EEPTR2(0x12)
#define EEPARB_DEL_START_ENA  EEPTR2(0x14)
#define EEPARB_DEL_START_TMR1 EEPTR2(0x16)
#define EEPARB_DEL_START_TIME EEPTR2(0x18)

#define NO_INIT      (0xff)
#define USE_ETH      (0x00)
#define USE_USB      (0xff)

void check_and_init_eeprom(void);
void read_eeprom_parameters(void);
void write_eeprom_parameters(void);

#endif /* EEP_H_ */