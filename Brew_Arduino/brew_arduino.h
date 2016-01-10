/*==================================================================
  File Name    : $Id$
  Function name: -
  Author       : E. van de Logt
  ------------------------------------------------------------------
  Purpose      : This is the header-file for brew_arduino.h
  Compatibility: R1.7 <-> ebrew R1.65
				 R1.8 <-> ebrew R1.66
  ------------------------------------------------------------------
  $Log$
  Revision 1.11  2015/06/28 12:27:35  Emile
  - Moving_average filters now work with Q8.7 instead of Q8.4 format
  - One-wire functions now work with DS18B20
  - Separate ow_task() added for one-wire communication
  - I2C clock made adjustable

  Revision 1.10  2015/05/31 10:28:57  Emile
  - Bugfix: Flowsensor reading counted rising and falling edges.
  - Bugfix: Only valve V8 is written to at init (instead of all valves).

  Revision 1.9  2014/11/09 15:38:34  Emile
  - PUMP_LED removed from PD2, PUMP_230V has same function
  - Interface for 2nd waterflow sensor added to PD2
  - Command A6 (read waterflow in E-2 L) added
  - FLOW_PER_L changed to 330 (only rising edge is counted)

  Revision 1.8  2014/06/15 14:52:20  Emile
  - Commands E0 and E1 (Disable/Enable Ethernet module) added
  - Interface for waterflow sensor added to PC3/ADC3
  - Command A5 (read waterflow in E-2 L) added

  Revision 1.7  2014/06/01 13:46:02  Emile
  Bug-fix: do not call udp routines when in COM port mode!

  Revision 1.6  2014/05/03 11:27:44  Emile
  - Ethernet support added for W550io module
  - No response for L, N, P, W commands anymore
  - All source files now have headers

  ================================================================== */ 
#ifndef _BREW_ARDUINO_H_
#define _BREW_ARDUINO_H_
//-----------------------------------------------------------------------------
//                       Brew Arduino Pin Mapping Arduino NANO
//
//                                ----ICSP----
// Dig.01 (TX)          (TXD) PD1 [01]    [26] VIN
// Dig.00 (RX)          (RXD) PD0 [02]    [27] GND
//                    (RESET) PC6 [03]    [28] PC6 (RESET) 
//                            GND [04]    [27] VCC
// Dig.02 - - -        (INT0) PD2 [05]    [26] PC2 (ADC7)     - - - analog 7
// Dig.03 - - -        (INT1) PD3 [06]    [25] PC1 (ADC6)     LM35  analog 6
// Dig.04 ALIVE_LED  (XCK/TO) PD4 [07]    [24] PC5 (ADC5/SCL) SCL   analog 5
// Dig.05 - - -          (T1) PD5 [08]    [23] PC4 (ADC4/SDA) SDA   analog 4
// Dig.06 - - -        (AIN0) PD6 [09]    [22] PC3 (ADC3)     FLOW1 analog 3
// Dig.07 SPI_SS       (AIN1) PD7 [10]    [21] PC2 (ADC2)     FLOW2 analog 2
// Dig.08 WIZ550_RESET (ICP1) PB0 [11]    [20] PC1 (ADC1)     FLOW3 analog 1
// Dig.09 HLT_PWM      (OC1A) PB1 [12]    [19] PC0 (ADC0)     FLOW4 analog 0
// Dig.10 BOIL_PWM     (OC1B) PB2 [13]    [18] AREF
// Dig.11 SPI_MOSI (MOSI/OC2) PB3 [14]    [17] 3V3
// Dig.12 SPI_MISO     (MISO) PB4 [15]    [16] PB5 (SCK)      SPI_CLK
//                               -----USB----
//                               Arduino NANO
//-----------------------------------------------------------------------------
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/atomic.h>
#include <string.h>
#include "delay.h"         /* for delay_msec() */
#include "adc.h"
#include "i2c.h"
#include "pwm.h"
#include "spi.h"
#include "usart.h"
#include "command_interpreter.h"
#include "misc.h"
#include "scheduler.h"

// Uncomment the next line if a WIZ550io module is present
//#define WIZ550io_PRESENT

#define RS232_USB    (true)
#define ETHERNET_UDP (false)

//-----------------------------
// PORTB defines
//-----------------------------
#define WIZ550_HW_RESET (0x01)

//-----------------------------
// PORTD defines
//-----------------------------
#define ALIVE_LED  (0x10)

//-----------------------------
// PORTB of MCP23017 defines
//-----------------------------
#define HLT_NMOD   (0x01)
#define HLT_230V   (0x02)
#define BOIL_NMOD  (0x04)
#define BOIL_230V  (0x08)
#define PUMP_230V  (0x10)

//-----------------------------
// E-brew System Mode
//-----------------------------
#define GAS_MODULATING     (0)
#define GAS_NON_MODULATING (1)
#define ELECTRICAL_HEATING (2)

//-----------------------------
// pwm_2_time() States
//-----------------------------
#define IDLE       (0)
#define EL_HTR_OFF (1)
#define EL_HTR_ON  (2)

// INIT_TEMP  20 °C
// INIT_VOL10 80 E-1 L
// LM35_CONV: 11000 E-2 °C / 1023
#define INIT_TEMP  (20)
#define INIT_VOL10 (80)
#define LM35_CONV  (10.75268817)

//---------------------------------------------------
// Q = 5.5 Hz for 1 L, 11 pulses per L per sec.
// Per minute: 5.5 x 1 rising pulse x 60 sec. = 330
// An interrupt is generated for a rising AND
// falling edge, but only the rising edge is counted.
//---------------------------------------------------
#define FLOW_PER_L     (330)
#define FLOW_ROUND_OFF (FLOW_PER_L>>1)

void print_ebrew_revision(char *ver);
void init_WIZ550IO_module(void);

#endif /* _BREW_ARDUINO_H_ */