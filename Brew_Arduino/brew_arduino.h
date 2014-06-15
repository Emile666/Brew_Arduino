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
//                         Brew Arduino Pin Mapping ATMEGA328P
//
//                                -----\/-----
//                    (RESET) PC6 [01]    [28] PC5 (ADC5/SCL) SCL  analog 5
// Dig.00 (RX)          (RXD) PD0 [02]    [27] PC4 (ADC4/SDA) SDA  analog 4
// Dig.01 (TX)          (TXD) PD1 [03]    [26] PC3 (ADC3)     ---  analog 3
// Dig.02 PUMP_LED     (INT0) PD2 [04]    [25] PC2 (ADC2)     VMLT analog 2
// Dig.03 HEATER_LED   (INT1) PD3 [05]    [24] PC1 (ADC1)     VHLT analog 1
// Dig.04 ALIVE_LED  (XCK/TO) PD4 [06]    [23] PC0 (ADC0)     LM35 analog 0
//                            VCC [07]    [22] GND
//                            GND [08]    [21] AREF
//              (XTAL1/TOSC1) PB6 [09]    [20] AVCC
//              (XTAL2/TOSC2) PB7 [10]    [19] PB5 (SCK)      SCK  dig.13 (LED)
// Dig.05 NON_MOD        (T1) PD5 [11]    [18] PB4 (MISO)     MISO dig.12
// Dig.06 PUMP         (AIN0) PD6 [12]    [17] PB3 (MOSI/OC2) MOSI dig.11 (PWM)
// Dig.07 HEATER       (AIN1) PD7 [13]    [16] PB2 (SS/OC1B)  SS   dig.10 (PWM)
// Dig.08 WIZ550_RESET (ICP1) PB0 [14]    [15] PB1 (OC1A)     PWM  dig.09 (PWM)
//                               ------------
//                                ATmega328P
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
#define PUMP_LED   (0x04)
#define HEATER_LED (0x08)
#define ALIVE_LED  (0x10)
#define NON_MOD    (0x20)
#define PUMP       (0x40)
#define HEATER     (0x80)

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

#define INIT_TEMP  (20)
#define INIT_VOL    (8)

//---------------------------------------------
// Q = 5.5 Hz for 1 L, 11 pulses per L per sec.
// Per minute: 5.5 x 2 pulses x 60 sec. = 660
//---------------------------------------------
#define FLOW_PER_L  (660)

void print_ebrew_revision(char *ver);
void init_WIZ550IO_module(void);

#endif /* _BREW_ARDUINO_H_ */