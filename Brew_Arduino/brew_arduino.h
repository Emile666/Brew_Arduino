/*==================================================================
  File Name    : brew_arduino.h
  Author       : Emile
  ------------------------------------------------------------------
  Purpose      : This is the header-file for brew_arduino.h
  ================================================================== */ 
#ifndef _BREW_ARDUINO_H_
#define _BREW_ARDUINO_H_
//-----------------------------------------------------------------------------
//                       Brew Arduino Pin Mapping Arduino NANO
//
//                                ----ICSP----
// Dig.00 (TX)           (TXD) PD0 [01]    [30] VIN
// Dig.01 (RX)           (RXD) PD1 [02]    [29] GND
//                     (RESET) PB6 [03]    [28] PB6 (RESET) 
//                             GND [04]    [27] VCC
// Dig.02 BK_EH1        (INT0) PD2 [05]    [26] PC7 (ADC7)     ---   analog 7
// Dig.03 BUZZER        (INT1) PD3 [06]    [25] PC6 (ADC6)     LM35  analog 6
// Dig.04 ALIVE_LED_B (XCK/TO) PD4 [07]    [24] PC5 (ADC5/SCL) SCL   analog 5
// Dig.05 ALIVE_LED_G     (T1) PD5 [08]    [23] PC4 (ADC4/SDA) SDA   analog 4
// Dig.06 ALIVE_LED_R   (AIN0) PD6 [09]    [22] PC3 (ADC3)     FLOW1 analog 3
// Dig.07 SPI_SS        (AIN1) PD7 [10]    [21] PC2 (ADC2)     FLOW2 analog 2
// Dig.08 WIZ550_RESET  (ICP1) PB0 [11]    [20] PC1 (ADC1)     FLOW3 analog 1
// Dig.09 HLT_PWM       (OC1A) PB1 [12]    [19] PC0 (ADC0)     FLOW4 analog 0
// Dig.10 BOIL_PWM      (OC1B) PB2 [13]    [18] AREF
// Dig.11 SPI_MOSI  (MOSI/OC2) PB3 [14]    [17] 3V3
// Dig.12 SPI_MISO      (MISO) PB4 [15]    [16] PB5 (SCK)      SPI_CLK Dig.13
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
#include "eep.h"

//---------------------------------
// Defines for WIZ550IO ETH module
//---------------------------------
#define RS232_USB     (true)
#define ETHERNET_UDP  (false)
#define EBREW_PORT_NR (8888)

//-----------------------------
// PORTB defines
//-----------------------------
#define WIZ550_HW_RESET (0x01)

//-----------------------------
// PORTD defines
//-----------------------------
#define ALIVE_LED_R  (0x40) /* Delayed-start red LED blinking */
#define BK_EH2_230V  (0x20) /* Slow SSR signal BK Electrical Heater 2 */
#define ALIVE_LED_B  (0x10) /* L0/L1 Alive-LED blue blinking */
#define BUZZER       (0x08)
#define BK_EH1_230V  (0x04) /* Slow SSR signal BK Electrical Heater 1 */

//-----------------------------
// PORTB of MCP23017 defines
//-----------------------------
#define HLT_NMOD     (0x01) /* On/Off non-modulating HLT gas-valve 24Vac */
#define HLT_230V     (0x02) /* On/Off control modulating HLT gas-valve 230V */
#define BOIL_NMOD    (0x04) /* On/Off non-modulating BK gas-valve 24Vac */
#define BOIL_230V    (0x08) /* On/Off control modulating gas-valve 230V */
#define PUMP_230V    (0x10) /* On/Off Main pump Triac/SSR */
#define PUMP2_230V   (0x20) /* On/Off HLT pump Triac/SSR */
#define HLT_EH1_230V (0x40) /* Slow SSR signal HLT Electrical Heater 1 */
#define HLT_EH2_230V (0x80) /* Slow SSR signal HLT Electrical Heater 2 */

#define ON1ST        (true) /* Create PWM signal by starting with 1 signal */
#define OFF1ST      (false) /* Create PWM signal by starting with 0 signal */

//------------------------------------
// Bit defines for elec_htrs variable
//------------------------------------
#define HTR_BK1     (0x01)
#define HTR_BK2     (0x02)
#define HTR_BK3     (0x04)
#define HTR_HLT1    (0x08)
#define HTR_HLT2    (0x10)
#define HTR_HLT3    (0x20)

typedef struct _pwmtime
{
	uint8_t std;      // STD state number
	uint8_t mask;     // port mask of MCP23017 Port B pin
	bool    on1st;    // true = make 1 first, false = make 0 first
} pwmtime;
	
//-------------------------------------------------------------------
// These defines are used by the B and H commands and indicate  
// which energy-sources are used.
// Note: These defines should be the same as in the PC-program!
//-------------------------------------------------------------------
#define GAS_MODU               (0x01) /* Modulating gas-valve */
#define GAS_ONOFF              (0x02) /* Non-modulating gas-valve */
#define ELEC_HTR1              (0x04) /* First electric heating-element */
#define ELEC_HTR2              (0x08) /* Second electric heating-element */
#define ELEC_HTR3              (0x10) /* Not implemented yet */

//-----------------------------
// Buzer STD modes
//-----------------------------
#define BZ_OFF   (0)
#define BZ_ON    (1)
#define BZ_ON2   (2)
#define BZ_BURST (3)
#define BZ_SHORT (4)

//-----------------------------
// Delayed Start STD modes
//-----------------------------
#define DEL_START_INIT   (0)
#define DEL_START_TMR    (1)
#define DEL_START_BURN   (2)
#define DEL_START_MAX_DELAY_TIME (54000) /* Max. time is 30 hours * 60 minutes * 30 * 2 seconds */
#define DEL_START_MAX_BURN_TIME   (3600) /* Max. time is 120 minutes * 30 * 2 seconds */
#define DEL_START_ELEC_PWM          (40) /* PWM signal for HLT electric heaters during delayed start */

//-----------------------------
// pwm_2_time() States
//-----------------------------
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

void    init_pwm_time(pwmtime *p, uint8_t mask, bool on1st);
void    pwm_2_time(pwmtime *p, uint8_t cntr, uint8_t pwm);
void    print_ebrew_revision(char *ver);
uint8_t init_WIZ550IO_module(void);
void    print_IP_address(uint8_t *ip);

#endif /* _BREW_ARDUINO_H_ */