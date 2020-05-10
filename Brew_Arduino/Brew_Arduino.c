//-----------------------------------------------------------------------------
// Created: 20-4-2013 22:32:11
// Author : Emile
// File   : Brew_Arduino.c
//-----------------------------------------------------------------------------
// Revision 1.35  2020/05/10 14:38:00  Emile
// - delayed-start function added with eeprom save
// - D0, D1 and D2 commands for delayed-start added
//
// Revision 1.34  2018/10/25 11:12:00  Emile
// - Bugfix MCP23017 addressing, IOCON definition was for BANK==1. Now all
//          definitions are for BANK==0, which is the default at power-up.
// - Buzzer alarm added to PD3 + Xx command added
//
// Revision 1.33  2018/03/11 15:40:00  Emile
// - Bugfix: for some reason, the MCP23017_init() routine needs to be called twice.
//           With Ethernet operations, there's no reset anymore from the USB, and
//           setting IO on the MCP23017 did not work. Corrected by calling the
//           MCP23017_init() twice. Not nice, but it works.
//
// Revision 1.32  2018/02/18 15:55:00  Emile
// - Better debugging info for ETH start, DHCP timeouts larger, E2 command added.
// - Bug-fix Ethernet_begin(): SHAR register should be used for reading MAC address.
// - R0 (reset flows) command added
//
// Revision 1.31  2018/02/10 16:24:12  Emile
// - LocalPort and LocalIP no longer stored in EEPROM: LocalIP is init by DHCP
//   and LocalPort should always be set to 8888.
//
// Revision 1.30  2016/10/30 11:01:21  Emile
// - More relaxed I2C addressing per channel. Any LM92 address is now accepted
//   for HLT and MLT temperature sensor.
//
// Revision 1.29  2016/08/07 13:46:51  Emile
// - Pump 2 (HLT heat-exchanger pump) is now supported with Px command.
//
// Revision 1.28  2016/06/11 16:50:07  Emile
// - I2C_start() performance improved, one-wire duration from 22 -> 6 msec.
// - Network communication now works using DHCP
//
// Revision 1.28  2016/05/22 12:14:58  Emile
// - Baud-rate to 38400 Baud.
// - Temperature error value now set to '-99.99'.
//
// Revision 1.27  2016/05/15 12:24:20  Emile
// - I2C clock speed now adjustable
// - IP address and port now stored in eeprom
//
// Revision 1.26  2016/04/16 19:39:04  Emile
// - Bugfix reading flowsensor values
// - Bugfix Tcfc and Tboil reading tasks.
//
// Revision 1.25  2016/04/16 11:22:58  Emile
// - One temperature slope parameter for all temps. Now fixed value (2 degrees/second).
// - Temp. Offset parameters removed.
// - All parameters > 12 removed from parameter list. Now only pars 1-6 left.
// - Tasks for CFC and Boil Temperature added.
//
// Revision 1.24  2016/04/01 14:06:08  Emile
// - Bugfix PWM generation. Now both PWM signals work with PCB v3.30.
//
// Revision 1.23  2016/01/10 16:00:23  Emile
// First version (untested!) for new HW PCB 3.30 with 4 x temperature, 4 x flowsensor and 2 PWM outputs.
// - Added: owb_task(), owc_task(), tcfc_ and tboil_ variables. Removed: vhlt_ and vhlt_ variables.
// - A5..A8 commands added (flowsensors), A1..A4 commands re-arranged.
// - Wxxx command is now Hxxx command, new Bxxx command added.
// - pwm_write() and pwm_2_time() now for 2 channels (HLT and Boil): OCR1A and OCR1B timers used.
// - SPI_SS now from PB2 to PD7 (OC1B/PB2 used for 2nd PWM signal). PWM freq. now set to 25 kHz.
// - PCINT1_vect now works with 4 flowsensors: flow_cfc_out and flow4 added.
// - MCP23017 instead of MCP23008: PORTB used for HLT_NMOD, HLT_230V, BOIL_NMOD, BOIL_230V and PUMP_230V.
// - set_parameter(): parameters 7-12 removed.
//
// Revision 1.22  2015/08/07 13:32:04  Emile
// - OW_task() split into owh_task() and owm_task(). Blocking time now reduced
//   from 34 msec. to 24 msec.
//
// Revision 1.21  2015/08/06 14:41:16  Emile
// - Adapted for MCP23008 instead of MCP23017.
//
// Revision 1.20  2015/07/01 21:03:46  Emile
// - Bug-fix in scheduler time-measurement. Now reads proper time in msec
// - Usart comm. now IRQ driven, so that all receiving commands are handled
// - DS18B20 reads only 2 bytes (instead of 9). Total time taken is now 28 msec.
//   This was 60 msec. and caused multiple reads at PC side.
//
// Revision 1.19  2015/06/28 12:27:35  Emile
// - Moving_average filters now work with Q8.7 instead of Q8.4 format
// - One-wire functions now work with DS18B20
// - Separate ow_task() added for one-wire communication
// - I2C clock made adjustable
//
// Revision 1.18  2015/06/05 13:51:04  Emile
// - Headers added to one_wire sources
//
// Revision 1.17  2015/05/31 10:28:57  Emile
// - Bugfix: Flowsensor reading counted rising and falling edges.
// - Bugfix: Only valve V8 is written to at init (instead of all valves).
//
// Revision 1.16  2015/05/12 14:18:37  Emile
// - HW-bugfix: MCP23017 output latches needs to be written first with 0xFF.
//
// Revision 1.15  2015/05/09 14:37:37  Emile
// - I2C Channel & HW-address update for MCP23017 to reflect changes in HW PCB V3.01
//
// Revision 1.14  2015/05/09 13:36:54  Emile
// - MCP23017 Port B now always input. Port A Pull-up resistors enabled.
// - Bug-fix for HW PCB V3.01
//
// Revision 1.13  2014/11/30 20:44:45  Emile
// - Vxxx command added to write valve output bits
// - mcp23017 (16 bit I2C IO-expander) routines + defines added
//
// Revision 1.12  2014/11/09 15:38:34  Emile
// - PUMP_LED removed from PD2, PUMP_230V has same function
// - Interface for 2nd waterflow sensor added to PD2
// - Command A6 (read waterflow in E-2 L) added
// - FLOW_PER_L changed to 330 (only rising edge is counted)
//
// Revision 1.11  2014/10/26 12:44:47  Emile
// - A3 (Thlt) and A4 (Tmlt) commands now return '99.99' in case of I2C HW error.
//
// Revision 1.10  2014/06/15 14:52:19  Emile
// - Commands E0 and E1 (Disable/Enable Ethernet module) added
// - Interface for waterflow sensor added to PC3/ADC3
// - Command A5 (read waterflow in E-2 L) added
//
// Revision 1.9  2014/06/01 13:51:49  Emile
// - Update CVS revision number
//
// Revision 1.8  2014/05/03 11:27:43  Emile
// - Ethernet support added for W550io module
// - No response for L, N, P, W commands anymore
// - All source files now have headers
//
// Revision 1.7  2013/07/24 13:46:39  Emile
// - Minor changes in S1, S2 and S3 commands to minimize comm. overhead.
// - Version ready for Integration Testing with PC program!
//
// Revision 1.6  2013/07/23 19:33:17  Emile
// - Bug-fix slope-limiter function. Tested on all measurements.
//
// Revision 1.5  2013/07/21 13:10:43  Emile
// - Reading & Writing of 17 parameters now fully works with set_parameter()
// - VHLT and VMLT tasks added
// - Scheduler: actual & max. times now printed in msec. instead of usec.
// - THLT and TMLT now in E-2 Celsius for PC program
// - All lm92 test routines removed, only one lm92_read() remaining
//
// Revision 1.4  2013/07/20 14:51:59  Emile
// - LM35, THLT and TMLT tasks are now working
// - Max. duration added to scheduler
// - slope_limiter & lm92_read() now work with uint16_t instead of float
//
// Revision 1.3  2013/07/19 10:51:01  Emile
// - I2C frequency 50 50 kHz to get 2nd LM92 working
// - Command Mx removed, command N0 x added, commands N0..N3 renamed to N1..N4
// - Command S3 added, list_all_tasks. To-Do: get timing-measurement working
// - Scheduler added with 3 tasks: lm35, led_blink and pwm_2_time
//
// Revision 1.2  2013/06/23 08:56:03  Emile
// - Working version (with PC program) with new command-set.
//-----------------------------------------------------------------------------
#include "brew_arduino.h"
#include "w5500.h"
#include "Ethernet.h"
#include "Udp.h"
#include "one_wire.h"

extern char rs232_inbuf[];

// Global variables
uint8_t      local_ip[4]      = {0,0,0,0}; // local IP address, gets a value from init_WIZ550IO_module() -> dhcp_begin()
unsigned int local_port;                   // local port number read back from wiz550i module
const char  *ebrew_revision   = "$Revision: 1.35 $";   // ebrew CVS revision number
uint8_t      system_mode      = GAS_MODULATING; // Default to Modulating Gas-valve
bool         ethernet_WIZ550i = false;		    // Default to No WIZ550i present

// The following variables are defined in Udp.c
extern uint8_t  remoteIP[4]; // remote IP address for the incoming packet whilst it's being processed 
extern uint16_t remotePort;  // remote port for the incoming packet whilst it's being processed 
extern uint8_t  _sock;       // socket ID for Wiz5100
unsigned char   udp_rcv_buf[UDP_TX_PACKET_MAX_SIZE]; //buffer to hold incoming packet]

//---------------------------------------------------------------------------------
// system_mode == GAS_NON_MODULATING: a hysteresis block is used to determine when
//                                    the RELAY (that controls the 24 Vac for the 
//                                    non-modulating gas-burner) is switched on.
//                                    This happens when PID-Output > Hysteresis.
//---------------------------------------------------------------------------------
uint8_t  gas_non_mod_llimit = 30; // Hysteresis lower-limit parameter
uint8_t  gas_non_mod_hlimit = 35; // Hysteresis upper-limit parameter
//---------------------------------------------------------------------------------
// system_mode == GAS_MODULATING:    a hysteresis block is used to determine when
//                                   the HLT_230V LED and HLT_230V TRIAC is switched on.
//                                   The TRIAC is needed to energize the gas-valve.
//---------------------------------------------------------------------------------
uint8_t  gas_mod_pwm_llimit = 2;  // Modulating gas-valve Hysteresis lower-limit parameter
uint8_t  gas_mod_pwm_hlimit = 4;  // Modulating gas-valve Hysteresis upper-limit parameter

uint8_t    btmr_on_val  = 0;        // ON-timer  for PWM to Time-Division Boil-kettle
uint8_t    btmr_off_val = 0;        // OFF-timer for PWM to Time-Division Boil-kettle
uint8_t    htmr_on_val  = 0;        // ON-timer  for PWM to Time-Division HLT
uint8_t    htmr_off_val = 0;        // OFF-timer for PWM to Time-Division HLT

//-----------------------------------
// LM35 parameters and variables
//-----------------------------------
ma       lm35_ma;                   // struct for LM35 moving_average filter
uint16_t lm35_temp;                 // LM35 Temperature in E-2 °C
uint16_t triac_llimit  = 6500;      // Hysteresis lower-limit for triac_too_hot in E-2 °C
uint16_t triac_hlimit  = 7500;	    // Hysteresis upper-limit for triac_too_hot in E-2 °C
uint8_t  triac_too_hot = FALSE;     // 1 = TRIAC temperature (read by LM35) is too high

//------------------------------------------------
// THLT parameters and variables (I2C or One-Wire)
//------------------------------------------------
int16_t  temp_slope_87  = 512;      // Temperature slope-limiter is 4 °C/ 2 sec. * 128
ma       thlt_ma;                   // struct for THLT moving_average filter
int16_t  thlt_old_87;               // Previous value of thlt_temp_87
int16_t  thlt_temp_87;              // THLT Temperature from LM92 in °C * 128
int16_t  thlt_ow_87;                // THLT Temperature from DS18B20 in °C * 128
uint8_t  thlt_err       = 0;        // 1 = Read error from LM92
uint8_t  thlt_ow_err    = 0;	    // 1 = Read error from DS18B20
 
//------------------------------------------------
// TMLT parameters and variables (I2C or One-Wire)
//------------------------------------------------
ma       tmlt_ma;                   // struct for TMLT moving_average filter
int16_t  tmlt_old_87;               // Previous value of tmlt_temp_87
int16_t  tmlt_temp_87;              // TMLT Temperature from LM92 in °C * 128
int16_t  tmlt_ow_87;                // TMLT Temperature from DS18B20 in °C * 128
uint8_t  tmlt_err       = 0;        // 1 = Read error from LM92
uint8_t  tmlt_ow_err    = 0;	    // 1 = Read error from DS18B20

//------------------------------------------------
// TCFC parameters and variables (One-Wire only)
//------------------------------------------------
ma       tcfc_ma;                   // struct for TCFC moving_average filter
int16_t  tcfc_old_87;               // Previous value of tcfc_temp_87
int16_t  tcfc_temp_87;              // TCFC Temperature in °C * 128
uint8_t  tcfc_err       = 0;        // 1 = Read error from DS18B20  

//------------------------------------------------
// TBOIL parameters and variables (One-Wire only)
//------------------------------------------------
ma       tboil_ma;                  // struct for TBOIL moving_average filter
int16_t  tboil_old_87;              // Previous value of tboil_temp_87
int16_t  tboil_temp_87;             // TBOIL Temperature in °C * 128
uint8_t  tboil_err       = 0;       // 1 = Read error from DS18B20

unsigned long    t2_millis     = 0UL;
unsigned long    flow_hlt_mlt  = 0UL;
unsigned long    flow_mlt_boil = 0UL;
unsigned long    flow_cfc_out  = 0UL; // Count from flow-sensor at output of CFC
unsigned long    flow4         = 0UL; // Count from FLOW4 (future use)
volatile uint8_t old_flows     = 0x0F; // default is high because of the pull-up

//------------------------------------------------
// Buzzer variables
//------------------------------------------------
bool     bz_on = false;    // true = buzzer is enabled
bool     bz_dbl;           // true = generate 2nd beep
uint8_t  bz_std = BZ_OFF;  // std number
uint8_t  bz_rpt;           // buzzer repeat counter
uint8_t  bz_rpt_max;       // number of beeps to make
uint16_t bz_tmr;           // buzzer msec counter

//------------------------------------------------
// Delayed-start variables
//------------------------------------------------
bool     delayed_start_enable  = false;          // true = delayed start is enabled
uint16_t delayed_start_time    = 0;              // delayed start time in 2 sec. counts 
uint16_t delayed_start_timer1;                   // timer to countdown until delayed start
uint8_t  delayed_start_std     = DEL_START_INIT; // std number, start in INIT state

/*------------------------------------------------------------------
  Purpose  : This is the delayed-start routine which is called by 
             lm35_task() every 2 seconds. 
  Variables: -
  Returns  : -
  ------------------------------------------------------------------*/
void process_delayed_start(void)
{
	uint8_t         pwm         = 0; // duty-cycle for HLT burner
	uint8_t         led_tmr_max = 0; // blink every x seconds  
    static uint8_t  led_tmr     = 0; // Timer for blinking of RED led
	static uint16_t eep_tmr;         // minute counter for eeprom write
	static uint16_t timer2;          // timer for max. burn in minutes (max. 120 minutes)
	
	switch (delayed_start_std)
	{
		case DEL_START_INIT: // Delayed-start is not enabled
		         if (delayed_start_enable)
				 {
				    delayed_start_timer1 = eeprom_read_word(EEPARB_DEL_START_TMR1); // read value from eeprom
				    delayed_start_time   = eeprom_read_word(EEPARB_DEL_START_TIME); // read value from eeprom
					eep_tmr              = 0;                      // reset eeprom minute timer
					delayed_start_std    = DEL_START_TMR;          // start countdown timer
				 } // if				 
				 led_tmr_max = 0; // no blinking
			     break;

		case DEL_START_TMR: // The delayed-start timer is running, no time-out yet
			     if (!delayed_start_enable)
				 {
				     eeprom_write_byte(EEPARB_DEL_START_ENA,false); // reset enable in eeprom
				     delayed_start_std = DEL_START_INIT;
				 } // if				 
				 else if (++delayed_start_timer1 >= delayed_start_time)
				 {   // delayed-start counted down
					 timer2            = 0;              // init. burn-timer 
					 delayed_start_std = DEL_START_BURN; // start HLT burner
				 } // if
				 if (++eep_tmr >= 30)
				 {   // save in eeprom every minute
					 eep_tmr = 0;
					 eeprom_write_word(EEPARB_DEL_START_TMR1,delayed_start_timer1);
				 } // if
				 led_tmr_max = 5; // blink once every 10 seconds
				 break;

		case DEL_START_BURN: // Delayed-start time-out, HLT-burner is burning
			     if (!delayed_start_enable)
				 {
				     eeprom_write_byte(EEPARB_DEL_START_ENA,false); // reset enable in eeprom
					 delayed_start_std = DEL_START_INIT;
				 } // if
				 else 
				 {		 
					pwm = 80; // HLT burner with fixed percentage of 80 %
				    if (++timer2 >= DEL_START_MAX_BURN_TIME) 
					{   // Safety feature, set burn-time to max. of 2 hours
						delayed_start_enable = false; // prevent another burn
						delayed_start_std    = DEL_START_INIT;
						eeprom_write_byte(EEPARB_DEL_START_ENA,false); // reset enable in eeprom
					} // if
				 } // else				 
				 led_tmr_max = 1; // blink once every 4 seconds
				 break;

		default: delayed_start_std = DEL_START_INIT;
		         break;
	} // switch
	if ((led_tmr_max > 0) && (++led_tmr > led_tmr_max))
	{    // blink once every led_tmr_max seconds
		 led_tmr = 0;
		 PORTD |= ALIVE_LED_R;
	} // if
	else PORTD &= ~ALIVE_LED_R;
	process_pwm_signal(PWMH,pwm); // Enable HLT burner with fixed percentage
} // process_delayed_start()

/*------------------------------------------------------------------
  Purpose  : This is the buzzer routine which runs every msec. 
             (f=1 kHz). It is used by 1 kHz interrupt routine.
  Variables: -
  Returns  : -
  ------------------------------------------------------------------*/
void buzzer(void)
{
	switch (bz_std)
	{
		case BZ_OFF:   PORTD &= ~BUZZER;
					   if (bz_on) 
					   {
						   bz_tmr = 0;
						   bz_rpt = 0;
						   bz_dbl = false;
						   bz_std = BZ_ON;
					   } // if
					   break;
		case BZ_ON:    PORTD |= BUZZER;
					   if (++bz_tmr > 50) 
					   {
						   bz_tmr = 0;
						   if (!bz_dbl)
						        bz_std = BZ_SHORT;
						   else bz_std = BZ_BURST;
					   } // if
					   else bz_std = BZ_ON2;					 
					   break;
		case BZ_ON2:   PORTD &= ~BUZZER;
					   bz_std = BZ_ON;
					   break;
		case BZ_SHORT: PORTD &= ~BUZZER;
					   bz_dbl = true;
					   if (++bz_tmr >= 50)
					   {
						   bz_tmr = 0;
						   bz_std = BZ_ON;
					   } // if
					   break;		
		case BZ_BURST: PORTD &= ~BUZZER;
					   if (++bz_tmr > 500)
					   {
						 bz_tmr = 0;  
						 bz_dbl = false;
						 if (++bz_rpt >= bz_rpt_max) 
						 {
							  bz_on  = false;
							  bz_std = BZ_OFF;
						 } // if
						 else bz_std = BZ_ON;
					   } // if
					   break;					   						 
	} // switch
} // buzzer()

/*------------------------------------------------------------------
  Purpose  : This is the Timer-Interrupt routine which runs every msec. 
             (f=1 kHz). It is used by the task-scheduler.
  Variables: -
  Returns  : -
  ------------------------------------------------------------------*/
ISR(TIMER2_COMPA_vect)
{
	t2_millis++;     // update millisecond counter
	buzzer();        // sound alarm through buzzer
	scheduler_isr(); // call the ISR routine for the task-scheduler
} // ISR()

/*------------------------------------------------------------------
  Purpose  : This is the State-change interrupt routine for PORTC. 
             It is used to count pulses from FLOW1 (PC3), FLOW2 (PC2) 
			 FLOW3 (PC1) and FLOW4 (PC0).
  Variables: -
  Returns  : -
  ------------------------------------------------------------------*/
ISR (PCINT1_vect)
{
	uint8_t temp,flows;
	
	temp      = PINC & 0x0F;       // Read only Flow-sensor port-pins
	flows     = temp & ~old_flows; // Detect rising edge for all flows
	old_flows = temp;              // Save flow-sensor port-pins
	if (flows & 0x08)
	{   // Rising edge on FLOW1
		flow_hlt_mlt++;  /* PC3/PCINT11 rising edge */
	} // if
	if (flows & 0x04)
	{   // Rising edge on FLOW2
		flow_mlt_boil++; /* PC2/PCINT10 rising edge */
	} // if
	if (flows & 0x02)
	{   // Rising edge on FLOW3
		flow_cfc_out++; /* PC1/PCINT9  rising edge */
	} // if
	if (flows & 0x01)
	{   // Rising edge on FLOW4
		flow4++;       /* PC0/PCINT8  rising edge */
	} // if
} // ISR(PCINT1_vect)

/*------------------------------------------------------------------
  Purpose  : This function returns the number of milliseconds since
			 power-up. It is defined in delay.h
  Variables: -
  Returns  : The number of milliseconds since power-up
  ------------------------------------------------------------------*/
unsigned long millis(void)
{
	unsigned long m;
	// disable interrupts while we read timer0_millis or we might get an
	// inconsistent value (e.g. in the middle of a write to timer0_millis)
	ATOMIC_BLOCK(ATOMIC_FORCEON)
	{
		m = t2_millis;
	}
	return m;
} // millis()

/*------------------------------------------------------------------
  Purpose  : This function waits a number of milliseconds. It is 
             defined in delay.h. Do NOT use this in an interrupt.
  Variables: 
         ms: The number of milliseconds to wait.
  Returns  : -
  ------------------------------------------------------------------*/
void delay_msec(uint16_t ms)
{
	unsigned long start = millis();

	while ((millis() - start) < ms) ;
} // delay_msec()

/*-----------------------------------------------------------------------------
  Purpose  : Converts a PWM signal into a time-division signal of 100 * 50 msec.
             This routine should be called from the timer-interrupt every 50 msec.
			 It uses the tmr_on_val and tmr_off_val values which were set by the
			 process_pwm_signal. If Electrical Heating (with a heating element)
			 is NOT enabled, this routine returns immediately without doing 
			 anything. Called by pwm_task().
  Variables: std_td     : the STD state number
			 htimer     : the ON-timer
			 ltimer     : the OFF-timer
             tmr_on_val : the ON-time value
             tmr_off_val: the OFF-time value
  Returns  : -
  ---------------------------------------------------------------------------*/
void pwm_2_time(uint8_t *std_td, uint8_t *htimer, uint8_t *ltimer, uint8_t tmr_on_val, uint8_t tmr_off_val, uint8_t mask)
{
	uint8_t portb = mcp23017_read(GPIOB);

	switch (*std_td)
	{
		case IDLE:
			//-------------------------------------------------------------
			// This is the default state after power-up. Main function
			// of this state is to initialize the electrical heater states.
			//-------------------------------------------------------------
			portb  &= ~mask;       // set electrical heater OFF
			*ltimer = tmr_off_val; // init. low-timer
			*std_td = EL_HTR_OFF;  // goto OFF-state
			break;

		case EL_HTR_OFF:
			//---------------------------------------------------------
			// Goto ON-state if timeout_ltimer && !triac_too_hot &&
			//      !0%_gamma
			//---------------------------------------------------------
			if (*ltimer == 0)
			{  // OFF-timer has counted down
				if (tmr_on_val == 0)
				{   // indication for 0%, remain in this state
					*ltimer = tmr_off_val; // init. timer again
					portb  &= ~mask;       // set electrical heater OFF
				} // if
				else if (!triac_too_hot)
				{
					portb  |= mask;       // set electrical heater ON
					*htimer = tmr_on_val; // init. high-timer
					*std_td = EL_HTR_ON;  // go to ON-state
				} // else if
				// Remain in this state if timeout && !0% && triac_too_hot
			} // if
			else
			{   // timer has not counted down yet, continue
				portb &= ~mask;  // set electrical heater OFF
				*ltimer--;       // decrement low-timer
			} // else
			break;

		case EL_HTR_ON:
			//---------------------------------------------------------
			// Goto OFF-state if triac_too_hot OR 
			//      (timeout_htimer && !100%_gamma)
			//---------------------------------------------------------
			if (triac_too_hot)
			{
				portb  &= ~mask;       // set electrical heater OFF
				*ltimer = tmr_off_val; // init. low-timer
				*std_td = EL_HTR_OFF;  // go to 'OFF' state
			} // if
			else if (*htimer == 0)
			{   // timer has counted down
				if (tmr_off_val == 0)
				{  // indication for 100%, remain in this state
					*htimer = tmr_on_val; // init. high-timer again
					portb  |= mask;       // set electrical heater ON
				} // if
				else
				{   // tmr_off_val > 0
					portb  &= ~mask;       // set electrical heater OFF
					*ltimer = tmr_off_val; // init. low-timer
					*std_td = EL_HTR_OFF;  // go to 'OFF' state
				} // else
			} // if
			else
			{  // timer has not counted down yet, continue
				portb  |= mask; // set electrical heater ON
				*htimer--;      // decrement high-timer
			} // else
			break;

		default: break;	
	} // switch 
	mcp23017_write(GPIOB,portb); // write updated bit-values back to MCP23017 PORTB
} // pwm_2_time()

/*-----------------------------------------------------------------------------
  Purpose  : This task converts a PWM signal into a time-division signal of 
             100 * 50 msec. This routine should be called from the timer-interrupt 
			 every 50 msec. It uses the tmr_on_val and tmr_off_val values which 
			 were set by the process_pwm_signal and is only executed when
			 Electrical Heating (with a heating element) is enabled.
  Variables: - 
  Returns  : -
  ---------------------------------------------------------------------------*/
void pwm_task(void)
{
   static uint8_t stdb  = IDLE; // STD number for Boil
   static uint8_t stdh  = IDLE; // STD number for HLT
   static uint8_t htmrb = 0;    // The ON-timer for Boil
   static uint8_t ltmrb = 0;    // The OFF-timer for Boil
   static uint8_t htmrh = 0;    // The ON-timer for HLT
   static uint8_t ltmrh = 0;    // The OFF-timer for HLT
	
   if (system_mode == ELECTRICAL_HEATING)
   {    // only run STD when ELECTRICAL HEATING is enabled
		pwm_2_time(&stdb, &htmrb, &ltmrb, btmr_on_val, btmr_off_val, BOIL_230V);	
		pwm_2_time(&stdh, &htmrh, &ltmrh, htmr_on_val, htmr_off_val, HLT_230V);
   } // if
} // pwm_task()

/*--------------------------------------------------------------------
  Purpose  : This is the task that processes the temperature from
             the LM35 temperature sensor. It is called once every 2 seconds.
			 This sensor is connected to ADC0. The LM35 outputs 10 mV/°C,
			 VREF=1.1 V, therefore => Max. Temp. = 11000 E-2 °C
			 Conversion constant = 11000 / 1023 = 10.7526882

			 It also executes the delayed-start function, which will fire the
			 HLT burner after a number of minutes set by the D0 command.

  Variables: lm35_temp    : contains temperature in E-2 °C
			 lm35_frac    : contains fractional temperature in E-2 °C
			 lm35_ma      : moving-average filter struct for lm35_temp
			 triac_llimit : lower limit (hysteresis) for triac_too_hot
			 triac_hlimit : upper limit (hysteresis) for triac_too_hot
             triac_too_hot: is set/reset when the temp. is too high
  Returns  : -
  --------------------------------------------------------------------*/
void lm35_task(void)
{
    float tmp; // temporary variable
	
	tmp       = adc_read(LM35) * LM35_CONV;
	lm35_temp = (uint16_t)moving_average(&lm35_ma, tmp);
	if (triac_too_hot)
	{  // reset hysteresis if temp < lower limit
		triac_too_hot = (lm35_temp >= triac_llimit);	
	} // if
	else 
	{  // set hysteresis if temp > upper limit
		triac_too_hot = (lm35_temp > triac_hlimit);
	} // else
    process_delayed_start(); // process delayed-start function
} // lm35_task()

/*--------------------------------------------------------------------
  Purpose  : This is the task that processes the temperature from
             the Hot-Liquid Tun (HLT). The first sensor to read is the
			 LM92 (I2C sensor). If that one is not present, the DS18B20
			 (One-Wire sensor) is tried. Since both sensors have 4
			 fractional bits (1/2, 1/4, 1/8, 1/16), a signed Q8.4 format
			 would be sufficient. However all variables are stored in a
			 Q8.7 format for accuracy reasons when filtering. All variables 
			 with this format have the extension _87.
			 The HLT temperature is both filtered and slope-limited.
			 This task is called every 2 seconds and uses the output from
			 ow_task(), which delivers a new DS18B20 reading every 2 seconds.
  Variables: thlt_old_87   : previous value of thlt_temp_87
			 temp_slope_87 : the slope-limit: 2 °C/sec.
			 thlt_err      : 1 = error reading from LM92 (I2C)
			 thlt_ow_err   : 1 = error reading from DS18B20 (One-Wire)
			 thlt_ow_87    : Temperature value from DS18B20 (One-Wire)
			 thlt_ma       : the moving-average filter struct for THLT
			 thlt_temp_87  : the processed HLT temperature in °C
  Returns  : -
  --------------------------------------------------------------------*/
void thlt_task(void)
{
	int16_t  tmp; // temporary variable (signed Q8.7 format)
	
	thlt_old_87 = thlt_temp_87; // copy previous value of thlt_temp
	tmp         = lm92_read(THLT, &thlt_err); // returns a signed Q8.7 format
	if (thlt_err && !thlt_ow_err)
	{   // No I2C sensor (LM92) found, one-wire sensor is present
		thlt_err = FALSE; // reset error
		tmp      = thlt_ow_87;
	} // if
	if (!thlt_err) 
	{	// filter if either I2C or One-Wire has been read successfully
		slope_limiter(temp_slope_87, thlt_old_87, &tmp);
		thlt_temp_87 = (int16_t)(moving_average(&thlt_ma, (float)tmp) + 0.5);
	} // if
} // thlt_task()

/*--------------------------------------------------------------------
  Purpose  : This is the task that processes the temperature from
			 the Mash-Lauter Tun (MLT). The first sensor to read is the
		     LM92 (I2C sensor). If that one is not present, the DS18B20
		     (One-Wire sensor) is tried. Since both sensors have 4
			 fractional bits (1/2, 1/4, 1/8, 1/16), a signed Q8.4 format
			 would be sufficient. However all variables are stored in a
			 Q8.7 format for accuracy reasons when filtering. All variables
			 with this format have the extension _87.
			 The MLT temperature is both filtered and slope-limited.
			 This task is called every 2 seconds and uses the output from
			 ow_task(), which delivers a new DS18B20 reading every 2 seconds.
  Variables: tmlt_old_87   : previous value of tmlt_temp_87
			 temp_slope_87 : the slope-limit: 2 °C/sec.
			 tmlt_err      : 1 = error reading from LM92 (I2C)
			 tmlt_ow_err   : 1 = error reading from DS18B20 (One-Wire)
			 tmlt_ow_87    : Temperature value from DS18B20 (One-Wire)
			 tmlt_ma       : the moving-average filter struct for TMLT
			 tmlt_temp_87  : the processed MLT temperature in °C
  Returns  : -
  --------------------------------------------------------------------*/
void tmlt_task(void)
{
	int16_t tmp; // temporary variable (signed Q8.7 format)
	
	tmlt_old_87 = tmlt_temp_87; // copy previous value of tmlt_temp
	tmp         = lm92_read(TMLT, &tmlt_err); // returns a signed Q8.7 format
	if (tmlt_err && !tmlt_ow_err)
	{   // No I2C sensor (LM92) found, one-wire sensor is ok
		tmlt_err = FALSE; // reset error
		tmp      = tmlt_ow_87;
	} // if
	if (!tmlt_err)
	{	// filter if either I2C or One-Wire has been read successfully
		slope_limiter(temp_slope_87, tmlt_old_87, &tmp);
		tmlt_temp_87 = (int16_t)(moving_average(&tmlt_ma, (float)tmp) + 0.5);
	} // else
} // tmlt_task()

/*--------------------------------------------------------------------
  Purpose  : This is the task that processes the temperatures from
			 the HLT One-Wire sensor. The sensor has its
			 own DS2482 I2C-to-One-Wire bridge. Since this sensor has 4
			 fractional bits (1/2, 1/4, 1/8, 1/16), a signed Q8.4 format
			 would be sufficient. However all variables are stored in a
			 Q8.7 format for accuracy reasons when filtering. All variables
			 with this format have the extension _87.
			 This task is called every second so that every 2 seconds a new
			 temperature is present.
  Variables: thlt_ow_87 : HLT temperature read from sensor in Q8.7 format
			 thlt_ow_err: 1=error
  Returns  : -
  --------------------------------------------------------------------*/
void owh_task(void)
{
	static int owh_std = 0; // internal state
	
	switch (owh_std)
	{   
		case 0: // Start Conversion
				ds18b20_start_conversion(DS2482_THLT_BASE);
				owh_std = 1;
				break;
		case 1: // Read Thlt device
			    thlt_ow_87 = ds18b20_read(DS2482_THLT_BASE, &thlt_ow_err,1);
				owh_std = 0;
				break;
	} // switch
} // owh_task()

/*--------------------------------------------------------------------
  Purpose  : This is the task that processes the temperatures from
			 the MLT One-Wire sensor. The sensor has its
			 own DS2482 I2C-to-One-Wire bridge. Since this sensor has 4
			 fractional bits (1/2, 1/4, 1/8, 1/16), a signed Q8.4 format
			 would be sufficient. However all variables are stored in a
			 Q8.7 format for accuracy reasons when filtering. All variables
			 with this format have the extension _87.
			 This task is called every second so that every 2 seconds a new
			 temperature is present.
  Variables: tmlt_ow_87 : MLT temperature read from sensor in Q8.7 format
			 tmlt_ow_err: 1=error
  Returns  : -
  --------------------------------------------------------------------*/
void owm_task(void)
{
	static int owm_std = 0; // internal state
	
	switch (owm_std)
	{   
		case 0: // Start conversion
				ds18b20_start_conversion(DS2482_TMLT_BASE);
				owm_std = 1;
				break;
		case 1: // Read Tmlt device
			    tmlt_ow_87 = ds18b20_read(DS2482_TMLT_BASE, &tmlt_ow_err,1);
				owm_std = 0;
				break;
	} // switch
} // owm_task()

/*--------------------------------------------------------------------
  Purpose  : This is the task that processes the temperatures from
			 the Boil-kettle One-Wire sensor. The sensor has its
			 own DS2482 I2C-to-One-Wire bridge. Since this sensor has 4
			 fractional bits (1/2, 1/4, 1/8, 1/16), a signed Q8.4 format
			 would be sufficient. However all variables are stored in a
			 Q8.7 format for accuracy reasons when filtering. All variables
			 with this format have the extension _87.
			 This task is called every second so that every 2 seconds a new
			 temperature is present.
  Variables: tboil_temp_87 : Boil-kettle temperature read from sensor in Q8.7 format
			 tboil_err     : 1=error
  Returns  : -
  --------------------------------------------------------------------*/
void owb_task(void)
{
	static int owb_std = 0; // internal state
	int16_t    tmp;         // temporary variable (signed Q8.7 format)
	
	switch (owb_std)
	{   
		case 0: // Start conversion
				ds18b20_start_conversion(DS2482_TBOIL_BASE);
				owb_std = 1;
				break;
		case 1: // Read Tboil device
		        tboil_old_87 = tboil_temp_87; // copy previous value of tboil_temp
			    tmp = ds18b20_read(DS2482_TBOIL_BASE, &tboil_err,1);
				if (!tboil_err)
				{
					slope_limiter(temp_slope_87, tboil_old_87, &tmp);
					tboil_temp_87 = (int16_t)(moving_average(&tboil_ma, (float)tmp) + 0.5);
				} // if
				owb_std = 0;
				break;
	} // switch
} // owb_task()

/*--------------------------------------------------------------------
  Purpose  : This is the task that processes the temperatures from
			 the Counterflow Chiller (CFC) One-Wire sensor. The sensor has
			 its own DS2482 I2C-to-One-Wire bridge. Since this sensor has 4
			 fractional bits (1/2, 1/4, 1/8, 1/16), a signed Q8.4 format
			 would be sufficient. However all variables are stored in a
			 Q8.7 format for accuracy reasons when filtering. All variables
			 with this format have the extension _87.
			 This task is called every second so that every 2 seconds a new
			 temperature is present.
  Variables: tcfc_temp_87 : CFC temperature read from sensor in Q8.7 format
			 tcfc_err     : 1=error
  Returns  : -
  --------------------------------------------------------------------*/
void owc_task(void)
{
	static int owc_std = 0; // internal state
	int16_t    tmp;         // temporary variable (signed Q8.7 format)
	
	switch (owc_std)
	{   
		case 0: // Start conversion
				ds18b20_start_conversion(DS2482_TCFC_BASE);
				owc_std = 1;
				break;
		case 1: // Read Tcfc device
		        tcfc_old_87 = tcfc_temp_87; // copy previous value of tcfc_temp
			    tmp = ds18b20_read(DS2482_TCFC_BASE, &tcfc_err,1);
				if (!tcfc_err)
				{
					slope_limiter(temp_slope_87, tcfc_old_87, &tmp);
					tcfc_temp_87 = (int16_t)(moving_average(&tcfc_ma, (float)tmp) + 0.5);
				} // if
				owc_std = 0;
				break;
	} // switch
} // owc_task()

/*------------------------------------------------------------------
  Purpose  : This function initializes all the Arduino ports that 
             are used by the E-brew hardware and it initializes the
			 timer-interrupt to 1 msec. (1 kHz)
			 ADC6 (LM35), ADC0/PC0/PCINT8 (FLOW4), ADC1/PC1/PCINT9 (FLOW3), 
			 ADC2/PC2/PCINT10 (FLOW2) and ADC3/PC3/PCINT11 are inputs.
  Variables: -
  Returns  : -
  ------------------------------------------------------------------*/
void init_hardware(void)
{
	// Init. port C for reading LM35 Temp. / Flow-sensor counts
	DDRC   &= 0xB0; // ADC0/PC0, ADC1/PC1, ADC2/PC2, ADC3/PC3 and ADC6 are inputs
	PORTC  &= 0xBF; // Disable Pull-up resistor on ADC6 (LM35)
	PORTC  |= 0x0F; // Enable pull-up resistors on PC3, PC2, PC1 and PC0
	
	// Init. port C for reading 4 flow-sensor counts
	PCICR  |= (1<<PCIE1);   // set PCIE1 to enable PCMSK1 scan
	PCMSK1 |= (1<<PCINT11) | (1<<PCINT10) | (1<<PCINT9) | (1<<PCINT8); // Enable pin change interrupts

	//------------------------------------------------------
	// Init. Buzzer and Alive_LED outputs
	//------------------------------------------------------
	DDRD  |=  (ALIVE_LED_R | ALIVE_LED_G | ALIVE_LED_B | BUZZER); // Set to output
	PORTD &= ~(ALIVE_LED_R | ALIVE_LED_G | ALIVE_LED_B | BUZZER); // Init. both outputs to 0

	// Set Timer 2 to 1000 Hz interrupt frequency: ISR(TIMER2_COMPA_vect)
	TCCR2A |= (0x01 << WGM21);    // CTC mode, clear counter on TCNT2 == OCR2A
	TCCR2B =  (0x01 << CS22) | (0x01 << CS20); // set pre-scaler to 128
	OCR2A  =  124;   // this should set interrupt frequency to 1000 Hz
	TCNT2  =  0;     // start counting at 0
	TIMSK2 |= (0x01 << OCIE2A);   // Set interrupt on Compare Match
} // init_hardware()

/*------------------------------------------------------------------
  Purpose  : This function prints a welcome message to the serial
             port together with the current CVS revision number.
  Variables: 
        ver: string with version info
  Returns  : -
  ------------------------------------------------------------------*/
void print_ebrew_revision(char *ver)
{
	uint8_t len;
	char s[20];
	
	strcpy(ver, "E-Brew V2.0 rev.");  // welcome message, assure that all is well!
	len = strlen(ebrew_revision) - 13;  // just get the rev. number
	strncpy(s,&ebrew_revision[11],len); // example: " 1.3"
	s[len]   = '\n';
	s[len+1] = '\0';
	strcat(ver,s);
} // print_ebrew_revision()

/*------------------------------------------------------------------
  Purpose  : This function prints the IP address to the serial port.
  Variables: The IP-address to print
  Returns  : -
  ------------------------------------------------------------------*/
void print_IP_address(uint8_t *ip)
{
	char s[30];
	uint8_t i;
	
	for (i = 0; i < 4; i++)
	{
		sprintf(s,"%d",ip[i]);
		xputs(s);
		if (i < 3) xputs(".");
	} // for
} // print_IP_address()


/*-----------------------------------------------------------------------
  Purpose  : This function inits the WIZ550i Ethernet module by
             calling w5500_init() and spi_init(). If a valid MAC address
			 is detected, then dhcp_begin() is started.
  Variables: The IP-address to print
  Returns  : 1: success, 0: WIZ550IO not present
  -----------------------------------------------------------------------*/
uint8_t init_WIZ550IO_module(void)
{
	char     s[30];   // Needed for xputs() and sprintf()
	uint8_t  bufr[8]; // Needed for w5500_read_common_register()
	uint8_t  ret,x;
	uint16_t y;
	
	//---------------------------------------------------------------
	// Reset WIZ550IO Ethernet module
	//---------------------------------------------------------------
	DDRB  |=  WIZ550_HW_RESET; // Set HW_RESET pin as output-pin
	PORTB &= ~WIZ550_HW_RESET; // Active RESET for WIZ550io
	delay_msec(1);
	PORTB |=  WIZ550_HW_RESET; // Disable RESET for WIZ550io
	delay_msec(150);           // Giver W5500 time to configure itself

	ret = Ethernet_begin();    // includes w5500_init(), spi_init() & dhcp_begin()
	if (ret == 0)              // Error, no WIZ550IO module found
	{
		ethernet_WIZ550i = false;  // No ETH mode, switch back to USB
	    write_eeprom_parameters(); // save value in eeprom
		xputs("switching back to USB mode\n");
		return 0;
	} // if	
	
	x = udp_begin(EBREW_PORT_NR);           // init. UDP protocol
	w5500_read_common_register(SHAR, bufr); // get MAC address
	sprintf(s,"MAC:%02x:%02x:%02x:%02x:%02x:%02x ",bufr[0],bufr[1],bufr[2],bufr[3],bufr[4],bufr[5]);
	xputs(s);
	y = w5500_read_common_register(VERSIONR,bufr);
	sprintf(s,"W5500 VERSIONR: 0x%02x\n",y);          xputs(s);
	sprintf(s,"udp_begin():%d, sock=%d\n",x,_sock);   xputs(s);
	x = w5500_read_socket_register(_sock, Sn_MR, bufr);
	sprintf(s,"Sn_MR=%d, ",x);                        xputs(s);
	local_port = w5500_read_socket_register(_sock, Sn_PORT, bufr);
	sprintf(s,"Sn_PORT=%d, ",local_port);			  xputs(s);
	y = w5500_getTXFreeSize(_sock);
	sprintf(s,"Sn_TXfree=%d, ",y);					  xputs(s);
	y = w5500_getRXReceivedSize(_sock);
	sprintf(s,"Sn_RXrecv=%d\n",y);					  xputs(s);
	w5500_read_common_register(SIPR, bufr);
	xputs("Local IP address  :"); print_IP_address(bufr);
	for (x = 0; x < 4; x++) local_ip[x] = bufr[x]; // copy IP address
	w5500_read_common_register(GAR, bufr);
	xputs("\nGateway IP address:"); print_IP_address(bufr);
	w5500_read_common_register(SUBR, bufr);
	xputs("\nSubnet Mask       :"); print_IP_address(bufr);	
	xputs("\n");
	return 1; // success
} // init_WIZ550IO_module()

//int freeRam (void) 
//{
//	extern int __heap_start, *__brkval;
//	int v;
//	return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval);
//}

/*------------------------------------------------------------------
  Purpose  : This is the main() function for the E-brew hardware.
  Variables: -
  Returns  : should never return
  ------------------------------------------------------------------*/
int main(void)
{
	char    s[30];     // Needed for xputs() and sprintf()
	int	    udp_packet_size;
	
	init_hardware();         // Initialize Interrupts and all hardware devices
	i2c_init(SCL_CLK_50KHZ); // Init. I2C bus, I2C CLK = 50 kHz
	adc_init();              // Init. internal 10-bits AD-Converter
	pwm_init();              // Init. PWM function
	pwm_write(PWMB,0);	     // Start with 0 % duty-cycle for Boil-kettle
	pwm_write(PWMH,0);	     // Start with 0 % duty-cycle for HLT

    check_and_init_eeprom(); // EEPROM init.
	read_eeprom_parameters();// Read EEPROM value for ETHUSB and delayed-start
	
	//---------------------------------------------------------------
	// Init. Moving Average Filters for Measurements
	//---------------------------------------------------------------
	init_moving_average(&lm35_ma,10, (float)INIT_TEMP * 100.0); // Init. MA10-filter with 20 °C
	init_moving_average(&thlt_ma,10, (float)INIT_TEMP * 128.0); // Init. MA10-filter with 20 °C
	init_moving_average(&tmlt_ma,10, (float)INIT_TEMP * 128.0); // Init. MA10-filter with 20 °C
	init_moving_average(&tcfc_ma,10, (float)INIT_TEMP * 128.0); // Init. MA10-filter with 20 °C
	init_moving_average(&tboil_ma,10,(float)INIT_TEMP * 128.0); // Init. MA10-filter with 20 °C
	lm35_temp     = INIT_TEMP * 100;
	thlt_temp_87  = INIT_TEMP << 7;
	tmlt_temp_87  = INIT_TEMP << 7;
	tcfc_temp_87  = INIT_TEMP << 7;
	tboil_temp_87 = INIT_TEMP << 7;

	// Initialize Serial Communication, See usart.h for BAUD
	// F_CPU should be a Project Define (-DF_CPU=(xxxL)
	usart_init(MYUBRR); // Initializes the serial communication

	// Initialize all the tasks for the E-Brew system
	add_task(pwm_task  ,"pwm_task"  , 10,   50); // Electrical Heating Time-Division every 50 msec.
	add_task(lm35_task ,"lm35_task" , 30, 2000); // Process Temperature from LM35 sensor
	add_task(owh_task  ,"owh_task"  ,120, 1000); // Process Temperature from DS18B20 HLT sensor
	add_task(owm_task  ,"owm_task"  ,220, 1000); // Process Temperature from DS18B20 MLT sensor
	add_task(owb_task  ,"owb_task"  ,320, 1000); // Process Temperature from DS18B20 Boil-kettle sensor
	add_task(owc_task  ,"owc_task"  ,420, 1000); // Process Temperature from DS18B20 CFC-output sensor
	add_task(thlt_task ,"thlt_task" ,520, 2000); // Process Temperature from THLT sensor (I2C and/or OW)
	add_task(tmlt_task ,"tmlt_task" ,620, 2000); // Process Temperature from TMLT sensor (I2C and/or OW)
	
	sei();                      // set global interrupt enable, start task-scheduler
	if (mcp23017_init())        // Initialize IO-expander for valves (port A output, port B input)
	{
		xputs("mcp23017_init() error\n");
	} // if
	
	if (ethernet_WIZ550i)        // Initialize Ethernet adapter
	{
		print_ebrew_revision(s); // print revision number
		xputs(s);				 // Output to COM-port for debugging
		xputs("init_wiz550io_module()\n");
		init_WIZ550IO_module();
		xputs("starting main()   :");
		print_IP_address(local_ip);
		sprintf(s,":%d\n",local_port); // add local port as read back from wiz550io module
		xputs(s);
		bz_rpt_max = 1; // Sound buzzer once to indicate ethernet connection ready
		bz_on      = true;
	} // if
		
    while(1)
    {
	    dispatch_tasks(); // run the task-scheduler
		switch (rs232_command_handler()) // run command handler continuously
		{
			case ERR_CMD: xputs("Command Error\n"); break;
			case ERR_NUM: sprintf(s,"Number Error (%s)\n",rs232_inbuf);
						  xputs(s);  
						  break;
			case ERR_I2C: break; // do not print anything 
			default     : break;
		} // switch
		if (ethernet_WIZ550i) // only true after an E1 command
		{
			udp_packet_size = udp_parsePacket();
			if (udp_packet_size)
			{
				udp_read(udp_rcv_buf, UDP_TX_PACKET_MAX_SIZE);
				udp_rcv_buf[udp_packet_size] = '\0';
				ethernet_command_handler((char *)udp_rcv_buf);
			} // if	
			//delay_msec(10);	
		} // if
    } // while()
} // main()