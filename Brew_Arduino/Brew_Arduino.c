//-----------------------------------------------------------------------------
// Created: 20-4-2013 22:32:11
// Author : Emile
// File   : $Id$
//-----------------------------------------------------------------------------
// $Log$
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
// - PUMP_LED removed from PD2, PUMP has same function
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
uint8_t      localIP[4]     = {192,168,1,177};    // local IP address
unsigned int localPort      = 8888;               // local port to listen on 	
const char  *ebrew_revision = "$Revision$"; // ebrew CVS revision number
uint8_t      system_mode    = GAS_MODULATING;     // Default to Modulating Gas-valve
bool         ethernet_WIZ550i = false;			  // Default to No WIZ550i present

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
//                                   the HEATER LED and HEATER TRIAC is switched on.
//                                   The TRIAC is needed to energize the gas-valve.
//---------------------------------------------------------------------------------
uint8_t  gas_mod_pwm_llimit = 2;  // Modulating gas-valve Hysteresis lower-limit parameter
uint8_t  gas_mod_pwm_hlimit = 4;  // Modulating gas-valve Hysteresis upper-limit parameter

uint8_t  tmr_on_val    = 0;         // ON-timer value  for PWM to Time-Division signal
uint8_t  tmr_off_val   = 0;         // OFF-timer value for PWM to Time-Division signal

//-----------------------------------
// LM35 parameters and variables
//-----------------------------------
ma       lm35_ma;                   // struct for LM35 moving_average filter
uint16_t lm35_temp;                 // LM35 Temperature in E-2 °C
uint16_t triac_llimit  = 6500;      // Hysteresis lower-limit for triac_too_hot in E-2 °C
uint16_t triac_hlimit  = 7500;	    // Hysteresis upper-limit for triac_too_hot in E-2 °C
uint8_t  triac_too_hot = FALSE;     // 1 = TRIAC temperature (read by LM35) is too high

//-----------------------------------
// VHLT parameters and variables
//-----------------------------------
ma       vhlt_ma;                   // struct for VHLT moving_average filter
uint16_t vhlt_old_10;               // Previous value of vhlt_10
uint16_t vhlt_10;                   // VHLT Volume in E-1 L
int16_t  vhlt_offset_10 = 80;       // VHLT offset-correction in E-1 L
uint16_t vhlt_max_10    = 1400;     // VHLT MAX Volume in E-1 L
int16_t  vhlt_slope_10  = 10;       // VHLT slope-limiter is 1 L/sec.

//-----------------------------------
// VMLT parameters and variables
//-----------------------------------
ma       vmlt_ma;                   // struct for VMLT moving_average filter
uint16_t vmlt_old_10;               // Previous value of vmlt_10
uint16_t vmlt_10;                   // VMLT Volume in E-1 L
int16_t  vmlt_offset_10 = 80;       // VMLT offset-correction in E-1 L
uint16_t vmlt_max_10    = 1100;     // VMLT MAX Volume in E-1 L
int16_t  vmlt_slope_10  = 10;       // VMLT slope-limiter is 1 L/sec.

//-----------------------------------
// THLT parameters and variables
//-----------------------------------
ma       thlt_ma;                   // struct for THLT moving_average filter
int16_t  thlt_old_87;               // Previous value of thlt_temp_87
int16_t  thlt_temp_87;              // THLT Temperature from LM92 in °C * 128
int16_t  thlt_ow_87;                // THLT Temperature from DS18B20 in °C * 128
int16_t  thlt_offset_87 = 0;        // THLT offset-correction in °C * 128
int16_t  thlt_slope_87  = 512;      // THLT slope-limiter is 4 °C/ 2 sec. * 128
uint8_t  thlt_err       = 0;        // 1 = Read error from LM92
uint8_t  thlt_ow_err    = 0;	    // 1 = Read error from DS18B20
 
//-----------------------------------
// TMLT parameters and variables
//-----------------------------------
ma       tmlt_ma;                   // struct for TMLT moving_average filter
int16_t  tmlt_old_87;               // Previous value of tmlt_temp_87
int16_t  tmlt_temp_87;              // TMLT Temperature from LM92 in °C * 128
int16_t  tmlt_ow_87;                // TMLT Temperature from DS18B20 in °C * 128
int16_t  tmlt_offset_87 = 0;        // TMLT offset-correction in °C * 128
int16_t  tmlt_slope_87  = 512;      // TMLT slope-limiter is 4 °C/ 2 sec. * 128
uint8_t  tmlt_err       = 0;        // 1 = Read error from LM92
uint8_t  tmlt_ow_err    = 0;	    // 1 = Read error from DS18B20

unsigned long    t2_millis     = 0UL;
unsigned long    flow_hlt_mlt  = 0UL;
unsigned long    flow_mlt_boil = 0UL;
volatile uint8_t old_pc3       = 0x08; // default is high because of the pull-up
volatile uint8_t old_pd2       = 0x04; // default is high because of the pull-up

/*------------------------------------------------------------------
  Purpose  : This is the Timer-Interrupt routine which runs every msec. 
             (f=1 kHz). It is used by the task-scheduler.
  Variables: -
  Returns  : -
  ------------------------------------------------------------------*/
ISR(TIMER2_COMPA_vect)
{
	t2_millis++;     // update millisecond counter
	scheduler_isr(); // call the ISR routine for the task-scheduler
} // ISR()

/*------------------------------------------------------------------
  Purpose  : This is the State-change interrupt routine for PORTC. 
             It is used to count pulses from FLOW1 (PC3), which is 
			 the water flow sensor between the HLT and the MLT.
  Variables: -
  Returns  : -
  ------------------------------------------------------------------*/
ISR (PCINT1_vect)
{
	uint8_t pc3, dpc3;
	
	pc3     = PINC & (1 << PINC3); // is either 0x00 or 0x08
	dpc3    = pc3 ^ old_pc3;       // reads 0x08 on rising & falling edge
	old_pc3 = pc3;                 // save current value of PC3
	
	if (dpc3 && pc3)
	{   // PC3 is 1 => rising edge detected
		flow_hlt_mlt++; /* PC3/PCINT11 rising edge */
	} // if
} // ISR(PCINT1_vect)

/*------------------------------------------------------------------
  Purpose  : This is the State-change interrupt routine for PORTD. 
             It is used to count pulses from FLOW2 (PD2), which is 
			 the water flow sensor between the MLT and the boil-kettle.
  Variables: -
  Returns  : -
  ------------------------------------------------------------------*/
ISR (PCINT2_vect)
{
	uint8_t pd2, dpd2;

	pd2     = PIND & (1 << PIND2); // is either 0x00 or 0x04
	dpd2    = pd2 ^ old_pd2;       // reads 0x04 on rising & falling edge
	old_pd2 = pd2;                 // save current value of PD2
	
	if (dpd2 && pd2)
	{   // PD2 is 1 => rising edge detected
		flow_mlt_boil++; /* PD2/PCINT18 rising edge */
	} // if
} // ISR(PCINT2_vect)

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
			 anything.
  Variables: tmr_on_val : the ON-time value
             tmr_off_val: the OFF-time value
			 htimer     : the ON-timer
			 ltimer     : the OFF-timer
  Returns  : -
  ---------------------------------------------------------------------------*/
void pwm_2_time(void)
{
   static uint8_t  htimer      = 0;    // The ON-timer
   static uint8_t  ltimer      = 0;    // The OFF-timer
   static uint8_t  std_td      = IDLE; // STD State number

   if (system_mode == ELECTRICAL_HEATING)
   {   // only run STD when ELECTRICAL HEATING is enabled
	   switch (std_td)
	   {
		   case IDLE:
				//-------------------------------------------------------------
				// This is the default state after power-up. Main function
				// of this state is to initialize the electrical heater states.
				//-------------------------------------------------------------
				PORTD &= ~(HEATER | HEATER_LED); // set electrical heater OFF
				ltimer = tmr_off_val;            // init. low-timer
				std_td = EL_HTR_OFF;             // goto OFF-state
				break;

		   case EL_HTR_OFF:
			   //---------------------------------------------------------
			   // Goto ON-state if timeout_ltimer && !triac_too_hot &&
			   //      !0%_gamma
			   //---------------------------------------------------------
			   if (ltimer == 0)
			   {  // OFF-timer has counted down
				   if (tmr_on_val == 0)
				   {   // indication for 0%, remain in this state
					   ltimer = tmr_off_val; // init. timer again
					   PORTD &= ~(HEATER | HEATER_LED); // set heater OFF
				   } // if
				   else if (!triac_too_hot)
				   {
					   PORTD |= (HEATER | HEATER_LED); // set heater ON
					   htimer = tmr_on_val; // init. high-timer
					   std_td = EL_HTR_ON;  // go to ON-state
				   } // else if
				   // Remain in this state if timeout && !0% && triac_too_hot
			   } // if
			   else
			   {   // timer has not counted down yet, continue
				   PORTD &= ~(HEATER | HEATER_LED); // set heater OFF
				   ltimer--;         // decrement low-timer
			   } // else
			   break;

		   case EL_HTR_ON:
			   //---------------------------------------------------------
			   // Goto OFF-state if triac_too_hot OR 
			   //      (timeout_htimer && !100%_gamma)
			   //---------------------------------------------------------
			   if (triac_too_hot)
			   {
				   PORTD &= ~(HEATER | HEATER_LED); // set heater OFF
				   ltimer = tmr_off_val; // init. low-timer
				   std_td = EL_HTR_OFF;  // go to 'OFF' state
			   } // if
			   else if (htimer == 0)
			   {   // timer has counted down
				   if (tmr_off_val == 0)
				   {  // indication for 100%, remain in this state
					   htimer = tmr_on_val; // init. high-timer again
					   PORTD |= (HEATER | HEATER_LED); // Set heater ON
				   } // if
				   else
				   {   // tmr_off_val > 0
					   PORTD &= ~(HEATER | HEATER_LED); // set heater OFF
					   ltimer = tmr_off_val; // init. low-timer
					   std_td = EL_HTR_OFF;  // go to 'OFF' state
				   } // else
			   } // if
			   else
			   {  // timer has not counted down yet, continue
				   PORTD |= (HEATER | HEATER_LED); // Set heater ON
				   htimer--;        // decrement high-timer
			   } // else
			   break;

		   default: break;	
	   } // switch 
   } // if
} // pwm_2_time()

/*--------------------------------------------------------------------
  Purpose  : This is the task that processes the temperature from
             the LM35 temperature sensor. This sensor is connected
			 to ADC0. The LM35 outputs 10 mV/°C ; VREF=1.1 V,
			 therefore => Max. Temp. = 11000 E-2 °C
			 Conversion constant = 11000 / 1023 = 10.7526882
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
} // lm35_task()

/*--------------------------------------------------------------------
  Information about Volume Measurements (vhlt_task() & vmlt_task())

  The volume is measured by a MPX2010 pressure sensor that is 
  connected to an AD-channel of the ATmega328
  
  The MPX2010 delivers 25 mV at 1.45 psi = 10 kPa = 101.978 cm H2O.

  The MPX2010 signal is amplified by an AD620 PGA with A=184 (Rg=270R).
  The results in a full-scale span of 2.26 Volts @ 50 cm H2O.
  TO-DO: adjust the gain of the AD620:
	     60 cm H2O => 1.1 Volt: A = 1.1/(60*0.025/101.978)
	     A = 74.8 ; Rg = 680 Ohms (A=73.6)
  --------------------------------------------------------------------*/

/*--------------------------------------------------------------------
  Purpose  : This is the task that processes the HLT volume. 
             The HLT volume is measured by the MPX2010 pressure sensor
			 that is connected to ADC channel 1.
  Variables: vhlt_ma       : struct for VHLT moving_average filter
             vhlt_old_10   : Previous value of vhlt_10
             vhlt_10       : VHLT Volume in E-1 L
             vhlt_offset_10: VHLT offset-correction in E-1 L
             vhlt_max_10   : VHLT MAX Volume in E-1 L
             vhlt_slope_10 : VHLT slope-limiter is 1 L/sec.
  Returns  : -
  --------------------------------------------------------------------*/
void vhlt_task(void)
{
    int16_t tmp; // temporary variable
	
	tmp         = adc_read(VHLT);
	tmp         = (uint16_t)((unsigned long)vhlt_max_10 * tmp / 1023);
	tmp        += vhlt_offset_10;
	vhlt_old_10 = vhlt_10; // copy previous value of vhlt
	slope_limiter(vhlt_slope_10, vhlt_old_10, &tmp);
	vhlt_10     = (uint16_t)moving_average(&vhlt_ma,(float)tmp);
} // vhlt_task()

/*--------------------------------------------------------------------
  Purpose  : This is the task that processes the MLT volume. 
             The MLT volume is measured by the MPX2010 pressure sensor
			 that is connected to ADC channel 2.
  Variables: vmlt_ma       : struct for VMLT moving_average filter
             vmlt_old_10   : Previous value of vmlt_10
             vmlt_10       : VMLT Volume in E-1 L
             vmlt_offset_10: VMLT offset-correction in E-1 L
             vmlt_max_10   : VMLT MAX Volume in E-1 L
             vmlt_slope_10 : VMLT slope-limiter is 1 L/sec.
  Returns  : -
  --------------------------------------------------------------------*/
void vmlt_task(void)
{
    int16_t tmp; // temporary variable
	
	tmp         = adc_read(VMLT);
	tmp         = (uint16_t)((unsigned long)vmlt_max_10 * tmp / 1023);
	tmp        += vmlt_offset_10;
	vmlt_old_10 = vmlt_10; // copy previous value of vmlt
	slope_limiter(vmlt_slope_10, vmlt_old_10, &tmp);
	vmlt_10     = (uint16_t)moving_average(&vmlt_ma,(float)tmp);
} // vmlt_task()

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
			 thlt_offset_87: the correction in temperature in °C
			 tmlt_slope_87 : the slope-limit in °C/sec.
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
	{   // I2C sensor (LM92) returned an error, one-wire sensor is ok
		thlt_err = FALSE; // reset error
		tmp      = thlt_ow_87;
	} // if
	if (!thlt_err) 
	{	// filter if either I2C or One-Wire has been read successfully
		tmp += thlt_offset_87; // add offset
		slope_limiter(thlt_slope_87, thlt_old_87, &tmp);
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
			 tmlt_offset_87: the correction in temperature in °C
			 tmlt_slope_87 : the slope-limit in °C/sec.
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
	{   // I2C sensor (LM92) returned an error, one-wire sensor is ok
		tmlt_err = FALSE; // reset error
		tmp      = tmlt_ow_87;
	} // if
	if (!tmlt_err)
	{	// filter if either I2C or One-Wire has been read successfully
		tmp += tmlt_offset_87; // add offset
		slope_limiter(tmlt_slope_87, tmlt_old_87, &tmp);
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

/*------------------------------------------------------------------
  Purpose  : This function initializes all the Arduino ports that 
             are used by the E-brew hardware and it initializes the
			 timer-interrupt to 1 msec. (1 kHz)
  Variables: -
  Returns  : -
  ------------------------------------------------------------------*/
void init_interrupt(void)
{
	// Init. port C for reading analog values
	DDRC  &= 0xF8; // ADC0, ADC1, ADC2 inputs
	PORTC &= 0xF8; // Disable Pull-up resistors on ADC0..ADC2
	
	// Init. port C for reading flow sensor counts
	DDRC   &= ~0x08;        // PC3 is input
	PORTC  |= 0x08;         // Enable pull-up resistor on PC3
	PCICR  |= (1<<PCIE1);   // set PCIE1 to enable PCMSK1 scan
	PCMSK1 |= (1<<PCINT11); // Enable PC3 pin change interrupt
	
	// Init. port D for reading flow sensor counts
	DDRD   &= ~0x04;        // PD2 is input
	PORTD  |= 0x04;         // Enable pull-up resistor on PD2
	PCICR  |= (1<<PCIE2);   // set PCIE2 to enable PCMSK2 scan
	PCMSK2 |= (1<<PCINT18); // Enable PD2 pin change interrupt

	TCCR2A |= (0x01 << WGM21);    // CTC mode, clear counter on TCNT2 == OCR2A
	TCCR2B =  (0x01 << CS22) | (0x01 << CS20); // set pre-scaler to 128
	OCR2A  =  124;   // this should set interrupt frequency to 1000 Hz
	TCNT2  =  0;     // start counting at 0
	TIMSK2 |= (0x01 << OCIE2A);   // Set interrupt on Compare Match
} // init_interrupt()

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

void init_WIZ550IO_module(void)
{
	char     s[30];     // Needed for xputs() and sprintf()
	uint8_t  x,bufr[8]; // Needed for w5500_read_common_register()
	uint16_t y;
	
	//---------------------------------------------------------------
	// Reset WIZ550IO Ethernet module
	//---------------------------------------------------------------
	DDRB  |=  WIZ550_HW_RESET; // Set HW_RESET pin as output-pin
	PORTB &= ~WIZ550_HW_RESET; // Active RESET for WIZ550io
	delay_msec(1);
	PORTB |=  WIZ550_HW_RESET; // Disable RESET for WIZ550io
	delay_msec(150);           // Giver W5500 time to configure itself

	Ethernet_begin_ip(localIP); // includes w5500_init() and spi_init()
	x = udp_begin(localPort);   // init. UDP protocol

	w5500_read_common_register(SHAR, bufr); // get MAC address
	sprintf(s,"MAC:%02x:%02x:%02x:%02x:%02x:%02x ",bufr[0],bufr[1],bufr[2],bufr[3],bufr[4],bufr[5]);
	xputs(s);
	y = w5500_read_common_register(VERSIONR,bufr);
	sprintf(s,"W5500 VERSIONR: 0x%02x\n",y);          xputs(s);
	sprintf(s,"udp_begin():%d, sock=%d\n",x,_sock);   xputs(s);
	x = w5500_read_socket_register(_sock, Sn_MR, bufr);
	sprintf(s,"Sn_MR=%d, ",x);                        xputs(s);
	y = w5500_read_socket_register(_sock, Sn_PORT, bufr);
	sprintf(s,"Sn_PORT=%d, ",y);					  xputs(s);
	y = w5500_getTXFreeSize(_sock);
	sprintf(s,"Sn_TXfree=%d, ",y);					  xputs(s);
	y = w5500_getRXReceivedSize(_sock);
	sprintf(s,"Sn_RXrecv=%d\n",y);					  xputs(s);
	w5500_read_common_register(SIPR, bufr);
	xputs("Local IP address  :"); print_IP_address(bufr);
	w5500_read_common_register(GAR, bufr);
	xputs("\nGateway IP address:"); print_IP_address(bufr);
	w5500_read_common_register(SUBR, bufr);
	xputs("\nSubnet Mask       :"); print_IP_address(bufr);	
	xputs("\n");
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
	
	init_interrupt();        // Initialize Interrupts and all hardware devices
	i2c_init(SCL_CLK_50KHZ); // Init. I2C bus, I2C CLK = 50 kHz
	adc_init();              // Init. internal 10-bits AD-Converter
	pwm_init();              // Init. PWM function
	pwm_write(0);	         // Start with 0 % duty-cycle

	//---------------------------------------------------------------
	// Init. Moving Average Filters for Measurements
	//---------------------------------------------------------------
	init_moving_average(&lm35_ma,10, (float)INIT_TEMP * 100.0); // Init. MA10-filter with 20 °C
	init_moving_average(&vhlt_ma, 5, (float)INIT_VOL10);        // Init. MA5-filter with 8 L
	init_moving_average(&vmlt_ma, 5, (float)INIT_VOL10);        // Init. VMLT MA5-filter with 8 L
	init_moving_average(&thlt_ma,10, (float)INIT_TEMP * 128.0); // Init. MA10-filter with 20 °C
	init_moving_average(&tmlt_ma,10, (float)INIT_TEMP * 128.0); // Init. MA10-filter with 20 °C
	lm35_temp    = INIT_TEMP * 100;
	vhlt_10      = INIT_VOL10;
	vmlt_10      = INIT_VOL10;
	thlt_temp_87 = INIT_TEMP << 7;
	tmlt_temp_87 = INIT_TEMP << 7;
	
	//------------------------------------------------------
	// PD3..PD4: LEDs ; PD5..PD7: Digital switching outputs
	// - Init. all IO pins as output
	// - Init. all IO pins to 0  
	//------------------------------------------------------
	DDRD  |=  (HEATER_LED | ALIVE_LED | NON_MOD | PUMP | HEATER);
	PORTD &= ~(HEATER_LED | ALIVE_LED | NON_MOD | PUMP | HEATER);
		
	// Initialize Serial Communication, See usart.h for BAUD
	// F_CPU should be a Project Define (-DF_CPU=(xxxL)
	usart_init(MYUBRR); // Initializes the serial communication
	
	// Initialize all the tasks for the E-Brew system
	add_task(pwm_2_time      ,"pwm_2_time", 10,   50); // Electrical Heating PWM every 50 msec.
	add_task(lm35_task       ,"lm35_task" , 30, 2000); // Process Temperature from LM35 sensor
	add_task(vhlt_task       ,"vhlt_task" , 50, 1000); // Process Volume from VHLT sensor
	add_task(vmlt_task       ,"vmlt_task" , 70, 1000); // Process Volume from VMLT sensor
	add_task(owh_task        ,"owh_task"  ,320, 1000); // Process Temperature from DS18B20 HLT sensor
	add_task(owm_task        ,"owm_task"  ,420, 1000); // Process Temperature from DS18B20 MLT sensor
	add_task(thlt_task       ,"thlt_task" ,520, 2000); // Process Temperature from THLT sensor
	add_task(tmlt_task       ,"tmlt_task" ,620, 2000); // Process Temperature from TMLT sensor
	
	sei();                      // set global interrupt enable, start task-scheduler
	print_ebrew_revision(s);    // print revision number    
	if (mcp23008_init())        // Initialize IO-expander for valves (port A output, port B input)
	{
		xputs("mcp23008_init() error\n");
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
			delay_msec(10);	
		} // if
    } // while()
} // main()