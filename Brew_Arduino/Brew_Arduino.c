//-----------------------------------------------------------------------------
// Created: 20-4-2013 22:32:11
// Author : Emile
// File   : $Id$
//
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
//              (XTAL2/TOSC2) PB7 [10]    [19] PB5 (SCK)      SCK  dig.13 (SPI)
// Dig.05 NON_MOD        (T1) PD5 [11]    [18] PB4 (MISO)     MISO dig.12 (SPI)
// Dig.06 PUMP         (AIN0) PD6 [12]    [17] PB3 (MOSI/OC2) MOSI dig.11 (SPI)
// Dig.07 HEATER       (AIN1) PD7 [13]    [16] PB2 (SS/OC1B)  SS   dig.10 (SPI)
// Dig.08 WIZ550_RESET (ICP1) PB0 [14]    [15] PB1 (OC1A)     PWM  dig.09 (PWM)
//                               ------------
//                                ATmega328P
//-----------------------------------------------------------------------------
// $Log$
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

// Global variables
uint8_t      localIP[4]     = {192,168,1,177};    // local IP address
unsigned int localPort      = 8888;               // local port to listen on 	
const char  *ebrew_revision = "$Revision$"; // ebrew CVS revision number
uint8_t      system_mode    = GAS_MODULATING;     // Default to Modulating Gas-valve

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
uint16_t vhlt_max_10    = 1100;     // VHLT MAX Volume in E-1 L
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
int16_t  thlt_old_16;               // Previous value of thlt_temp_16
int16_t  thlt_temp_16;              // THLT Temperature in °C * 16
int16_t  thlt_offset_16 = 0;        // THLT offset-correction in °C * 16
int16_t  thlt_slope_16  = 32;       // THLT slope-limiter is 2 °C/sec. * 16
 
//-----------------------------------
// TMLT parameters and variables
//-----------------------------------
ma       tmlt_ma;                   // struct for TMLT moving_average filter
int16_t  tmlt_old_16;               // Previous value of tmlt_temp
int16_t  tmlt_temp_16;              // TMLT Temperature in °C * 16
int16_t  tmlt_offset_16 = 16;       // TMLT offset-correction in °C * 16
int16_t  tmlt_slope_16  = 32;       // TMLT slope-limiter is 2 °C/sec. * 16

unsigned long t2_millis = 0UL;

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
			 Conversion constant = 11000 / 1023 = 1000/93
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
    int16_t tmp; // temporary variable
	
	tmp       = adc_read(LM35);
	tmp       = (uint16_t)((unsigned long)tmp * 1000 / 93);
	lm35_temp = moving_average(&lm35_ma, tmp);
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
	
	vhlt_old_10 = vhlt_10; // copy previous value of vhlt
	tmp         = adc_read(VHLT);
	tmp         = (uint16_t)((unsigned long)vhlt_max_10 * tmp / 1023);
	tmp        += vhlt_offset_10;
	slope_limiter(vhlt_slope_10, vhlt_old_10, &tmp);
	vhlt_10   = moving_average(&vhlt_ma,tmp);
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
	
	vmlt_old_10 = vmlt_10; // copy previous value of vmlt
	tmp         = adc_read(VMLT);
	tmp         = (uint16_t)((unsigned long)vmlt_max_10 * tmp / 1023);
	tmp        += vmlt_offset_10;
	slope_limiter(vmlt_slope_10, vmlt_old_10, &tmp);
	vmlt_10   = moving_average(&vmlt_ma,tmp);
} // vmlt_task()

/*--------------------------------------------------------------------
  Purpose  : This is the task that processes the temperature from
             the LM92 temperature sensor. Since the LM92 has 4
			 fractional bits (1/2, 1/4, 1/8, 1/16), all values are
			 stored in a signed Q8.4 format. All variables with this
			 format have the extension _16.
			 The HLT temperature is both filtered and slope-limited.
  Variables: thlt_old_16   : previous value of thlt_temp_16
			 thlt_offset_16: the correction in temperature in °C
			 thlt_ma       : the moving-average filter struct for THLT
			 thlt_temp_16  : the processed HLT temperature in °C
  Returns  : -
  --------------------------------------------------------------------*/
void thlt_task(void)
{
	uint8_t  err; // error return value
	int16_t  tmp; // temporary variable (signed Q8.4 format)
	
	thlt_old_16 = thlt_temp_16; // copy previous value of thlt_temp
	tmp         = lm92_read(THLT, &err); // returns a signed Q8.4 format
	if (err) tmp = 0;
	else
	{	
		tmp += thlt_offset_16;
		slope_limiter(thlt_slope_16, thlt_old_16, &tmp);
		thlt_temp_16 = moving_average(&thlt_ma, tmp);
	} // else
} // thlt_task()

/*--------------------------------------------------------------------
  Purpose  : This is the task that processes the temperature from
             the LM92 temperature sensor. Since the LM92 has 4
			 fractional bits (1/2, 1/4, 1/8, 1/16), all values are
			 stored in a signed Q8.4 format. All variables with this
			 format have the extension _16.
			 The MLT temperature is both filtered and slope-limited.
  Variables: tmlt_old_16   : previous value of tmlt_temp_16
			 tmlt_offset_16: the correction in temperature in °C
			 tmlt_ma       : the moving-average filter struct for TMLT
			 tmlt_temp_16  : the processed MLT temperature in °C
  Returns  : -
  --------------------------------------------------------------------*/
void tmlt_task(void)
{
	uint8_t err; // error return value
	int16_t tmp; // temporary variable (signed Q8.4 format)
	
	tmlt_old_16 = tmlt_temp_16; // copy previous value of tmlt_temp
	tmp         = lm92_read(TMLT, &err); // returns a signed Q8.4 format
	if (err) tmp = 0;
	else
	{	
		tmp += tmlt_offset_16;
		slope_limiter(tmlt_slope_16, tmlt_old_16, &tmp);
		tmlt_temp_16 = moving_average(&tmlt_ma, tmp);
	} // else
} // tmlt_task()

/*------------------------------------------------------------------
  Purpose  : This function initializes all the Arduino ports that 
             are used by the E-brew hardware and it initializes the
			 timer-interrupt to 1 msec. (1 kHz)
  Variables: -
  Returns  : -
  ------------------------------------------------------------------*/
void init_interrupt(void)
{
	DDRC  &= 0xF8; // ADC0, ADC1, ADC2 inputs
	PORTC &= 0xF8; // Disable Pull-up resistors on ADC0..ADC2
	
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
	char     s[30];     // Needed for xputs() and sprintf()
	uint8_t  x,bufr[8]; // Needed for w5500_read_common_register()
	uint16_t y, cnt = 0;
	int	     udp_packet_size;
	
	init_interrupt(); // Initialize Interrupts and all hardware devices
	i2c_init();       // Init. I2C bus
	adc_init();       // Init. internal 10-bits AD-Converter
	pwm_init();       // Init. PWM function
	pwm_write(0);	  // Start with 0 % duty-cycle

	//---------------------------------------------------------------
	// Init. Moving Average Filters for Measurements
	//---------------------------------------------------------------
	init_moving_average(&lm35_ma,10, INIT_TEMP * 100); // Init. MA10-filter with 20 °C
	init_moving_average(&vhlt_ma, 5, INIT_VOL  *  10); // Init. MA5-filter with 8 L
	init_moving_average(&vmlt_ma, 5, INIT_VOL  *  10); // Init. VMLT MA5-filter with 8 L
	init_moving_average(&thlt_ma,10, INIT_TEMP << 4);  // Init. MA10-filter with 20 °C
	init_moving_average(&tmlt_ma,10, INIT_TEMP << 4);  // Init. MA10-filter with 20 °C
	lm35_temp    = INIT_TEMP * 100;
	vhlt_10      = INIT_VOL  *  10;
	vmlt_10      = INIT_VOL  *  10;
	thlt_temp_16 = INIT_TEMP << 4;
	tmlt_temp_16 = INIT_TEMP << 4;
	
	//------------------------------------------------------
	// PD2..PD4: LEDs ; PD5..PD7: Digital switching outputs
	// - Init. all IO pins as output
	// - Init. all IO pins to 0  
	//------------------------------------------------------
	DDRD  |=  (PUMP_LED | HEATER_LED | ALIVE_LED | NON_MOD | PUMP | HEATER);
	PORTD &= ~(PUMP_LED | HEATER_LED | ALIVE_LED | NON_MOD | PUMP | HEATER);
		
	// Initialize Serial Communication, See usart.h for BAUD
	// F_CPU should be a Project Define (-DF_CPU=(xxxL)
	usart_init(MYUBRR); // Initializes the serial communication
	
	// Initialize all the tasks for the E-Brew system
	add_task(pwm_2_time      ,"pwm_2_time", 10,   50); // Electrical Heating PWM every 50 msec.
	add_task(lm35_task       ,"lm35_task" , 30, 1000); // Process Temperature from LM35 sensor
	add_task(vhlt_task       ,"vhlt_task" , 50, 1000); // Process Volume from VHLT sensor
	add_task(vmlt_task       ,"vmlt_task" , 70, 1000); // Process Volume from VMLT sensor
	add_task(thlt_task       ,"thlt_task" , 90, 1000); // Process Temperature from THLT sensor
	add_task(tmlt_task       ,"tmlt_task" ,110, 1000); // Process Temperature from TMLT sensor
	
	sei();                      // set global interrupt enable, start task-scheduler
	print_ebrew_revision(s);    // print revision number

#ifdef WIZ550io_PRESENT
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
#endif

    while(1)
    {
		dispatch_tasks(); // run the task-scheduler
		switch (rs232_command_handler()) // run command handler continuously
		{
			case ERR_CMD: xputs("Command Error\n"); break;
			case ERR_NUM: xputs("Number Error\n");  break;
			case ERR_I2C: break; // do not print anything 
			default     : break;
		} // switch
		
#ifdef WIZ550io_PRESENT
		udp_packet_size = udp_parsePacket();
		if (udp_packet_size)
		{
			//sprintf(s,"\nUDP: %d bytes from ",udp_packet_size);         xputs(s);
			//print_IP_address(remoteIP); sprintf(s, ":%u\n",remotePort); xputs(s);
			// read the packet into packetBufffer
			udp_read(udp_rcv_buf, UDP_TX_PACKET_MAX_SIZE);
			udp_rcv_buf[udp_packet_size] = '\0';
			//sprintf(s,"Contents:[%s]\n",udp_rcv_buf);  					xputs(s);
			ethernet_command_handler((char *)udp_rcv_buf);
			// send a reply, to the IP address that sent us the packet we received
			// But with the localPort specified here
			//udp_beginPacketIP(remoteIP, localPort);
			//sprintf(s,"ack[%d]",cnt++);
			//udp_write((uint8_t *)s,strlen(s));
			//udp_endPacket();
		} // if	
		delay_msec(10);	
#endif
    } // while()
} // main()