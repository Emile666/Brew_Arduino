//-----------------------------------------------------------------------------
// Created: 20-4-2013 22:32:11
// Author : Emile
// File   : Brew_Arduino.c
//                         Brew Arduino Pin Mapping ATMEGA328P
//
//                               -----\/-----
//                   (RESET) PC6 [01]    [28] PC5 (ADC5/SCL) SCL  analog 5
// Dig.00 (RX)         (RXD) PD0 [02]    [27] PC4 (ADC4/SDA) SDA  analog 4
// Dig.01 (TX)         (TXD) PD1 [03]    [26] PC3 (ADC3)     ---  analog 3
// Dig.02 PUMP_LED    (INT0) PD2 [04]    [25] PC2 (ADC2)     VMLT analog 2
// Dig.03 HEATER_LED  (INT1) PD3 [05]    [24] PC1 (ADC1)     VHLT analog 1
// Dig.04 ALIVE_LED (XCK/TO) PD4 [06]    [23] PC0 (ADC0)     LM35 analog 0
//                           VCC [07]    [22] GND
//                           GND [08]    [21] AREF
//             (XTAL1/TOSC1) PB6 [09]    [20] AVCC
//             (XTAL2/TOSC2) PB7 [10]    [19] PB5 (SCK)      SCK  dig.13 (LED)
// Dig.05 NON_MOD       (T1) PD5 [11]    [18] PB4 (MISO)     MISO dig.12
// Dig.06 PUMP        (AIN0) PD6 [12]    [17] PB3 (MOSI/OC2) MOSI dig.11 (PWM)
// Dig.07 HEATER      (AIN1) PD7 [13]    [16] PB2 (SS/OC1B)  SS   dig.10 (PWM)
// Dig.08 ---         (ICP1) PB0 [14]    [15] PB1 (OC1A)     PWM  dig.09 (PWM)
//                               ------------
//                                ATmega328P
//-----------------------------------------------------------------------------
// $Log$
// Revision 1.2  2013/06/23 08:56:03  Emile
// - Working version (with PC program) with new command-set.
//
//-----------------------------------------------------------------------------
#include "brew_arduino.h"

// Global variables
const char *ebrew_revision = "$Revision$"; // ebrew CVS revision number
uint8_t    system_mode     = GAS_MODULATING;   // Default to Modulating Gas-valve

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
uint8_t  gas_mod_pwm_llimit = 4;  // Modulating gas-valve Hysteresis lower-limit parameter
uint8_t  gas_mod_pwm_hlimit = 6;  // Modulating gas-valve Hysteresis upper-limit parameter

uint8_t  tmr_on_val    = 0;         // ON-timer value  for PWM to Time-Division signal
uint8_t  tmr_off_val   = 0;         // OFF-timer value for PWM to Time-Division signal
uint8_t  triac_llimit  = 60;        // Hysteresis lower-limit for triac_too_hot signal
uint8_t  triac_hlimit  = 70;	    // Hysteresis upper-limit for triac_too_hot signal
uint8_t  triac_too_hot = FALSE;     // 1 = TRIAC temperature (read by LM35) is too high

ma       lm35_ma;                   // struct for LM35 moving_average filter
int8_t   lm35_temp;                 // LM35 Temperature in °C
uint16_t lm35_frac;                 // LM35 Temperature fraction part in E-2 °C
 
/*------------------------------------------------------------------
  Purpose  : This is the Timer-Interrupt routine which runs every msec. 
             (f=1 kHz). It is used by the task-scheduler.
  Variables: -
  Returns  : -
  ------------------------------------------------------------------*/
ISR(TIMER2_COMPA_vect)
{
	scheduler_isr(); // call the ISR routine for the task-scheduler
} // ISR()

/*------------------------------------------------------------------
  Purpose  : This is the task that toggles the LED on the Arduino
             every 500 msec. (f=1 Hz).
  Variables: -
  Returns  : -
  ------------------------------------------------------------------*/
void toggle_led_task(void)
{
	PORTB ^= 0x20; // Toggle PB5 (SCK) with orange Arduino LED
} // toggle_led_task()

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

/*------------------------------------------------------------------
  Purpose  : This is the task that processes the temperature from
             the LM35 temperature sensor. This sensor is connected
			 to ADC0. The LM35 outputs 10 mV/°C ; VREF=1.1 V,
			 therefore => Max. Temp. = 11000 E-2 °C
			 Conversion constant = 110 / 1023 = 10/93
  Variables: lm35_temp: contains temperature in °C
			 lm35_frac: contains fractional temperature in E-2 °C
             triac_too_hot: is set/reset
  Returns  : -
  ------------------------------------------------------------------*/
void lm35_task(void)
{
    uint16_t tmp2; // temporary variable

	tmp2      = adc_read(LM35);
	tmp2      = moving_average(&lm35_ma,tmp2);
	lm35_temp = (int8_t)(tmp2 * 10 / 93);
	tmp2     -= (int16_t)lm35_temp * 93 / 10;
	lm35_frac = (tmp2 * 1000 / 93); // (10/93)*100
	if (triac_too_hot)
	{  // reset hysteresis if temp < lower limit
		triac_too_hot = (lm35_temp >= triac_llimit);	
	} // if
	else 
	{  // set hysteresis if temp > upper limit
		triac_too_hot = (lm35_temp > triac_hlimit);
	} // else
} // lm35_task()

/*------------------------------------------------------------------
  Purpose  : This function initializes all the Arduino ports that 
             are used by the E-brew hardware and it initializes the
			 timer-interrupt to 1 msec. (1 kHz)
  Variables: -
  Returns  : -
  ------------------------------------------------------------------*/
void init_interrupt(void)
{
	DDRB   = 0x20;           // initialize PB5 (Arduino LED) for output, PB0 for input
	PORTB &= 0xDE;           // Disable Pull-up resistor on PORTB0 + LED off
	DDRC  &= 0xF8;           // ADC0, ADC1, ADC2 inputs
	PORTC &= 0xF8;           // Disable Pull-up resistors on ADC0..ADC2
	
	TCCR2A |= (0x01 << WGM21);    // CTC mode, clear counter on TCNT2 == OCR2A
	TCCR2B =  (0x01 << CS22) | (0x01 << CS20); // set pre-scaler to 128
	OCR2A  =  124;   // this should set interrupt frequency to 1000 Hz
	TCNT2  =  0;     // start counting at 0
	TIMSK2 |= (0x01 << OCIE2A);   // Set interrupt on Compare Match
} // init_interrupt()

/*------------------------------------------------------------------
  Purpose  : This is the main() function for the E-brew hardware.
  Variables: -
  Returns  : should never return
  ------------------------------------------------------------------*/
int main(void)
{
	init_interrupt(); // Initialize Interrupts and all hardware devices
	i2c_init();       // Init. I2C bus
	adc_init();       // Init. internal 10-bits AD-Converter
	pwm_init();       // Init. PWM function
	pwm_write(0);	  // Start with 0 % duty-cycle
	init_moving_average(&lm35_ma,10,20); // Init. LM35 MA10-filter with 20 °C
	
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
	add_task(toggle_led_task ,"led_blink" , 10,  500); // Toggle led every 500 msec.
	add_task(pwm_2_time      ,"pwm_2_time", 30,   50); // Electrical Heating PWM every 50 msec.
	add_task(lm35_task       ,"lm35_task" , 50, 1000); // Process Temperature from LM35 sensor
	
	sei(); // set global interrupt enable, start task-scheduler
	xputs("E-Brew V2.0!\n"); // welcome message, assure that all is well!

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
    } // while()
} // main()