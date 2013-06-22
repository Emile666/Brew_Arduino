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
//-----------------------------------------------------------------------------
#include "brew_arduino.h"
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>         // for _delay_ms()

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
uint8_t  triac_too_hot = 0;		    // 1 = TRIAC temperature (read by LM35) is too high
uint16_t irq_cnt       = 0;         // interrupt counter

ma lm35_ma;                 // struct for LM35 moving_average filter


/*-----------------------------------------------------------------------------
  Purpose  : Convert a PWM signal into a time-division signal of 100 * 50 msec.
             This routine should be called from the timer-interrupt every 50 msec.
			 It uses the tmr_on_val and tmr_off_val values which were set by the
			 process_pwm_signal. This routine should only be called when 
			 system_mode is set to ELECTRICAL_HEATING.
  Variables: tmr_on_val : the ON-time value
             tmr_off_val: the OFF-time value
			 htimer   : the ON-timer
			 ltimer   : the OFF-timer
  Returns  : -
  ---------------------------------------------------------------------------*/
void pwm_2_time(void)
{
   static uint8_t  htimer      = 0;    // The ON-timer
   static uint8_t  ltimer      = 0;    // The OFF-timer
   static uint8_t  std_td      = IDLE; // STD State number

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
} // pwm_2_time()

ISR(TIMER2_COMPA_vect)
{
	if (((irq_cnt % 50) == 1) && (system_mode == ELECTRICAL_HEATING))
	{   // call every 50 msec and only in case of ELECTRICAL HEATING
		pwm_2_time();	
	} // if
	if (++irq_cnt == IRQ_CNT)
	{
		PORTB ^= 0x20; // Toggle PB5 (SCK) with orange Arduino LED
		irq_cnt = 0;
	} // if
} // ISR()

void i2c_scan(uint8_t ch)
{
    char    s[50]; // needed for printing to serial terminal
	uint8_t x = 0;
	int     i;     // Leave this as an int!
    enum i2c_acks retv;
	
	retv = i2c_select_channel(ch);
	if (ch == PCA9544_NOCH)
		 sprintf(s,"I2C[-]: ");
	else sprintf(s,"I2C[%1d]: ",ch-PCA9544_CH0);
	xputs(s);
	for (i = 0x00; i < 0xff; i+=2)
	{
		if (i2c_start(i) == I2C_ACK)
		{
			if ((ch == PCA9544_NOCH) || ((ch != PCA9544_NOCH) && (i != PCA9544)))
			{
				sprintf(s,"0x%0x ",i);
				xputs(s);
				x++;
			}
		} // if
		i2c_stop();
	} // for
	if (!x) xputs("no devices detected");
	xputs("\n");
} // i2c_scan()

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
	sei();                        // set global interrupt enable	
} // init_interrupt()

int main(void)
{
	// Initialize all hardware devices
	init_interrupt();
	i2c_init();       // init. I2C bus
	adc_init();       // init. internal 10-bits AD-Converter
	pwm_init();       // init. PWM function
	pwm_write(0);	  // Start with 0 % duty-cycle
	init_moving_average(&lm35_ma,10,20); // Init. LM35 MA10-filter
	
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
	xputs("E-Brew v2.00 Brewing System!\nStarting i2c_scan()...\n");

	i2c_scan(PCA9544_NOCH); // Start with main I2C channel
	i2c_scan(PCA9544_CH0);  // PCA9544 channel 0
	i2c_scan(PCA9544_CH1);  // PCA9544 channel 1
	i2c_scan(PCA9544_CH2);  // PCA9544 channel 2
	i2c_scan(PCA9544_CH3);  // PCA9544 channel 3
	xputs("\nStarting rs232_command_handler()\n");
    while(1)
    {
		switch (rs232_command_handler()) // run command handler continuously
		{
			case ERR_CMD: xputs("Command Error\n"); break;
			case ERR_NUM: xputs("Number Error\n");  break;
			case ERR_I2C: xputs("I2C Error\n");     break;
			default     : break;
		} // switch
    } // while()
} // main()