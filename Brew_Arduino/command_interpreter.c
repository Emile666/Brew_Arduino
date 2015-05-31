//-----------------------------------------------------------------------------
// Created: 22-4-2013 07:26:24
// Author : Emile
// File   : command_interpreter.c
//-----------------------------------------------------------------------------
// $Log$
// Revision 1.14  2014/11/30 20:44:45  Emile
// - Vxxx command added to write valve output bits
// - mcp23017 (16 bit I2C IO-expander) routines + defines added
//
// Revision 1.13  2014/11/09 15:38:34  Emile
// - PUMP_LED removed from PD2, PUMP has same function
// - Interface for 2nd waterflow sensor added to PD2
// - Command A6 (read waterflow in E-2 L) added
// - FLOW_PER_L changed to 330 (only rising edge is counted)
//
// Revision 1.12  2014/10/26 12:44:47  Emile
// - A3 (Thlt) and A4 (Tmlt) commands now return '99.99' in case of I2C HW error.
//
// Revision 1.11  2014/06/15 14:52:20  Emile
// - Commands E0 and E1 (Disable/Enable Ethernet module) added
// - Interface for waterflow sensor added to PC3/ADC3
// - Command A5 (read waterflow in E-2 L) added
//
// Revision 1.10  2014/06/01 13:46:02  Emile
// Bug-fix: do not call udp routines when in COM port mode!
//
// Revision 1.9  2014/05/03 11:27:44  Emile
// - Ethernet support added for W550io module
// - No response for L, N, P, W commands anymore
// - All source files now have headers
//
// Revision 1.8  2013/07/24 13:46:40  Emile
// - Minor changes in S1, S2 and S3 commands to minimize comm. overhead.
// - Version ready for Integration Testing with PC program!
//
// Revision 1.7  2013/07/23 19:33:18  Emile
// - Bug-fix slope-limiter function. Tested on all measurements.
//
// Revision 1.6  2013/07/21 13:10:43  Emile
// - Reading & Writing of 17 parameters now fully works with set_parameter()
// - VHLT and VMLT tasks added
// - Scheduler: actual & max. times now printed in msec. instead of usec.
// - THLT and TMLT now in E-2 Celsius for PC program
// - All lm92 test routines removed, only one lm92_read() remaining
//
// Revision 1.5  2013/07/20 14:51:59  Emile
// - LM35, THLT and TMLT tasks are now working
// - Max. duration added to scheduler
// - slope_limiter & lm92_read() now work with uint16_t instead of float
//
// Revision 1.4  2013/07/19 10:51:02  Emile
// - I2C frequency 50 50 kHz to get 2nd LM92 working
// - Command Mx removed, command N0 x added, commands N0..N3 renamed to N1..N4
// - Command S3 added, list_all_tasks. To-Do: get timing-measurement working
// - Scheduler added with 3 tasks: lm35, led_blink and pwm_2_time
//
// Revision 1.3  2013/06/23 09:08:51  Emile
// - Headers added to files
//
//
//-----------------------------------------------------------------------------
#include <string.h>
#include <ctype.h>
#include <util/atomic.h>
#include <stdio.h>
#include "command_interpreter.h"
#include "misc.h"
#include "Udp.h"

extern uint8_t      remoteIP[4]; // remote IP address for the incoming packet whilst it's being processed
extern unsigned int localPort;   // local port to listen on 	

extern uint8_t    system_mode;         // from Brew_Arduino.c
extern bool       ethernet_WIZ550i;
extern const char *ebrew_revision;     // ebrew CVS revision number
extern uint8_t    gas_non_mod_llimit; 
extern uint8_t    gas_non_mod_hlimit;
extern uint8_t    gas_mod_pwm_llimit;
extern uint8_t    gas_mod_pwm_hlimit;
extern uint8_t    tmr_on_val;          // ON-timer  for PWM to Time-Division signal
extern uint8_t    tmr_off_val;         // OFF-timer for PWM to Time-Division signal

extern uint16_t   lm35_temp;           // LM35 Temperature in E-2 °C
extern uint16_t   triac_llimit;        // Hysteresis lower-limit for triac_too_hot in E-2 °C
extern uint16_t   triac_hlimit;	       // Hysteresis upper-limit for triac_too_hot in E-2 °C

extern uint16_t   vhlt_10;             // VHLT Volume in E-1 L
extern int16_t    vhlt_offset_10;      // VHLT offset-correction in E-1 L
extern uint16_t   vhlt_max_10;         // VHLT MAX Volume in E-1 L
extern int16_t    vhlt_slope_10;       // VHLT slope-limiter in E-1 L/sec.

extern uint16_t   vmlt_10;             // VMLT Volume in E-1 L
extern int16_t    vmlt_offset_10;      // VMLT offset-correction in E-1 L
extern uint16_t   vmlt_max_10;         // VMLT MAX Volume in E-1 L
extern int16_t    vmlt_slope_10;       // VMLT slope-limiter in E-1 L/sec.

//------------------------------------------------------
// The extension _16 indicates a signed Q8.4 format!
// This is used for both the HLT and MLT temperatures
//------------------------------------------------------
extern int16_t    thlt_temp_16;        // THLT Temperature in °C * 16
extern int16_t    thlt_offset_16;      // THLT offset-correction in °C * 16
extern int16_t    thlt_slope_16;       // THLT slope-limiter is 2 °C/sec. * 16
extern uint8_t    thlt_err;

extern int16_t    tmlt_temp_16;        // TMLT Temperature in °C * 16
extern int16_t    tmlt_offset_16;      // TMLT offset-correction in °C * 16
extern int16_t    tmlt_slope_16;       // TMLT slope-limiter in °C/sec.
extern uint8_t    tmlt_err;

extern unsigned long flow_hlt_mlt;     // Count from flow-sensor between HLT and MLT
extern unsigned long flow_mlt_boil;    // Count from flow-sensor between MLT and boil-kettle

char    rs232_inbuf[USART_BUFLEN];     // buffer for RS232 commands
uint8_t rs232_ptr = 0;                 // index in RS232 buffer

/*-----------------------------------------------------------------------------
  Purpose  : Scan all devices on the I2C bus on all channels of the PCA9544
  Variables: 
         ch: the I2C channel number, 0 is the main channel
  rs232_udp: [RS232_USB, ETHERNET_UDP] Response via RS232/USB or Ethernet/Udp
 Returns  : -
  ---------------------------------------------------------------------------*/
void i2c_scan(uint8_t ch, bool rs232_udp)
{
	char    s[50]; // needed for printing to serial terminal
	uint8_t x = 0;
	int     i;     // Leave this as an int!
	const uint8_t none[] = "none";
	
	if (i2c_select_channel(ch) == I2C_NACK)
	{
		sprintf(s,"Could not open I2C[%d]\n",ch);
	    rs232_udp == RS232_USB ? xputs(s) : udp_write((uint8_t *)s,strlen(s));
	} // if
	else
	{
		if (ch == PCA9544_NOCH)
		sprintf(s,"I2C[-]: ");
		else sprintf(s,"I2C[%1d]: ",ch-PCA9544_CH0);
	    rs232_udp == RS232_USB ? xputs(s) : udp_write((uint8_t *)s,strlen(s));
		for (i = 0x00; i < 0xff; i+=2)
		{
			if (i2c_start(i) == I2C_ACK)
			{
				if ((ch == PCA9544_NOCH) || ((ch != PCA9544_NOCH) && (i != PCA9544)))
				{
					sprintf(s,"0x%0x ",i);
		  		    rs232_udp == RS232_USB ? xputs(s) : udp_write((uint8_t *)s,strlen(s));
					x++;
				} // if
			} // if
			i2c_stop();
		} // for
		if (!x) 
		{
		  	rs232_udp == RS232_USB ? xputs(none) : udp_write(none,strlen(none));
		} // if			
	  	rs232_udp == RS232_USB ? xputs("\n") : udp_write("\n",1);
	} // else	
} // i2c_scan()

/*-----------------------------------------------------------------------------
  Purpose  : Process PWM signal for all system-modes
             MODULATING GAS BURNER:     the HEATER bit energizes the gas-valve 
										and the PWM signal is generated by a timer.
		     NON MODULATING GAS BURNER: ON/OFF signal (bit NON_MOD) controlled
			                            by a RELAY that is used to switch 24 VAC.
			 ELECTRICAL HEATING:        The HEATER Triac is switched with a 5 sec.
			                            period, the PWM signal is converted into
										a time-division signal of 100 * 50 msec.
  Variables: pwm: the PWM signal [0%..100%]
  Returns  : -
  ---------------------------------------------------------------------------*/
void process_pwm_signal(uint8_t pwm)
{
	switch (system_mode)
	{
		case GAS_MODULATING: // Modulating gas-burner
							 if (PORTD & HEATER_LED)
							 {   // HEATER is ON
								 if (pwm < gas_mod_pwm_llimit)
								 {   // set HEATER and HEATER_LED off
									 PORTD &= ~(HEATER | HEATER_LED);
								 } // if
								 // else do nothing (hysteresis)
							 } // if
							 else
							 {	 // HEATER is OFF					 
								 if (pwm > gas_mod_pwm_hlimit)
								 {   // set HEATER and HEATER_LED on
									 PORTD |= (HEATER | HEATER_LED);
								 } // if
								 // else do nothing (hysteresis)
							 } // else
							 pwm_write(pwm); // write PWM value to Timer register
		                     break;
							 
		case GAS_NON_MODULATING: // Non-Modulating gas-burner
							 if (PORTD & HEATER_LED)
							 {   // HEATER is ON
								 if (pwm < gas_non_mod_llimit)
								 {   // set RELAY and HEATER_LED off
									 PORTD &= ~(NON_MOD | HEATER_LED);
								 } // if
								 // else do nothing (hysteresis)
							 } // if
							 else
							 {	 // HEATER is OFF
								 if (pwm > gas_non_mod_hlimit)
								 {   // set RELAY and HEATER_LED on
									 PORTD |= (NON_MOD | HEATER_LED);
								 } // if
								 // else do nothing (hysteresis)
							 } // else
		                     break;

		case ELECTRICAL_HEATING: // Electrical heating
		                     ATOMIC_BLOCK(ATOMIC_FORCEON)
							 {  // set values for pwm_2_time() task
								tmr_on_val  = pwm;
								tmr_off_val = 100 - tmr_on_val;
							 } 	// ATOMIC_BLOCK						 
							 break;

		default: // This should not happen
							 break;
	} // switch
} // process_pwm_signal()

/*-----------------------------------------------------------------------------
  Purpose  : Non-blocking RS232 command-handler via the USB port
  Variables: -
  Returns  : [NO_ERR, ERR_CMD, ERR_NUM, ERR_I2C]
  ---------------------------------------------------------------------------*/
uint8_t rs232_command_handler(void)
{
  char    ch;
  static uint8_t cmd_rcvd = 0;
  
  if (!cmd_rcvd && usart_kbhit())
  { // A new character has been received
    ch = tolower(usart_getc()); // get character as lowercase
	switch (ch)
	{
		case '\r': break;
		case '\n': cmd_rcvd  = 1;
		           rs232_inbuf[rs232_ptr] = '\0';
		           rs232_ptr = 0;
				   break;
		default  : rs232_inbuf[rs232_ptr++] = ch;
				   break;
	} // switch
  } // if
  if (cmd_rcvd)
  {
	  cmd_rcvd = 0;
	  return execute_single_command(rs232_inbuf, RS232_USB);
  } // if
  else return NO_ERR;
} // rs232_command_handler()

/*-----------------------------------------------------------------------------
  Purpose  : Non-blocking command-handler via the Ethernet UDP port
  Variables: -
  Returns  : [NO_ERR, ERR_CMD, ERR_NUM, ERR_I2C]
  ---------------------------------------------------------------------------*/
uint8_t ethernet_command_handler(char *s)
{
  uint8_t rval = NO_ERR;
  char    *s1;
  char    s2[20];
  uint8_t i, cnt = 1;
  
  s1 = strtok(s,"\n"); // get the first command
  while (s1 != NULL)
  {   // walk through other commands
	  for (i = 0; i < strlen(s1); i++) s1[i] = tolower(s1[i]);
	  sprintf(s2,"eth%1d=[%s]\n",cnt++,s1); 
	  xputs(s2);
	  rval = execute_single_command(s1, ETHERNET_UDP);
	  s1 = strtok(NULL, "\n");
  } // while
  return rval;
} // ethernet_command_handler()

/*-----------------------------------------------------------------------------
  Purpose  : Receive a value from the command-line and assign this value to 
             the proper parameter.
  Variables: num: the parameter number
             val: the value for the parameter
  Returns  : [NO_ERR, ERR_NUM]
  ---------------------------------------------------------------------------*/
uint8_t set_parameter(uint8_t num, uint16_t val)
{
	uint8_t rval = NO_ERR;
	
	switch (num)
	{
		case 0:  // Ebrew System-Mode
				if (val > 2) rval = ERR_NUM;
				else 
				{   // [GAS_MODULATING, GAS_NON_MODULATING, ELECTRICAL_HEATING]
					system_mode = val;
				} // else
				break;
		case 1:  // non-modulating gas valve: hysteresis lower-limit [%]
				gas_non_mod_llimit = val;
				break;
		case 2:  // non-modulating gas valve: hysteresis upper-limit [%]
				gas_non_mod_hlimit = val;
				break;
		case 3:  // Modulating gas-valve Hysteresis lower-limit [%]
				gas_mod_pwm_llimit = val;
				break;
		case 4:  // Modulating gas-valve Hysteresis upper-limit [%]
				gas_mod_pwm_hlimit = val;
				break;
		case 5:  // Lower-limit for switching of Electrical Heating (triac_too_hot) [°C]
				triac_llimit = val;
				break;
		case 6:  // Upper-limit for switching of Electrical Heating (triac_too_hot) [°C]
				triac_hlimit = val;
				break;
		case 7:  // Offset to add to HLT Volume measurement [E-1 L]
				vhlt_offset_10 = val;
				break;
		case 8:  // Max. volume of HLT kettle [E-1 L]
				vhlt_max_10 = val;
				break;
		case 9:  // Slope-Limiter for HLT Volume measurement [E-1 L/sec.]
				vhlt_slope_10 = val;
				break;
		case 10: // Offset to add to MLT Volume measurement [E-1 L]
				vmlt_offset_10 = val;
				break;
		case 11: // Max. volume of MLT kettle [E-1 L]
				vmlt_max_10 = val;
				break;
		case 12: // Slope-Limiter for MLT Volume measurement [E-1 L/sec.]
				vmlt_slope_10 = val;
				break;
		case 13: // Offset (Q8.4) to add to HLT Temp. measurement [°C/16]
				thlt_offset_16 = val;
				break;
		case 14: // Slope Limiter (Q8.4) for HLT Temp. measurement [°C/(16.sec.)]
				thlt_slope_16 = val;
				break;
		case 15: // Offset (Q8.4) to add to MLT Temp. measurement [°C/16]
				tmlt_offset_16 = val;
				break;
		case 16: // Slope Limiter (Q8.4) for MLT Temp. measurement [°C/(16.sec.)]
				tmlt_slope_16 = val;
				break;
		default: break;
	} // switch
	return rval;
} // set_parameter()

/*-----------------------------------------------------------------------------
  Purpose: interpret commands which are received via the USB serial terminal:
   - A0           : Read Analog value: LM35 temperature sensor
   - A1 / A2      : Read Analog value: VHLT / VMLT Volumes
   - A3 / A4      : Read Temperature sensor LM92: THLT / TMLT temperature
   - A5           : Read flow sensor connected between HLT to MLT
   - A6           : Read flow sensor connected between MLT to boil-kettle

   - E0 / E1      : Ethernet with WIZ550io Disabled / Enabled

   - L0 / L1      : ALIVE Led OFF / ON

   - N0           : System-Mode: 0=Modulating, 1=Non-Modulating, 2=Electrical
     N1 / N2      : Hysteresis Lower-Limit / Upper-Limit for Non-Modulating gas-valve
     N3 / N4      : Hysteresis Lower-Limit / Upper-Limit for Electrical heating

   - P0 / P1      : set Pump OFF / ON

   - S0           : Ebrew hardware revision number
	 S1			  : List value of parameters that can be set with Nx command
	 S2           : List all connected I2C devices  
	 S3           : List all tasks
	 	
   - V0...V255    : Output bits for valves V1 until V8
   
   - W0...W100    : PID-output, needed for:
			        - PWM output for modulating gas-valve (N0=0)
				    - Time-Division ON/OFF signal for Non-Modulating gas-valve (N0=1)
				    - Time-Division ON/OFF signal for Electrical heating-element (N0=2)
   
  Variables: 
          s: the string that contains the command from RS232 serial port 0
  rs232_udp: [RS232_USB, ETHERNET_UDP] Response via RS232/USB or Ethernet/Udp
  Returns  : [NO_ERR, ERR_CMD, ERR_NUM, ERR_I2C] or ack. value for command
             cmd ack   cmd ack   cmd ack   cmd ack   cmd      ack
			 A0  33     L0  40    N0  43    P0  65   W0..W100 75
			 A1  34     L1  41    ..  ..    P1  66   E0       76
			 A2  35     N0  42    N16 59    S0  67   E1       77
			 A3  36     N1  43              S1  68   Vx       78 
			 A4  37     N2  44              S2  69
			 A5  38     N3  45              S3  70
			 A6  39     N4  46
  ---------------------------------------------------------------------------*/
uint8_t execute_single_command(char *s, bool rs232_udp)
{
   uint8_t  num  = atoi(&s[1]); // convert number in command (until space is found)
   uint8_t  rval = NO_ERR, err;
   uint16_t temp, frac_16;
   char     s2[40]; // Used for printing to RS232 port
   
   switch (s[0])
   {
	   case 'a': // Read analog (LM35, VHLT, VMLT) + digital (THLT, TMLT) values
				 if (rs232_udp == ETHERNET_UDP) 
				 {
					 udp_beginPacketIP(remoteIP, localPort); // send response back
				 } // if				 
			     rval = 33 + num;
				 switch (num)
				 {
				    case 0: // LM35. Processing is done by lm35_task()
							temp = lm35_temp / 100;
							sprintf(s2,"Lm35=%d.%02d\n",temp,lm35_temp-100*temp);
							break;
					case 1: // VHLT. Processing is done by vhlt_task()
							temp = vhlt_10 / 10;
							sprintf(s2,"Vhlt=%d.%1d\n",temp,vhlt_10-10*temp);
							break;
					case 2: // VMLT. Processing is done by vmlt_task()
							temp = vmlt_10 / 10;
							sprintf(s2,"Vmlt=%d.%1d\n",temp,vmlt_10-10*temp);
							break;
					case 3: // THLT. Processing is done by thlt_task()
							if (thlt_err)
							{
								sprintf(s2,"Thlt=99.99\n");
							}
							else
							{
								temp     = thlt_temp_16 >> 4;     // The integer part of THLT
								frac_16  = thlt_temp_16 & 0x000f; // The fractional part of THLT
								frac_16 *= 50;                    // 100 / 16 = 50 / 8
								frac_16 +=  4;                    // 0.5 for rounding
								frac_16 >>= 3;                    // SHR 3 = divide by 8
								sprintf(s2,"Thlt=%d.%02d\n",temp,frac_16);
							} // else							
							break;
					case 4: // TMLT. Processing is done by tmlt_task()
							if (tmlt_err)
							{
								sprintf(s2,"Tmlt=99.99\n");
							}
							else
							{
								temp     = tmlt_temp_16 >> 4;     // The integer part of TMLT
								frac_16  = tmlt_temp_16 & 0x000f; // The fractional part of TMLT
								frac_16 *= 50;                    // 100 / 16 = 50 / 8
								frac_16 +=  4;                    // 0.5 for rounding
								frac_16 >>= 3;                    // SHR 3 = divide by 8
								sprintf(s2,"Tmlt=%d.%02d\n",temp,frac_16);
							} // else							
							break;
					case 5: // Flow-sensor processing is done by ISR routine PCINT1_vect
							temp     = (uint16_t)((flow_hlt_mlt * 100 + FLOW_ROUND_OFF) / FLOW_PER_L); // Flow in E-2 Litres
							frac_16  = temp / 100;
							frac_16  = temp - 100 * frac_16;
							sprintf(s2,"Flow1=%d.%02d\n",temp/100,frac_16);
							break;
					case 6: // Flow-sensor processing is done by ISR routine PCINT2_vect
							temp     = (uint16_t)((flow_mlt_boil * 100 + FLOW_ROUND_OFF) / FLOW_PER_L); // Flow in E-2 Litres
							frac_16  = temp / 100;
							frac_16  = temp - 100 * frac_16;
							sprintf(s2,"Flow2=%d.%02d\n",temp/100,frac_16);
							break;
					default: rval = ERR_NUM;
					         break;
				 } // switch
				 if (rval != ERR_NUM)
				 {		 
					rs232_udp == RS232_USB ? xputs(s2) : udp_write((uint8_t *)s2,strlen(s2));
				 } // if
				 if (rs232_udp == ETHERNET_UDP)
				 {
					 udp_endPacket(); // send response
				 } // if				 
			     break;

	   case 'e': // Enable/Disable Ethernet with WIZ550io
			   if (num > 1) rval = ERR_NUM;
			   else
			   {
				   rval = 76 + num;
				   if (num)
				   {   
					   init_WIZ550IO_module(); // this hangs the system if no WIZ550io is present!!!
					   ethernet_WIZ550i = true;
				   }				   	
				   else	ethernet_WIZ550i = false;
			   } // else
			   break;

	   case 'l': // ALIVE-Led
				 if (num > 1) rval = ERR_NUM;
				 else
			  	 {
					 rval = 40 + num;
					 if (num) PORTD |=  ALIVE_LED;
					 else     PORTD &= ~ALIVE_LED;
				 } // else
				 break;

	   case 'n': // Set parameters / variables to a new value
				 if (((num >  9) && ((s[3] != ' ') || (strlen(s) < 5))) ||
				     ((num < 10) && ((s[2] != ' ') || (strlen(s) < 4))))
				 {  // check for error in command: 'nx yy' or 'nxx yy'
					rval = ERR_CMD; 
				 } // if				 
	             else if (num > 16)
				 {
					 rval = ERR_NUM;
				 } // else if
				 else
				 {
					temp = atoi(&s[(num > 9) ? 3 : 2]); // convert to number
					err  = set_parameter(num,temp);     // set parameter to a new value
					if (err != NO_ERR) rval = err;
					else
					{
						rval = 43 + num;
					} // else
				 } // else
	             break;

	   case 'p': // Pump
	             rval = 65 + num;
				 if (num > 1) rval = ERR_NUM;
				 else 
				 {
					 if (num == 0) PORTD &= ~(PUMP);
					 else          PORTD |=  (PUMP);
				 } // else				 
	             break;

	   case 's': // System commands
				 if (rs232_udp == ETHERNET_UDP)
				 {
					 udp_beginPacketIP(remoteIP, localPort); // send response back
				 } // if
	             rval = 67 + num;
				 switch (num)
				 {
					 case 0: // Ebrew revision
							 print_ebrew_revision(s2); // print CVS revision number
				             rs232_udp == RS232_USB ? xputs(s2) : udp_write((uint8_t *)s2,strlen(s2));
							 break;
					 case 1: // List parameters
							 sprintf(s2,"SYS:%01d,%d,%d,%d,%d,%d,%d\n",system_mode, 
					                    gas_non_mod_llimit, gas_non_mod_hlimit,
					                    gas_mod_pwm_llimit, gas_mod_pwm_hlimit,
										triac_llimit, triac_hlimit);
				             rs232_udp == RS232_USB ? xputs(s2) : udp_write((uint8_t *)s2,strlen(s2));
							 sprintf(s2,"VHLT:o=%d,m=%d,s=%d\n",
							             vhlt_offset_10,vhlt_max_10,vhlt_slope_10); 
				             rs232_udp == RS232_USB ? xputs(s2) : udp_write((uint8_t *)s2,strlen(s2));
							 sprintf(s2,"VMLT:o=%d,m=%d,s=%d\n",
							             vmlt_offset_10,vmlt_max_10,vmlt_slope_10); 
				             rs232_udp == RS232_USB ? xputs(s2) : udp_write((uint8_t *)s2,strlen(s2));
							 sprintf(s2,"TxLT:o=%d,s=%d,o=%d,s=%d\n",
									     thlt_offset_16,thlt_slope_16,
										 tmlt_offset_16,tmlt_slope_16);
				             rs232_udp == RS232_USB ? xputs(s2) : udp_write((uint8_t *)s2,strlen(s2));
							 break;
					 case 2: // List all I2C devices
					         i2c_scan(PCA9544_NOCH, rs232_udp); // Start with main I2C channel
					         i2c_scan(PCA9544_CH0 , rs232_udp);  // PCA9544 channel 0
					         i2c_scan(PCA9544_CH1 , rs232_udp);  // PCA9544 channel 1
					         i2c_scan(PCA9544_CH2 , rs232_udp);  // PCA9544 channel 2
					         i2c_scan(PCA9544_CH3 , rs232_udp);  // PCA9544 channel 3
							 break;
					 case 3: // List all tasks
							 list_all_tasks(rs232_udp); 
							 break;				 
					 default: rval = ERR_NUM;
							  break;
				 } // switch
				 if (rs232_udp == ETHERNET_UDP)
				 {
					 udp_endPacket(); // send response
				 } // if
				 break;

	   case 'v': // Output Valve On-Off signals to MCP23017 16-bit IO
			   rval = 78;
			   mcp23017_write(OLATA,num); // write valve bits to IO-expander PORTA
			   break;

	   case 'w': // PWM signal for Modulating Gas-Burner
	             rval = 75 + num;
				 if (num > 100) 
				      rval = ERR_NUM;
				 else 
				 {
					 process_pwm_signal(num);
				 } // else				 
	             break;

	   default: rval = ERR_CMD;
	            break;
   } // switch
   return rval;	
} // execute_single_command()