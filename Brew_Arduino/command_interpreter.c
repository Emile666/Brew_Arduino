//-----------------------------------------------------------------------------
// Created: 22-4-2013 07:26:24
// Author : Emile
// File   : command_interpreter.c
//-----------------------------------------------------------------------------
// $Log$
// Revision 1.23  2016/05/22 12:14:58  Emile
// - Baud-rate to 38400 Baud.
// - Temperature error value now set to '-99.99'.
//
// Revision 1.22  2016/05/15 12:24:20  Emile
// - I2C clock speed now adjustable
// - IP address and port now stored in eeprom
//
// Revision 1.21  2016/04/17 12:55:32  Emile
// - Version after Integration Testing. Works with ebrew r1.81.
// - Temps (A0) and Flows (A9) now combined instead of separate readings
//
// Revision 1.20  2016/04/16 11:22:58  Emile
// - One temperature slope parameter for all temps. Now fixed value (2 degrees/second).
// - Temp. Offset parameters removed.
// - All parameters > 12 removed from parameter list. Now only pars 1-6 left.
// - Tasks for CFC and Boil Temperature added.
//
// Revision 1.19  2016/01/10 16:00:24  Emile
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
// Revision 1.18  2015/08/06 14:41:16  Emile
// - Adapted for MCP23008 instead of MCP23017.
//
// Revision 1.17  2015/07/01 21:03:46  Emile
// - Bug-fix in scheduler time-measurement. Now reads proper time in msec
// - Usart comm. now IRQ driven, so that all receiving commands are handled
// - DS18B20 reads only 2 bytes (instead of 9). Total time taken is now 28 msec.
//   This was 60 msec. and caused multiple reads at PC side.
//
// Revision 1.16  2015/06/28 12:27:35  Emile
// - Moving_average filters now work with Q8.7 instead of Q8.4 format
// - One-wire functions now work with DS18B20
// - Separate ow_task() added for one-wire communication
// - I2C clock made adjustable
//
// Revision 1.15  2015/05/31 10:28:57  Emile
// - Bugfix: Flowsensor reading counted rising and falling edges.
// - Bugfix: Only valve V8 is written to at init (instead of all valves).
//
// Revision 1.14  2014/11/30 20:44:45  Emile
// - Vxxx command added to write valve output bits
// - mcp23017 (16 bit I2C IO-expander) routines + defines added
//
// Revision 1.13  2014/11/09 15:38:34  Emile
// - PUMP_LED removed from PD2, PUMP_230V has same function
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
#include "one_wire.h"
#include "eep.h"

extern uint8_t      localIP[4];  // local IP address
extern uint8_t      remoteIP[4]; // remote IP address for the incoming packet whilst it's being processed
extern unsigned int localPort;   // local port to listen on 	
extern uint8_t      ROM_NO[];    // One-wire hex-address
extern uint8_t      crc8;

extern uint8_t    system_mode;         // from Brew_Arduino.c
extern bool       ethernet_WIZ550i;
extern const char *ebrew_revision;     // ebrew CVS revision number
extern uint8_t    gas_non_mod_llimit; 
extern uint8_t    gas_non_mod_hlimit;
extern uint8_t    gas_mod_pwm_llimit;
extern uint8_t    gas_mod_pwm_hlimit;
extern uint8_t    btmr_on_val;          // ON-timer  for PWM to Time-Division Boil-kettle
extern uint8_t    btmr_off_val;         // OFF-timer for PWM to Time-Division Boil-kettle
extern uint8_t    htmr_on_val;          // ON-timer  for PWM to Time-Division HLT
extern uint8_t    htmr_off_val;         // OFF-timer for PWM to Time-Division HLT

extern uint16_t   lm35_temp;           // LM35 Temperature in E-2 °C
extern uint16_t   triac_llimit;        // Hysteresis lower-limit for triac_too_hot in E-2 °C
extern uint16_t   triac_hlimit;	       // Hysteresis upper-limit for triac_too_hot in E-2 °C

//------------------------------------------------------
// The extension _87 indicates a signed Q8.7 format!
// This is used for both the HLT and MLT temperatures
//------------------------------------------------------
extern int16_t    thlt_temp_87;        // THLT Temperature in °C * 128
extern uint8_t    thlt_err;

extern int16_t    tmlt_temp_87;        // TMLT Temperature in °C * 128
extern uint8_t    tmlt_err;

extern int16_t    tcfc_temp_87;        // TCFC Temperature in °C * 128
extern uint8_t    tcfc_err;

extern int16_t    tboil_temp_87;        // TBOIL Temperature in °C * 128
extern uint8_t    tboil_err;

extern unsigned long flow_hlt_mlt;     // Count from flow-sensor between HLT and MLT
extern unsigned long flow_mlt_boil;    // Count from flow-sensor between MLT and boil-kettle
extern unsigned long flow_cfc_out;     // Count from flow-sensor at output of CFC
extern unsigned long flow4;            // Count from FLOW4 (future use)

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
	const uint8_t none[] = "-";
	
	if (i2c_select_channel(ch, HSPEED) == I2C_NACK)
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
	  //sprintf(s2,"eth%1d=[%s]\n",cnt++,s1); 
	  //xputs(s2);
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
		default: break;
	} // switch
	return rval;
} // set_parameter()

void find_OW_device(uint8_t i2c_addr)
{
    char    s2[40]; // Used for printing to RS232 port
	uint8_t i,rval;
	
	rval = OW_first(i2c_addr); // Find ROM ID of first DS18B20
	if (rval) 
	{
		for (i= 0; i < 8; i++)
		{
			sprintf(s2,"%02X ",ROM_NO[i]); // global array
			xputs(s2);
		} // for								 
	} // if
	else xputs("-");
	xputs("\n");							 
} // find_OW_device()

/*-----------------------------------------------------------------------------
  Purpose  : Process PWM signal for all system-modes. 
             Executed when a Bxxx or Hxxx command is received.
             MODULATING GAS BURNER:     the 230V-signal energizes the gas-valve 
										and the PWM signal is generated by a timer.
		     NON MODULATING GAS BURNER: ON/OFF signal (bits HLT_NMOD and BOIL_NMOD) 
			                            controlled by a RELAY that is used to switch 24 VAC.
			 ELECTRICAL HEATING:        The 230V signal for the Triac is switched 
										with a 5 sec. period, the PWM signal is 
										converted into a time-division signal of 
										100 * 50 msec.
  Variables: 
     pwm_ch: [PWMB,PWMH]. Selects the PWM channel (HLT or Boil-kettle)
	pwm_val: the PWM signal [0%..100%]
  Returns  : -
  ---------------------------------------------------------------------------*/
void process_pwm_signal(uint8_t pwm_ch, uint8_t pwm_val)
{
	uint8_t portb = mcp230xx_read(GPIOB);
	uint8_t mod_mask, nmod_mask;
	
	if (pwm_ch == PWMB)
	{   // boil-kettle PWM
		mod_mask  = BOIL_230V;
		nmod_mask = BOIL_NMOD;
	}
	else
	{   // HLT PWM
		mod_mask  = HLT_230V;
		nmod_mask = HLT_NMOD;
	} // else
	
	switch (system_mode)
	{
		case GAS_MODULATING: // Modulating gas-burner
							 if (portb & mod_mask)
							 {   // 230V-signal is ON
								 if (pwm_val < gas_mod_pwm_llimit)
								 {   // set 230V-signal off
									 portb &= ~mod_mask;
								 } // if
								 // else do nothing (hysteresis)
							 } // if
							 else
							 {	 // 230V-signal is OFF					 
								 if (pwm_val > gas_mod_pwm_hlimit)
								 {   // set 230V-signal on
									 portb |= mod_mask;
								 } // if
								 // else do nothing (hysteresis)
							 } // else
							 pwm_write(pwm_ch, pwm_val);  // write PWM value to Timer register
							 mcp230xx_write(OLATB,portb); // write updated bit-values back to MCP23017 PORTB
		                     break;
							 
		case GAS_NON_MODULATING: // Non-Modulating gas-burner
							 if (portb & nmod_mask)
							 {   // 24V-relay is ON
								 if (pwm_val < gas_non_mod_llimit)
								 {   // set RELAY off
									 portb &= ~nmod_mask;
								 } // if
								 // else do nothing (hysteresis)
							 } // if
							 else
							 {	 // 24V-relay is OFF
								 if (pwm_val > gas_non_mod_hlimit)
								 {   // set RELAY on
									 portb |= nmod_mask;
								 } // if
								 // else do nothing (hysteresis)
							 } // else
							 mcp230xx_write(OLATB,portb); // write updated bit-values back to MCP23017 PORTB
		                     break;

		case ELECTRICAL_HEATING: // Electrical heating
		                     ATOMIC_BLOCK(ATOMIC_FORCEON)
							 {  // set values for pwm_task() / pwm_2_time()
								if (pwm_ch == PWMB)
								{   // boil-kettle PWM
									btmr_on_val  = pwm_val;
									btmr_off_val = 100 - btmr_on_val;
								}
								else
								{   // HLT PWM
									htmr_on_val  = pwm_val;
									htmr_off_val = 100 - htmr_on_val;
								} // else																
							 } 	// ATOMIC_BLOCK						 
							 break;

		default: // This should not happen
							 break;
	} // switch
} // process_pwm_signal()

/*-----------------------------------------------------------------------------
  Purpose  : Builds a string that can be returned via RS232/Ethernet.
  Variables:
		err: 1 = error when reading the temperature sensor
       name: the string with the sensor name. Also used to return the result.
	 val_87: the actual value of the temperature sensor in Q8.7 format
  Returns  : -
  ---------------------------------------------------------------------------*/
void process_temperatures(uint8_t err, char *name, int16_t val_87, uint8_t last)
{
   uint16_t temp, frac_16;
   char     s2[20];
   
   if (err)
   {   // error
	   if (last) strcat(name,"-99.9\n");
	   else      strcat(name,"-99.9,");
   }
   else
   {
	   temp     = val_87 >> 7;     // The integer part of the sensor value
	   frac_16  = val_87 & 0x007f; // The fractional part of the sensor value
	   frac_16 *= 25;              // 100 / 128 = 25 / 32
	   frac_16 += 16;              // 0.5 for rounding
	   frac_16 >>= 5;              // SHR 5 = divide by 32
	   if (last) sprintf(s2,"%d.%02d\n",temp,frac_16);
	   else      sprintf(s2,"%d.%02d," ,temp,frac_16);
	   strcat(name,s2);            // store result back in *name
   } // else
} // process_temperatures()

/*-----------------------------------------------------------------------------
  Purpose  : Builds a string that can be returned via RS232/Ethernet.
  Variables:
  	 flownr: [1..4] Number of flowsensor
       name: the string with the sensor name. Also used to return the result.
  Returns  : -
  ---------------------------------------------------------------------------*/
void process_flows(uint32_t flow_val, char *name, uint8_t last)
{
   uint16_t temp, frac_16;
   char     s2[20];
   
   temp     = (uint16_t)((flow_val * 100 + FLOW_ROUND_OFF) / FLOW_PER_L); // Flow in E-2 Litres
   frac_16  = temp / 100;
   frac_16  = temp - 100 * frac_16;
   if (last)
        sprintf(s2,"%d.%02d\n",temp/100,frac_16);
   else sprintf(s2,"%d.%02d," ,temp/100,frac_16);
   strcat(name,s2);  // store result back in *name
} // process_flows()

/*-----------------------------------------------------------------------------
  Purpose: interpret commands which are received via the USB serial terminal:
   - A0           : Read Temperature sensors: THLT / TMLT / TBOIL / TCFC
   - A9           : Read flow sensors: HLT->MLT, MLT->BOIL, CFC-out, Flow4
   - B0...B100    : PID-output for Boil-kettle, needed for:
				    - PWM output for modulating gas-valve (N0=0)
				    - Time-Division ON/OFF signal for Non-Modulating gas-valve (N0=1)
				    - Time-Division ON/OFF signal for Electrical heating-element (N0=2)
   - E0 / E1      : Ethernet with WIZ550io Disabled / Enabled
   - H0...H100    : PID-output for HLT, needed for:
				    - PWM output for modulating gas-valve (N0=0)
				    - Time-Division ON/OFF signal for Non-Modulating gas-valve (N0=1)
				    - Time-Division ON/OFF signal for Electrical heating-element (N0=2)
   - L0 / L1      : ALIVE Led OFF / ON
   - N0           : System-Mode: 0=Modulating, 1=Non-Modulating, 2=Electrical
     N1..N6       : Parameter settings
   - P0 / P1      : set Pump OFF / ON
   - S0           : Ebrew hardware revision number
	 S1			  : List value of parameters that can be set with Nx command
	 S2           : List all connected I2C devices  
	 S3           : List all tasks
	 S4           : List One-Wire devices
   - V0...V255    : Output bits for valves V1 until V8
 
  Variables: 
          s: the string that contains the command from RS232 serial port 0
  rs232_udp: [RS232_USB, ETHERNET_UDP] Response via RS232/USB or Ethernet/Udp
  Returns  : [NO_ERR, ERR_CMD, ERR_NUM, ERR_I2C] or ack. value for command
  ---------------------------------------------------------------------------*/
uint8_t execute_single_command(char *s, bool rs232_udp)
{
   uint8_t  num  = atoi(&s[1]); // convert number in command (until space is found)
   uint8_t  rval = NO_ERR, err, portb, i;
   uint16_t temp;
   char     s2[40]; // Used for printing to RS232 port
   char     *s1;
   
   switch (s[0])
   {
	   case 'a': // Read Temperatures and Flows
				 if (rs232_udp == ETHERNET_UDP) 
				 {
					 udp_beginPacketIP(remoteIP, localPort); // send response back
				 } // if				 
				 switch (num)
				 {
				    case 0: // Temperature Processing, send all Temperatures to PC
							temp = lm35_temp / 100;
							sprintf(s2,"T=%d.%02d,",temp,lm35_temp-100*temp);   // LM35
							process_temperatures(thlt_err,s2,thlt_temp_87,0);   // Thlt
					        process_temperatures(tmlt_err,s2,tmlt_temp_87,0);   // Tmlt
					        process_temperatures(tboil_err,s2,tboil_temp_87,0); // Tboil
					        process_temperatures(tcfc_err,s2,tcfc_temp_87,1);   // Tcfc
							break;
					case 9: // FLOW Processing, send all flows to PC
					        strcpy(s2,"F=");
							process_flows(flow_hlt_mlt ,s2,0);
							process_flows(flow_mlt_boil,s2,0);
							process_flows(flow_cfc_out ,s2,0);
							process_flows(flow4        ,s2,1);
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

	   case 'b': // PWM signal for Boil-Kettle Modulating Gas-Burner
				if (num > 100) 
					 rval = ERR_NUM;
				else process_pwm_signal(PWMB,num);
				break;

	   case 'e': // Enable/Disable Ethernet with WIZ550io
			   if (num < 2)
			   {
				   if (num) ethernet_WIZ550i = true;
				   else     ethernet_WIZ550i = false;
				   write_eeprom_parameters(); // save value in eeprom
				   if (ethernet_WIZ550i) init_WIZ550IO_module(); // this hangs the system if no WIZ550io is present!!!
			   }
			   else if (num == 2) 
			   {   // IP address, e.g. "192.168.1.177:8888"
				   s1 = strtok(&s[3],"."); localIP[0] = atoi(s1);
				   for (i = 1; i < 4; i++) 
				   {
					   s1 = strtok(NULL ,".:"); 
					   localIP[i] = atoi(s1);
				   } // for i				
				   s1 = strtok(NULL,":");
				   localPort = atoi(s1);   
				   write_eeprom_parameters(); // save IP address and Port in eeprom
				   sprintf(s2,"IP:%d.%d.%d.%d:%d\n",localIP[0],localIP[1],localIP[2],localIP[3],localPort);
				   xputs(s2);
			   } // else if
			   else rval = ERR_NUM;
			   break;

	   case 'h': // PWM signal for HLT Modulating Gas-Burner
			   if (num > 100)
					rval = ERR_NUM;
			   else process_pwm_signal(PWMH,num);
			   break;

	   case 'l': // ALIVE-Led
				 if (num > 1) rval = ERR_NUM;
				 else
			  	 {
					 if (num) PORTD |=  ALIVE_LED;
					 else     PORTD &= ~ALIVE_LED;
				 } // else
				 break;

	   case 'n': // Set parameters / variables to a new value
				 if ((s[2] != ' ') || (strlen(s) < 4))
				 {  // check for error in command: 'nx yy'
					rval = ERR_CMD; 
				 } // if				 
	             else if (num > 6)
				 {
					 rval = ERR_NUM;
				 } // else if
				 else
				 {
					temp = atoi(&s[2]);             // convert to number
					err  = set_parameter(num,temp); // set parameter to a new value
					if (err != NO_ERR) rval = err;
				 } // else
	             break;

	   case 'p': // Pump
				 if (num > 1) rval = ERR_NUM;
				 else 
				 {
					 portb = mcp230xx_read(GPIOB);
					 if (num == 0) portb &= ~PUMP_230V;
					 else          portb |=  PUMP_230V;
					 mcp230xx_write(GPIOB, portb);
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
					 case 4: // List all One-Wire Devices (finding a sensor costs approx. 350 msec.)
							 find_OW_device(DS2482_THLT_BASE);  // Find ROM ID of HLT  DS18B20
							 find_OW_device(DS2482_TBOIL_BASE); // Find ROM ID of BOIL DS18B20
							 find_OW_device(DS2482_TCFC_BASE);  // Find ROM ID of CFC  DS18B20
							 find_OW_device(DS2482_TMLT_BASE);  // Find ROM ID of MLT  DS18B20
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
			   mcp230xx_write(OLATA,num); // write valve bits to IO-expander PORT
			   break;

	   default: rval = ERR_CMD;
				sprintf(s2,"ERR.CMD[%s]\n",s);
				xputs(s2);
	            break;
   } // switch
   return rval;	
} // execute_single_command()