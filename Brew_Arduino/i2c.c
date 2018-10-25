/*==================================================================
  File Name    : i2c.c
  Author       : Peter Fleury <pfleury@gmx.ch> http://jump.to/fleury
                 other i2c-routines: Emile
  ------------------------------------------------------------------
  Purpose : I2C master library using hardware TWI interface
  ================================================================== */ 
#include <inttypes.h>
#include <compat/twi.h>
#include "i2c.h"
#include "delay.h"         /* for delay_msec() */

/*************************************************************************
 Initialization of the I2C bus interface. 
*************************************************************************/
void i2c_init(uint8_t clk)
{
  /* initialize TWI clock: TWPS = 0x01 => prescaler = 4 */
  
  if (clk == SCL_CLK_400KHZ) TWSR = 0x00; /* pre-scaler = 1 */
  else                       TWSR = 0x01; /* pre-scaler = 4 */
  TWBR = clk;  /* must be > 10 for stable operation */
} /* i2c_init() */

/*************************************************************************	
  Issues a start condition and sends address and transfer direction.
  return:  I2C_ACK : device accessible
           I2C_NACK: failed to access device
*************************************************************************/
enum i2c_acks i2c_start(unsigned char address)
{
    uint8_t   twst;
    uint8_t   retries = 0;

	// send START condition
	TWCR = (1<<TWINT) | (1<<TWSTA) | (1<<TWEN);
	delay_usec(30); // At 50 (400) kHz this takes approx. 200 (25) usec.
	
	// wait until transmission completed
	while ((!(TWCR & (1<<TWINT))) && (retries < I2C_RETRIES))
	{	
		delay_usec(200);
		retries++;
	} // while	

	// check value of TWI Status Register. Mask pre-scaler bits.
	twst = TW_STATUS & 0xF8;
	if (((twst != TW_START) && (twst != TW_REP_START)) || (retries >= I2C_RETRIES)) 
		return I2C_NACK;

	// send device address
	TWDR = address;
	TWCR = (1<<TWINT) | (1<<TWEN);

	// wail until transmission completed and ACK/NACK has been received
	while(!(TWCR & (1<<TWINT)));

	// check value of TWI Status Register. Mask pre-scaler bits.
	twst = TW_STATUS & 0xF8;
	if ( (twst != TW_MT_SLA_ACK) && (twst != TW_MR_SLA_ACK) ) return I2C_NACK;

	return I2C_ACK;
} /* i2c_start() */

/*************************************************************************
 Issues a start condition and sends address and transfer direction.
 If device is busy, use ACK polling to wait until device is ready
 
 Input:   address and transfer direction of I2C device
*************************************************************************/
void i2c_start_wait(unsigned char address)
{
    uint8_t   twst;

    while ( 1 )
    {
	    // send START condition
	    TWCR = (1<<TWINT) | (1<<TWSTA) | (1<<TWEN);
    
    	// wait until transmission completed
    	while(!(TWCR & (1<<TWINT)));
    
    	// check value of TWI Status Register. Mask pre-scaler bits.
    	twst = TW_STATUS & 0xF8;
    	if ( (twst != TW_START) && (twst != TW_REP_START)) continue;
    
    	// send device address
    	TWDR = address;
    	TWCR = (1<<TWINT) | (1<<TWEN);
    
    	// wail until transmission completed
    	while(!(TWCR & (1<<TWINT)));
    
    	// check value of TWI Status Register. Mask pre-scaler bits.
    	twst = TW_STATUS & 0xF8;
    	if ( (twst == TW_MT_SLA_NACK )||(twst ==TW_MR_DATA_NACK) ) 
    	{    	    
    	    /* device busy, send stop condition to terminate write operation */
	        TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWSTO);
	        
	        // wait until stop condition is executed and bus released
	        while(TWCR & (1<<TWSTO));
	        
    	    continue;
    	} // if
    	//if( twst != TW_MT_SLA_ACK) return 1;
    	break;
     } // while
} /* i2c_start_wait() */

/*************************************************************************
 Issues a repeated start condition and sends address and transfer direction 

 Input:   address and transfer direction of I2C device
 
 Return:  I2C_ACK : device accessible
          I2C_NACK: failed to access device
*************************************************************************/
enum i2c_acks i2c_rep_start(unsigned char address)
{
    return i2c_start( address );
} /* i2c_rep_start() */

/*************************************************************************
 Terminates the data transfer and releases the I2C bus
*************************************************************************/
void i2c_stop(void)
{
    /* send stop condition */
	TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWSTO);
	
	// wait until stop condition is executed and bus released
	while(TWCR & (1<<TWSTO));
} /* i2c_stop() */

/*************************************************************************
  Send one byte to I2C device
  
  Input:    byte to be transferred
  Return:   I2C_ACK : write successful 
            I2C_NACK: write failed
*************************************************************************/
enum i2c_acks i2c_write( unsigned char data )
{	
    uint8_t   twst;
    
	// send data to the previously addressed device
	TWDR = data;
	TWCR = (1<<TWINT) | (1<<TWEN);

	// wait until transmission completed
	while(!(TWCR & (1<<TWINT)));

	// check value of TWI Status Register. Mask pre-scaler bits
	twst = TW_STATUS & 0xF8;
	if( twst != TW_MT_DATA_ACK) return I2C_NACK;
	return I2C_ACK;
} /* i2c_write() */

/*************************************************************************
 Read one byte from the I2C device, request more data from device 
 
 Return:  byte read from I2C device
*************************************************************************/
unsigned char i2c_readAck(void)
{
	TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWEA);
	while(!(TWCR & (1<<TWINT)));    

    return TWDR;
} /* i2c_readAck() */

/*************************************************************************
 Read one byte from the I2C device, read is followed by a stop condition 
 
 Return:  byte read from I2C device
*************************************************************************/
unsigned char i2c_readNak(void)
{
	TWCR = (1<<TWINT) | (1<<TWEN);
	while(!(TWCR & (1<<TWINT)));
	
    return TWDR;
} /* i2c_readNak() */

//---------------------------------------------------------------------
// The PCA9544 IC is an I2C Channel Multiplexer. Prior to addressing an
// individual I2C IC, the proper PCA9544 channel should be selected.
// Variables:
//  ch       : [PCA9544_CH0, PCA9544_CH1, PCA9544_CH2, PCA9544_CH3]
//  Returns  : I2C_ACK, I2C_NACK
//---------------------------------------------------------------------
enum i2c_acks i2c_select_channel(uint8_t ch, uint8_t speed)
{
   if (speed == HSPEED) 
        i2c_init(SCL_CLK_400KHZ); // I2C-clock is 400 kHz
   else i2c_init(SCL_CLK_50KHZ);  // I2C-clock is  50 kHz

   if (i2c_start(PCA9544) == I2C_NACK) // generate I2C start + output address to I2C bus
   {
	   return I2C_NACK; // NACK received, return error code
   }
   i2c_write(ch);       // Set proper PCA9544 channel
   i2c_stop();			// Close I2C bus
   return I2C_ACK;      // All is well!
}; // i2c_select_channel()

int16_t lm92_read(uint8_t dvc, uint8_t *err)
/*------------------------------------------------------------------
  Purpose  : This function reads the LM92 13-bit Temp. Sensor and
             returns the temperature.
             Reading Register 0 of the LM92 results in the following bits:
              15   14  13 12      3   2    1   0
             Sign MSB B10 B9 ... B0 Crit High Low
  Variables:
       dvc : THLT = Read from the LM92 at 0x92/0x93 (This is the HLT Temp.)
             TMLT = Read from the LM92 at 0x94/0x95 (This is the MLT Temp.)
  Returns  : The temperature from the LM92 in a signed Q8.7 format.
             Q8.7 is chosen here for accuracy reasons when filtering.
  ------------------------------------------------------------------*/
{
   uint8_t  buffer[2]; // array to store data from i2c_read()
   uint16_t temp_int;  // the temp. from the LM92 as an integer
   uint8_t  sign;      // sign of temperature
   int16_t  temp = 0;  // the temp. from the LM92 as an int
   uint8_t  adr;       // i2c address
       
   // Start with selecting the proper channel on the PCA9544
   *err = FALSE;	
   if (dvc == THLT)
   {
	  *err = (i2c_select_channel(THLT_I2C_CH, LSPEED) != I2C_ACK);
   } // if
   else if (dvc == TMLT)
   {
	  *err = (i2c_select_channel(TMLT_I2C_CH, LSPEED) != I2C_ACK);
   } // else if
   else *err = TRUE;
   
   if (!*err) 
   {
       *err = 1;                      // assume no I2C device found 
	   adr  = LM92_0_BASE | I2C_READ; // First possible LM92 address
       while (*err && (adr <= LM92_3_BASE+1))
	   {
	      *err = (i2c_start(adr) == I2C_NACK); // generate I2C start + output address to I2C bus
		  if (*err) adr += 2;                  // no device found, try next possible I2C address 
	   }; // while
   } // if
   // adr contains address of LM92 found or *err is true (no LM92 present)     
   if (!*err)	
   {
      buffer[0] = i2c_readAck();		// Read 1st byte, request for more
      buffer[1] = i2c_readNak();		// Read 2nd byte, generate I2C stop condition
      temp_int = buffer[0] & 0x00FF;    // store {Sign, MSB, bit 10..5} at bits temp_int bits 7..0
      temp_int <<= 8;                   // SHL 8, Sign now at bit 15
      temp_int &= 0xFF00;               // Clear bits 7..0
      temp_int |= (buffer[1] & 0x00FF); // Add bits D4..D0 to temp_int bits 7..3
      temp_int &= 0xFFF8;               // Clear Crit High & Low bits
      sign = ((temp_int & LM92_SIGNb) == LM92_SIGNb);
      if (sign)
      {
         temp_int &= ~LM92_SIGNb;        // Clear sign bit
         temp_int  = LM92_FS - temp_int; // Convert two complement number
      } // if
      //temp = temp_int >> 3;
	  temp = temp_int; // without shifting! Returns Q8.7 format
      if (sign)
      {
         temp = -temp; // negate number
      } // if
	  i2c_stop();
   } // else
   return temp;     // Return value now in °C << 7
} // lm92_read()

uint8_t mcp23017_init(void)
/*------------------------------------------------------------------
  Purpose  : This function inits the MCP23017 16-bit IO-expander
  Variables: -
  Returns  : 0: no error, 1: error
  ------------------------------------------------------------------*/
{
	uint8_t err;
	err = mcp23017_write(IOCON, MCP23017_INIT);
	if (!err)
	{
		err = mcp23017_write(IODIRB, 0x00); // all PORTB bits are output
		err = mcp23017_write(IODIRA, 0x00); // all PORTA bits are output
		err = mcp23017_write(GPIOA,  0x00); // All valves are OFF at power-up
		err = mcp23017_write(GPIOB,  0x00); // All IO is OFF at power-up
	} // if
	return err;	
} // mcp23017_init()

uint8_t mcp23017_read(uint8_t reg)
/*------------------------------------------------------------------
  Purpose  : This function reads one register from the MCP23017 IO-expander.
  Variables:
       reg : The register to read from
  Returns  : the value returned from the register
  ------------------------------------------------------------------*/
{
	uint8_t err, ret = 0;
	
	err = (i2c_select_channel(MCP23017_I2C_CH, HSPEED) != I2C_ACK);
	if (!err) 
	{   // generate I2C start + output address to I2C bus
		err = (i2c_start(MCP23017_BASE | I2C_WRITE) == I2C_NACK); 
	} // if
	if (!err)
	{
		err = (i2c_write(reg)  == I2C_NACK); // write register address
		i2c_rep_start(MCP23017_BASE | I2C_READ);
		ret = i2c_readNak(); // Read byte, generate I2C stop condition
		i2c_stop();
	} // if
	return ret;
} // mcp23017_read()

uint8_t mcp23017_write(uint8_t reg, uint8_t data)
/*------------------------------------------------------------------
  Purpose  : This function write one data byte to a specific register 
			 of the MCP23017 IO-expander.
  Variables:
       reg : The register to write to
	   data: The data byte to write into the register
  Returns  : 0: no error, 1: error
  ------------------------------------------------------------------*/
{
	uint8_t err;
		
	err = (i2c_select_channel(MCP23017_I2C_CH, HSPEED) != I2C_ACK);
	if (!err) 
	{   // generate I2C start + output address to I2C bus
		err = (i2c_start(MCP23017_BASE | I2C_WRITE) == I2C_NACK);
	} // if
	if (!err)
	{
		err = (i2c_write(reg)  == I2C_NACK); // write register address
		err = (i2c_write(data) == I2C_NACK); // write register value
		i2c_stop();
	} // if
	return err;
} // mcp23017_write()

//--------------------------------------------------------------------------
// Perform a device reset on the DS2482
//
// Device Reset
//   S AD,0 [A] DRST [A] Sr AD,1 [A] [SS] A\ P
//  [] indicates from slave
//  SS status byte to read to verify state
//
// Input: addr: the I2C address of the DS2482 to reset
// Returns: TRUE if device was reset
//          FALSE device not detected or failure to perform reset
//--------------------------------------------------------------------------
int8_t ds2482_reset(uint8_t addr)
{
   uint8_t err, ret;

	err = (i2c_select_channel(DS2482_I2C_CH, HSPEED) != I2C_ACK);
	if (!err) 
	{   // generate I2C start + output address to I2C bus
		err = (i2c_start(addr | I2C_WRITE) == I2C_NACK);
	} // if
	if (!err)
	{
		err  = (i2c_write(CMD_DRST)  == I2C_NACK); // write register address
		i2c_rep_start(addr | I2C_READ);
		ret = i2c_readNak(); // Read byte, generate I2C stop condition
		i2c_stop();
   } // if
   // check for failure due to incorrect read back of status
   if (!err && ((ret & 0xF7) == 0x10))
   		return TRUE;
   else return FALSE;	
} // ds2482_reset()
  
//--------------------------------------------------------------------------
// Write the configuration register in the DS2482. The configuration 
// options are provided in the lower nibble of the provided config byte. 
// The uppper nibble in bitwise inverted when written to the DS2482.
//  
// Write configuration (Case A)
//   S AD,0 [A] WCFG [A] CF [A] Sr AD,1 [A] [CF] A\ P
//  [] indicates from slave
//  CF configuration byte to write
//
// Input: addr: the I2C address of the DS2482 to reset
// Returns:  TRUE: config written and response correct
//           FALSE: response incorrect
//--------------------------------------------------------------------------
int8_t ds2482_write_config(uint8_t addr)
{
   uint8_t err, read_config;

	err = (i2c_select_channel(DS2482_I2C_CH, HSPEED) != I2C_ACK);
	if (!err) 
	{   // generate I2C start + output address to I2C bus
		err = (i2c_start(addr | I2C_WRITE) == I2C_NACK);
	} // if
	if (!err)
	{
		err  = (i2c_write(CMD_WCFG)  == I2C_NACK); // write register address
		err |= (i2c_write(DS2482_CONFIG)  == I2C_NACK); // write register address
		i2c_rep_start(addr | I2C_READ);
		read_config = i2c_readNak(); // Read byte, generate I2C stop condition
		i2c_stop();
   } // if
   // check for failure due to incorrect read back
   if (err || (read_config != DS2482_CONFIG))
   {
      ds2482_reset(addr); // handle error
      return FALSE;
   } // if
   return TRUE;
} // ds2482_write_config()

//--------------------------------------------------------------------------
// DS2428 Detect routine that performs a device reset followed by writing 
// the default configuration settings (active pullup enabled)
//
// Input: addr: the I2C address of the DS2482 to reset
// Returns: TRUE if device was detected and written
//          FALSE device not detected or failure to write configuration byte
//--------------------------------------------------------------------------
int8_t ds2482_detect(uint8_t addr)
{
   if (!ds2482_reset(addr)) // reset the DS2482
      return FALSE;

   if (!ds2482_write_config(addr)) // write default configuration settings
        return FALSE;
   else return TRUE;
} // ds2482_detect()

//--------------------------------------------------------------------------
// Use the DS2482 help command '1-Wire triplet' to perform one bit of a 1-Wire
// search. This command does two read bits and one write bit. The write bit
// is either the default direction (all device have same bit) or in case of 
// a discrepancy, the 'search_direction' parameter is used. 
//
// Returns: The DS2482 status byte result from the triplet command
//--------------------------------------------------------------------------
uint8_t ds2482_search_triplet(uint8_t search_direction, uint8_t addr)
{
   uint8_t err, status;
   int poll_count = 0;

   // 1-Wire Triplet (Case B)
   //   S AD,0 [A] 1WT [A] SS [A] Sr AD,1 [A] [Status] A [Status] A\ P
   //                                         \--------/        
   //                           Repeat until 1WB bit has changed to 0
   //  [] indicates from slave
   //  SS indicates byte containing search direction bit value in msbit
   err = (i2c_select_channel(DS2482_I2C_CH, HSPEED) != I2C_ACK);
   if (!err) 
   {   // generate I2C start + output address to I2C bus
	   err = (i2c_start(addr | I2C_WRITE) == I2C_NACK);
   } // if
   if (!err)
   {
	   err  = (i2c_write(CMD_1WT) == I2C_NACK); // write register address
   	   err |= (i2c_write(search_direction ? 0x80 : 0x00) == I2C_NACK);
	   i2c_rep_start(addr | I2C_READ);
   	   // loop checking 1WB bit for completion of 1-Wire operation 
   	   // abort if poll limit reached
	   status = i2c_readAck(); // Read byte
	   do
	   {
	     if (status & STATUS_1WB) status = i2c_readAck();
	   }
	   while ((status & STATUS_1WB) && (poll_count++ < DS2482_OW_POLL_LIMIT));
	   status = i2c_readNak();
	   i2c_stop();
   	   // check for failure due to poll limit reached
   	   if (poll_count >= DS2482_OW_POLL_LIMIT)
   	   {
      	  ds2482_reset(addr); // handle error
      	  return FALSE;
   	   } // if
   	   return status;
   } // if
   else return FALSE;
} // ds2482_search_triplet()