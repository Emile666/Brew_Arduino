/*==================================================================
  File Name    : $Id$
  Function name: -
  Author       : Peter Fleury <pfleury@gmx.ch> http://jump.to/fleury
  ------------------------------------------------------------------
  Purpose : I2C master library using hardware TWI interface
  ------------------------------------------------------------------
  $Log$
  Revision 1.5  2014/05/03 11:27:44  Emile
  - Ethernet support added for W550io module
  - No response for L, N, P, W commands anymore
  - All source files now have headers

  ================================================================== */ 
#include <inttypes.h>
#include <compat/twi.h>
#include "i2c.h"
#include "delay.h"         /* for delay_msec() */

/* I2C clock in Hz */
#define SCL_CLOCK  50000L

/*************************************************************************
 Initialization of the I2C bus interface. Need to be called only once
*************************************************************************/
void i2c_init(void)
{
  /* initialize TWI clock: 100 kHz clock, TWPS = 0 => prescaler = 1 */
  
  TWSR = 0;                         /* no pre-scaler */
  TWBR = ((F_CPU/SCL_CLOCK)-16)/2;  /* must be > 10 for stable operation */
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

	// wait until transmission completed
	while ((!(TWCR & (1<<TWINT))) && (retries < I2C_RETRIES))
	{
		delay_msec(1);
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
enum i2c_acks i2c_select_channel(uint8_t ch)
{
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
       dvc : 0 = Read from the LM92 at 0x92/0x93 (This is the HLT Temp.)
             1 = Read from the LM92 at 0x94/0x95 (This is the MLT Temp.)
  Returns  : The temperature from the LM92 in a signed Q8.4 format
  ------------------------------------------------------------------*/
{
   uint8_t  buffer[2]; // array to store data from i2c_read()
   uint16_t temp_int;  // the temp. from the LM92 as an integer
   uint8_t  sign;      // sign of temperature
   int16_t  temp = 0;  // the temp. from the LM92 as a double
   uint8_t  adr;       // i2c address
       
   // Start with selecting the proper channel on the PCA9544
   *err = FALSE;	
   if (dvc == THLT)
   {
      adr  = THLT_BASE | I2C_READ; 
	  *err = (i2c_select_channel(THLT_I2C_CH) != I2C_ACK);
   } // if
   else if (dvc == TMLT)
   {
	  adr  = TMLT_BASE | I2C_READ; 
	  *err = (i2c_select_channel(TMLT_I2C_CH) != I2C_ACK);
   } // else if
   else *err = TRUE;
   
   if (!*err) *err = (i2c_start(adr) == I2C_NACK); // generate I2C start + output address to I2C bus
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
      temp = temp_int >> 3;
      if (sign)
      {
         temp = -temp; // negate number
      } // if
	  i2c_stop();
   } // else
   return temp;     // Return value now in °C * 16
} // lm92_read()

uint8_t mcp23017_init(void)
/*------------------------------------------------------------------
  Purpose  : This function inits the MCP23017 16-bit IO-expander
  Variables: -
  Returns  : 0: no error, 1: error
  ------------------------------------------------------------------*/
{
	uint8_t err;
	err = mcp23017_write(IOCON, IOCON_INIT);
	if (!err)
	{
		err = mcp23017_write(IODIRA, 0x00); // all PORTA bits are output
		err = mcp23017_write(IODIRB, 0x00); // all PORTB bits are output
	} // if
	return err;	
} // mcp23017_init()

uint8_t mcp23017_read(uint8_t reg)
/*------------------------------------------------------------------
  Purpose  : This function reads one register from the MCP23017
             16-bit IO-expander
  Variables:
       reg : The register to read from
  Returns  : the value returned from the register
  ------------------------------------------------------------------*/
{
	uint8_t err, ret = 0;
	
	err = (i2c_select_channel(MCP23017_I2C_CH) != I2C_ACK);
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
			 of the MCP23017 16-bit IO-expander
  Variables:
       reg : The register to write to
	   data: The data byte to write into the register
  Returns  : 0: no error, 1: error
  ------------------------------------------------------------------*/
{
	uint8_t err;
		
	err = (i2c_select_channel(MCP23017_I2C_CH) != I2C_ACK);
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
