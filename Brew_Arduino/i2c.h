/*==================================================================
  File Name    : $Id$
  Function name: -
  Author       : Peter Fleury <pfleury@gmx.ch> http://jump.to/fleury
  ------------------------------------------------------------------
  Purpose : This is the header-file for the I2C master interface (i2c.c)
  ------------------------------------------------------------------
  $Log$
  Revision 1.4  2014/05/03 11:27:44  Emile
  - Ethernet support added for W550io module
  - No response for L, N, P, W commands anymore
  - All source files now have headers

  ================================================================== */ 
#ifndef _I2C_H
#define _I2C_H   1
#include <avr/io.h>

//-------------------------------------------------------------------------
// PCA9544 I2C Multiplexer
// Channel 0: Internal I2C for MAX1238 (ADC) and PCF8574 (Valve Outputs)
// Channel 1: I2C for SAA1064 7-segment displays
// Channel 2: I2C for LM92_0
// Channel 3: I2C for LM92_1
//-------------------------------------------------------------------------
#define PCA9544      (0xE0)  /* I2C Base-address */
#define PCA9544_NOCH (0x00)
#define PCA9544_CH0  (0x04)
#define PCA9544_CH1  (0x05)
#define PCA9544_CH2  (0x06)
#define PCA9544_CH3  (0x07)

/** Define all I2C Hardware addresses **/
#define PCF8574     (0x44) /* CH0: 8-bit IO for Valve Outputs (Optional) */
#define LM92_0_BASE (0x90) /* CH2: LM92 */
#define LM92_1_BASE (0x92) /* CH2: LM92 */
#define LM92_2_BASE (0x94) /* CH2: LM92 */
#define LM92_3_BASE (0x96) /* CH2: LM92 */

#define THLT        (0)
#define THLT_BASE   (LM92_1_BASE)
#define THLT_I2C_CH (PCA9544_CH2)
#define TMLT        (1)
#define TMLT_BASE   (LM92_2_BASE)
#define TMLT_I2C_CH (PCA9544_CH3)

//-----------------------------------------------------------------
// The LM92 sign bit is normally bit 12. The value read from the
// LM92 is SHL3. Therefore the sign bit is at bit 15
// Same for the Full Scale value, normally 2^12 SHL3 = 2^15.
//-----------------------------------------------------------------
#define LM92_SIGNb     (0x8000)
#define LM92_FS        (32768)
#define LM92_ERR       (0x4000)

/** defines the data direction (reading from I2C device) in i2c_start(), i2c_rep_start() */
#define I2C_READ    (1)
#define I2C_WRITE   (0)

#define FALSE (0)
#define TRUE  (!FALSE)
#define I2C_RETRIES (3)

// I2C slave HW responds with an ACK (0) or an NACK (1)
enum i2c_acks {I2C_ACK, I2C_NACK};

void          i2c_init(void); // Initializes the I2C Interface. Needs to be called only once
void          i2c_stop(void); // Terminates the data transfer and releases the I2C bus
enum i2c_acks i2c_start(unsigned char addr); // Issues a start condition and sends address and transfer direction
enum i2c_acks i2c_rep_start(unsigned char addr); // Issues a repeated start condition and sends address and transfer direction
void          i2c_start_wait(unsigned char addr); // i2c_start() + If device is busy, use ack polling to wait until device ready
enum i2c_acks i2c_write(unsigned char data); // Send one byte to I2C device
unsigned char i2c_readAck(void); // read one byte from the I2C device, request more data from device
unsigned char i2c_readNak(void); // read one byte from the I2C device, read is followed by a stop condition
unsigned char i2c_read(unsigned char ack); // read one byte from the I2C device
enum i2c_acks i2c_select_channel(uint8_t ch); // Set PCA9544 channel
int16_t       lm92_read(uint8_t dvc, uint8_t *err);

//  Implemented as a macro, which calls either i2c_readAck or i2c_readNak
#define i2c_read(ack)  (ack==I2C_ACK) ? i2c_readAck() : i2c_readNak(); 

#endif
