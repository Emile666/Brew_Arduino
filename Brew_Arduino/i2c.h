/*==================================================================
  File Name    : $Id$
  Function name: -
  Author       : Peter Fleury <pfleury@gmx.ch> http://jump.to/fleury
  ------------------------------------------------------------------
  Purpose : This is the header-file for the I2C master interface (i2c.c)
  ------------------------------------------------------------------
  $Log$
  Revision 1.7  2015/05/09 14:37:37  Emile
  - I2C Channel & HW-address update for MCP23017 to reflect changes in HW PCB V3.01

  Revision 1.6  2014/11/30 20:44:45  Emile
  - Vxxx command added to write valve output bits
  - mcp23017 (16 bit I2C IO-expander) routines + defines added

  Revision 1.5  2014/10/26 12:44:47  Emile
  - A3 (Thlt) and A4 (Tmlt) commands now return '99.99' in case of I2C HW error.

  Revision 1.4  2014/05/03 11:27:44  Emile
  - Ethernet support added for W550io module
  - No response for L, N, P, W commands anymore
  - All source files now have headers

  ================================================================== */ 
#ifndef _I2C_H
#define _I2C_H   1
#include <avr/io.h>

//-------------------------------------------------------------------------
// MCP23017 16-BIT IO Expander: Register names when BANK == 1
//
#define IOCON    (0x05)
// Bank addressing, seq. operation disabled, slew rate enabled
// HW addressing enabled
#define IOCON_INIT (0xAA)

#define IODIRA   (0x00)
#define IPOLA    (0x01)
#define GPINTENA (0x02)
#define DEFVALA  (0x03)
#define INTCONA  (0x04)
#define GPPUA    (0x06)
#define INTFA    (0x07)
#define INTCAPA  (0x08)
#define GPIOA    (0x09)
#define OLATA    (0x0A)

#define IODIRB   (IODIRA   + 0x10)
#define IPOLB    (IPOLA    + 0x10)
#define GPINTENB (GPINTENA + 0x10)
#define DEFVALB  (DEFVALA  + 0x10)
#define INTCONB  (INTCONA  + 0x10)
#define GPPUB    (GPPUA    + 0x10)
#define INTFB    (INTFA    + 0x10)
#define INTCAPB  (INTCAPA  + 0x10)
#define GPIOB    (GPIOA    + 0x10)
#define OLATB    (OLATA    + 0x10)


//-------------------------------------------------------------------------
// PCA9544 I2C Multiplexer
// Channel 0: CON14 SPI_I2C_BUS (as of HW PCB V3.01)
// Channel 1: MCP23017 (0x40)   (as of HW PCB V3.01)
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
/** NOTE: 0x44 on older PCBs, 0x40 as of HW PCB V3.01 **/
#define MCP23017_BASE   (0x40) /* CH0: 16-bit IO for Valve Outputs (Optional) */
#define MCP23017_I2C_CH (PCA9544_CH1)

#define DS2482_THLT_BASE (0x30)
#define DS2482_TMLT_BASE (0x36)
#define DS2482_I2C_CH    (PCA9544_CH0)

#define LM92_0_BASE   (0x90) /* Not used */
#define LM92_1_BASE   (0x92) /* CH2: LM92 */
#define LM92_2_BASE   (0x94) /* CH3: LM92 */
#define LM92_3_BASE   (0x96) /* Not used */

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

// DS2482 Configuration Register
// Standard speed (1WS==0), Strong Pullup disabled (SPU==0), Active Pullup enabled (APU==1)
#define DS2482_CONFIG (0xE1)
#define DS2482_OW_POLL_LIMIT (200)

// DS2482 commands
#define CMD_DRST   0xF0
#define CMD_WCFG   0xD2
#define CMD_CHSL   0xC3
#define CMD_SRP    0xE1
#define CMD_1WRS   0xB4
#define CMD_1WWB   0xA5
#define CMD_1WRB   0x96
#define CMD_1WSB   0x87
#define CMD_1WT    0x78

// DS2482 status bits 
#define STATUS_1WB  0x01
#define STATUS_PPD  0x02
#define STATUS_SD   0x04
#define STATUS_LL   0x08
#define STATUS_RST  0x10
#define STATUS_SBR  0x20
#define STATUS_TSB  0x40
#define STATUS_DIR  0x80

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

uint8_t       mcp23017_init(void);
uint8_t		  mcp23017_read(uint8_t reg);
uint8_t       mcp23017_write(uint8_t reg, uint8_t data);

int8_t        ds2482_reset(uint8_t addr);
int8_t        ds2482_write_config(uint8_t addr);
int8_t        ds2482_detect(uint8_t addr);
uint8_t ds2482_search_triplet(uint8_t search_direction, uint8_t addr);

//  Implemented as a macro, which calls either i2c_readAck or i2c_readNak
#define i2c_read(ack)  (ack==I2C_ACK) ? i2c_readAck() : i2c_readNak(); 

#endif
