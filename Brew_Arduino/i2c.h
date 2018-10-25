/*==================================================================
  File Name    : i2c.h
  Function name: -
  Author       : Peter Fleury <pfleury@gmx.ch> http://jump.to/fleury
  ------------------------------------------------------------------
  Purpose : This is the header-file for the I2C master interface (i2c.c)
  ================================================================== */ 
#ifndef _I2C_H
#define _I2C_H   1
#include <avr/io.h>

//-------------------------------------------
// Set the prescaler (TWSR register) to 0x01, 
// so that the prescaler value equals 4.
// 100 kHz: TWBR = 18 (should be > 10)
//  10 kHz: TWBR = 198 (should be < 255)
//
// For 400 kHz: TWSR should be set to 0x00,
//              so that the prescaler equals 1!
//-------------------------------------------
#define SCL_CLK_10KHZ  (((F_CPU/10000)-16)/8)
#define SCL_CLK_20KHZ  (((F_CPU/20000)-16)/8)
#define SCL_CLK_30KHZ  (((F_CPU/30000)-16)/8)
#define SCL_CLK_40KHZ  (((F_CPU/40000)-16)/8)
#define SCL_CLK_50KHZ  (((F_CPU/50000)-16)/8)
#define SCL_CLK_60KHZ  (((F_CPU/60000)-16)/8)
#define SCL_CLK_70KHZ  (((F_CPU/70000)-16)/8)
#define SCL_CLK_80KHZ  (((F_CPU/80000)-16)/8)
#define SCL_CLK_90KHZ  (((F_CPU/90000)-16)/8)
#define SCL_CLK_100KHZ (((F_CPU/100000)-16)/8)
#define SCL_CLK_400KHZ (((F_CPU/400000)-16)/2)

#if SCL_CLK_100KHZ < 11
#error "SCL_CLK_100KHZ value too small"
#endif
#if SCL_CLK_10KHZ > 255
#error "SCL_CLK_10KHZ value too big"
#endif

#define LSPEED (0)
#define HSPEED (1)

//-------------------------------------------------------------------------
// MCP23017 16-BIT IO Expander: 
// Register names when IOCON.BANK == 0 (default situation at power-up)
//-------------------------------------------------------------------------
#define MCP23017_INIT (0x00) /* No bank addressing, seq.operation enabled */

#define IODIRA   (0x00) /* I/O direction register, 1=input, 0=output */
#define IODIRB   (0x01)
#define IPOLA    (0x02) /* Input polarity port register, 1= opposite state */
#define IPOLB    (0x03)
#define GPINTENA (0x04) /* Interrupt-on-change register */
#define GPINTENB (0x05)
#define DEFVALA  (0x06) /* Default compare register for interrupt-on-change */
#define DEFVALB  (0x07)
#define INTCONA  (0x08) /* Interrupt control register */
#define INTCONB  (0x09)
#define IOCON    (0x0A) /* Configuration register */
#define GPPUA    (0x0C) /* Pull-up resistor configuration register */
#define GPPUB    (0x0D)
#define INTFA    (0x0E) /* Interrupt flag register */
#define INTFB    (0x0F)
#define INTCAPA  (0x10) /* Interrupt capture register */
#define INTCAPB  (0x11)
#define GPIOA    (0x12) /* Port register */
#define GPIOB    (0x13)
#define OLATA    (0x14) /* Output latch register */
#define OLATB    (0x15)

//-------------------------------------------------------------------------
// PCA9544 I2C Multiplexer
// Channel 0: CON14 SPI_I2C_BUS (as of HW PCB V3.01)
//            DS2482   (0x30)   (as of HW PCB V3.03)
//            DS2482   (0x36)   (as of HW PCB V3.03)
//            DS2482   (0x32)   (as of HW PCB V3.30)
//            DS2482   (0x34)   (as of HW PCB V3.30)
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
#define MCP23017_BASE   (0x40) /* CH0: IO-Expander for Valve Outputs */
#define MCP23017_I2C_CH (PCA9544_CH1)

#define DS2482_THLT_BASE  (0x30)
#define DS2482_TBOIL_BASE (0x32)
#define DS2482_TCFC_BASE  (0x34)
#define DS2482_TMLT_BASE  (0x36)
#define DS2482_I2C_CH     (PCA9544_CH0)

// List of possible LM92 addresses
#define LM92_0_BASE   (0x90)
#define LM92_1_BASE   (0x92)
#define LM92_2_BASE   (0x94)
#define LM92_3_BASE   (0x96)

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
#define DS2482_CONFIG        (0xE1)
#define DS2482_OW_POLL_LIMIT  (200)

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

void          i2c_init(uint8_t clk); // Initializes the I2C Interface. Needs to be called only once
void          i2c_stop(void); // Terminates the data transfer and releases the I2C bus
enum i2c_acks i2c_start(unsigned char addr); // Issues a start condition and sends address and transfer direction
enum i2c_acks i2c_rep_start(unsigned char addr); // Issues a repeated start condition and sends address and transfer direction
void          i2c_start_wait(unsigned char addr); // i2c_start() + If device is busy, use ack polling to wait until device ready
enum i2c_acks i2c_write(unsigned char data); // Send one byte to I2C device
unsigned char i2c_readAck(void); // read one byte from the I2C device, request more data from device
unsigned char i2c_readNak(void); // read one byte from the I2C device, read is followed by a stop condition
unsigned char i2c_read(unsigned char ack); // read one byte from the I2C device
enum i2c_acks i2c_select_channel(uint8_t ch, uint8_t speed); // Set PCA9544 channel and I2C-clock

int16_t       lm92_read(uint8_t dvc, uint8_t *err);

uint8_t       mcp23017_init(void);
uint8_t		  mcp23017_read(uint8_t reg);
uint8_t       mcp23017_write(uint8_t reg, uint8_t data);

int8_t        ds2482_reset(uint8_t addr);
int8_t        ds2482_write_config(uint8_t addr);
int8_t        ds2482_detect(uint8_t addr);
uint8_t       ds2482_search_triplet(uint8_t search_direction, uint8_t addr);

//  Implemented as a macro, which calls either i2c_readAck or i2c_readNak
#define i2c_read(ack)  (ack==I2C_ACK) ? i2c_readAck() : i2c_readNak(); 

#endif
