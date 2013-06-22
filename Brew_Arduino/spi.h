#ifndef _SPI_H_
#define _SPI_H_
#include <avr/io.h>

//-----------------------------------------------------------------------------
// Register for the MAX6951 8-digit LED Display Driver
//-----------------------------------------------------------------------------
#define MAX6951_DECODE     (0x01 << 8)
#define MAX6951_INTENSITY  (0x02 << 8)
#define MAX6951_SCAN_LIMIT (0x03 << 8)
#define MAX6951_CONFIG     (0x04 << 8)
#define MAX6951_DISP_TEST  (0x07 << 8)
#define MAX6951_DIGIT0     (0x60 << 8)
#define MAX6951_DIGIT1     (0x61 << 8)
#define MAX6951_DIGIT2     (0x62 << 8)
#define MAX6951_DIGIT3     (0x63 << 8)
#define MAX6951_DIGIT4     (0x64 << 8)
#define MAX6951_DIGIT5     (0x65 << 8)
#define MAX6951_DIGIT6     (0x66 << 8)
#define MAX6951_DIGIT7     (0x67 << 8)

//-----------------------------------------------------------------------------
// Constants for the MAX6951 8-digit LED Display Driver
//-----------------------------------------------------------------------------
#define MAX6951_DISP_TEST_ON     (MAX6951_DISP_TEST | 0x0001)
#define MAX6951_DISP_TEST_OFF    (MAX6951_DISP_TEST & 0xFFFE)
#define MAX6951_DECODE_ALL       (MAX6951_DECODE    | 0x00FF)
#define MAX6951_DECODE_NONE      (MAX6951_DECODE    & 0xFF00)
#define MAX6951_DECODE_D5D0      (MAX6951_DECODE    | 0x003F)
#define MAX6951_DECODE_D3D0      (MAX6951_DECODE    | 0x000F)

#define MAX6951_DECODE_TOPb      (0x00F0)
#define MAX6951_NO_DECODE_D7b    (0xFF7F)
#define MAX6951_DECODE_D7b       (0x0080)
#define MAX6951_DECODE_D654b     (0x0070)
#define MAX6951_DECODE_BOTTOMb   (0x000F)
#define MAX6951_NO_DECODE_D3b    (0xFFF7)
#define MAX6951_DECODE_D3b       (0x0008)
#define MAX6951_DECODE_D210b     (0x0007)
#define MAX6951_MINUS_SIGN       (0x01)

#define MAX6951_NORMAL_OPERATION (1 << 0)
#define MAX6951_FAST_BLINKING    (1 << 2)
#define MAX6951_BLINK_ENABLE     (1 << 3)
#define MAX6951_SYNC_BLINKING    (1 << 4)
#define MAX6951_CLEAR_DIGITS     (1 << 5)

#define DISPLAY_BOTTOM           (0)
#define DISPLAY_TOP              (1)
#define NO_BLINKING              (0)

//-----------------------------------------------------------------------------
// Register for the MCP23S08 GPIO-8 IC
//-----------------------------------------------------------------------------
#define MCP23S08_BASE    (0x40)
#define MCP23S08_IODIR   (0x00)
#define MCP23S08_IPOL    (0x01)
#define MCP23S08_GPINTEN (0x02)
#define MCP23S08_DEFVAL  (0x03)
#define MCP23S08_INTCON  (0x04)
#define MCP23S08_IOCON   (0x05)
#define MCP23S08_GPPU    (0x06)
#define MCP23S08_INTF    (0x07)
#define MCP23S08_INTCAP  (0x08)
#define MCP23S08_GPIO    (0x09)
#define MCP23S08_OLAT    (0x0A)

void    spi_init();
uint8_t spi_read (uint8_t * dataout, uint8_t len);
void    spi_transmit_sync (uint8_t * dataout, uint8_t len);
uint8_t spi_fast_shift (uint8_t data);

void    max6951_init(void);
void    max6951_print_nr(int16_t d, int dp, uint8_t nr, uint8_t blink);

void    mcp23s08_init(void);
void    mcp23s08_write(uint8_t mcp_reg, uint8_t data);
uint8_t mcp23s08_read(uint8_t mcp_reg);

#endif /* _SPI_H_ */