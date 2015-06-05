#ifndef ONE_WIRE_H
#define ONE_WIRE_H

#include <inttypes.h>

// 1-Wire API for DS2482 function prototypes
uint8_t OW_reset(uint8_t addr);
uint8_t OW_touch_bit(uint8_t sendbit, uint8_t addr);
void    OW_write_bit(uint8_t sendbit, uint8_t addr);
uint8_t OW_read_bit(uint8_t addr);
uint8_t OW_write_byte(uint8_t sendbyte, uint8_t addr);
uint8_t OW_read_byte(uint8_t addr);
uint8_t OW_touch_byte(uint8_t sendbyte, uint8_t addr);
void    OW_block(uint8_t *tran_buf, uint8_t tran_len, uint8_t addr);
uint8_t OW_first(uint8_t addr);
uint8_t OW_next(uint8_t addr);
uint8_t OW_verify(uint8_t addr);
void    OW_target_setup(uint8_t family_code);
void    OW_family_skip_setup(void);
uint8_t OW_search(uint8_t addr);

// Helper functions
uint8_t calc_crc8(uint8_t data);

#endif
