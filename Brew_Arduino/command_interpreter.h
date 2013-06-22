#ifndef _CI_H
#define _CI_H   1
#include <avr/io.h>

#include <stdlib.h>
#include "brew_arduino.h"

#define NO_ERR  (0x00)
#define ERR_CMD (0x01)
#define ERR_NUM (0x02)
#define ERR_I2C (0x04)

uint8_t rs232_command_handler(void);
uint8_t execute_rs232_command(char *s);

#endif
