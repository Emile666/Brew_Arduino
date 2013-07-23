#ifndef _CI_H
#define _CI_H   1
#include <avr/io.h>

#include <stdlib.h>
#include "brew_arduino.h"
#include "scheduler.h"

uint8_t rs232_command_handler(void);
uint8_t execute_rs232_command(char *s);

#endif
