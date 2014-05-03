/*==================================================================
  File Name    : $Id$
  Function name: -
  Author       : E v.d. Logt
  ------------------------------------------------------------------
  Purpose : HEader file for command_interpreter.c
  ------------------------------------------------------------------
  $Log$
  ================================================================== */ 
#ifndef _CI_H
#define _CI_H   1
#include <avr/io.h>

#include <stdlib.h>
#include "brew_arduino.h"
#include "scheduler.h"

uint8_t rs232_command_handler(void);
uint8_t ethernet_command_handler(char *s);
uint8_t execute_single_command(char *s, bool rs232_udp);

#endif
