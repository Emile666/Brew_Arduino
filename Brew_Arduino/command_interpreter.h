/*==================================================================
  File Name    : command_interpreter.h
  Author       : Emile
  ------------------------------------------------------------------
  Purpose : Header file for command_interpreter.c
  ------------------------------------------------------------------
  $Log$
  Revision 1.3  2014/05/03 11:27:44  Emile
  - Ethernet support added for W550io module
  - No response for L, N, P, W commands anymore
  - All source files now have headers

  ================================================================== */ 
#ifndef _CI_H
#define _CI_H   1
#include <avr/io.h>

#include <stdlib.h>
#include "brew_arduino.h"
#include "scheduler.h"

void    i2c_scan(uint8_t ch, bool rs232_udp);
uint8_t rs232_command_handler(void);
uint8_t ethernet_command_handler(char *s);
uint8_t set_parameter(uint8_t num, uint16_t val);
void    find_OW_device(uint8_t i2c_addr);
void    process_pwm_signal(uint8_t pwm_ch, uint8_t pwm_val);
void    process_temperature(uint8_t err, char *name, int16_t val_87);
void    process_flow(uint32_t flow_val, char *name);
uint8_t execute_single_command(char *s, bool rs232_udp);

#endif
