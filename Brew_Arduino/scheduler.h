/*==================================================================
  File Name    : scheduler.h
  Function name: -
  File name    : $Id$
  Author       : E. van de Logt
  ------------------------------------------------------------------
  Purpose : This is the header-file for scheduler.c
  ------------------------------------------------------------------
  $Log$
  Revision 1.5  2014/05/03 11:27:44  Emile
  - Ethernet support added for W550io module
  - No response for L, N, P, W commands anymore
  - All source files now have headers

  Revision 1.4  2013/07/24 13:46:40  Emile
  - Minor changes in S1, S2 and S3 commands to minimize comm. overhead.
  - Version ready for Integration Testing with PC program!

  Revision 1.3  2013/07/21 13:10:44  Emile
  - Reading & Writing of 17 parameters now fully works with set_parameter()
  - VHLT and VMLT tasks added
  - Scheduler: actual & max. times now printed in msec. instead of usec.
  - THLT and TMLT now in E-2 Celsius for PC program
  - All lm92 test routines removed, only one lm92_read() remaining

  Revision 1.2  2013/07/20 14:52:00  Emile
  - LM35, THLT and TMLT tasks are now working
  - Max. duration added to scheduler
  - slope_limiter & lm92_read() now work with uint16_t instead of float

  Revision 1.1  2013/07/19 10:51:02  Emile
  - I2C frequency 50 50 kHz to get 2nd LM92 working
  - Command Mx removed, command N0 x added, commands N0..N3 renamed to N1..N4
  - Command S3 added, list_all_tasks. To-Do: get timing-measurement working
  - Scheduler added with 3 tasks: lm35, led_blink and pwm_2_time

  ==================================================================
*/ 
#ifndef _SCHEDULER_H
#define _SCHEDULER_H

#include <string.h>
#include <avr/io.h>
#include <stdbool.h>
#include "usart.h"
#include "Udp.h"
#include "pwm.h"
#include "brew_arduino.h"

#define MAX_TASKS	  (8)
#define MAX_MSEC      (60000)
#define TICKS_PER_SEC (1000L)
#define NAME_LEN      (12) 

#define TASK_READY    (0x01)
#define TASK_ENABLED  (0x02)

#define NO_ERR        (0x00)
#define ERR_CMD		  (0x01)
#define ERR_NUM		  (0x02)
#define ERR_I2C		  (0x03)
#define ERR_NAME      (0x04)
#define ERR_EMPTY     (0x05)
#define ERR_MAX_TASKS (0x06)

typedef struct _task_struct
{
	void     (* pFunction)(); // Function pointer
	char     Name[NAME_LEN];  // Task name
	uint16_t Period;          // Period between 2 calls in msec.
	uint16_t Delay;           // Initial delay before Counter starts in msec.
	uint16_t Counter;         // Running counter, is init. from Period
	uint8_t	 Status;          // bit 1: 1=enabled ; bit 0: 1=ready to run
	uint16_t Duration;        // Measured task-duration in clock-ticks
	uint16_t Duration_Max;    // Max. measured task-duration
} task_struct;

void    scheduler_isr(void);  // run-time function for scheduler
void    dispatch_tasks(void); // run all tasks that are ready
uint8_t add_task(void (*task_ptr), char *Name, uint16_t delay, uint16_t period);
uint8_t set_task_time_period(uint16_t Period, char *Name);
uint8_t enable_task(char *Name);
uint8_t disable_task(char *Name);
void    list_all_tasks(bool rs232_udp);

#endif
