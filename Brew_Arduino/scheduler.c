/*==================================================================
  File Name    : scheduler.c
  Function name: scheduler_isr(), dispatch_tasks(), add_task(),
                 enable_task()  , disable_task()  , set_task_time_period(),
				 list_all_tasks()
  File name    : $Id$
  Author       : E. van de Logt
  ------------------------------------------------------------------
  Purpose : This files contains all the functions for adding and
            executing tasks in a cooperative (non pre-emptive) way.
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
#include "scheduler.h"

task_struct task_list[MAX_TASKS]; // struct with all tasks
uint8_t     max_tasks = 0;

/*-----------------------------------------------------------------------------
  Purpose  : Run-time function for scheduler. Should be called from within
             an ISR. This function goes through the task-list and decrements
             eacht task counter. On time-out, the ready flag is set.
  Variables: task_list[] structure
  Returns  : -
  ---------------------------------------------------------------------------*/
void scheduler_isr(void)
{
	uint8_t index = 0; // index in task_list struct

	while(task_list[index].pFunction)
	{
		//First go through the initial delay
		if(task_list[index].Delay > 0)
		{
			task_list[index].Delay--;
		} // if
		else
		{	//now we decrement the actual period counter 

			task_list[index].Counter--;
			if(task_list[index].Counter == 0)
			{
				//Set the flag and reset the counter;
				task_list[index].Status |= TASK_READY;
			} // if
		} // else
		index++;
	} // while
} // scheduler_isr()

/*-----------------------------------------------------------------------------
  Purpose  : Run all tasks for which the ready flag is set. Should be called 
             from within the main() function, not from an interrupt routine!
  Variables: task_list[] structure
  Returns  : -
  ---------------------------------------------------------------------------*/
void dispatch_tasks(void)
{
	uint8_t index = 0;
	uint32_t time1; // Measured #clock-ticks of 50 usec. (TMR1 frequency)
	uint32_t time2;

	//go through the active tasks
	while(task_list[index].pFunction)
	{
		if((task_list[index].Status & (TASK_READY | TASK_ENABLED)) == (TASK_READY | TASK_ENABLED))
		{
			time1 = millis(); // Read msec. timer
			task_list[index].pFunction(); // run the task
			task_list[index].Status  &= ~TASK_READY; // reset the task when finished
			task_list[index].Counter  = task_list[index].Period; // reset counter
			time2 = millis(); // read msec. timer
			if (time2 < time1) time2 += UINT32_MAX - time1; // overflows every 49.7 days, unlikely
			else               time2 -= time1; 
			task_list[index].Duration  = (uint16_t)time2; // time difference in milliseconds
			if (time2 > task_list[index].Duration_Max)
			{
				task_list[index].Duration_Max = time2;
			} // if
		} // if
		index++;
	} // while
	//go to sleep till next tick!
} // dispatch_tasks()

/*-----------------------------------------------------------------------------
  Purpose  : Add a function to the task-list struct. Should be called upon
  		     initialization.
  Variables: task_ptr: pointer to function
             delay   : initial delay in msec.
             period  : period between two calls in msec.
  Returns  : [NO_ERR, ERR_MAX_TASKS]
  ---------------------------------------------------------------------------*/
uint8_t add_task(void (*task_ptr), char *Name, uint16_t delay, uint16_t period)
{
	uint8_t  index = 0;
	uint16_t temp1 = (uint16_t)(delay  * TICKS_PER_SEC / 1000);
	uint16_t temp2 = (uint16_t)(period * TICKS_PER_SEC / 1000);

	if (max_tasks >= MAX_TASKS)
		return ERR_MAX_TASKS;
		
	//go through the active tasks
	if(task_list[index].Period != 0)
	{
		while(task_list[++index].Period != 0) ;
	} // if

	if(task_list[index].Period == 0)
	{
		task_list[index].pFunction    = task_ptr;       // Pointer to Function
		task_list[index].Period       = temp2;          // Period in msec.
		task_list[index].Counter      = temp2;	        // Countdown timer
		task_list[index].Delay        = temp1;          // Initial delay before start
		task_list[index].Status      |= TASK_ENABLED;   // Enable task by default
		task_list[index].Status      &= ~TASK_READY;    // Task not ready to run
		task_list[index].Duration     = 0;              // Actual Task Duration
		task_list[index].Duration_Max = 0;              // Max. Task Duration
		strncpy(task_list[index].Name, Name, NAME_LEN); // Name of Task
		max_tasks++; // increase number of tasks
	} // if
	return NO_ERR;
} // add_task()

/*-----------------------------------------------------------------------------
  Purpose  : Enable a task.
  Variables: Name: Name of task to enable
  Returns  : error [NO_ERR, ERR_NAME, ERR_EMPTY]
  ---------------------------------------------------------------------------*/
uint8_t enable_task(char *Name)
{
	uint8_t index = 0;
	bool    found = false;
	
	//go through the active tasks
	if(task_list[index].Period != 0)
	{
		while ((task_list[index].Period != 0) && !found)
		{
			if (!strcmp(task_list[index].Name,Name))
			{
				task_list[index].Status |= TASK_ENABLED;
				found = true;
			} // if
			index++;
		} // while
	} // if
	else return ERR_EMPTY;
	if (!found)
	     return ERR_NAME;
	else return NO_ERR;	
} // enable_task()

/*-----------------------------------------------------------------------------
  Purpose  : Disable a task.
  Variables: Name: Name of task to disable
  Returns  : error [NO_ERR, ERR_NAME, ERR_EMPTY]
  ---------------------------------------------------------------------------*/
uint8_t disable_task(char *Name)
{
	uint8_t index = 0;
	bool    found = false;
	
	//go through the active tasks
	if(task_list[index].Period != 0)
	{
		while ((task_list[index].Period != 0) && !found)
		{
			if (!strcmp(task_list[index].Name,Name))
			{
				task_list[index].Status &= ~TASK_ENABLED;
				found = true;
			} // if
			index++;
		} // while
	} // if
	else return ERR_EMPTY;
	if (!found)
	     return ERR_NAME;
	else return NO_ERR;	
} // disable_task()

/*-----------------------------------------------------------------------------
  Purpose  : Set the time-period (msec.) of a task.
  Variables: msec: the time in milliseconds
             name: the name of the task to set the time for
  Returns  : error [NO_ERR, ERR_NAME, ERR_EMPTY]
  ---------------------------------------------------------------------------*/
uint8_t set_task_time_period(uint16_t Period, char *Name)
{
	uint8_t index = 0;
	bool    found = false;
	
	//go through the active tasks
	if(task_list[index].Period != 0)
	{
		while ((task_list[index].Period != 0) && !found)
		{
			if (!strcmp(task_list[index].Name,Name))
			{
				task_list[index].Period = (uint16_t)(Period * TICKS_PER_SEC / 1000);
				found = true;
			} // if
			index++;
		} // while
	} // if
	else return ERR_EMPTY;
	if (!found)
	     return ERR_NAME;
	else return NO_ERR;	
} // set_task_time_period()

/*-----------------------------------------------------------------------------
  Purpose  : list all tasks and send result to USB-RS232 using xputs().
  Variables: 
  rs232_udp: [RS232_USB, ETHERNET_UDP] Response via RS232/USB or Ethernet/Udp
 Returns  : -
  ---------------------------------------------------------------------------*/
void list_all_tasks(bool rs232_udp)
{
	uint8_t index = 0;
	char    s[50];
	const char hdr[] = "Task-Name   T(ms) Stat T(ms) M(ms)\n";

	rs232_udp == RS232_USB ? xputs(hdr) : udp_write((uint8_t *)hdr,strlen(hdr));
	//go through the active tasks
	if(task_list[index].Period != 0)
	{
		while (task_list[index].Period != 0)
		{
			sprintf(s,"%-11s %5d 0x%02x   %03d   %03d\n", task_list[index].Name, 
					  task_list[index].Period  , task_list[index].Status, 
					  task_list[index].Duration, task_list[index].Duration_Max);
			rs232_udp == RS232_USB ? xputs(s) : udp_write((uint8_t *)s,strlen(s));
			index++;
		} // while
	} // if
} // list_all_tasks()