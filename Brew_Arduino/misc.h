//-----------------------------------------------------------------------------
// Created: 20-4-2013 22:32:11
// Author : Emile
// File   : $Id$
//-----------------------------------------------------------------------------
// $Log$
// Revision 1.5  2014/05/03 11:27:44  Emile
// - Ethernet support added for W550io module
// - No response for L, N, P, W commands anymore
// - All source files now have headers
//
// Revision 1.4  2013/07/23 19:33:18  Emile
// - Bug-fix slope-limiter function. Tested on all measurements.
//
// Revision 1.3  2013/07/21 13:10:44  Emile
// - Reading & Writing of 17 parameters now fully works with set_parameter()
// - VHLT and VMLT tasks added
// - Scheduler: actual & max. times now printed in msec. instead of usec.
// - THLT and TMLT now in E-2 Celsius for PC program
// - All lm92 test routines removed, only one lm92_read() remaining
//
// Revision 1.2  2013/07/20 14:51:59  Emile
// - LM35, THLT and TMLT tasks are now working
// - Max. duration added to scheduler
// - slope_limiter & lm92_read() now work with uint16_t instead of float
//
// Revision 1.1.1.1  2013/06/22 19:32:09  Emile
// First import of Brew_Arduino directory (Atmel Studio 6)
//
//-----------------------------------------------------------------------------
#ifndef _MISC_H
#define _MISC_H

#include <stdint.h>

#define MAX_MA (4)
typedef struct _ma
{
	float   T[MAX_MA]; // array with delayed values of input signal
	uint8_t index;     // index in T[] where to store the new input value
	uint8_t N;         // The order of the MA filter. Note that N < MAX_MA
	float   sum;       // The running sum of the MA filter
} ma;

void  init_moving_average(ma *p, uint8_t N, float init_val);
float moving_average(ma *p, float x);
void  slope_limiter(const int16_t lim, const int16_t Told, int16_t *Tnew);

#endif