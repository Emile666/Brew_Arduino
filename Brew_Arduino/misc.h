//-----------------------------------------------------------------------------
// Created: 20-4-2013 22:32:11
// Author : Emile
// File   : misc.h
//-----------------------------------------------------------------------------
// $Log$
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

#define MAX_MA (10)
typedef struct _ma
{
	int16_t T[MAX_MA]; // array with delayed values of input signal
	uint8_t  index;    // index in T[] where to store the new input value
	uint8_t  N;        // The order of the MA filter. Note that N < MAX_MA
	int16_t sum;       // The running sum of the MA filter
} ma;

void     init_moving_average(ma *p, uint8_t N, int16_t init_val);
int16_t  moving_average(ma *p, int16_t x);
void     init_sample_delay(ma *p, int TD);
double   sample_delay(ma *p, double x);
void     slope_limiter(const int16_t lim, const int16_t Told, int16_t *Tnew);

#endif