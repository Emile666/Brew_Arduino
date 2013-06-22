//-----------------------------------------------------------------------------
// Created: 20-4-2013 22:32:11
// Author : Emile
// File   : misc.h
//-----------------------------------------------------------------------------
// $Log$
//-----------------------------------------------------------------------------
#ifndef _MISC_H
#define _MISC_H

#include <stdint.h>

#define MAX_MA (10)
typedef struct _ma
{
	uint16_t T[MAX_MA]; // array with delayed values of input signal
	uint8_t  index;     // index in T[] where to store the new input value
	uint8_t  N;         // The order of the MA filter. Note that N < MAX_MA
	uint16_t sum;       // The running sum of the MA filter
} ma;

void     init_moving_average(ma *p, uint8_t N, uint16_t init_val);
uint16_t moving_average(ma *p, uint16_t x);
void   init_sample_delay(ma *p, int TD);
double sample_delay(ma *p, double x);
void   slope_limiter(const double lim, const double Told, double *Tnew);

#endif