//-----------------------------------------------------------------------------
// Created: 20-4-2013 22:32:11
// Author : Emile
// File   : Brew_Arduino.c
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
#include "misc.h"

void init_moving_average(ma *p, uint8_t N, int16_t init_val)
/*------------------------------------------------------------------
  Purpose  : This function initializes the Moving Average (MA) struct
             that is used in the moving_average() function
  Variables:
        *p : Pointer to the ma struct (specify a new struct for every
             new filter!)
         N : The order of the Moving Average filter (1 = No filtering)
   init_val: The value to init. the MA filter to
  Returns  : -
  ------------------------------------------------------------------*/
{
   uint8_t i; // temp. var.

   p->N     = N;        // order of MA filter
   p->index = 0;        // index in cyclic array
   p->sum   = init_val; // running sum
   for (i = 0; i < N; i++)
   {
      p->T[i] = init_val / N; // set cyclic array to init. value
   } // for
} // init_moving_average()

int16_t moving_average(ma *p, int16_t x)
/*------------------------------------------------------------------
  Purpose  : This function calculates the Moving Average (MA) of the
             input signal x. An MA filter of order N is a low-pass
             filter with a zero at fs / N (or 2.pi/N). It's Z transform
             function is:
                                 -1         -(N-1)
             Y[z] = X[z] . (1 + z  + ... + Z      )

             Initialization: p->N must have a value.
                             p->sum and p->index should be init. to 0.
  Variables:
        *p : Pointer to the ma struct (specify a new struct for every
             new filter!)
         x : The actual input value
  Returns  : Filter output value
  ------------------------------------------------------------------*/
{
   p->sum -= p->T[p->index];                // subtract value from running sum
   p->T[p->index] = (x + (p->N>>1)) / p->N; // store new value in array
   p->sum += p->T[p->index];                // update running sum with new value
   if (++(p->index) >= p->N)                // update index in cyclic array
   {
      p->index = 0; // restore to 1st position
   } // if
   return p->sum;   // return value = filter output
} // moving_average()

void init_sample_delay(ma *p, int TD)
/*------------------------------------------------------------------
  Purpose  : This function initializes the sample delay function.
             This function uses the same struct as the moving_average()
             function uses (the MA struct).
  Variables:
        *p : Pointer to the ma struct (specify a new struct for every
             new filter!)
        TD : The number of sample-time delays (should be < MAX_MA)
  Returns  : -
  ------------------------------------------------------------------*/
{
   int i; // temp. var.

   if (TD < MAX_MA) p->N = TD;    // number of sample-time delays
   else             p->N = MAX_MA;
   p->index = 0;                  // index in cyclic array
   for (i = 0; i < p->N; i++)
   {
      p->T[i] = 0.0; // init. all stores to 0
   } // for
} // init_sample_delay()

double sample_delay(ma *p, double x)
/*------------------------------------------------------------------
  Purpose  : This function delays a signal N samples and outputs the
             value x[k-N] (N is defined in init_sample_delay()):

             Initialisation: p->N must have a value.
                             p->index should be init. to 0.0.
  Variables:
        *p : Pointer to the ma struct (specify a new struct for every
             new filter!)
         x : The actual input value x[k]
  Returns  : the delayed input value x[k-N]
  ------------------------------------------------------------------*/
{
   double xn;     // get x[k-N]
   if (p->N == 0) // no delay
   {
       xn = x; // return input value directly
   }
   else
   {
      xn = p->T[p->index]; // get x[k-N]
      p->T[p->index] = x;         // store x[k] in array
      if (++(p->index) >= p->N)   // update index in cyclic array
      {
         p->index = 0; // restore to 1st position
      } // if
   } // else
   return xn;      // return value = x[k-N]
} // sample_delay()

void slope_limiter(const int16_t lim, const int16_t Told, int16_t *Tnew)
/*------------------------------------------------------------------
  Purpose  : This function limits the increase of Tnew by lim.

                              Tnew
                               |  ------------  lim
                               |/
              -----------------/------------------ -> Tin - Tout
                              /|
              -lim ----------  |

  Variables:
       lim : The limiting value
      Told : The previous value of the variable to limit
      Tnew : The output value of the variable to limit
  Returns  : none
  ------------------------------------------------------------------*/
{
   int16_t diff = *Tnew - Told; // calculate difference

   if      (diff > lim)  *Tnew =  Told + lim;
   else if (diff < -lim) *Tnew =  Told - lim;
   // else: do nothing, Tnew is within slope limits
} // slope_limiter()
