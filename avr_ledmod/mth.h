
#ifndef MTH_H
#define MTH_H

/**
 * Some helpful functions.
 *
 * Matthias Hannig in the end of 2013 
 */

#include <math.h>

inline float m_pulse( float k, float x ) 
{
  return k*x * exp( 1.0f - (k*x) );
}


inline float m_pulse_to( float k, float base, float x )
{
  return k*x*(1.0f - base)*exp( 1.0f - (k*x) ) + base;
}


// k = something like 6
inline float m_decay( float k, float x )
{
  return exp( -k*x );
}

inline float m_smoothstep( float x0, float x1, float x ) 
{
  x = (x-x0)/(x1-x0);
  return x*x * (3.0f - 2.0f*x);
}


#endif 

