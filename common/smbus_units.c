/*
 * smbus_units.c
 *
 *  Created on: Jul 17, 2019
 *      Author: pw94
 */
#include "common/smbus_units.h"
#include "common/utils.h"

float linear11_to_float(linear11_val_t t)
{
  // this gymnastics allows us to avoid a call to powf and use
  // bitshifts. 
  float val = (float)(1<<ABS((signed char)t.linear.mantissa));
  if ( t.linear.mantissa < 0.f )
    return t.linear.base/val;
  else 
    return t.linear.base * val;
}

#define LIN16U_MANTISSA -13.0f
float linear16u_to_float(uint16_t t )
{
  return 1.0f*t*(float)powf(2.0f,LIN16U_MANTISSA);
}

uint16_t float_to_linear11_2(float t)
{
  return (uint16_t)(t * (1 << 5));
}


/*
 * Convert a floating point value into a * LinearFloat5_11 formatted word
 */
#define MAX_MANTISSA	(1023)
#define MIN_MANTISSA	(511)
#define INITIAL_EXPONENT -16
uint16_t float_to_linear11(float input_val)
{
  // set exponent to -16
  int exponent = -16;
  bool negative = false;
  if ( input_val < 0 ) {
    negative = true;
    input_val = - input_val;
  }
  // extract mantissa from input value
  int mantissa = (int)(input_val / (float)powf(2.0, INITIAL_EXPONENT));

  // Search for an exponent that produces
  // a valid 11-bit mantissa

  // Reduce large mantissa until it fits into 10 bit 
  while (mantissa >= MAX_MANTISSA && exponent < 15) {
    exponent++;
    mantissa >>= 1;
  }
  // Increase small mantissa to improve precision
  while (mantissa < MIN_MANTISSA && exponent > -15) {
    exponent--;
    mantissa <<= 1;
  }

  // restore sign
  if (negative) {
    mantissa = -mantissa;
  }

  // Format the exponent of the L11
  uint16_t uExponent = exponent << 11;
  // Format the mantissa of the L11
  uint16_t uMantissa = mantissa & 0x07FF;
  // Compute value as exponent | mantissa
  return uExponent | uMantissa;
}
