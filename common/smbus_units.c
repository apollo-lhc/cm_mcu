/*
 * smbus_units.c
 *
 *  Created on: Jul 17, 2019
 *      Author: pw94
 */
#include "common/smbus_units.h"

float linear11_to_float(linear11_val_t t)
{
  // can't use bit shifts here as mantissa is often negative
  return t.linear.base * pow(2, t.linear.mantissa);
}

float linear16u_to_float(uint16_t t )
{
  const int mantissa = -13; // can't use shifts here since <0
  return 1.0*t*pow(2,mantissa);
}

uint16_t float_to_linear11_2(float t)
{
  return (uint16_t)(t * (1 << 5));
}


/*
 * Convert a floating point value into a * LinearFloat5_11 formatted word
 */
uint16_t float_to_linear11(float input_val)
{
  // set exponent to -16
  int exponent = -16;
  // extract mantissa from input value
  int mantissa = (int)(input_val / pow(2.0, exponent));
  // Search for an exponent that produces
  // a valid 11-bit mantissa
  do
  {
    if ((mantissa >= -1024) && (mantissa <= 1023)) {
      break; // stop if mantissa valid
    }
    exponent++;
    mantissa = (int)(input_val / pow(2.0, exponent));
  } while (exponent < 15);
  // Format the exponent of the L11
  uint16_t uExponent = exponent << 11; // Format the mantissa of the L11
  uint16_t uMantissa = mantissa & 0x07FF;
  // Compute value as exponent | mantissa
  return uExponent | uMantissa;
}
