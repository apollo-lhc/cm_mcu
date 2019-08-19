/*
 * smbus_units.c
 *
 *  Created on: Jul 17, 2019
 *      Author: pw94
 */
#include "common/smbus_units.h"

float linear11_to_float(linear11_val_t t)
{
  return t.linear.base * pow(2, t.linear.mantissa);
}

float linear16u_to_float(uint16_t t )
{
  const float mantissa = -13;
  return 1.0*t*pow(2,mantissa);
}

uint16_t float_to_linear11(float t)
{
  return (uint16_t)(t * (1 << 5));
}
