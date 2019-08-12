/*
 * smbus_units.h
 *
 *  Created on: May 23, 2019
 *      Author: wittich
 */

#ifndef _COMMON_SMBUS_UNITS_H
#define _COMMON_SMBUS_UNITS_H
#include <stdbool.h>
#include <stdint.h>
#include <math.h>


typedef struct
{
  int16_t base : 11;
  int16_t mantissa : 5;
} linear11_t;

typedef union
{
  linear11_t linear;
  uint16_t raw;
} linear11_val_t;

float linear11_to_float(linear11_val_t t);

float linear16u_to_float(uint16_t t );

uint16_t float_to_linear11(float t);

#endif // _COMMON_SMBUS_UNITS_H
