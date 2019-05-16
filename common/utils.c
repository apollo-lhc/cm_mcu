/*
 * utils.c
 *
 *  Created on: May 9, 2019
 *      Author: wittich
 */

#include "common/utils.h"
#include "common/pinsel.h"
#include "driverlib/gpio.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "driverlib/debug.h"

// write by pin number or name
void write_gpio_pin(int pin, uint8_t value)
{
  ASSERT(value==1||value==0);
  uint32_t gport;
  uint8_t  gpin;
  pinsel(pin, &gport, &gpin);
  uint8_t pinval;
  if ( value == 1 )
	  pinval = gpin;
  else
	  pinval = 0;
  MAP_GPIOPinWrite(gport, gpin, pinval);
  return;
}

// write by pin number or name
uint8_t read_gpio_pin(int pin)
{
  uint32_t gport;
  uint8_t  gpin;
  uint8_t value;
  pinsel(pin, &gport, &gpin);
  value = MAP_GPIOPinRead(gport, gpin);
  if ( value ) value = 1;
  return value;
}

// write by pin number or name
uint8_t toggle_gpio_pin(int pin)
{
  uint8_t val = read_gpio_pin(pin);
  if ( val ) val = 0;
  else
	  val = 1;
  write_gpio_pin(pin, val);
  return val;
}

// Set up all the active low pins to be off (i.e., high)
void setupActiveLowPins(void)
{
  const int pins[] = {
      _FPGA_I2C_RESET,
      _PWR_I2C_RESET,
      _CLOCKS_I2C_RESET,
      _V_OPTICS_I2C_RESET,
      _K_OPTICS_I2C_RESET
  };
  const int8_t npins = sizeof(pins)/pins[0];
  for (int8_t i = 0; i < npins; ++i ) {
    write_gpio_pin(pins[i], 0x1);
  }
}

