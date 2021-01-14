/*
 * parameters.h
 *
 *  Created on: Jan 13, 2021
 *      Author: fatimayousuf
 */

#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_types.h"
#include "inc/hw_memmap.h"
#include "inc/hw_nvic.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "driverlib/systick.h"
#include "driverlib/sysctl.h"
#include "driverlib/eeprom.h"

int snprintf(char *buf, unsigned int count, const char *format, ...);

#define MAX_INPUT_LENGTH  50
#define MAX_OUTPUT_LENGTH 512


