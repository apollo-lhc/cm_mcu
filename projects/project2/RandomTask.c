/*
 * RandomTask.c
 *
 *  Created on: May 19, 2019
 *      Author: wittich
 */

// includes for types
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>

// memory mappings
#include "inc/hw_types.h"
#include "inc/hw_memmap.h"

// FreeRTOS
#include "FreeRTOS.h"
#include "FreeRTOSConfig.h"
#include "task.h"

// local includes
#include "common/i2c_reg.h"
#include "common/smbus.h"
#include "common/smbus_units.h"



// playground to test various things
void RandomTask(void *parameters)
{
  // initialize to the current tick time
  TickType_t xLastWakeTime = xTaskGetTickCount();

  for (;;) {
    // do stuff
    vTaskDelayUntil( &xLastWakeTime, pdMS_TO_TICKS( 1000 ) );

  }

}
