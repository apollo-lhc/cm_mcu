/*
 * InitTask.c
 *
 *  Created on: Feb 7, 2020
 *      Author: glg62
 */

// includes for types
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>

#include "FreeRTOS.h"
#include "FreeRTOSConfig.h"
#include "task.h"

#include "driverlib/eeprom.h"
#include "common/utils.h"
#include "Tasks.h"


void InitTask(void *parameters)
{

  // Initialize eeprom error buffer
  errbuffer_init(ebuf,EBUF_MINBLK,EBUF_MAXBLK);
  errbuffer_put(ebuf,RESTART,0);


  // Delete this task
  vTaskDelete(xTaskGetCurrentTaskHandle());
}
