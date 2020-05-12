/*
 * InitTask.c
 *
 *  Created on: Feb 7, 2020
 *      Author: glg62
 */

// includes for types
#include <stdint.h>
#include <stdbool.h>

#include "FreeRTOS.h"
#include "FreeRTOSConfig.h"
#include "task.h"

#include "driverlib/rom.h"
#include "driverlib/eeprom.h"
#include "driverlib/sysctl.h"
#include "common/utils.h"
#include "Tasks.h"


void InitTask(void *parameters)
{

  // store the reboot into the error buffer, including the reason for the reset
  uint32_t r =  ROM_SysCtlResetCauseGet();
  uint16_t restart_reason = (uint16_t) 0xFFFFUL&r;
  // clear RESC register
  ROM_SysCtlResetCauseClear(r);
  errbuffer_put(ebuf,EBUF_RESTART, restart_reason);


  // Delete this task
  vTaskDelete(xTaskGetCurrentTaskHandle());
}
