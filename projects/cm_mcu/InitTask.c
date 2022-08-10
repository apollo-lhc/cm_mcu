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
#include "common/log.h"
#include "Tasks.h"


void InitTask(void *parameters)
{

  //  store the reboot into the error buffer, including the reason for the reset
  uint32_t r = ROM_SysCtlResetCauseGet();
  uint16_t restart_reason = (uint16_t)0xFFFFUL & r;
  // clear RESC register
  ROM_SysCtlResetCauseClear(r);
  errbuffer_put(EBUF_RESTART, restart_reason);
  log_info(LOG_SERVICE, "REC register=0x%08x\r\n", restart_reason);

// wait for 3.3V power to come up. Wait indefinitely.
// in Rev1 the clocks cannot be accessed before the 3.3 V is on.
#ifdef REV1 
  while ( getPowerControlState() != POWER_ON) {
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
#endif // REV1
  init_registers_clk(); // initalize I/O expander for clocks
  log_info(LOG_SERVICE, "Clock I/O expander initialized\r\n");
#ifdef REV2
  init_load_clk(0);
  init_load_clk(2);
  log_info(LOG_SERVICE, "Clocks configured\r\n");
#endif // REV2
  vTaskSuspend(NULL);

  // Delete this task
  vTaskDelete(xTaskGetCurrentTaskHandle());
}
