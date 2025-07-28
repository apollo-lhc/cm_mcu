/*
 * InitTask.c
 *
 *  Created on: Feb 7, 2020
 *      Author: glg62
 */

// includes for types
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include "FireflyUtils.h"
#include "task.h"

#include "driverlib/rom.h"
#include "common/utils.h"
#include "common/log.h"
#include "Tasks.h"
#include "clocksynth.h"

void InitTask(void *parameters)
{
  //  store the reboot into the error buffer, including the reason for the reset
  uint32_t r = ROM_SysCtlResetCauseGet();
  uint16_t restart_reason = (uint16_t)0xFFFFUL & r;
  // clear RESC register
  ROM_SysCtlResetCauseClear(r);
  errbuffer_put(EBUF_RESTART, restart_reason);
  log_info(LOG_SERVICE, "REC register=0x%08x\r\n", restart_reason);

  // get board information
  uint32_t id;
  uint32_t rev;
  get_board_info(&rev, &id);
  log_info(LOG_SERVICE, "Board ID: %d, Revision: %d\r\n", id, rev);

// wait for 3.3V power to come up. Wait indefinitely.
// in Rev1 the clocks cannot be accessed before the 3.3 V is on.
#ifdef REV1
  while (getPowerControlState() != POWER_ON) {
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
#endif                               // REV1
  init_registers_ff();               // initialize I/O expander for fireflies -- with FF monitoring via I2C in other threads, it grabs semaphore inside
  int status = init_registers_clk(); // initialize I/O expander for clocks
  readFFpresent();
  if (status == 0)
    log_info(LOG_SERVICE, "Clock I/O expander initialized\r\n");
  else {
    log_info(LOG_SERVICE, "Clock I/O expander failed\r\n");
    errbuffer_put(EBUF_CLKINIT_FAILURE, 0);
  }
  vTaskSuspend(NULL);
  // Delete this task
  vTaskDelete(xTaskGetCurrentTaskHandle());
}
