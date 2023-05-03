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
#include "MonitorI2CTask.h"
#include "Semaphore.h"
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
  }
#ifdef REV2
  // grab the semaphore to ensure unique access to I2C controller
  // otherwise, block its operations indefinitely until it's available
  acquireI2CSemaphoreBlock(i2c2_sem);

  for (int i = 0; i < 5; ++i) {
    init_load_clk(i); // load each clock config from EEPROM
  }
  status = clear_clk_stickybits();
  if (status == 0)
    log_info(LOG_SERVICE, "Clear clock sticky bits\r\n");
  else {
    log_info(LOG_SERVICE, "Clear clock sticky bits failed\r\n");
  }
  // check if we have the semaphore
  if (xSemaphoreGetMutexHolder(i2c2_sem) == xTaskGetCurrentTaskHandle()) {
    xSemaphoreGive(i2c2_sem);
  }

  log_info(LOG_SERVICE, "Clocks configured\r\n");

  // check 4-ch FF parts from vendors on FPGA1/2
  getFFpart_FPGA1();
  getFFpart_FPGA2();

#endif // REV2
  vTaskSuspend(NULL);
  // Delete this task
  vTaskDelete(xTaskGetCurrentTaskHandle());
}
