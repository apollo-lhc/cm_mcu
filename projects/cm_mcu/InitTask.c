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
  init_registers_ff(); // initalize I/O expander for fireflies -- with FF monitoring via I2C in other threads, it grabs semaphore inside

  init_registers_clk(); // initalize I/O expander for clocks -- with clock monitoring via I2C in other threads, it grabs semaphore inside
  log_info(LOG_SERVICE, "Clock I/O expander initialized\r\n");
#ifdef REV2
  init_load_clk(0); // load clock r0a config from EEPROM
  // init_load_clk(1); // load clock r0b config from EEPROM but no data to load as of 08.04.222
  init_load_clk(2); // load clock r1a config from EEPROM
  init_load_clk(3); // load clock r1b config from EEPROM
  // init_load_clk(4); // load clock r1c config from EEPROM but no data  to load as of 08.04.222
  log_info(LOG_SERVICE, "Clocks configured\r\n");
  getFFpart(); // the order of where to check FF part matters -- it won't be able to read anything if check sooner
#endif         // REV2
  vTaskSuspend(NULL);

  // Delete this task
  vTaskDelete(xTaskGetCurrentTaskHandle());
}
