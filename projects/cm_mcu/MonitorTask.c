/*
 * MonitorTask.c
 *
 *  Created on: May 19, 2019
 *      Author: wittich
 *
 * Monitor temperatures, voltages, currents, via I2C/PMBUS. Generic,
 * pass in addresses via parameter to the task.
 *
 * It only handles PMBUS, not raw I2C.
 *
 */

// includes for types
#include <stdint.h>
#include <stdbool.h>

// memory mappings
#include "inc/hw_types.h"
#include "inc/hw_memmap.h"

// FreeRTOS
#include "FreeRTOS.h"
#include "FreeRTOSConfig.h"
#include "task.h"

// local includes
#include "common/i2c_reg.h"
#include "common/utils.h"
#include "common/smbus.h"
#include "common/log.h"
#include "common/printf.h"
#include "common/smbus_units.h"
#include "common/power_ctl.h"
#include "MonitorTask.h"
#include "Tasks.h"

// Todo: rewrite to get away from awkward/bad SMBUS implementation from TI

// the PAGE command is an SMBUS standard at register 0
#define PAGE_COMMAND 0x0

// break out of loop, releasing semaphore if we have it
#define release_break()           \
  {                               \
    if (args->xSem != NULL)       \
      xSemaphoreGive(args->xSem); \
    break;                        \
  }

void MonitorTask(void *parameters)
{
  // initialize to the current tick time
  TickType_t xLastWakeTime = xTaskGetTickCount();
  uint8_t data[2];

  struct MonitorTaskArgs_t *args = parameters;

  configASSERT(args->name != 0);

  bool log = true;
  args->updateTick = xLastWakeTime; // initial value

  // wait for the power to come up
  vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(500));

  bool isFullyPowered = false; // assume not fully powered
  for (;;) {
    // grab the semaphore to ensure unique access to I2C controller
    if (args->xSem != NULL) {
      while (xSemaphoreTake(args->xSem, (TickType_t)10) == pdFALSE)
        ;
    }
    args->updateTick = xTaskGetTickCount(); // current time in ticks
    // loop over devices
    for (int ps = 0; ps < args->n_devices; ++ps) {
      // handle case where only management power is on
      if (args->requirePower) { // if this device requires more than management power
        enum power_system_state power_state = getPowerControlState();
        if (power_state != POWER_ON) { // if the power state is not fully on
          if (isFullyPowered) {        // was previously on
            log_info(LOG_MON, "%s: PWR off. Disabling I2C monitoring.\r\n", args->name);
            isFullyPowered = false;
          }
          release_break(); // skip this iteration
        }
        else {                   // if the power state is fully on
          if (!isFullyPowered) { // was previously off
            log_info(LOG_MON, "%s: PWR on. (Re)starting I2C monitoring.\r\n", args->name);
            isFullyPowered = true;
          }
        }
        // if the power state is unknown, don't do anything
      }

      // select the appropriate output for the mux
      data[0] = 0x1U << args->devices[ps].mux_bit;
      log_trace(LOG_MON, "%s: mux to 0x%02x\r\n", args->name, data[0]);
      tSMBusStatus r = SMBusMasterI2CWrite(args->smbus, args->devices[ps].mux_addr, data, 1);
      if (r != SMBUS_OK) {
        log_debug(LOG_MON, "%s: I2CBus command failed  (setting mux)\r\n", args->name);
        continue;
      }
      int tries = 0;
      while (SMBusStatusGet(args->smbus) == SMBUS_TRANSFER_IN_PROGRESS) {
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(10)); // wait
        if (++tries > 500) {
          log_warn(LOG_MON, "timed out (SMBUSxfer mux) (%s, dev=%d)\r\n",
                   args->name, ps);
          break;
        }
      }
      if (*args->smbus_status != SMBUS_OK) {
        log_trace(LOG_MON, "%s:Mux w error %d, break (ps=%d)\r\n", args->name, *args->smbus_status,
                  ps);
        release_break();
      }

      // loop over pages on the supply
      for (uint8_t page = 0; page < args->n_pages; ++page) {
        r = SMBusMasterByteWordWrite(args->smbus, args->devices[ps].dev_addr, PAGE_COMMAND, &page,
                                     1);
        if (r != SMBUS_OK) {
          log_warn(LOG_MON, "SMBUS page failed %s\r\n", args->name);
        }
        while (SMBusStatusGet(args->smbus) == SMBUS_TRANSFER_IN_PROGRESS) {
          vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(10)); // wait
        }
        // this is checking the return from the interrupt
        if (*args->smbus_status != SMBUS_OK) {
          log_warn(LOG_MON, "%s: Page SMBUS ERROR: %d\r\n", args->name, *args->smbus_status);
        }
        log_trace(LOG_MON, "%s: Page %d\r\n", args->name, page);

        // loop over commands
        for (int c = 0; c < args->n_commands; ++c) {
          int index = ps * (args->n_commands * args->n_pages) + page * args->n_commands + c;
          args->pm_values[index] = __builtin_nanf("");

          data[0] = 0x0U;
          data[1] = 0x0U;
          r = SMBusMasterByteWordRead(args->smbus, args->devices[ps].dev_addr,
                                      args->commands[c].command, data, args->commands[c].size);
          if (r != SMBUS_OK) {
            log_warn(LOG_MON, "%s: SMBUS failed (master/bus busy, (ps=%d,c=%d,p=%d)\r\n", args->name,
                     ps, c, page);
            continue; // abort reading this register
          }
          tries = 0;
          while (SMBusStatusGet(args->smbus) == SMBUS_TRANSFER_IN_PROGRESS) {
            vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(10)); // wait
            if (++tries > 500) {
              log_warn(LOG_MON, "timed out (SMBUSxfer) (%s, c=%d,dev=%d)\r\n",
                       args->name, c, ps);
              break;
            }
          }
          if (*args->smbus_status != SMBUS_OK) {
            log_warn(LOG_MON, "%s: Error %d, break out of loop (ps=%d,c=%d,p=%d) ...\r\n", args->name,
                     *args->smbus_status, ps, c, page);
            // abort reading this device
            if (log)
              errbuffer_put(EBUF_I2C, (uint16_t)args->name[0]);
            release_break();
          }
          float val;
          if (args->commands[c].type == PM_LINEAR11) {
            linear11_val_t ii;
            ii.raw = (data[1] << 8) | data[0];
            val = linear11_to_float(ii);
          }
          else if (args->commands[c].type == PM_LINEAR16U) {
            uint16_t ii = (data[1] << 8) | data[0];
            val = linear16u_to_float(ii);
          }
          else if (args->commands[c].type == PM_STATUS) {
            // Note: this assumes 2 byte xfer and endianness and converts and int to a float
            val = (float)((data[1] << 8) | data[0]); // ugly is my middle name
          }
          else {
            val = -98.0f; // should never get here
          }
          args->pm_values[index] = val;
          // wait here for the x msec, where x is 2nd argument below.
          vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(10));
        }                   // loop over commands
      }                     // loop over pages
    }                       // loop over power supplies
    if (args->xSem != NULL) // if we have a semaphore, give it
      xSemaphoreGive(args->xSem);

    CHECK_TASK_STACK_USAGE(args->stack_size);

    vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(250));
  } // infinite loop
}
