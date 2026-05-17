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

// FreeRTOS
#include "FreeRTOS.h" // IWYU pragma: keep
#include "FreeRTOSConfig.h"
#include "task.h"

// local includes
#include "common/utils.h"
#include "common/smbus.h"
#include "common/smbus_helper.h"
#include "common/log.h"
#include "common/smbus_units.h"
#include "MonitorTask.h"
#include "Tasks.h"
#include "Semaphore.h"
#include "I2CCommunication.h"

// Todo: rewrite to get away from awkward/bad SMBUS implementation from TI

// the PAGE command is an SMBUS standard at register 0
#define PAGE_COMMAND 0x0

void MonitorTask(void *parameters)
{
  // initialize to the current tick time
  TickType_t xLastWakeTime = xTaskGetTickCount();
  uint8_t data[2];

  struct MonitorTaskArgs_t *args = parameters;

  configASSERT(args->name != 0);

  // derive the I2C bus index (1-6) from the controller pointer so we can use
  // the notification-based apollo_i2c_ctl_* wrappers.
  uint8_t i2c_dev = smbus_get_device_index(args->smbus);
  configASSERT(i2c_dev != 0);

  bool log = false;
  args->updateTick = xLastWakeTime; // initial value

  // wait for the power to come up
  vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(500));

  bool isFullyPowered = false; // assume not fully powered
  for (;;) {
    // grab the semaphore to ensure unique access to I2C controller
    if (args->xSem != NULL) {
      if (acquireI2CSemaphore(args->xSem) == pdFAIL) {
        log_warn(LOG_SERVICE, "%s could not get semaphore in time; continue\r\n", args->name);
        continue;
      }
    }

    // loop over devices
    for (int ps = 0; ps < args->n_devices; ++ps) {
      // handle case where only management power is on
      if (args->requirePower) { // if this device requires more than management power
        enum power_system_state power_state = getPowerControlState();
        if (power_state != POWER_ON) { // if the power state is not fully on
          if (isFullyPowered) {        // was previously on
            log_info(LOG_MON, "%s: PWR off. Disabling *BUS monitoring.\r\n", args->name);
            isFullyPowered = false;
            // clear the now-stale data by setting to sentinel value -999
            for (int i = 0; i < args->n_values; ++i)
              args->pm_values[i] = -999.f;
          }
          break; // skip this iteration
        }
        else {                   // if the power state is fully on
          if (!isFullyPowered) { // was previously off
            log_info(LOG_MON, "%s: PWR on. (Re)start PMBUS mon\r\n", args->name);
            isFullyPowered = true;
          }
        }
        // if the power state is unknown, don't do anything
      }

      // select the appropriate output for the mux
      data[0] = 0x1U << args->devices[ps].mux_bit;
      log_trace(LOG_MON, "%s: mux to 0x%02x\r\n", args->name, data[0]);
      int r = apollo_i2c_ctl_w(i2c_dev, args->devices[ps].mux_addr, 1, data[0]);
      if (r != SMBUS_OK) { // mux should always be accessible
        log_warn(LOG_MON, "%s:Mux w error %s, break (ps=%d)\r\n", args->name, SMBUS_get_error(r), ps);
        break;
      }

      // loop over pages on the supply
      for (uint8_t page = 0; page < args->n_pages; ++page) {
        r = apollo_i2c_ctl_reg_w(i2c_dev, args->devices[ps].dev_addr, 1, PAGE_COMMAND, 1, page);
        if (r != SMBUS_OK) {
          if (!args->ignoreNACK && SMBUS_is_NACK(r)) {
            log_warn(LOG_MON, "%s: Page SMBUS ERROR: %s\r\n", args->name, SMBUS_get_error(r));
          }
          continue;
        }
        log_trace(LOG_MON, "%s: Page %d\r\n", args->name, page);

        // loop over commands
        for (int c = 0; c < args->n_commands; ++c) {
          int index = ps * (args->n_commands * args->n_pages) + page * args->n_commands + c;

          uint32_t output_raw = 0;
          r = apollo_i2c_ctl_reg_r(i2c_dev, args->devices[ps].dev_addr, 1,
                                   args->commands[c].command, args->commands[c].size, &output_raw);
          if (r != SMBUS_OK) {
            if (!args->ignoreNACK && SMBUS_is_NACK(r)) {
              log_warn(LOG_MON, "%s: Error %s, break out of loop (ps=%d,c=%d,p=%d) ...\r\n", args->name,
                       SMBUS_get_error(r), ps, c, page);
            }
            // abort reading this device
            args->pm_values[index] = __builtin_nanf("");
            if (log)
              errbuffer_put(EBUF_I2C, (uint16_t)args->name[0]);
            break;
          }
          data[0] = output_raw & 0xFFU;
          data[1] = (output_raw >> 8) & 0xFFU;
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
          args->updateTick = xTaskGetTickCount(); // current time in ticks, for the sake of stale data
          vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(10));
        } // loop over commands
      }   // loop over pages
    }     // loop over power supplies
    // if we have a semaphore, give it
    if (xSemaphoreGetMutexHolder(args->xSem) == xTaskGetCurrentTaskHandle()) {
      xSemaphoreGive(args->xSem);
    }

    CHECK_TASK_STACK_USAGE(args->stack_size);

    vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(250));
  } // infinite loop
}
