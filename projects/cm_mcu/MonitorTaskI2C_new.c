/*
 * MonitorI2CTask_new.c
 *
 *  Created on: June 30, 2022
 *      Author: pkotamnives
 *      Monitor temperatures, and statuses of firefly ports via I2C BUS
 * pass in addresses via parameter to the task.
 */

// includes for types
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

// FreeRTOS
#include "FreeRTOS.h" // IWYU pragma: keep
#include "FreeRTOSConfig.h"
#include "task.h"

// local includes
#include "common/smbus_helper.h"
#include "MonitorTaskI2C_new.h"
#include "common/log.h"
#include "Tasks.h"
#include "I2CCommunication.h"
#include "Semaphore.h"

// local prototype

// read-only accessor functions for Firefly names and values.
#if 0
bool getFFch_low(uint8_t val, int channel)
{
  configASSERT(channel < 8);
  if (!((1 << channel) & val)) {
    return false;
  }
  return true;
}

bool getFFch_high(uint8_t val, int channel)
{
  configASSERT(channel >= 8);
  if (!((1 << (channel - 8)) & val)) {
    return false;
  }
  return true;
}
#endif

// Monitor registers of FF temperatures, voltages, currents, and ClK statuses via I2C
void MonitorI2CTask_new(void *parameters)
{

  struct MonitorI2CTaskArgs_new_t *args = parameters;

  configASSERT(args->name != 0);

  // watchdog info
  task_watchdog_register_task(kWatchdogTaskID_MonitorI2CTask);

  // wait for the power to come up
  vTaskDelayUntil(&(args->updateTick), pdMS_TO_TICKS(5000));

  // initialize to the current tick time
  args->updateTick = xTaskGetTickCount();

  bool good = false;
  for (;;) {

    log_debug(LOG_MONI2C, "%s: grab sem\r\n", args->name);

    // grab the semaphore to ensure unique access to I2C controller
    if (args->xSem != NULL) {
      if (acquireI2CSemaphore(args->xSem) == pdFAIL) {
        log_debug(LOG_SERVICE, "%s could'nt get sem; delay & continue\r\n", args->name);
        vTaskDelayUntil(&(args->updateTick), pdMS_TO_TICKS(10)); // wait
        continue;
      }
    }

    // -------------------------------
    // loop over devices in the device-type instance
    // -------------------------------
    for (int device = 0; device < args->n_devices; ++device) {
      if (args->presentCallback && !args->presentCallback(device)) {
        log_debug(LOG_MONI2C, "%s: device %d not present\r\n", args->name, device);
        continue;
      }
      log_debug(LOG_MONI2C, "%s: device %d powercheck\r\n", args->name, device);
      if (getPowerControlState() != POWER_ON) {
        if (good) {
          log_info(LOG_MONI2C, "%s: PWR off. Disable I2Cmon.\r\n", args->name);
          good = false;
          task_watchdog_unregister_task(kWatchdogTaskID_MonitorI2CTask);
        }
        if (xSemaphoreGetMutexHolder(args->xSem) == xTaskGetCurrentTaskHandle()) {
          xSemaphoreGive(args->xSem);
        }
        break;
      }
      else if (getPowerControlState() == POWER_ON) { // power is on, and ...
        if (!good) {                                 // ... was not good, but is now good
          task_watchdog_register_task(kWatchdogTaskID_MonitorI2CTask);
          log_info(LOG_MONI2C, "%s: PWR on. (Re)start I2Cmon.\r\n", args->name);
          good = true;
        }
      }

      // select the appropriate output for the mux
      uint8_t data;
      data = 0x1U << args->devices[device].mux_bit;
      log_debug(LOG_MONI2C, "Mux set to 0x%02x\r\n", data);
      int res = apollo_i2c_ctl_w(args->i2c_dev, args->devices[device].mux_addr, 1, data);
      if (res != 0) {
        log_warn(LOG_MONI2C, "Mux write error %s, break (instance=%s,ps=%d)\r\n", SMBUS_get_error(res), args->name, device);
        break;
      }
      uint8_t last_page_reg_value = 0xff;

      // what kind of device is this
      uint32_t devtype_mask = args->typeCallback(device);
      uint32_t clz = __builtin_clz(devtype_mask);
      uint32_t devtype = 32 - __builtin_clz( devtype_mask); // highest bit set FIXME: this is backwards
      // Loop to read I2C registers/commands
      for (int c = 0; c < args->n_commands; ++c) {
        // check if the command is for this device
        // what kind of device do we have (e.g., 4 ch FF, 12 ch 25 G FF, 12 ch CERN-B FF, etc.)
        if ((args->commands[c].devicelist() & devtype_mask) == 0) {
          continue; // not for me!
        }

        // set page register if it's different than the last time
        log_debug(LOG_MONI2C, "%s: reg %s\r\n", args->name, args->commands[c].name);
        uint8_t page_reg_value = args->commands[c].page;
        if (page_reg_value != last_page_reg_value) {
          int r = apollo_i2c_ctl_reg_w(args->i2c_dev, args->devices[device].dev_addr, 1, args->selpage_reg, 1, page_reg_value);
          if (r != 0) {
            log_error(LOG_MONI2C, "%s : page fail %s\r\n", args->devices[device].name, SMBUS_get_error(r));
            break;
          }
          last_page_reg_value = page_reg_value;
        }

        // get the data from the I2C register
        uint32_t output_raw;
        int res = apollo_i2c_ctl_reg_r(args->i2c_dev, args->devices[device].dev_addr, args->commands[c].reg_size,
                                       args->commands[c].command[devtype], args->commands[c].size, &output_raw);

        if (res != 0) {
          log_error(LOG_MONI2C, "%s: %s read Error %s, break (ps=%d)\r\n",
                    args->name, args->commands[c].name, SMBUS_get_error(res), device);
          args->commands[c].storeData(0xffff, device); // store error value
          break;
        }
        else {
          uint16_t masked_output = output_raw & args->commands[c].bit_mask;
          args->commands[c].storeData(masked_output, device);
        }

      } // loop over commands

      log_debug(LOG_MONI2C, "%s: end loop commands\r\n", args->name);
      args->updateTick = xTaskGetTickCount(); // current time in ticks

      // clear out the I2C mux
      res = apollo_i2c_ctl_w(args->i2c_dev, args->devices[device].mux_addr, 1, 0);
      if (res != 0) {
        log_warn(LOG_MONI2C, "Mux write error %s, break (instance=%s,ps=%d)\r\n", SMBUS_get_error(res), args->name, device);
        break;
      }
      log_debug(LOG_MONI2C, "%s: reset mux\r\n", args->name);

    } // loop over devices

    // if we have a semaphore, give it
    if (xSemaphoreGetMutexHolder(args->xSem) == xTaskGetCurrentTaskHandle()) {
      xSemaphoreGive(args->xSem);
    }

    // monitor stack usage for this task
    CHECK_TASK_STACK_USAGE(args->stack_size);

    // task_watchdog_feed_task(kWatchdogTaskID_MonitorI2CTask);
    vTaskDelayUntil(&(args->updateTick), pdMS_TO_TICKS(250));
  } // infinite loop for task
}
