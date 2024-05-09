/*
 * MonitorI2CTask.c
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
#include "MonitorI2CTask.h"
#include "common/log.h"
#include "Tasks.h"
#include "I2CCommunication.h"
#include "Semaphore.h"

// local prototype

// read-only accessor functions for Firefly names and values.

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

// Monitor registers of FF temperatures, voltages, currents, and ClK statuses via I2C
void MonitorI2CTask(void *parameters)
{

  struct MonitorI2CTaskArgs_t *args = parameters;

  configASSERT(args->name != 0);

  // watchdog info
  task_watchdog_register_task(kWatchdogTaskID_MonitorI2CTask);

  // wait for the power to come up
  vTaskDelayUntil(&(args->updateTick), pdMS_TO_TICKS(5000));

  int IsCLK = (strstr(args->name, "CLK") != NULL);  // the instance is of CLK-device type
  int IsFF12 = (strstr(args->name, "_12") != NULL); // the instance is of FF 12-ch part type
  int IsFFDAQ = (strstr(args->name, "_4") != NULL); // the instance is of FF 4-ch part type

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

      // for firefly devices, skip if FF is not enabled
      if (!IsCLK) {                           // Fireflies need to be checked if the links are connected or not
        if (args->i2c_dev == I2C_DEVICE_F1) { // FPGA #1
#ifdef REV1
          int NFIREFLIES_IT_F1_P1 = NFIREFLIES_IT_F1 - 2;
          if (!isEnabledFF((IsFFDAQ * (device + NFIREFLIES_IT_F1_P1)) + (IsFF12 * (device < NFIREFLIES_IT_F1 - 3) * (device)) + (IsFF12 * (device > NFIREFLIES_IT_F1 - 3) * (device + NFIREFLIES_DAQ_F1)))) // skip the FF if it's not enabled via the FF config
            continue;
#elif defined(REV2)
          if (!isEnabledFF((IsFFDAQ * (device + NFIREFLIES_IT_F1)) + (IsFF12 * (device)))) // skip the FF if it's not enabled via the FF config
            continue;
#else
#error "Define either Rev1 or Rev2"
#endif
        }
        if (args->i2c_dev == I2C_DEVICE_F2) { // FPGA #2
#ifdef REV1
          if (!isEnabledFF(NFIREFLIES_F1 + (IsFFDAQ * (device)) + (IsFF12 * (device + NFIREFLIES_DAQ_F2)))) // skip the FF if it's not enabled via the FF config
            continue;
#elif defined(REV2)
          if (!isEnabledFF(NFIREFLIES_F1 + (IsFFDAQ * (device + NFIREFLIES_IT_F2)) + (IsFF12 * (device)))) // skip the FF if it's not enabled via the FF config
            continue;
#else
#error "Define either Rev1 or Rev2"
#endif
        }
      }

      // for firefly devices, check the reset pin
      if (!IsCLK) {
        // mux setting
        int result = apollo_i2c_ctl_w(args->i2c_dev, 0x71, 1, 0x40);
        if (result) {
          log_warn(LOG_MONI2C, "mux err %d\r\n", result);
          break;
        }
        uint32_t val;
        // reading reset-FF pin
        int res = apollo_i2c_ctl_reg_r(args->i2c_dev, 0x21, 1, 0x3, 1, &val);
        if (res) {
          log_warn(LOG_MONI2C, "%s read reset-FF pin failed %d\r\n", args->name, res);
          break;
        }
        if ((val & 0x1) != 0x1) {
          log_warn(LOG_MONI2C, "%s reset-FF pin is down \r\n", args->name);
          break;
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

      // Read I2C registers/commands
      for (int c = 0; c < args->n_commands; ++c) {

        int index = device * (args->n_commands * args->n_pages) + c;

        log_debug(LOG_MONI2C, "%s: reg %s\r\n", args->name, args->commands[c].name);
        uint8_t page_reg_value = args->commands[c].page;
        int r = apollo_i2c_ctl_reg_w(args->i2c_dev, args->devices[device].dev_addr, 1, args->selpage_reg, 1, page_reg_value);
        if (r != 0) {
          log_error(LOG_MONI2C, "%s : page fail %s\r\n", args->devices[device].name, SMBUS_get_error(r));
          break;
        }

        uint32_t output_raw;
        int res = apollo_i2c_ctl_reg_r(args->i2c_dev, args->devices[device].dev_addr, args->commands[c].reg_size,
                                       args->commands[c].command, args->commands[c].size, &output_raw);

        if (res != 0) {
          log_error(LOG_MONI2C, "%s: %s read Error %s, break (ps=%d)\r\n",
                    args->name, args->commands[c].name, SMBUS_get_error(res), device);
          args->sm_values[index] = 0xffff;
          break;
        }
        else {
          uint16_t masked_output = output_raw & args->commands[c].bit_mask;
          args->sm_values[index] = masked_output;
          if (args->commands[c].devicelist()) {
            args->commands[c].storeData(masked_output, device);
          }
        }

      } // loop over commands

      log_debug(LOG_MONI2C, "%s: end loop commands\r\n", args->name);
      args->updateTick = xTaskGetTickCount(); // current time in ticks

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
