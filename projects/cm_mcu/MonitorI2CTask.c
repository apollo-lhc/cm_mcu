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
#include "common/smbus_helper.h"
#include "common/smbus_units.h"
#include "MonitorI2CTask.h"
#include "common/power_ctl.h"
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

extern struct zynqmon_data_t zynqmon_data[ZM_NUM_ENTRIES];

// Monitor registers of FF temperatures, voltages, currents, and ClK statuses via I2C
void MonitorI2CTask(void *parameters)
{

  struct MonitorI2CTaskArgs_t *args = parameters;

  configASSERT(args->name != 0);

  // watchdog info
  task_watchdog_register_task(kWatchdogTaskID_MonitorI2CTask);

  // initialize to the current tick time
  args->updateTick = xTaskGetTickCount();

  // wait for the power to come up
  vTaskDelayUntil(&(args->updateTick), pdMS_TO_TICKS(5000));

  int IsCLK = (strstr(args->name, "CLK") != NULL);    // the instance is of CLK-device type
  int IsFF12 = (strstr(args->name, "FF12") != NULL);  // the instance is of FF 12-ch part type
  int IsFFDAQ = (strstr(args->name, "FFDA") != NULL); // the instance is of FF 4-ch part type (DAQ links) -- not being used currently

  // reset the wake time to account for the time spent in any work in i2c tasks

  bool good = false;
  for (;;) {
    log_debug(LOG_MONI2C, "%s: grab semaphore\r\n", args->name);
    // grab the semaphore to ensure unique access to I2C controller
    if (acquireI2CSemaphore(args->xSem) == pdFAIL) {
      log_warn(LOG_SERVICE, "%s could not get semaphore in time; continue\r\n", args->name);
      continue;
    }

    // -------------------------------
    // loop over devices in the device-type instance
    // -------------------------------
    for (int ps = 0; ps < args->n_devices; ++ps) {
      log_debug(LOG_MONI2C, "%s: device %d\r\n", args->name, ps);

      if (ps == args->n_devices - 1 && getPowerControlState() != POWER_ON) { // avoid continues to infinite loops due to multi-threading when pwr is not on
        break;
      }
      if (!IsCLK) {                           // Fireflies need to be checked if the links are connected or not
        if (args->i2c_dev == I2C_DEVICE_F1) { // FPGA #1
#ifdef REV1
          int NFIREFLIES_IT_F1_P1 = NFIREFLIES_IT_F1 - 2;
          if (!isEnabledFF((IsFFDAQ * (ps + NFIREFLIES_IT_F1_P1)) + (IsFF12 * (ps < NFIREFLIES_IT_F1 - 3) * (ps)) + (IsFF12 * (ps > NFIREFLIES_IT_F1 - 3) * (ps + NFIREFLIES_DAQ_F1)))) // skip the FF if it's not enabled via the FF config
            continue;
#elif defined(REV2)
          if (!isEnabledFF((IsFFDAQ * (ps + NFIREFLIES_IT_F1)) + (IsFF12 * (ps)))) // skip the FF if it's not enabled via the FF config
            continue;
#else
#error "Define either Rev1 or Rev2"
#endif
        }
        if (args->i2c_dev == I2C_DEVICE_F2) { // FPGA #2
#ifdef REV1
          if (!isEnabledFF(NFIREFLIES_F1 + (IsFFDAQ * (ps)) + (IsFF12 * (ps + NFIREFLIES_DAQ_F2)))) // skip the FF if it's not enabled via the FF config
            continue;
#elif defined(REV2)
          if (!isEnabledFF(NFIREFLIES_F1 + (IsFFDAQ * (ps + NFIREFLIES_IT_F2)) + (IsFF12 * (ps)))) // skip the FF if it's not enabled via the FF config
            continue;
#else
#error "Define either Rev1 or Rev2"
#endif
        }
      }
      log_debug(LOG_MONI2C, "%s: powercheck\r\n", args->name);

      if (getPowerControlState() != POWER_ON) {
        if (good) {
          log_info(LOG_MONI2C, "%s: PWR off. Disabling I2C monitoring.\r\n", args->name);
          good = false;
          task_watchdog_unregister_task(kWatchdogTaskID_MonitorI2CTask);
        }
        vTaskDelay(pdMS_TO_TICKS(500));
        continue;
      }
      else if (getPowerControlState() == POWER_ON) { // power is on, and ...
        if (!good) {                                 // ... was not good, but is now good
          task_watchdog_register_task(kWatchdogTaskID_MonitorI2CTask);
          log_info(LOG_MONI2C, "%s: PWR on. (Re)starting I2C monitoring.\r\n", args->name);
          good = true;
        }
      }
      // if the power state is unknown, don't do anything
      else {
        log_info(LOG_MONI2C, "%s: power state %d unknown\r\n", args->name,
                 getPowerControlState());
        vTaskDelay(10);
        continue;
      }

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
      data = 0x1U << args->devices[ps].mux_bit;
      log_debug(LOG_MONI2C, "Mux set to 0x%02x\r\n", data);
      int res = apollo_i2c_ctl_w(args->i2c_dev, args->devices[ps].mux_addr, 1, data);
      if (res != 0) {
        log_warn(LOG_MONI2C, "Mux write error %s, break (instance=%s,ps=%d)\r\n", SMBUS_get_error(res), args->name, ps);
        break;
      }

      // Read I2C registers/commands
      for (int c = 0; c < args->n_commands; ++c) {
        int index = ps * (args->n_commands * args->n_pages) + c;

        log_debug(LOG_MONI2C, "%s: command %s.\r\n", args->name, args->commands[c].name);
        uint8_t page_reg_value = args->commands[c].page;
        int r = apollo_i2c_ctl_reg_w(args->i2c_dev, args->devices[ps].dev_addr, 1, args->selpage_reg, 1, page_reg_value);
        if (r != 0) {
          log_error(LOG_MONI2C, "%s : page fail %s\r\n", args->devices[ps].name, SMBUS_get_error(r));
          break;
        }

        uint32_t output_raw;
        int res = apollo_i2c_ctl_reg_r(args->i2c_dev, args->devices[ps].dev_addr, args->commands[c].reg_size,
                                       args->commands[c].command, args->commands[c].size, &output_raw);
        uint16_t masked_output = output_raw & args->commands[c].bit_mask;

        if (res != 0) {
          log_error(LOG_MONI2C, "%s: %s read Error %s, break (ps=%d)\r\n",
                    args->name, args->commands[c].name, SMBUS_get_error(res), ps);
          args->sm_values[index] = 0xffff;
          break;
        }
        else {
          args->sm_values[index] = (uint16_t)masked_output;
        }

      } // loop over commands

#ifdef REV2
      // get optical power information from 25Gbs FFs
      if (IsFFDAQ || (IsFF12 && (args->ffpart_bit_mask & (0x1U << (int)ps / 2)) && (ps % 2 == 1))) {
        uint8_t page_reg_value = args->commands[FF_OPT_POW_C].page;
        int r = apollo_i2c_ctl_reg_w(args->i2c_dev, args->devices[ps].dev_addr, 1, args->selpage_reg, 1, page_reg_value);
        if (r != 0) {
          log_error(LOG_MONI2C, "%s : page fail %s\r\n", args->devices[ps].name, SMBUS_get_error(r));
          break;
        }
        for (int ch = 0; ch < args->n_rxchs; ++ch) {
          uint32_t output_raw;
          uint16_t opt_pw_command;
          if (IsFFDAQ) {
            opt_pw_command = args->commands[FF_OPT_POW_C].command + 2 * ch;
          }
          else {
            ps = (ps - 1) / 2;
            opt_pw_command = args->commands[FF_OPT_POW_C].command - 2 * ch;
          }
          int res = apollo_i2c_ctl_reg_r(args->i2c_dev, args->devices[ps].dev_addr, args->commands[FF_OPT_POW_C].reg_size,
                                         opt_pw_command, args->commands[FF_OPT_POW_C].size, &output_raw);

          if (res != 0) {
            log_error(LOG_MONI2C, "%s: %s read Error %s, break (ps=%d)\r\n", args->name, args->commands[FF_OPT_POW_C].name, SMBUS_get_error(res), ps);
            args->opt_pow_values[ch + ps * (args->n_rxchs)] = 0xffff;
            break;
          }
          else {
            args->opt_pow_values[ch + ps * (args->n_rxchs)] = output_raw;
          }
        }
      }
#endif // REV2
      log_debug(LOG_MONI2C, "%s: end loop commands\r\n", args->name);
      args->updateTick = xTaskGetTickCount(); // current time in ticks

      res = apollo_i2c_ctl_w(args->i2c_dev, args->devices[ps].mux_addr, 1, 0);
      if (res != 0) {
        log_warn(LOG_MONI2C, "Mux write error %s, break (instance=%s,ps=%d)\r\n", SMBUS_get_error(res), args->name, ps);
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
