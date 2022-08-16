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

// local prototype

void Print(const char *str);

// break out of loop, releasing semaphore if we have it

#define release_break()           \
  {                               \
    if (args->xSem != NULL) {     \
      xSemaphoreGive(args->xSem); \
    }                             \
    break;                        \
  }

// get a mask of a register address given a starting bit field (shift) and the width

void make_bitmask(unsigned int width, unsigned int shift, uint8_t *mask)
{
  unsigned int t = width ? (2u << (width - 1)) - 1u : 0u;
  *mask = t << shift;
}

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

#define TMPBUFFER_SZ 96

// Monitor registers of FF temperatures, voltages, currents, and ClK statuses via I2C
void MonitorI2CTask(void *parameters)
{
  // initialize to the current tick time
  TickType_t xLastWakeTime = xTaskGetTickCount();

  struct MonitorI2CTaskArgs_t *args = parameters;

  configASSERT(args->name != 0);

  // watchdog info
  task_watchdog_register_task(kWatchdogTaskID_MonitorI2CTask);
  args->updateTick = xLastWakeTime;

  // wait for the power to come up
  vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(2500));

  int IsCLK = (strstr(args->name, "CLK") != NULL);   // the instance is of CLK-device type
  int IsFF12 = (strstr(args->name, "FF12") != NULL); // the instance is of FF 12-ch part type
  // int IsFFDAQ =  (strstr(args->name, "FFDAQ") != NULL);  //the instance is of FF 4-ch part type (DAQ links) -- not being used currently

  vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(2500));

  // reset the wake time to account for the time spent in any work in i2c tasks

  bool good = false;
  for (;;) {
    char tmp[TMPBUFFER_SZ];

    // grab the semaphore to ensure unique access to I2C controller
    if (args->xSem != NULL) {
      while (xSemaphoreTake(args->xSem, (TickType_t)10) == pdFALSE)
        ;
    }
    args->updateTick = xTaskGetTickCount(); // current time in ticks
    // -------------------------------
    // loop over devices in the device-type instance
    // -------------------------------
    for (int ps = 0; ps < args->n_devices; ++ps) {

      if (!IsCLK) { // Fireflies need to be checked if the links are connected or not
        int offsetFF12 = 1 - IsFF12;
        if (!isEnabledFF(ps + (offsetFF12 * (NFIREFLIES_IT_F1)) + ((args->i2c_dev - I2C_DEVICE_F1) * (-1) * (NFIREFLIES_F2)))) // skip the FF if it's not enabled via the FF config
          continue;
        args->updateTick = xTaskGetTickCount();
      }

      if (args->requirePower) {
        if (getPowerControlState() != POWER_ON) {
          if (good) {
            snprintf(tmp, TMPBUFFER_SZ, "MONI2C(%s): 3V3 died. Skipping I2C monitoring.\r\n", args->name);
            log_info(LOG_MONI2C, "%s: PWR off. Disabling I2C monitoring.\r\n", args->name);
            good = false;
            task_watchdog_unregister_task(kWatchdogTaskID_MonitorI2CTask);
          }
          vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(500));
          continue;
        }
        else if (getPowerControlState() == POWER_ON) { // power is on, and ...
          if (!good) {                                 // ... was not good, but is now good
            task_watchdog_register_task(kWatchdogTaskID_MonitorI2CTask);
            snprintf(tmp, TMPBUFFER_SZ, "MONI2C(%s): 3V3 came back. Restarting I2C monitoring.\r\n", args->name);
            log_info(LOG_MONI2C, "%s: PWR on. (Re)starting I2C monitoring.\r\n", args->name);
            good = true;
          }
        }
        // if the power state is unknown, don't do anything
      }

      // select the appropriate output for the mux
      uint8_t data[1];
      data[0] = 0x1U << args->devices[ps].mux_bit;
      log_debug(LOG_MONI2C, "Mux set to 0x%02x\r\n", data[0]);
      int res = apollo_i2c_ctl_w(args->i2c_dev, args->devices[ps].mux_addr, 1, data[0]);
      if (res != 0) {
        log_warn(LOG_MONI2C, "Mux write error %s, break (instance=%s,ps=%d)\r\n", SMBUS_get_error(res), args->name, ps);
        release_break();
      }

      // Read I2C registers/commands
      for (int c = 0; c < args->n_commands; ++c) {
        int index = ps * (args->n_commands * args->n_pages) + c;

        char tmp[64];
        snprintf(tmp, 64, "Debug: name = %s.\r\n", args->commands[c].name);
        uint8_t page_reg_value = args->commands[c].page;
        int r = apollo_i2c_ctl_reg_w(args->i2c_dev, args->devices[ps].dev_addr, 1, 0x01, 1, page_reg_value);
        if (r != 0) {
          log_warn(LOG_MONI2C, "SMBUS page failed %s\r\n", SMBUS_get_error(r));
          Print("SMBUS command failed  (setting page)\r\n");
        }

        uint32_t output_raw;
        int res = apollo_i2c_ctl_reg_r(args->i2c_dev, args->devices[ps].dev_addr, args->commands[c].reg_size, args->commands[c].command, args->commands[c].size, &output_raw);
        uint16_t masked_output = output_raw & args->commands[c].bit_mask;


        if (res != 0) {
          log_warn(LOG_MONI2C, "%s read Error %s, break (ps=%d)\r\n",
              args->commands[c].name, SMBUS_get_error(res), ps);
          args->sm_values[index] = 0xffff;
          release_break();
        }
        else {
          args->sm_values[index] = (uint16_t)masked_output;
        }

      } // loop over commands

    } // loop over devices

    if (args->xSem != NULL) // if we have a semaphore, give it
      xSemaphoreGive(args->xSem);

    // monitor stack usage for this task
    CHECK_TASK_STACK_USAGE(args->stack_size);

    // task_watchdog_feed_task(kWatchdogTaskID_MonitorI2CTask);
    vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(250));
  } // infinite loop for task
}
