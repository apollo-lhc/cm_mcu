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

#define DPRINT(x)


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

// this needs to be a macro so that the __LINE__ will resolve to the right 
// line (in a function call it would just resolve to the function call....)


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

void make_bitmask(unsigned int width, unsigned int shift, uint8_t *mask){
  unsigned int t = width ? (2u << (width-1))-1u : 0u;
  *mask = t << shift;
}

bool isEnabledFF(int ff)
{
  // firefly config stored in on-board EEPROM
  static bool configured = false;

  static uint32_t ff_config;
  if (!configured) {
    ff_config = read_eeprom_single(EEPROM_ID_FF_ADDR);
    configured = true;
  }
  if (!((1 << ff) & ff_config))
    return false;
  else
    return true;
}

TickType_t getFFupdateTick()
{
  return xTaskGetTickCount();
}

// FIXME: the current_error_count never goes down, only goes up.
static void SuppressedPrint(const char *str, int *current_error_cnt, bool *logging)
{
  const int error_max = 25;
  const int error_restart_threshold = error_max - 10;

  if (*current_error_cnt < error_max) {
    if (*logging == true) {
      Print(str);
      ++(*current_error_cnt);
      if (*current_error_cnt == error_max) {
        Print("\t--> suppressing further errors for now\r\n");
        log_warn(LOG_MON, "suppressing further errors for now\r\n");
      }
    }
    else { // not logging
      if (*current_error_cnt <= error_restart_threshold)
        *logging = true; // restart logging
    }
  }
  else { // more than error_max errors
    *logging = false;
  }

  return;
}

#define TMPBUFFER_SZ 96

// Monitor registers of FF temperatures, voltages, currents, and ClK statuses via I2C
void MonitorI2CTask(void *parameters) {
  // initialize to the current tick time
  TickType_t moni2c_updateTick = xTaskGetTickCount();

  struct MonitorI2CTaskArgs_t *args = parameters;

  configASSERT(args->name != 0);
  bool log = true;
  int current_error_cnt = 0;

  args->updateTick = moni2c_updateTick; // initial value
  // watchdog info
  task_watchdog_register_task(kWatchdogTaskID_MonitorI2CTask);

  // wait for the power to come up
  vTaskDelayUntil(&moni2c_updateTick, pdMS_TO_TICKS(2500));

  int IsCLK =  (strstr(args->name, "CLK") != NULL);
  int IsFF12 =  (strstr(args->name, "FF12") != NULL);
  int IsFFDAQ =  (strstr(args->name, "FFDAQ") != NULL);

  vTaskDelayUntil(&moni2c_updateTick, pdMS_TO_TICKS(2500));


  // reset the wake time to account for the time spent in any work in i2c tasks
  moni2c_updateTick = xTaskGetTickCount();
  bool good = false;
  for (;;) {
    char tmp[TMPBUFFER_SZ];

    // grab the semaphore to ensure unique access to I2C controller
    if (args->xSem != NULL) {
        while (xSemaphoreTake(args->xSem, (TickType_t) 10) == pdFALSE)
          ;
    }
    moni2c_updateTick = xTaskGetTickCount();
    // -------------------------------
    // loop over devices
    // -------------------------------
    for (uint8_t ps = 0; ps < args->n_devices; ++ps) {

      if (!IsCLK){
        int offsetFF12 = 1 - IsFF12;
        if (!isEnabledFF(ps + (offsetFF12*(NFIREFLIES_IT_F1)) + ((args->i2c_dev-I2C_DEVICE_F1)*(-1)*(NFIREFLIES_F2)))) // skip the FF if it's not enabled via the FF config
          continue;
      }


      if (args->requirePower){
        if (getPowerControlState() != POWER_ON) {
          if (good) {
            snprintf(tmp, TMPBUFFER_SZ, "MONI2C(%s): 3V3 died. Skipping I2C monitoring.\r\n", args->name);
            SuppressedPrint(tmp, &current_error_cnt, &log);
            log_info(LOG_MONI2C, "%s: PWR off. Disabling I2C monitoring.\r\n", args->name);
            good = false;
            task_watchdog_unregister_task(kWatchdogTaskID_MonitorI2CTask);
          }
          vTaskDelayUntil(&moni2c_updateTick, pdMS_TO_TICKS(500));
          continue;
        }
        else if (getPowerControlState() == POWER_ON) { // power is on, and ...
          if (!good) { // ... was not good, but is now good
            task_watchdog_register_task(kWatchdogTaskID_MonitorI2CTask);
            snprintf(tmp, TMPBUFFER_SZ, "MONI2C(%s): 3V3 came back. Restarting I2C monitoring.\r\n", args->name);
            SuppressedPrint(tmp, &current_error_cnt, &log);
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
        if (IsCLK){
          int r = apollo_i2c_ctl_reg_w(args->i2c_dev, args->devices[ps].dev_addr, 1, 0x01, 1, page_reg_value);
          if (r != 0) {
            log_warn(LOG_MONI2C, "SMBUS page failed %s\r\n", SMBUS_get_error(r));
            Print("SMBUS command failed  (setting page)\r\n");
          }
          while (SMBusStatusGet(args->smbus) == SMBUS_TRANSFER_IN_PROGRESS) {
            vTaskDelayUntil(&moni2c_updateTick, pdMS_TO_TICKS(10)); // wait
          }
          // this is checking the return from the interrupt
          if (*args->smbus_status != SMBUS_OK) {
            snprintf(tmp, TMPBUFFER_SZ, "MONI2C(%s): Page SMBUS ERROR: %d\r\n", args->name,
                *args->smbus_status);
            SuppressedPrint(tmp, &current_error_cnt, &log);
          }
        }

        uint32_t output_raw;
        uint8_t mask = 0;
        make_bitmask(args->commands[c].end_bit - args->commands[c].begin_bit + 1,args->commands[c].begin_bit, &mask);
        uint16_t full_mask = (0xff << 8) | mask;
        uint16_t masked_command =  args->commands[c].command & full_mask;
        int res = apollo_i2c_ctl_reg_r(args->i2c_dev, args->devices[ps].dev_addr, args->commands[c].reg_size, masked_command, args->commands[c].size, &output_raw);

        if (res != 0) {
          log_warn(LOG_MONI2C, "%s read Error %s, break (ps=%d)\r\n",
              args->commands[c].name, SMBUS_get_error(res), ps);
          args->sm_values[index] = 0xffff;
          release_break();
        }
        else{
          args->sm_values[index] = (uint16_t)output_raw;

        }

      } // loop over commands


    } // loop over devices

    if (args->xSem != NULL) // if we have a semaphore, give it
      xSemaphoreGive(args->xSem);

    // monitor stack usage for this task
    CHECK_TASK_STACK_USAGE(args->stack_size);

    //task_watchdog_feed_task(kWatchdogTaskID_MonitorI2CTask);
    vTaskDelayUntil(&moni2c_updateTick, pdMS_TO_TICKS(250));
  } // infinite loop for task
}
