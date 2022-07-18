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
//#include "FreeRTOSConfig.h"
#include "task.h"

// local includes
#include "common/i2c_reg.h"
#include "common/smbus_helper.h"
#include "common/smbus_units.h"
#include "MonitorI2CTask.h"
#include "common/power_ctl.h"
#include "common/log.h"
#include "Tasks.h"
#include "I2CCommunication.h"

#define NPAGES_FF    1
#define NCOMMANDS_FF 2

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

// the following are true for both Rev1 and Rev2
#define FF_I2CMUX_1_ADDR 0x70
#define FF_I2CMUX_2_ADDR 0x71

// i2c addresses
// ECUO-B04 XCVR: 0x50 7 bit I2C address
// ECUO-T12 Tx:   0x50 7 bit I2C address (both 14 and 25G)
// ECUO-R12 Rx:   0x54 7 bit I2C address (both 14 and 25G)
#ifdef REV1
// -------------------------------------------------
//
// REV 1
//
// -------------------------------------------------

struct dev_i2c_addr_t ff_moni2c_addrs[NFIREFLIES] = {
    {"K01  12 Tx GTH", FF_I2CMUX_1_ADDR, 0, 0x50}, //
    {"K01  12 Rx GTH", FF_I2CMUX_1_ADDR, 1, 0x54}, //
    {"K02  12 Tx GTH", FF_I2CMUX_1_ADDR, 2, 0x50}, //
    {"K02  12 Rx GTH", FF_I2CMUX_1_ADDR, 3, 0x54}, //
    {"K03  12 Tx GTH", FF_I2CMUX_1_ADDR, 4, 0x50}, //
    {"K03  12 Rx GTH", FF_I2CMUX_1_ADDR, 5, 0x54}, //
    {"K04 4 XCVR GTY", FF_I2CMUX_2_ADDR, 0, 0x50}, //
    {"K05 4 XCVR GTY", FF_I2CMUX_2_ADDR, 1, 0x50}, //
    {"K06 4 XCVR GTY", FF_I2CMUX_2_ADDR, 2, 0x50}, //
    {"K07  12 Tx GTY", FF_I2CMUX_2_ADDR, 3, 0x50}, //
    {"K07  12 Rx GTY", FF_I2CMUX_2_ADDR, 4, 0x54}, //
    {"V01 4 XCVR GTY", FF_I2CMUX_1_ADDR, 0, 0x50}, //
    {"V02 4 XCVR GTY", FF_I2CMUX_1_ADDR, 1, 0x50}, //
    {"V03 4 XCVR GTY", FF_I2CMUX_1_ADDR, 2, 0x50}, //
    {"V04 4 XCVR GTY", FF_I2CMUX_1_ADDR, 3, 0x50}, //
    {"V05 4 XCVR GTY", FF_I2CMUX_1_ADDR, 4, 0x50}, //
    {"V06 4 XCVR GTY", FF_I2CMUX_1_ADDR, 5, 0x50}, //
    {"V07 4 XCVR GTY", FF_I2CMUX_2_ADDR, 0, 0x50}, //
    {"V08 4 XCVR GTY", FF_I2CMUX_2_ADDR, 1, 0x50}, //
    {"V09 4 XCVR GTY", FF_I2CMUX_2_ADDR, 2, 0x50}, //
    {"V10 4 XCVR GTY", FF_I2CMUX_2_ADDR, 3, 0x50}, //
    {"V11  12 Tx GTY", FF_I2CMUX_1_ADDR, 6, 0x50}, //
    {"V11  12 Rx GTY", FF_I2CMUX_1_ADDR, 7, 0x54}, //
    {"V12  12 Tx GTY", FF_I2CMUX_2_ADDR, 4, 0x50}, //
    {"V12  12 Rx GTY", FF_I2CMUX_2_ADDR, 5, 0x54}, //
};
#elif defined (REV2)
// -------------------------------------------------
//
// REV 2
//
// -------------------------------------------------

// information for registers
// Temperature
// 25G 12 lane Tx     : 0x16, 2 bytes
// 25G 12 lane Rx     : 0x16, 2 bytes
// 14G 12 lane Tx     : 0x16, 1 byte
// 14G 12 lane Rx     : 0x16, 1 byte
// 25G 4 lane XCVR    : 0x16, 1 byte

// status register
// 25G 12 lane Tx     : 0x2, 2 bytes TBC
// 25G 12 lane Rx
// 14G 12 lane Tx     : 0x2, 2 bytes
// 14G 12 lane Rx     : 0x2, 2 bytes
// 25G 4 lane XCVR    : 0x2, 2 bytes

// Tx disable register
// 25G 12 lane Tx     : 0x34, 2 bytes TBC!!
// 14G 12 lane Tx     : 0x34, 2 byte
// 25G 4 lane XCVR    : 0x56, 1 byte

// Rx disable register
// 25G 12 lane Rx     : 0x34, 2 bytes TBC!!
// 14G 12 lane Rx     : 0x34, 2 bytes
// 25G 4 lane XCVR    : 0x35, 1 byte

// CDR enable/disable
// 25G 12 lane Tx     : 0x4a, 2 bytes TBC!!
// 25G 12 lane Rx     : 0x4a, 2 bytes TBC!!
// 25G 4 lane XCVR    : 0x62, 1 byte

// Add mask information? 

#else
#error "Define either Rev1 or Rev2"
#endif
// Register definitions
// -------------------------------------------------
// 8 bit 2's complement signed int, valid from 0-80 C, LSB is 1 deg C
// Same address for 4 XCVR and 12 Tx/Rx devices

// two bytes, 12 FF to be disabled
#define ECU0_14G_TX_DISABLE_REG      0x34U
// one byte, 4 FF to be enabled/disabled (only 4 LSB are used)
#define ECU0_25G_XVCR_TX_DISABLE_REG 0x56U
// two bytes, 12 FF to be disabled
#define ECU0_14G_RX_DISABLE_REG      0x34U
// one byte, 4 FF to be enabled/disabled (only 4 LSB are used)
#define ECU0_25G_XVCR_RX_DISABLE_REG 0x35U
// one byte, 4 FF to be enabled/disabled (4 LSB are Rx, 4 LSB are Tx)
#define ECU0_25G_XVCR_CDR_REG        0x62U
// two bytes, 12 FF to be enabled/disabled. The byte layout 
// is a bit weird -- 0-3 on byte 4a, 4-11 on byte 4b
#define ECU0_25G_TXRX_CDR_REG        0x4AU

#define ECU0_25G_XCVR_LOS_ALARM_REG     0x3
#define ECU0_25G_XCVR_CDR_LOL_ALARM_REG 0x5

#define ECU0_25G_TX_LOS_ALARM_REG_1  0x7
#define ECU0_25G_TX_LOS_ALARM_REG_2  0x8
#define ECU0_25G_CDR_LOL_ALARM_REG_1 0x14
#define ECU0_25G_CDR_LOL_ALARM_REG_2 0x15


extern struct zynqmon_data_t zynqmon_data[ZM_NUM_ENTRIES];

//static TickType_t ff_updateTick;

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

static int read_ff_register(void *parameters, const char *name, uint16_t packed_reg_addr, uint8_t *value, size_t size)
{
  struct MonitorI2CTaskArgs_t *args = parameters;
  memset(value, 0, size);
  // find the appropriate information for this FF device
  int ff;
  for (ff = 0; ff < args->n_devices; ++ff) {
    if (strncmp(args->devices[ff].name, name, 10) == 0)
      break;
  }
  if (ff == args->n_devices) {
    return -2; // no match found
  }

  int res;
  //xSemaphoreTake(xFFMutex, portMAX_DELAY);
  {
    // write to the mux
    // select the appropriate output for the mux
    uint8_t muxmask = 0x1U << args->devices[ff].mux_bit;
    res = apollo_i2c_ctl_w(args->i2c_dev, args->devices[ff].mux_addr, 1, muxmask);
    if (res != 0) {
      log_warn(LOG_MONI2C, "%s: Mux writing error %d (%s) (ff=%s) ...\r\n", __func__, res,
               SMBUS_get_error(res), args->devices[ff].name);
    }

    if (!res) {
      // Read from register.
      uint32_t uidata;
      res = apollo_i2c_ctl_reg_r(args->i2c_dev, args->devices[ff].dev_addr, 1,
                                 packed_reg_addr, size, &uidata);
      for (int i = 0; i < size; ++i) {
        value[i] = (uint8_t)((uidata >> (i * 8)) & 0xFFU);
      }
      if (res != 0) {
        log_warn(LOG_MONI2C, "%s: FF Regread error %d (%s) (ff=%s) ...\r\n", __func__, res,
                 SMBUS_get_error(res), args->devices[ff].name);
      }
    }
  }
  //xSemaphoreGive(xFFMutex);

  return res;
}

static int write_ff_register(void *parameters, const char *name, uint8_t reg, uint16_t value, int size)
{
  struct MonitorI2CTaskArgs_t *args = parameters;
  configASSERT(size <= 2);
  // find the appropriate information for this FF device
  int ff;
  for (ff = 0; ff < args->n_devices; ++ff) {
    if (strncmp(args->devices[ff].name, name, 10) == 0)
      break;
  }
  if (ff == args->n_devices) {
    return -2; // no match found
  }

  int res;
  //xSemaphoreTake(xFFMutex, portMAX_DELAY);
  {
    // write to the mux
    // select the appropriate output for the mux
    uint8_t muxmask = 0x1U << args->devices[ff].mux_bit;
    res = apollo_i2c_ctl_w(args->i2c_dev, args->devices[ff].mux_addr, 1, muxmask);
    if (res != 0) {
      log_warn(LOG_MONI2C, "%s: Mux writing error %d (%s) (ff=%s) ...\r\n", __func__, res,
               SMBUS_get_error(res), args->devices[ff].name);
    }

    // write to register. First word is reg address, then the data.
    // increment size to account for the register address
    if (!res) {
      res = apollo_i2c_ctl_reg_w(args->i2c_dev, args->devices[ff].dev_addr, 1, reg, size, (uint32_t)value);
      if (res != 0) {
        log_warn(LOG_MONI2C, "%s: FF writing error %d (%s) (ff=%s) ...\r\n", __func__, res,
                 SMBUS_get_error(res), args->devices[ff].name);
      }
    }
  }
  //xSemaphoreGive(xFFMutex);

  return res;
}

int8_t* test_read_vendor(void *parameters, const uint8_t i) {
  struct MonitorI2CTaskArgs_t *args = parameters;
  configASSERT(i < NFIREFLIES);
  return args->sm_vendor_part;
}


// FireFly temperatures, voltages, currents, via I2C
void MonitorI2CTask(void *parameters) {
  // initialize to the current tick time
  TickType_t ff_updateTick = xTaskGetTickCount();
  uint8_t data[2];

  struct MonitorI2CTaskArgs_t *args = parameters;

  configASSERT(args->name != 0);

  // create firefly mutex
  //xFFMutex = xSemaphoreCreateMutex();
  //configASSERT(xFFMutex != 0);

  args->updateTick = ff_updateTick; // initial value

  // wait for the power to come up
  vTaskDelayUntil(&ff_updateTick, pdMS_TO_TICKS(500));

  // watchdog info
  //task_watchdog_register_task(kWatchdogTaskID_MonitorI2CTask);

  // reset the wake time to account for the time spent in any work in i2c tasks
  ff_updateTick = xTaskGetTickCount();
  bool good = false;
  for (;;) {
    // grab the semaphore to ensure unique access to I2C controller
    if (args->xSem != NULL) {
      while (xSemaphoreTake(args->xSem, (TickType_t) 10) == pdFALSE)
        ;
    }
    // -------------------------------
    // loop over FireFly modules
    // -------------------------------
    for (uint8_t ff = 0; ff < args->n_devices; ++ff) {

      uint8_t ven_addr_start;
      uint8_t ven_addr_stop;

      int isFFIT = 1 - (strcmp(args->name, "FFIT") == 0);
      if (1 - isFFIT) {
        ven_addr_start = 171;
        ven_addr_stop = 187;
      }
      else{
        ven_addr_start = 168;
        ven_addr_stop = 184;
      }

      if (!isEnabledFF(ff + (isFFIT*(NFIREFLIES_IT_F1)) + ((args->i2c_dev-I2C_DEVICE_F1)*(-1)*(NFIREFLIES_F2)))) // skip the FF if it's not enabled via the FF config
        continue;

      if (getPowerControlState() != POWER_ON) {
        if (good) {
          log_warn(LOG_MONI2C, "No power, skip I2C monitor.\r\n");
          good = false;
          //task_watchdog_unregister_task(kWatchdogTaskID_MonitorI2CTask);
        }
        vTaskDelayUntil(&ff_updateTick, pdMS_TO_TICKS(500));
        continue;
      }
      else { // power is on, and ...
        if (!good) { // ... was not good, but is now good
          //task_watchdog_register_task(kWatchdogTaskID_MonitorI2CTask);
          log_warn(LOG_MONI2C, "Power on, resume I2C monitor.\r\n");
          good = true;
        }
      }
      ff_updateTick = xTaskGetTickCount();


      // select the appropriate output for the mux
      data[0] = 0x1U << args->devices[ff].mux_bit;
      log_debug(LOG_MONI2C, "Mux set to 0x%02x\r\n", data[0]);
      int res = apollo_i2c_ctl_w(args->i2c_dev, args->devices[ff].mux_addr, 1,
          data[0]);
      if (res != 0) {
        log_warn(LOG_MONI2C, "Mux write error %d, break (ff=%d)\r\n", res, ff);
        release_break()
      }

      // save the value of the PAGE register; to be restored at the bottom of the loop
      uint8_t page_reg_value;
      read_ff_register(args, args->devices[ff].name,
                       (uint16_t)args->commands[args->n_commands - 1].command, &page_reg_value,
                       1);
      // set the page register to 0, if needed
      if (page_reg_value != 0)
        write_ff_register(args, args->devices[ff].name,
            (uint16_t)args->commands[args->n_commands - 1].command, 0, 1);

      // Write device vendor part
      uint8_t vendor_data[4];

      uint32_t vendor_char;
      for (uint8_t i = ven_addr_start; i < ven_addr_stop; i++) { //171,187

        int res = apollo_i2c_ctl_reg_r(args->i2c_dev, args->devices[ff].dev_addr, 1, (uint16_t)i, 1, &vendor_char);
        //log_info(LOG_MONI2C, "Debug: res = %d.\r\n", res);
        if (res != 0){
          //log_info(LOG_MONI2C, "Debug: Fail reading registers.\r\n");
          char tmp[64];
          snprintf(tmp, 64, "FIF: %s: Error %d, break loop (ps=%d,c=%d) ...\r\n", __func__, res,
                   ff, 1);
          DPRINT(tmp);
          args->sm_vendor_part[i - ven_addr_start] = 0;
          release_break();
        }
        for (int i = 0; i < 4; ++i) {
          vendor_data[i] = (vendor_char>> (3-i) * 8) & 0xFF;
        }
        typedef union {
          uint8_t us;
          int8_t s;
        } convert_8_t;
        convert_8_t tmp1;

        tmp1.us = vendor_data[3]; //vendor_data[0]; // change from uint_8 to int8_t, preserving bit pattern
        args->sm_vendor_part[i - ven_addr_start] = tmp1.s;
      }


      // Read I2C registers/commands
      //log_info(LOG_MONI2C, "Debug: n_command is %d.\r\n", args->n_commands);
      for (int c = 0; c < args->n_commands - 1; ++c) { // exclude page reg
        int index = ff * (args->n_commands * args->n_pages) + c;
        //uint16_t buf = 0x0U;
        //args->sm_values[index] = __builtin_bswap16(buf);

        uint32_t output_raw;

        res = apollo_i2c_ctl_reg_r(args->i2c_dev, args->devices[ff].dev_addr,
            args->commands[c].reg_size, (uint16_t) args->commands[c].command,
            args->commands[c].size, &output_raw);


        uint8_t data[4];
        for (int i = 0; i < 4; ++i) {
          data[i] = (output_raw >> (3-i) * 8) & 0xFF;
        }

        if (res != 0) {
          log_warn(LOG_MONI2C, "%s read Error %d, break (ff=%d)\r\n",
              args->commands[c].name, res, ff);
          args->sm_values[index] = 0xff;
          release_break();
        }
        else if (res==0){
          args->sm_values[index] = data[3];
        }


      } // loop over commands

      // Read the serial number

      // Check the loss of signal alarm //PM_LINEAR16U for TX/RX and PM_STATUS for XCVR

      // Check the CDR loss of lock alarm //PM_LINEAR16U for TX/RX and PM_STATUS for XCVR


      // restore the page register to its value at the top of the loop, if it's non-zero
      if (page_reg_value != 0) {
        res = write_ff_register(args, args->devices[ff].name,
            (uint16_t)args->commands[args->n_commands - 1].command, page_reg_value,
            1);
        if (res != 0) {
          log_error(LOG_MONI2C, "page reg write error %d (ff=%d)\r\n", res, ff);
        }
      }

      // clear the I2C mux
      data[0] = 0x0;
      log_debug(LOG_MONI2C, "Output of mux set to 0x%02x\r\n", data[0]);
      res = apollo_i2c_ctl_w(args->i2c_dev, args->devices[ff].mux_addr, 1, data[0]);
      if (res != 0) {
        log_warn(LOG_MONI2C,
            "FIF: mux clearing error %d, end of loop (ff=%d)\r\n", res, ff);
      }


    } // loop over firefly modules

    if (args->xSem != NULL) // if we have a semaphore, give it
      xSemaphoreGive(args->xSem);

    // monitor stack usage for this task
    CHECK_TASK_STACK_USAGE(args->stack_size);

    //task_watchdog_feed_task(kWatchdogTaskID_MonitorI2CTask);
    vTaskDelayUntil(&ff_updateTick, pdMS_TO_TICKS(250));
  } // infinite loop for task
}
