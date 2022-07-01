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
#include "common/smbus_helper.h"
#include "MonitorI2CTask.h"
#include "common/power_ctl.h"
#include "common/log.h"
#include "Tasks.h"
#include "I2CCommunication.h"

#define NPAGES_FF    1
#define NCOMMANDS_FF 2

// I2C information -- which device on the MCU is for the FF for each FPGA
// this is what corresponds to I2C_BASE variables in the MCU
#ifdef REV1
#define I2C_DEVICE_F1 4
#define I2C_DEVICE_F2 3
#elif defined(REV2)
#define I2C_DEVICE_F1 4
#define I2C_DEVICE_F2 3
#endif

// local prototype
void Print(const char *str);

// the PAGE command is an SMBUS standard at register 0
#define PAGE_COMMAND 0x1

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
#define CHECKSTUCK_COUNT 30
#define CHECKSTUCK()                                                                               \
  {                                                                                                \
    ++tries;                                                                                       \
    if (tries > CHECKSTUCK_COUNT) {                                                                \
      log_warn(LOG_MONI2C, "stuck (%u, %u)\r\n", (unsigned)ff_updateTick, (unsigned)ff_updateTick);            \
      tries = 0;                                                                                   \
      break;                                                                                       \
    }                                                                                              \
  }

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

struct dev_i2c_addr_t ff_i2c_addrs[NFIREFLIES] = {
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
struct dev_i2c_addr_t ff_i2c_addrs[NFIREFLIES] = {
    {"F1_1  12 Tx", FF_I2CMUX_1_ADDR, 0, 0x50}, //
    {"F1_1  12 Rx", FF_I2CMUX_1_ADDR, 1, 0x54}, //
    {"F1_2  12 Tx", FF_I2CMUX_1_ADDR, 3, 0x50}, //
    {"F1_2  12 Rx", FF_I2CMUX_1_ADDR, 4, 0x54}, //
    {"F1_3  12 Tx", FF_I2CMUX_2_ADDR, 3, 0x50}, //
    {"F1_3  12 Rx", FF_I2CMUX_2_ADDR, 4, 0x54}, //
    {"F1_4 4 XCVR", FF_I2CMUX_1_ADDR, 2, 0x50}, //
    {"F1_5 4 XCVR", FF_I2CMUX_2_ADDR, 0, 0x50}, //
    {"F1_6 4 XCVR", FF_I2CMUX_2_ADDR, 1, 0x50}, //
    {"F1_7 4 XCVR", FF_I2CMUX_2_ADDR, 2, 0x50}, //
    {"F2_1  12 Tx", FF_I2CMUX_1_ADDR, 0, 0x50}, //
    {"F2_1  12 Rx", FF_I2CMUX_1_ADDR, 1, 0x54}, //
    {"F2_2  12 Tx", FF_I2CMUX_1_ADDR, 3, 0x50}, //
    {"F2_2  12 Rx", FF_I2CMUX_1_ADDR, 4, 0x54}, //
    {"F2_3  12 Tx", FF_I2CMUX_2_ADDR, 3, 0x50}, //
    {"F2_3  12 Rx", FF_I2CMUX_2_ADDR, 4, 0x54}, //
    {"F2_4 4 XCVR", FF_I2CMUX_1_ADDR, 2, 0x50}, //
    {"F2_5 4 XCVR", FF_I2CMUX_2_ADDR, 0, 0x50}, //
    {"F2_6 4 XCVR", FF_I2CMUX_2_ADDR, 1, 0x50}, //
    {"F2_7 4 XCVR", FF_I2CMUX_2_ADDR, 2, 0x50}, //

};



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
#define FF_STATUS_COMMAND_REG      0x02U
#define FF_STATUS_COMMAND_REG_MASK 0xFFU
#define FF_TEMP_COMMAND_REG        0x16U
#define FF_PAGE_REG                0x7FU // page register

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

static SemaphoreHandle_t xFFMutex = NULL;

extern struct zynqmon_data_t zynqmon_data[ZM_NUM_ENTRIES];

SemaphoreHandle_t getFFMutex()
{
  return xFFMutex;
}


static TickType_t ff_updateTick;

struct firefly_status_t {
  uint8_t status;
  int8_t temp;
  uint8_t los_alarm[2];
  uint8_t cdr_lol_alarm[2];
};
static struct firefly_status_t ff_stat[NFIREFLIES * NPAGES_FF];



// read-only accessor functions for Firefly names and values.

const char *getFFname(const uint8_t i)
{
  configASSERT(i < NFIREFLIES);
  return ff_i2c_addrs[i].name;
}

uint8_t getFFstatus(const uint8_t i)
{
  configASSERT(i < NFIREFLIES);
  return ff_stat[i].status;
}

int8_t getFFtemp(const uint8_t i)
{
  configASSERT(i < NFIREFLIES);
  return ff_stat[i].temp;
}



bool getFFlos(int i, int channel)
{
  configASSERT(i < NFIREFLIES);
  configASSERT(channel < 12);
  uint8_t *los_alarms = ff_stat[i].los_alarm;

  if (channel >= 8) {
    if (!((1 << (channel - 8)) & los_alarms[1])) {
      return false;
    }
    return true;
  }
  else {
    if (!((1 << channel) & los_alarms[0])) {
      return false;
    }
    return true;
  }
}

bool getFFlol(int i, int channel)
{
  configASSERT(i < NFIREFLIES);
  configASSERT(channel < 12);
  uint8_t *cdr_lol_alarms = ff_stat[i].cdr_lol_alarm;

  if (strstr(ff_i2c_addrs[i].name, "XCVR") == NULL && channel >= 8) {
    if (!((1 << (channel - 8)) & cdr_lol_alarms[1])) {
      return false;
    }
    return true;
  }
  else {
    if (!((1 << channel) & cdr_lol_alarms[0])) {
      return false;
    }
    return true;
  }
}


TickType_t getFFupdateTick()
{
  return ff_updateTick;
}

bool isEnabledFF(int ff)
{
  // firefly config stored in on-board EEPROM
  static bool configured = false;

  static uint32_t ff_config;
  if (!configured) {
    ff_config = read_eeprom_single(EEPROM_ID_FF_ADDR);
  }
  if (!((1 << ff) & ff_config))
    return false;
  else
    return true;
}

static int read_ff_register(const char *name, uint16_t packed_reg_addr, uint8_t *value, size_t size)
{
  memset(value, 0, size);
  // find the appropriate information for this FF device
  int ff;
  for (ff = 0; ff < NFIREFLIES; ++ff) {
    if (strncmp(ff_i2c_addrs[ff].name, name, 10) == 0)
      break;
  }
  if (ff == NFIREFLIES) {
    return -2; // no match found
  }

  int i2c_device;
  if (ff < NFIREFLIES_F1) {
    i2c_device = I2C_DEVICE_F1; // I2C_DEVICE_F1
  }
  else {
    i2c_device = I2C_DEVICE_F2; // I2C_DEVICE_F2
  }
  int res;
  //xSemaphoreTake(xFFMutex, portMAX_DELAY);
  {
    // write to the mux
    // select the appropriate output for the mux
    uint8_t muxmask = 0x1U << ff_i2c_addrs[ff].mux_bit;
    res = apollo_i2c_ctl_w(i2c_device, ff_i2c_addrs[ff].mux_addr, 1, muxmask);
    if ( res != 0 ) {
      log_warn(LOG_MONI2C, "%s: Mux writing error %d (%s) (ff=%s) ...\r\n", __func__, res,
               SMBUS_get_error(res), ff_i2c_addrs[ff].name);
    }

    if (!res) {
      // Read from register.
      uint32_t uidata;
      res = apollo_i2c_ctl_reg_r(i2c_device, ff_i2c_addrs[ff].dev_addr, 1,
                                 packed_reg_addr, size, &uidata);
      for (int i = 0; i < size; ++i) {
        value[i] = (uint8_t)((uidata >> (i * 8)) & 0xFFU);
      }
      if (res != 0) {
        log_warn(LOG_MONI2C, "%s: FF Regread error %d (%s) (ff=%s) ...\r\n", __func__, res,
                 SMBUS_get_error(res), ff_i2c_addrs[ff].name);
      }
    }
  }
  //xSemaphoreGive(xFFMutex);

  return res;
}

static int write_ff_register(const char *name, uint8_t reg, uint16_t value, int size)
{
  configASSERT(size <= 2);
  // find the appropriate information for this FF device
  int ff;
  for (ff = 0; ff < NFIREFLIES; ++ff) {
    if (strncmp(ff_i2c_addrs[ff].name, name, 10) == 0)
      break;
  }
  if (ff == NFIREFLIES) {
    return -2; // no match found
  }
  int i2c_device;
  if (ff < NFIREFLIES_F1) {
    i2c_device = I2C_DEVICE_F1; // I2C_DEVICE_F1
  }
  else {
    i2c_device = I2C_DEVICE_F2; // I2C_DEVICE_F2
  }
  int res;
  //xSemaphoreTake(xFFMutex, portMAX_DELAY);
  {
    // write to the mux
    // select the appropriate output for the mux
    uint8_t muxmask = 0x1U << ff_i2c_addrs[ff].mux_bit;
    res = apollo_i2c_ctl_w(i2c_device, ff_i2c_addrs[ff].mux_addr, 1, muxmask);
    if ( res != 0 ) {
      log_warn(LOG_MONI2C, "%s: Mux writing error %d (%s) (ff=%s) ...\r\n", __func__, res,
               SMBUS_get_error(res), ff_i2c_addrs[ff].name);
    }


    // write to register. First word is reg address, then the data.
    // increment size to account for the register address
    if ( ! res ) {
      res = apollo_i2c_ctl_reg_w(i2c_device, ff_i2c_addrs[ff].dev_addr, 1, reg, size, (uint32_t)value);
      if (res != 0) {
        log_warn(LOG_MONI2C, "%s: FF writing error %d (%s) (ff=%s) ...\r\n", __func__, res,
                 SMBUS_get_error(res), ff_i2c_addrs[ff].name);
      }
    }
  }
  //xSemaphoreGive(xFFMutex);


  return res;
}

static int disable_transmit(bool disable, int num_ff) 
{
  int ret = 0, i = num_ff, imax = num_ff + 1;
  // i and imax are used as limits for the loop below. By default, only iterate once, with i=num_ff.
  uint16_t value = 0x3ff;
  if (disable == false)
    value = 0x0;
  if (num_ff == NFIREFLIES) { // if NFIREFLIES is given for num_ff, loop over ALL transmitters.
    i = 0;
    imax = NFIREFLIES;
  }
  for (; i < imax; ++i) {
    if (!isEnabledFF(i)) // skip the FF if it's not enabled via the FF config
      continue;
    if (strstr(ff_i2c_addrs[i].name, "XCVR") != NULL) {
      ret += write_ff_register(ff_i2c_addrs[i].name, ECU0_25G_XVCR_TX_DISABLE_REG, value, 1);
    }
    else if (strstr(ff_i2c_addrs[i].name, "Tx") != NULL) {
      ret += write_ff_register(ff_i2c_addrs[i].name, ECU0_14G_TX_DISABLE_REG, value, 2);
    }
  }
  return ret;
}

static int disable_receivers(bool disable, int num_ff)
{
  int ret = 0, i = num_ff, imax = num_ff + 1;
  // i and imax are used as limits for the loop below. By default, only iterate once, with i=num_ff.
  uint16_t value = 0x3ff;
  if (disable == false)
    value = 0x0;
  if (num_ff == NFIREFLIES) { // if NFIREFLIES is given for num_ff, loop over ALL transmitters.
    i = 0;
    imax = NFIREFLIES;
  }
  for (; i < imax; ++i) {
    if (!isEnabledFF(i)) // skip the FF if it's not enabled via the FF config
      continue;
    if (strstr(ff_i2c_addrs[i].name, "XCVR") != NULL) {
        ret += write_ff_register(ff_i2c_addrs[i].name, ECU0_25G_XVCR_RX_DISABLE_REG, value, 1);
    }
    else if (strstr(ff_i2c_addrs[i].name, "Rx") != NULL) {
        ret += write_ff_register(ff_i2c_addrs[i].name, ECU0_14G_RX_DISABLE_REG, value, 2);
    }
  }
  return ret;
}


static int write_arbitrary_ff_register(uint16_t regnumber, uint8_t value, int num_ff)
{
  int ret = 0, i = num_ff, imax = num_ff + 1;
  // i and imax are used as limits for the loop below. By default, only iterate once, with i=num_ff.
  if (num_ff == NFIREFLIES) { // if NFIREFLIES is given for num_ff, loop over ALL transmitters.
    i = 0;
    imax = NFIREFLIES;
  }
  for (; i < imax; ++i) {
    if (!isEnabledFF(i)) { // skip the FF if it's not enabled via the FF config
      log_warn(LOG_MONI2C, "Skip writing to disabled FF %d\r\n", i);
      continue;
    }
    int ret1 = write_ff_register(ff_i2c_addrs[i].name, regnumber, value, 1);
    if ( ret1 ) {
      log_warn(LOG_MONI2C, "%s: error %s\r\n", __func__, SMBUS_get_error(ret1));
      ret += ret1;
    }
  }
  return ret;
}

// read a SINGLE firefly register, one byte only
static uint16_t read_arbitrary_ff_register(uint16_t regnumber, int num_ff, uint8_t * value, uint8_t size)
{
  if (num_ff >= NFIREFLIES) {
    return -1;
  }
  int ret = read_ff_register(ff_i2c_addrs[num_ff].name, regnumber, value, 1);
  return ret;
}


// FireFly temperatures, voltages, currents, via I2C
void MonitorI2CTask(void *parameters)
{
  // initialize to the current tick time
  ff_updateTick = xTaskGetTickCount();
  uint8_t data[2];

  struct MonitorTaskArgs_t *args = parameters;

  configASSERT(args->name != 0);

  // create firefly mutex
  //xFFMutex = xSemaphoreCreateMutex();
  //configASSERT(xFFMutex != 0);

  args->updateTick = ff_updateTick; // initial value

  // wait for the power to come up
  vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(500));

  // watchdog info
  task_watchdog_register_task(kWatchdogTaskID_FireFly);
  
  for (uint8_t i = 0; i < NFIREFLIES * NPAGES_FF; ++i) {
    for (int channel=0; channel<2; channel++) {
      ff_stat[i].los_alarm[channel] = 255;
      ff_stat[i].cdr_lol_alarm[channel] = 255;
    }
  }
  vTaskDelayUntil(&ff_updateTick, pdMS_TO_TICKS(2500));

  if (getPowerControlState() == POWER_ON) {
      // Disable all Firefly devices
      disable_transmit(true, NFIREFLIES);
      disable_receivers(true, NFIREFLIES);
      init_registers_ff();
      log_info(LOG_MONI2C, "initialization complete.\r\n");
  }
  else {
    log_warn(LOG_MONI2C, "Initialization skipped -- no power\r\n");
  }


  // reset the wake time to account for the time spent in any work in i2c tasks
  ff_updateTick = xTaskGetTickCount();
  bool good = false;
  for (;;) {
    args->devices = ff_i2c_addrs;
    // grab the semaphore to ensure unique access to I2C controller
    if (args->xSem != NULL) {
      while (xSemaphoreTake(args->xSem, (TickType_t)10) == pdFALSE)
        ;
    }
    // -------------------------------
    // loop over FireFly modules
    // -------------------------------
    for (uint8_t ff = 0; ff < NFIREFLIES; ++ff) {
      if (!isEnabledFF(ff)) // skip the FF if it's not enabled via the FF config
        continue;
      if (getPowerControlState() != POWER_ON) {
        if (good) {
          log_warn(LOG_MONI2C, "No power, skip I2C monitor.\r\n");
          good = false;
          task_watchdog_unregister_task(kWatchdogTaskID_FireFly);
        }
        vTaskDelayUntil(&ff_updateTick, pdMS_TO_TICKS(500));
        continue;
      }
      else { // power is on, and ...
        if ( ! good ) { // ... was not good, but is now good
          task_watchdog_register_task(kWatchdogTaskID_FireFly);
          log_warn(LOG_FFLY, "Power on, resume I2C monitor.\r\n");
          good = true;
        }
      }
      ff_updateTick = xTaskGetTickCount();
      int i2c_device;
      if (ff < NFIREFLIES_F1) {
        i2c_device = I2C_DEVICE_F1; // I2C_DEVICE_F1
      }
      else {
        i2c_device = I2C_DEVICE_F2; // I2C_DEVICE_F2
      }

      // select the appropriate output for the mux
      data[0] = 0x1U << ff_i2c_addrs[ff].mux_bit;
      log_debug(LOG_MONI2C, "Mux set to 0x%02x\r\n", data[0]);
      int res = apollo_i2c_ctl_w(i2c_device, ff_i2c_addrs[ff].mux_addr, 1, data[0]);
      if ( res != 0 ) {
        log_warn(LOG_MONI2C, "Mux write error %d, break (ff=%d)\r\n", res, ff);
        break;
      }

      // save the value of the PAGE register; to be restored at the bottom of the loop
      uint8_t page_reg_value;
      read_ff_register(ff_i2c_addrs[ff].name, FF_PAGE_REG, &page_reg_value, 1);
      // set the page register to 0, if needed
      if ( page_reg_value != 0 )
        write_ff_register(ff_i2c_addrs[ff].name, FF_PAGE_REG, 0, 1);

      typedef union {
        uint8_t us;
        int8_t s;
      } convert_8_t;
      convert_8_t tmp1;

      // Read the temperature
      uint32_t temp_raw;
      res = apollo_i2c_ctl_reg_r(i2c_device, ff_i2c_addrs[ff].dev_addr, 1, (uint16_t)FF_TEMP_COMMAND_REG, 2, &temp_raw);
      if (res != 0) {
        log_warn(LOG_MONI2C, "Temp read Error %d, break (ff=%d)\r\n", res, ff);
        ff_stat[ff].temp = -54;
        break;
      }
      tmp1.us = temp_raw & 0xFFU; // change from uint_8 to int8_t, preserving bit pattern
      ff_stat[ff].temp = tmp1.s;

      // read the status register
      uint32_t status_raw;
      res = apollo_i2c_ctl_reg_r(i2c_device, ff_i2c_addrs[ff].dev_addr, 1, (uint16_t)FF_STATUS_COMMAND_REG, 2, &status_raw);
      if (res != 0) {
        log_warn(LOG_MONI2C, "stat read Error %d, break (ff=%d)\r\n", res, ff);
        ff_stat[ff].status = -54;
        break;
      }
      ff_stat[ff].status = status_raw & FF_STATUS_COMMAND_REG_MASK;


      // Read the serial number

      // Check the loss of signal alarm
      uint32_t los_regs;
      BaseType_t nreg;

      if (strstr(ff_i2c_addrs[ff].name, "XCVR") == NULL)  {
        nreg = 2;
        los_regs = ECU0_25G_TX_LOS_ALARM_REG_2 | (ECU0_25G_TX_LOS_ALARM_REG_1 << 8);
      }
      else {
        nreg = 1;
        los_regs = ECU0_25G_XCVR_LOS_ALARM_REG;
      }

      uint32_t los_raw;
      res = apollo_i2c_ctl_reg_r(i2c_device, ff_i2c_addrs[ff].dev_addr, nreg, los_regs, 1,
                                 &los_raw);
      if (res != 0) {
        log_error(LOG_MONI2C, "los read Error %d, break (ff=%d)\r\n", res, ff);
        ff_stat[ff].los_alarm[0] = 0xff; // is this the right value to use?
        ff_stat[ff].los_alarm[1] = 0xff; // is this the right value to use?
      }
      else if (res == 0) {
        ff_stat[ff].los_alarm[0] = los_raw & 0xFFU;
        ff_stat[ff].los_alarm[1] = (los_raw >> 8) & 0xFFU;
      }

      // Check the CDR loss of lock alarm
      uint16_t cdr_lol_reg_addrs;
      if (strstr(ff_i2c_addrs[ff].name, "XCVR") == NULL)  {
        cdr_lol_reg_addrs = ECU0_25G_CDR_LOL_ALARM_REG_2 | (ECU0_25G_CDR_LOL_ALARM_REG_1 << 8);
        nreg = 2;
      }
      else{
        cdr_lol_reg_addrs = ECU0_25G_XCVR_CDR_LOL_ALARM_REG;
        nreg = 1;
      }

      uint32_t lol_raw;
      res = apollo_i2c_ctl_reg_r(i2c_device, ff_i2c_addrs[ff].dev_addr, nreg,
                                 cdr_lol_reg_addrs, 1, &lol_raw);
      if (res != 0) {
        log_error(LOG_MONI2C, "LOL read error %d (ff=%d)\r\n", res, ff);
        // what is a good value to set these to in case of error?
        ff_stat[ff].cdr_lol_alarm[0] = 0xff;
        ff_stat[ff].cdr_lol_alarm[0] = 0xff;
        break;
      }
      else if (res == 0) {
        ff_stat[ff].cdr_lol_alarm[0] = lol_raw & 0xFFU;
        ff_stat[ff].cdr_lol_alarm[1] = (lol_raw >> 8) & 0xFFU;
      }

      // monitor stack usage for this task
      static UBaseType_t vv = 4096;
      CHECK_TASK_STACK_USAGE(vv);

      // restore the page register to its value at the top of the loop, if it's non-zero
      if ( page_reg_value != 0 ) {
        res = write_ff_register(ff_i2c_addrs[ff].name, FF_PAGE_REG, page_reg_value, 1);
        if (res != 0) {
          log_error(LOG_MONI2C, "page reg write error %d (ff=%d)\r\n", res, ff);
        }
      }

      // clear the I2C mux
      data[0] = 0x0;
      log_debug(LOG_MONI2C, "Output of mux set to 0x%02x\r\n", data[0]);
      res = apollo_i2c_ctl_w(i2c_device, ff_i2c_addrs[ff].mux_addr, 1, data[0]);
      if (res != 0) {
        log_warn(LOG_MONI2C, "FIF: mux clearing error %d, end of loop (ff=%d)\r\n", res, ff);
      }

    } // loop over firefly modules

    if (args->xSem != NULL) // if we have a semaphore, give it
      xSemaphoreGive(args->xSem);

    task_watchdog_feed_task(kWatchdogTaskID_FireFly);
    vTaskDelayUntil(&ff_updateTick, pdMS_TO_TICKS(250));
  } // infinite loop for task
}
