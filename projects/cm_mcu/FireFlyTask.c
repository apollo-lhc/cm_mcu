/*
 * FireFlyTask.c
 *
 *  Created on: July 16, 2019
 *      Author: wittich
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
#include "MonitorTask.h"
#include "common/power_ctl.h"
#include "Tasks.h"
#include "I2CCommunication.h"

#define NPAGES_FF    1
#define NCOMMANDS_FF 2

#ifndef REV2
#define I2C_PULLUP_BUG2
#endif // REV2 

// I2C information -- which device on the MCU is for the FF for each FPGA
// this is what corresponds to I2C_BASE variables in the MCU
#ifndef REV2
#define I2C_DEVICE_F1 4
#define I2C_DEVICE_F2 3
#else
#error "define firefly i2c devices for Rev2"
#endif

// local prototype
void Print(const char *str);

//#define DEBUG_FIF
#ifdef DEBUG_FIF
// prototype of mutex'd print
#define DPRINT(x) Print(x)
#else // DEBUG_FIF
#define DPRINT(x)
#endif // DEBUG_FIF

// this needs to be a macro so that the __LINE__ will resolve to the right 
// line (in a function call it would just resolve to the function call....)
#define CHECKSTUCK_COUNT 30
#define CHECKSTUCK()                                                                               \
  {                                                                                                \
    ++tries;                                                                                       \
    if (tries > CHECKSTUCK_COUNT) {                                                                \
      char tmp[64];                                                                                \
      snprintf(tmp, 64, "FIF: stuck at line %d (%u, %u)\r\n", __LINE__, (unsigned)ff_updateTick,   \
               (unsigned)ff_updateTick);                                                           \
      Print(tmp);                                                                                  \
      tries = 0;                                                                                   \
      break;                                                                                       \
    }                                                                                              \
  }

// i2c addresses
// ECUO-B04 XCVR: 0x50 7 bit I2C address
// ECUO-T12 Tx:   0x50 7 bit I2C address (both 14 and 25G)
// ECUO-R12 Rx:   0x54 7 bit I2C address (both 14 and 25G)

// -------------------------------------------------
//
// REV 1
//
// -------------------------------------------------

struct dev_i2c_addr_t ff_i2c_addrs[NFIREFLIES] = {
    {"K01  12 Tx GTH", 0x70, 0, 0x50}, //
    {"K01  12 Rx GTH", 0x70, 1, 0x54}, //
    {"K02  12 Tx GTH", 0x70, 2, 0x50}, //
    {"K02  12 Rx GTH", 0x70, 3, 0x54}, //
    {"K03  12 Tx GTH", 0x70, 4, 0x50}, //
    {"K03  12 Rx GTH", 0x70, 5, 0x54}, //
    {"K04 4 XCVR GTY", 0x71, 0, 0x50}, //
    {"K05 4 XCVR GTY", 0x71, 1, 0x50}, //
    {"K06 4 XCVR GTY", 0x71, 2, 0x50}, //
    {"K07  12 Tx GTY", 0x71, 3, 0x50}, //
    {"K07  12 Rx GTY", 0x71, 4, 0x54}, //
    {"V01 4 XCVR GTY", 0x70, 0, 0x50}, //
    {"V02 4 XCVR GTY", 0x70, 1, 0x50}, //
    {"V03 4 XCVR GTY", 0x70, 2, 0x50}, //
    {"V04 4 XCVR GTY", 0x70, 3, 0x50}, //
    {"V05 4 XCVR GTY", 0x70, 4, 0x50}, //
    {"V06 4 XCVR GTY", 0x70, 5, 0x50}, //
    {"V07 4 XCVR GTY", 0x71, 0, 0x50}, //
    {"V08 4 XCVR GTY", 0x71, 1, 0x50}, //
    {"V09 4 XCVR GTY", 0x71, 2, 0x50}, //
    {"V10 4 XCVR GTY", 0x71, 3, 0x50}, //
    {"V11  12 Tx GTY", 0x70, 6, 0x50}, //
    {"V11  12 Rx GTY", 0x70, 7, 0x54}, //
    {"V12  12 Tx GTY", 0x71, 4, 0x50}, //
    {"V12  12 Rx GTY", 0x71, 5, 0x54}, //
};


// -------------------------------------------------
//
// REV 2
//
// -------------------------------------------------
// to be added here

// Register definitions
// 8 bit 2's complement signed int, valid from 0-80 C, LSB is 1 deg C
// Same address for 4 XCVR and 12 Tx/Rx devices
#define FF_STATUS_COMMAND_REG 0x2
#define FF_TEMP_COMMAND_REG   0x16

// two bytes, 12 FF to be disabled
#define ECU0_14G_TX_DISABLE_REG      0x34
// one byte, 4 FF to be enabled/disabled (only 4 LSB are used)
#define ECU0_25G_XVCR_TX_DISABLE_REG 0x56
// two bytes, 12 FF to be disabled
#define ECU0_14G_RX_DISABLE_REG      0x34
// one byte, 4 FF to be enabled/disabled (only 4 LSB are used)
#define ECU0_25G_XVCR_RX_DISABLE_REG 0x35
// one byte, 4 FF to be enabled/disabled (4 LSB are Rx, 4 LSB are Tx)
#define ECU0_25G_XVCR_CDR_REG        0x62
// two bytes, 12 FF to be enabled/disabled. The byte layout 
// is a bit weird -- 0-3 on byte 4a, 4-11 on byte 4b
#define ECU0_25G_TXRX_CDR_REG        0x4A

#define ECU0_25G_XCVR_LOS_ALARM_REG     0x3
#define ECU0_25G_XCVR_CDR_LOL_ALARM_REG 0x5

#define ECU0_25G_TX_LOS_ALARM_REG_1  0x7
#define ECU0_25G_TX_LOS_ALARM_REG_2  0x8
#define ECU0_25G_CDR_LOL_ALARM_REG_1 0x14
#define ECU0_25G_CDR_LOL_ALARM_REG_2 0x15

static TickType_t ff_updateTick;

struct firefly_status {
  int8_t status;
  int8_t temp;
  uint8_t los_alarm[2];
  uint8_t cdr_lol_alarm[2];
#ifdef DEBUG_FIF
  int8_t serial_num[16];
  int8_t test[20]; // Used for reading "Samtec Inc.    " for testing purposes
#endif
};
static struct firefly_status ff_status[NFIREFLIES * NPAGES_FF];

#ifdef DEBUG_FIF
static int8_t ff_temp_max[NFIREFLIES * NPAGES_FF];
static int8_t ff_temp_min[NFIREFLIES * NPAGES_FF];

static void update_max()
{
  for (uint8_t i = 0; i < NFIREFLIES * NPAGES_FF; ++i) {
    if (ff_temp_max[i] < ff_status[i].temp)
      ff_temp_max[i] = ff_status[i].temp;
  }
}
static void update_min()
{
  for (uint8_t i = 0; i < NFIREFLIES * NPAGES_FF; ++i) {
    if (ff_temp_min[i] > ff_status[i].temp)
      ff_temp_min[i] = ff_status[i].temp;
  }
}
#endif // DEBUG_FIF

// read-only accessor functions for Firefly names and values.

const char *getFFname(const uint8_t i)
{
  configASSERT(i < NFIREFLIES);
  return ff_i2c_addrs[i].name;
}

int8_t getFFstatus(const uint8_t i)
{
  configASSERT(i < NFIREFLIES);
  return ff_status[i].status;
}

int8_t getFFtemp(const uint8_t i)
{
  configASSERT(i < NFIREFLIES);
  return ff_status[i].temp;
}

#ifdef DEBUG_FIF
int8_t* getFFserialnum(const uint8_t i){
  configASSERT(i < NFIREFLIES);
  return ff_status[i].serial_num;
}
#endif // DEBUG_FIF

bool getFFlos(int i, int channel)
{
  configASSERT(i < NFIREFLIES);
  configASSERT(channel < 12);
  uint8_t *los_alarms = ff_status[i].los_alarm;

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
  uint8_t *cdr_lol_alarms = ff_status[i].cdr_lol_alarm;

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

#ifdef DEBUG_FIF
int8_t* test_read(const uint8_t i) {
  configASSERT(i < NFIREFLIES);
  return ff_status[i].test;
}
#endif

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

static int read_ff_register(const char *name, uint8_t reg_addr, uint8_t *value, size_t size)
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

  // write to the mux
  // select the appropriate output for the mux
  uint8_t muxmask = 0x1U << ff_i2c_addrs[ff].mux_bit;
  int res = apollo_i2c_ctl_w(i2c_device, ff_i2c_addrs[ff].mux_addr, 1, muxmask);
  if ( res != 0 ) {
    char tmp[64];
    snprintf(tmp, 64, "%s: Mux writing error %d  (ff=%s) ...\r\n", __func__, res,
             ff_i2c_addrs[ff].name);
    Print(tmp);
    return 1;
  }

  // Read from register.
  res = apollo_i2c_ctl_reg_r(i2c_device, ff_i2c_addrs[ff].dev_addr, reg_addr, size, value);

  //r = SMBusMasterI2CWriteRead(smbus, ff_i2c_addrs[ff].dev_addr, &reg_addr, 1, value, size);
  if (res != 0) {
    char tmp[128];
    snprintf(tmp, 128, "%s: FF Regread error %d  (ff=%s) ...\r\n", __func__, res,
             ff_i2c_addrs[ff].name);
    Print(tmp);
    return res;
  }
  return 0;
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

  // write to the mux
  // select the appropriate output for the mux
  uint8_t muxmask = 0x1U << ff_i2c_addrs[ff].mux_bit;
  int res = apollo_i2c_ctl_w(i2c_device, ff_i2c_addrs[ff].mux_addr, 1, muxmask);
  if ( res != 0 ) {
    char tmp[64];
    snprintf(tmp, 64, "%s: Mux writing error %d  (ff=%s) ...\r\n", __func__, res,
             ff_i2c_addrs[ff].name);
    Print(tmp);
    return 1;
  }


  // write to register. First word is reg address, then the data.
  // increment size to account for the register address
  res = apollo_i2c_ctl_reg_w(i2c_device, ff_i2c_addrs[ff].dev_addr, reg, size, (int)value);
  if (res != 0) {
    char tmp[64];
    snprintf(tmp, 64, "%s: FF writing error %d  (ff=%s) ...\r\n", __func__, res,
             ff_i2c_addrs[ff].name);
    Print(tmp);
    return 1;
  }


  return 0;
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
    else if (strstr(ff_i2c_addrs[i].name, "Tx") != NULL) { // change this back to rx
        ret += write_ff_register(ff_i2c_addrs[i].name, ECU0_14G_RX_DISABLE_REG, value, 2);
    }
  }
  return ret;
}

static int set_xcvr_cdr(uint8_t value, int num_ff) 
{
  int ret = 0, i = num_ff, imax = num_ff + 1;
  // i and imax are used as limits for the loop below. By default, only iterate once, with i=num_ff.
  if (num_ff == NFIREFLIES) { // if NFIREFLIES is given for num_ff, loop over ALL transmitters.
    i = 0;
    imax = NFIREFLIES;
  }
  for (; i < imax; ++i) {
    if (!isEnabledFF(i) // skip the FF if it's not enabled via the FF config
#ifdef TEST_FF12CHANNEL25G
        && !(i == 21 || i == 22)
#endif // TEST_FF12CHANNEL25G
    )
      continue;
    if (strstr(ff_i2c_addrs[i].name, "XCVR") != NULL) {
      // Print(ff_i2c_addrs[i].name); Print("\r\n");
      ret += write_ff_register(ff_i2c_addrs[i].name, ECU0_25G_XVCR_CDR_REG, value, 1);
    }
    else { // Tx/Rx
      uint16_t value16 = value == 0 ? 0U : 0xffffU; // hack
      Print(ff_i2c_addrs[i].name); Print("\r\n");
      ret += write_ff_register(ff_i2c_addrs[i].name, ECU0_25G_TXRX_CDR_REG, value16, 2);
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
    if (!isEnabledFF(i)) // skip the FF if it's not enabled via the FF config
      continue;
    ret += write_ff_register(ff_i2c_addrs[i].name, regnumber, value, 1);
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

QueueHandle_t xFFlyQueueIn = NULL;
QueueHandle_t xFFlyQueueOut = NULL;

// FireFly temperatures, voltages, currents, via I2C/PMBUS
void FireFlyTask(void *parameters)
{
  // initialize to the current tick time
  ff_updateTick = xTaskGetTickCount();
  uint8_t data[2];

  // watchdog info
  task_watchdog_register_task(kWatchdogTaskID_FireFly);
  
  for (uint8_t i = 0; i < NFIREFLIES * NPAGES_FF; ++i) {
#ifdef DEBUG_FIF
    ff_temp_max[i] = -99;
    ff_temp_min[i] = +99;
#endif // DEBUG_FIF
    ff_status[i].temp = -55;
    ff_status[i].status = 1;
#ifdef DEBUG_FIF
    for (int j = 0; j<16; j++){
    	ff_status[i].serial_num[j] = 0;
    }
#endif // DEBUG_FIF
    for (int channel=0; channel<2; channel++) {
      ff_status[i].los_alarm[channel] = 255;
      ff_status[i].cdr_lol_alarm[channel] = 255;
    }
  }
  vTaskDelayUntil(&ff_updateTick, pdMS_TO_TICKS(2500));

  if (getPSStatus(5) == PWR_ON) {
    // Disable all Firefly devices
    disable_transmit(true, NFIREFLIES);
    disable_receivers(true, NFIREFLIES);
  }

  // reset the wake time to account for the time spent in any work in i2c tasks
  ff_updateTick = xTaskGetTickCount();
  for (;;) {
#ifdef I2C_PULLUP_BUG2
    bool good = false;
#endif // I2C_PULLUP_BUG

    // -------------------------------
    // loop over FireFly modules
    // -------------------------------
    for (uint8_t ff = 0; ff < NFIREFLIES; ++ff) {
      if (!isEnabledFF(ff)) // skip the FF if it's not enabled via the FF config
        continue;
#ifdef I2C_PULLUP_BUG2
      if (getPSStatus(5) != PWR_ON) {
        if (good) {
          Print("FIF: 3V3 died. Skipping I2C monitoring.\r\n");
          good = false;
        }
        vTaskDelayUntil(&ff_updateTick, pdMS_TO_TICKS(500));
        continue;
      }
      else {
        good = true;
      }
#endif // I2C_PULLUP_BUG

      // -------------------------------
      // check for any messages.
      // -------------------------------
      uint32_t message;
      // TODO: what if I receive more than one message
      if (xQueueReceive(xFFlyQueueIn, &message, 0)) {
        // see Tasks.h for the data format
        uint8_t code = (uint8_t)((message >> FF_MESSAGE_CODE_OFFSET) & FF_MESSAGE_CODE_MASK);
        uint32_t data = (uint32_t)message & FF_MESSAGE_DATA_MASK;
        int channel = (data >> FF_MESSAGE_CODE_REG_FF_OFFSET) & FF_MESSAGE_CODE_REG_FF_MASK;
        if (channel > NFIREFLIES)
          channel = NFIREFLIES;
        switch (code) {
          case FFLY_ENABLE_CDR:
            set_xcvr_cdr(0xff, channel);
            break;
          case FFLY_DISABLE_CDR:
            set_xcvr_cdr(0x00, channel);
            break;
          case FFLY_DISABLE_TRANSMITTER:
            disable_transmit(true, channel);
            break;
          case FFLY_ENABLE_TRANSMITTER:
            disable_transmit(false, channel);
            break;
          case FFLY_DISABLE:
            disable_receivers(true, channel);
            break;
          case FFLY_ENABLE:
            disable_receivers(false, channel);
            break;
          case FFLY_WRITE_REGISTER: // high two bytes of data are register, low two bytes are value
          {
            uint16_t theReg =
                (data >> FF_MESSAGE_CODE_REG_REG_OFFSET) & FF_MESSAGE_CODE_REG_REG_MASK;
            uint8_t theValue =
                (data >> FF_MESSAGE_CODE_REG_DAT_OFFSET) & FF_MESSAGE_CODE_REG_DAT_MASK;
            write_arbitrary_ff_register(theReg, theValue, channel);
            break;
          }
          case FFLY_READ_REGISTER: // incoming message: high two bytes of data are register
          {
            // outgoing message: low byte is value; top byte is return code 
            uint16_t theReg =
                (data >> FF_MESSAGE_CODE_REG_REG_OFFSET) & FF_MESSAGE_CODE_REG_REG_MASK;
            uint8_t value;
            uint16_t ret = read_arbitrary_ff_register(theReg, channel, &value, 1);
            message = (uint32_t)value;
            if (ret != 0) {
              message |= (ret << 24);
            }
            xQueueSendToBack(xFFlyQueueOut, &message, pdMS_TO_TICKS(10));
            break;
          }
          case FFLY_TEST_READ: // test register read, dumped to stdout
          {
#define CHARLENGTH 64
            uint8_t theReg = (data >> FF_MESSAGE_CODE_TEST_REG_OFFSET) & FF_MESSAGE_CODE_TEST_REG_MASK;
            uint8_t theFF = (data >> FF_MESSAGE_CODE_TEST_FF_OFFSET) & FF_MESSAGE_CODE_TEST_FF_MASK;
            uint8_t theSZ = (data >> FF_MESSAGE_CODE_TEST_SIZE_OFFSET) & FF_MESSAGE_CODE_TEST_SIZE_MASK;
            if ( theSZ > CHARLENGTH ) {
              theSZ = CHARLENGTH;
            }
            if ( theFF > NFIREFLIES ) {
              theFF = 0;
            }
            char tmp[CHARLENGTH];
            snprintf(tmp, CHARLENGTH, "FF %s (%d)\r\n", ff_i2c_addrs[theFF].name, theFF);
            Print(tmp);
            snprintf(tmp, CHARLENGTH, "Register %d (size %d)\r\n", theReg, theSZ);
            Print(tmp);
            uint8_t regdata[CHARLENGTH];
            memset(regdata, 'x', CHARLENGTH);
            regdata[theSZ - 1] = '\0';
            int ret = read_ff_register(ff_i2c_addrs[theFF].name, theReg, &regdata[0], theSZ);
            if (ret != 0) {
              snprintf(tmp, CHARLENGTH, "read_ff_reg failed with %d\r\n", ret);
              Print(tmp);
              break;
            }
            regdata[CHARLENGTH - 1] = '\0'; // santity check
            Print((char*)regdata);
            Print("\r\n");
            break;
          }
          default:
            message = RED_LED_TOGGLE;
            // message I don't understand? Toggle red LED
            xQueueSendToBack(xLedQueue, &message, pdMS_TO_TICKS(10)); 
            break;
        }
      }
      // -------------------------------
      // continue loop over FF modules 
      // -------------------------------
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
      char tmp[64];
      snprintf(tmp, 64, "FIF: Output of mux set to 0x%02x\r\n", data[0]);
      DPRINT(tmp);
      int res = apollo_i2c_ctl_w(i2c_device, ff_i2c_addrs[ff].mux_addr, 1, data[0]);
      if ( res != 0 ) {
        snprintf(tmp, 64, "FIF: mux writing error %d, break out of loop (ff=%d)\r\n", res, ff);
        Print(tmp);
        break;
      }

      typedef union {
        uint8_t us;
        int8_t s;
      } convert_8_t;
      convert_8_t tmp1;

#define ERRSTR "FIF: %s: Error %d, break loop (ff=%d,c=%d) ...\r\n"
      // Read the temperature
      res = apollo_i2c_ctl_reg_r(i2c_device, ff_i2c_addrs[ff].dev_addr, FF_TEMP_COMMAND_REG, 1, data);
      if (res != 0) {
        snprintf(tmp, 64, ERRSTR, __func__, res, ff, 1);
        Print(tmp);
        ff_status[ff].temp = -54;
        break;
      }
      tmp1.us = data[0]; // change from uint_8 to int8_t, preserving bit pattern
      ff_status[ff].temp = tmp1.s;
#ifdef DEBUG_FIF
      snprintf(tmp, 64, "FIF: %d %s is 0x%02x\r\n", ff, ff_i2c_addrs[ff].name, data[0]);
      DPRINT(tmp);
#endif // DEBUG_FIF
      tmp1.us = data[0]; // change from uint_8 to int8_t, preserving bit pattern
      ff_status[ff].status = tmp1.s;

      // Read the serial number
#ifdef DEBUG_FIF
      data[0] = 0x0U;
      data[1] = 0x0U;
      for (uint8_t i = 189; i < 205; i++) {// change from 171-185 to 189-198 or 189-204 or 196-211
        res = apollo_i2c_ctl_reg_r(i2c_device, ff_i2c_addrs[ff].dev_addr, &i, 1, data);
        if (res == -1) {
          snprintf(tmp, 64, "FIF: %s: SMBUS failed (master/bus busy, ps=%d,c=%d)\r\n", __func__, ff,
              1);
          DPRINT(tmp);
          continue; // abort reading this register
        }
        else if (res==-2){
          snprintf(tmp, 64, "FIF: %s: Error %d, break loop (ps=%d,c=%d) ...\r\n", __func__, *p_status,
              ff, 1);
          DPRINT(tmp);
          ff_status[ff].serial_num[i - 196] = 0;
          break;
        }
        else if (res==0){
          convert_8_t tmp3;
          tmp3.us = data[0]; // change from uint_8 to int8_t, preserving bit pattern
          ff_status[ff].serial_num[i - 196] = tmp3.s;
        }
      }
#endif // DEBUG_FIF

      // Check the loss of signal alarm
      uint8_t los_regs[2];
      if (strstr(ff_i2c_addrs[ff].name, "XCVR") == NULL)  {
        los_regs[0] = ECU0_25G_TX_LOS_ALARM_REG_2;
        los_regs[1] = ECU0_25G_TX_LOS_ALARM_REG_1;
      }
      else{
        los_regs[0] = ECU0_25G_XCVR_LOS_ALARM_REG;
        los_regs[1] = 0;
      }

      // TODO: single multi-byte read rather than multiple reads 
      int reg_i=0;
      while(reg_i<2 && los_regs[reg_i] != 0){
        res = apollo_i2c_ctl_reg_r(i2c_device, ff_i2c_addrs[ff].dev_addr, los_regs[reg_i], 1, data);
        if (res != 0) {
          snprintf(tmp, 64, ERRSTR, __func__, res, ff, 3);
          DPRINT(tmp);
          ff_status[ff].los_alarm[reg_i] = 0xff;
          break;
        }
        else if (res==0){
          ff_status[ff].los_alarm[reg_i] = data[0];
        }
        reg_i+=1;
      }

      // Check the CDR loss of lock alarm
      int cdr_lol_regs[2];
      if (strstr(ff_i2c_addrs[ff].name, "XCVR") == NULL)  {
        cdr_lol_regs[0] = ECU0_25G_CDR_LOL_ALARM_REG_2;
        cdr_lol_regs[1] = ECU0_25G_CDR_LOL_ALARM_REG_1;
      }
      else{
        cdr_lol_regs[0] = ECU0_25G_XCVR_CDR_LOL_ALARM_REG;
        cdr_lol_regs[1] = 0;
      }

      reg_i=0;
      while(reg_i<2 && cdr_lol_regs[reg_i] != 0){
        res = apollo_i2c_ctl_reg_r(i2c_device, ff_i2c_addrs[ff].dev_addr, los_regs[reg_i], 1, data);
        if (res != 0) {
          snprintf(tmp, 64, ERRSTR, __func__, res, ff, 5);
          DPRINT(tmp);
          ff_status[ff].cdr_lol_alarm[reg_i] = 0xff;
          break;
        }
        else if(res==0){
          ff_status[ff].cdr_lol_alarm[reg_i] = data[0];
        }
        reg_i+=1;
      }

#ifdef DEBUG_FIF
      // Read the Samtec line - testing only
      data[0] = 0x0U;
      data[1] = 0x0U;

      for (uint8_t i = 148; i < 164; i++) {
        res = apollo_i2c_ctl_reg_r(i2c_device, ff_i2c_addrs[ff].dev_addr, i, 1, data);
        if (res != 0){
          snprintf(tmp, 64, "FIF: %s: Error %d, break loop (ps=%d,c=%d) ...\r\n", __func__, *p_status,
              ff, 1);
          DPRINT(tmp);
          ff_status[ff].test[i - 148] = 0;
          break;
        }
        tmp1.us = data[0]; // change from uint_8 to int8_t, preserving bit pattern
        ff_status[ff].test[i - 148] = tmp1.s;
      }
#endif

      // clear the I2C mux
      data[0] = 0x0;
      snprintf(tmp, 64, "FIF: Output of mux set to 0x%02x\r\n", data[0]);
      DPRINT(tmp);
      res = apollo_i2c_ctl_w(i2c_device, ff_i2c_addrs[ff].mux_addr, 1, data[0]);
      if (res != 0) {
        snprintf(tmp, 64, "FIF: mux clearing error %d, end of loop (ff=%d)\r\n", res, ff);
        Print(tmp);
      }

    } // loop over firefly modules
#ifdef DEBUG_FIF
    update_max();
    update_min();
#endif // DEBUG_FIF
    task_watchdog_feed_task(kWatchdogTaskID_FireFly);
    vTaskDelayUntil(&ff_updateTick, pdMS_TO_TICKS(250));
  } // infinite loop for task
}
