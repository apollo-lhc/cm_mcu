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
#include "common/smbus.h"
#include "common/smbus_units.h"
#include "MonitorTask.h"
#include "common/power_ctl.h"
#include "Tasks.h"


#define NPAGES_FF 1
#define NCOMMANDS_FF 1

// local prototype
void Print(const char* str);

//#define DEBUG_FIF
#ifdef DEBUG_FIF
// prototype of mutex'd print
# define DPRINT(x) Print(x)
#else // DEBUG_FIF
# define DPRINT(x)
#endif // DEBUG_FIF

// i2c addresses
// ECUO-B04 XCVR: 0x50 7 bit I2C address
// ECUO-T12 Tx:   0x50 7 bit I2C address
// ECUO-R12 Rx:   0x54 7 bit I2C address


struct dev_i2c_addr_t ff_i2c_addrs[NFIREFLIES] = {
    {"K01  12 Tx GTH", 0x70, 0, 0x50},
    {"K01  12 Rx GTH", 0x70, 1, 0x54},
    {"K02  12 Tx GTH", 0x70, 2, 0x50},
    {"K02  12 Rx GTH", 0x70, 3, 0x54},
    {"K03  12 Tx GTH", 0x70, 4, 0x50},
    {"K03  12 Rx GTH", 0x70, 5, 0x54},
    {"K04 4 XCVR GTY", 0x71, 0, 0x50},
    {"K05 4 XCVR GTY", 0x71, 1, 0x50},
    {"K06 4 XCVR GTY", 0x71, 2, 0x50},
    {"K07  12 Tx GTY", 0x71, 3, 0x50},
    {"K07  12 Rx GTY", 0x71, 4, 0x54},
    {"V01 4 XCVR GTY", 0x70, 0, 0x50},
    {"V02 4 XCVR GTY", 0x70, 1, 0x50},
    {"V03 4 XCVR GTY", 0x70, 2, 0x50},
    {"V04 4 XCVR GTY", 0x70, 3, 0x50},
    {"V05 4 XCVR GTY", 0x70, 4, 0x50},
    {"V06 4 XCVR GTY", 0x70, 5, 0x50},
    {"V07 4 XCVR GTY", 0x71, 0, 0x50},
    {"V08 4 XCVR GTY", 0x71, 1, 0x50},
    {"V09 4 XCVR GTY", 0x71, 2, 0x50},
    {"V10 4 XCVR GTY", 0x71, 3, 0x50},
    {"V11  12 Tx GTY", 0x70, 6, 0x50},
    {"V11  12 Rx GTY", 0x70, 7, 0x54},
    {"V12  12 Tx GTY", 0x71, 4, 0x50},
    {"V12  12 Rx GTY", 0x71, 5, 0x54},
};
// Register definitions
// 8 bit 2's complement signed int, valid from 0-80 C, LSB is 1 deg C
// Same address for 4 XCVR and 12 Tx/Rx devices
#define FF_TEMP_COMMAND_REG 0x16

// two bytes, 12 FF to be disabled
#define ECU0_14G_TX_DISABLE_REG 0x34
// one byte, 4 FF to be enabled/disabled (only 4 LSB are used)
#define ECU0_25G_XVCR_TX_DISABLE_REG 0x56
// one byte, 4 FF to be enabled/disabled (4 LSB are Rx, 4 LSB are Tx)
#define ECU0_25G_XVCR_CDR_REG 0x62

// I2C for VU7P optics
extern tSMBus g_sMaster3;
extern tSMBusStatus eStatus3 ;
// I2C for KU15P optics
extern tSMBus g_sMaster4;
extern tSMBusStatus eStatus4 ;


static int8_t ff_temp[NFIREFLIES*NPAGES_FF*NCOMMANDS_FF];
#ifdef DEBUG_FIF
static int8_t ff_temp_max[NFIREFLIES*NPAGES_FF*NCOMMANDS_FF];
static int8_t ff_temp_min[NFIREFLIES*NPAGES_FF*NCOMMANDS_FF];

static
void update_max() {
  for (uint8_t i = 0; i < NFIREFLIES*NPAGES_FF*NCOMMANDS_FF; ++i ) {
    if ( ff_temp_max[i] < ff_temp[i])
      ff_temp_max[i] = ff_temp[i];
  }
}
static
void update_min() {
  for (uint8_t i = 0; i < NFIREFLIES*NPAGES_FF*NCOMMANDS_FF; ++i ) {
    if ( ff_temp_min[i] > ff_temp[i])
      ff_temp_min[i] = ff_temp[i];
  }
}
#endif // DEBUG_FIF

// read-only accessor functions for Firefly names and values.

const char* getFFname(const uint8_t i)
{
  configASSERT(i<NFIREFLIES);
  return ff_i2c_addrs[i].name;
}

int8_t getFFvalue(const uint8_t i)
{
  configASSERT(i<NFIREFLIES);
  return ff_temp[i];
}

static TickType_t ff_updateTick = 0;
TickType_t getFFupdateTick()
{
  return ff_updateTick;
}
static int read_ff_register(const char *name, uint8_t reg, uint16_t *value,
                             int size) 
{
  *value = 0U;
  configASSERT(size <= 2);
  // find the appropriate information for this FF device
  int ff;
  for (ff = 0; ff < NFIREFLIES; ++ff) {
    if (strncmp(ff_i2c_addrs[ff].name, name, 3) == 0)
      break;
  }
  if (ff == NFIREFLIES) {
    return -2; // no match found
  }
  // i2c base -- two i2c controllers
  tSMBus *smbus;
  tSMBusStatus *p_status;

  if (ff < NFIREFLIES_KU15P) {
    smbus = &g_sMaster4;
    p_status = &eStatus4;
  } else {
    smbus = &g_sMaster3;
    p_status = &eStatus3;
  }
  uint8_t data[3];
  // write to the mux
  // select the appropriate output for the mux

  data[0] = 0x1U << ff_i2c_addrs[ff].mux_bit;
  tSMBusStatus r = SMBusMasterI2CWrite(smbus, ff_i2c_addrs[ff].mux_addr, data, 1);
  if (r != SMBUS_OK) {
    Print("write_ff_reg: I2CBus command failed  (setting mux)\r\n");
    return 1;
  }
  while (SMBusStatusGet(smbus) == SMBUS_TRANSFER_IN_PROGRESS) {
    vTaskDelay(pdMS_TO_TICKS(10)); // wait
  }
  if (*p_status != SMBUS_OK) {
    char tmp[64];
    snprintf(tmp, 64, "%s: Mux writing error %d  (ff=%s) ...\r\n", __func__,
             *p_status, ff_i2c_addrs[ff].name);
    Print(tmp);
    return 1;
  }
  // Write/Read from register. First word is reg address, then the data.
  // increment size to account for the register address
  data[0] = reg;
  r = SMBusMasterI2CWriteRead(smbus, ff_i2c_addrs[ff].dev_addr, data, 1,
                              &data[1], size);
  if (r != SMBUS_OK) {
    Print("write_ff_reg: I2CBus command failed  (FF register)\r\n");
    return 1;
  }
  while (SMBusStatusGet(smbus) == SMBUS_TRANSFER_IN_PROGRESS) {
    vTaskDelay(pdMS_TO_TICKS(10)); // wait
  }
  if (*p_status != SMBUS_OK) {
    char tmp[64];
    snprintf(tmp, 64, "%s: FF writing error %d  (ff=%s) ...\r\n", __func__,
             *p_status, ff_i2c_addrs[ff].name);
    Print(tmp);
    return 1;
  }
  if ( size == 1 )
    *value = data[1];
  else 
    *value = (data[2] << 8) | data[1];
  return 0;
}

static
int write_ff_register(const char *name, uint8_t reg, uint16_t value, int size)
{
  configASSERT(size <= 2);
  // find the appropriate information for this FF device
  int ff;
  for (ff = 0; ff < NFIREFLIES; ++ff ) {
    if ( strncmp(ff_i2c_addrs[ff].name, name, 3) == 0 )
      break;
  }
  if ( ff == NFIREFLIES ) {
    return -2 ; // no match found
  }
  // i2c base -- two i2c controllers
  tSMBus *smbus;
  tSMBusStatus *p_status;

  if ( ff < NFIREFLIES_KU15P ) {
    smbus = &g_sMaster4; p_status = &eStatus4;
  }
  else {
    smbus = &g_sMaster3; p_status = &eStatus3;
  }
  uint8_t data[3];
  // write to the mux
  // select the appropriate output for the mux

  data[0] = 0x1U << ff_i2c_addrs[ff].mux_bit;
  tSMBusStatus r = SMBusMasterI2CWrite(smbus, ff_i2c_addrs[ff].mux_addr, data, 1);
  if ( r != SMBUS_OK ) {
    Print("write_ff_reg: I2CBus command failed  (setting mux)\r\n");
    return 1;
  }
  while ( SMBusStatusGet(smbus) == SMBUS_TRANSFER_IN_PROGRESS) {
    vTaskDelay( pdMS_TO_TICKS( 10 )); // wait
  }
  if ( *p_status != SMBUS_OK ) {
    char tmp[64];
    snprintf(tmp, 64, "%s: Mux writing error %d  (ff=%s) ...\r\n",
             __func__, *p_status, ff_i2c_addrs[ff].name);
    Print(tmp);
    return 1;
  }
  // write to register. First word is reg address, then the data.
  // increment size to account for the register address
  data[0] = reg;
  data[1] = value & 0xFFU;
  data[2] = (value & 0xFF00U)>>8;
  r = SMBusMasterI2CWrite(smbus, ff_i2c_addrs[ff].dev_addr, data, size+1);
  if ( r != SMBUS_OK ) {
    Print("write_ff_reg: I2CBus command failed  (FF register)\r\n");
    return 1;
  }
  while ( SMBusStatusGet(smbus) == SMBUS_TRANSFER_IN_PROGRESS) {
    vTaskDelay( pdMS_TO_TICKS( 10 )); // wait
  }
  if ( *p_status != SMBUS_OK ) {
    char tmp[64];
    snprintf(tmp, 64, "%s: FF writing error %d  (ff=%s) ...\r\n",
             __func__, *p_status, ff_i2c_addrs[ff].name);
    Print(tmp);
    return 1;
  }
  return 0;
}


// static uint8_t ff_mon_register_list[] = {
//     FF_TEMP_COMMAND_REG,
//     ECU0_14G_TX_DISABLE_REG,
//     ECU0_25G_XVCR_TX_DISABLE_REG,
//     ECU0_25G_XVCR_CDR_REG,
// };

static
int disable_transmit(bool disable, int num_ff) // todo: actually test this
{
  int ret = 0, i=num_ff, imax=num_ff+1;
  // i and imax are used as limits for the loop below. By default, only iterate once, with i=num_ff.
  uint16_t value = 0x3ff;
  if ( disable == false )
    value = 0x0;
  if (num_ff==NFIREFLIES){ // if NFIREFLIES is given for num_ff, loop over ALL transmitters.
	  i=0;
	  imax = NFIREFLIES;
  }
  for (; i < imax; ++i) {
    if ( strstr(ff_i2c_addrs[i].name, "XCVR") != NULL ) {
      ret += write_ff_register(ff_i2c_addrs[i].name, ECU0_25G_XVCR_TX_DISABLE_REG,
          value, 1);
    }
    else if ( strstr(ff_i2c_addrs[i].name, "Tx") != NULL ) {
      ret += write_ff_register(ff_i2c_addrs[i].name, ECU0_14G_TX_DISABLE_REG,
          value, 2);
    }
  }
  return ret;
}

static
int set_xcvr_cdr(uint8_t value, int num_ff) // todo: actually test this
{
  int ret = 0, i=num_ff, imax=num_ff+1;
  // i and imax are used as limits for the loop below. By default, only iterate once, with i=num_ff.
  if (num_ff==NFIREFLIES){ // if NFIREFLIES is given for num_ff, loop over ALL transmitters.
	  i=0;
	  imax = NFIREFLIES;
  }
  for ( ; i < imax; ++ i) {
    if ( strstr(ff_i2c_addrs[i].name, "XCVR") != NULL ) {
      //Print(ff_i2c_addrs[i].name); Print("\r\n");
      ret += write_ff_register(ff_i2c_addrs[i].name, ECU0_25G_XVCR_CDR_REG,
          value, 1);
    }
  }
  return ret;
}

static
int write_arbitrary_ff_register(uint16_t regnumber, uint8_t value, int num_ff) 
{
  int ret = 0, i = num_ff, imax = num_ff + 1;
  // i and imax are used as limits for the loop below. By default, only iterate once, with i=num_ff.
  if (num_ff == NFIREFLIES) { // if NFIREFLIES is given for num_ff, loop over ALL transmitters.
    i = 0;
    imax = NFIREFLIES;
  }
  for (; i < imax; ++i) {
    ret += write_ff_register(ff_i2c_addrs[i].name, regnumber, value, 1);
  }
  return ret;
}

// read a SINGLE firefly register, one byte only 
static 
uint16_t read_arbitrary_ff_register(uint16_t regnumber, int num_ff) 
{
  uint16_t value;
  uint16_t ret = read_ff_register(ff_i2c_addrs[num_ff].name, regnumber, &value, 1);
  if ( ret == 0 ) {
    return value;
  }
  else {
    return -1;
  }
    
}

QueueHandle_t xFFlyQueueIn  = NULL;
QueueHandle_t xFFlyQueueOut = NULL;

// FireFly temperatures, voltages, currents, via I2C/PMBUS
void FireFlyTask(void *parameters)
{
  // initialize to the current tick time
  TickType_t xLastWakeTime = xTaskGetTickCount();
  uint8_t data[2];

  for ( uint8_t i = 0; i < NFIREFLIES*NPAGES_FF*NCOMMANDS_FF; ++i ) {
#ifdef DEBUG_FIF
    ff_temp_max[i] = -99;
    ff_temp_min[i] = +99;
#endif // DEBUG_FIF
    ff_temp    [i] = -55;
  }

  vTaskDelayUntil( &xLastWakeTime, pdMS_TO_TICKS( 2500 ) );
  // firefly config stored in on-board EEPROM
  uint32_t ff_config = read_eeprom_single(EEPROM_ID_FF_ADDR);

  for (;;) {
    tSMBus *smbus;
    tSMBusStatus *p_status;
    bool good = false;
    // loop over FireFly modules
    for ( uint8_t ff = 0; ff < NFIREFLIES; ++ ff ) {
      if (!((1 << ff) & ff_config)) // skip the FF if it's not enabled via the FF config
        continue;
      if ( ff < NFIREFLIES_KU15P ) {
        smbus = &g_sMaster4; p_status = &eStatus4;
      }
      else {
        smbus = &g_sMaster3; p_status = &eStatus3;
      }
      if ( getPSStatus(5) != PWR_ON) {
        if ( good ) {
          Print("FIF: 3V3 died. Skipping I2C monitoring.\r\n");
          good = false;
        }
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(500));
        continue;
      }
      else {
        good = true;
      }
      // check for any messages
      uint32_t message;
      if ( xQueueReceive(xFFlyQueueIn, &message, 0) ) { // TODO: what if I receive more than one message
        uint8_t  code = (uint8_t) ((message >> FF_MESSAGE_CODE_OFFSET)& FF_MESSAGE_CODE_MASK);   // see Tasks.h
        uint32_t data = (uint32_t)  message & FF_MESSAGE_DATA_MASK;
        switch (code ) {
        case FFLY_ENABLE_CDR:
          set_xcvr_cdr(0xff, data);
          break;
        case FFLY_DISABLE_CDR:
          set_xcvr_cdr(0x00, data);
          break;
        case FFLY_DISABLE_TRANSMITTER:
          disable_transmit(true, data);
          break;
        case FFLY_ENABLE_TRANSMITTER:
          disable_transmit(false, data);
          break;
        case FFLY_WRITE_REGISTER: // high two bytes of data are register, low two bytes are value
        {
          uint16_t theReg = (data>>FF_MESSAGE_CODE_REG_REG_OFFSET) & FF_MESSAGE_CODE_REG_REG_MASK;
          uint8_t theValue = (data >> FF_MESSAGE_CODE_REG_DAT_OFFSET) & FF_MESSAGE_CODE_REG_DAT_MASK;
          uint8_t theFF = (data >> FF_MESSAGE_CODE_REG_FF_OFFSET) & FF_MESSAGE_CODE_REG_FF_MASK;
          write_arbitrary_ff_register(theReg, theValue, theFF);
          break;
        }
        case FFLY_READ_REGISTER: // high two bytes of data are register, low
                                 // two bytes are value
        {
          uint16_t theReg = (data >> FF_MESSAGE_CODE_REG_REG_OFFSET) &
                            FF_MESSAGE_CODE_REG_REG_MASK;
          uint8_t theFF = (data >> FF_MESSAGE_CODE_REG_FF_OFFSET) &
                          FF_MESSAGE_CODE_REG_FF_MASK;
          uint16_t regdata = read_arbitrary_ff_register(theReg, theFF);
          message = (uint32_t)regdata;
          xQueueSendToBack(xFFlyQueueOut, &message, pdMS_TO_TICKS(10));
          break;
        }
        default:
          message = RED_LED_TOGGLE;
          xQueueSendToBack(xLedQueue, &message, pdMS_TO_TICKS(10)); // message I don't understand? Toggle red LED
          break;
        }
      }
      ff_updateTick = xTaskGetTickCount();
      // select the appropriate output for the mux
      data[0] = 0x1U << ff_i2c_addrs[ff].mux_bit;
      char tmp[64];
      snprintf(tmp, 64, "FIF: Output of mux set to 0x%02x\r\n", data[0]);
      DPRINT(tmp);
      tSMBusStatus r = SMBusMasterI2CWrite(smbus, ff_i2c_addrs[ff].mux_addr, data, 1);
      if ( r != SMBUS_OK ) {
        Print("FIF: I2CBus command failed  (setting mux)\r\n");
        continue;
      }
      while ( SMBusStatusGet(smbus) == SMBUS_TRANSFER_IN_PROGRESS) {
        vTaskDelayUntil( &xLastWakeTime, pdMS_TO_TICKS( 10 )); // wait
      }
      if ( *p_status != SMBUS_OK ) {
        snprintf(tmp, 64, "FIF: Mux writing error %d, break out of loop (ps=%d) ...\r\n", *p_status, ff);
        Print(tmp);
        break;
      }

#ifdef DEBUG_FIF
      data[0] = 0xAAU;
      r = SMBusMasterI2CRead(smbus, ff_i2c_addrs[index].mux_addr, data, 1);
      if ( r != SMBUS_OK ) {
        Print("FIF: Read of MUX output failed\r\n");
      }
      while ( SMBusStatusGet(smbus) == SMBUS_TRANSFER_IN_PROGRESS) {
        vTaskDelayUntil( &xLastWakeTime, pdMS_TO_TICKS( 10 )); // wait
      }
      if ( *p_status != SMBUS_OK ) {
        snprintf(tmp, 64, "FIF: Mux read error %d, break out of loop (ps=%d) ...\r\n", *p_status, index);
        Print(tmp);
        break;
      }
      else {
        snprintf(tmp, 64, "FIF: read back register on mux to be %02x\r\n", data[0]);
        DPRINT(tmp);
      }
#endif // DEBUG_FIF

      // loop over commands. Currently just one command.
      for (int c = 0; c < NCOMMANDS_FF; ++c ) {

        data[0] = 0x0U; data[1] = 0x0U;
        uint8_t reg_addr = FF_TEMP_COMMAND_REG;
        r = SMBusMasterI2CWriteRead(smbus, ff_i2c_addrs[ff].dev_addr, &reg_addr, 1, data, 1);

        if ( r != SMBUS_OK ) {
          snprintf(tmp, 64, "FIF: %s: SMBUS failed (master/bus busy, ps=%d,c=%d)\r\n", __func__, ff,c);
          DPRINT(tmp);
          continue; // abort reading this register
        }
        while ( SMBusStatusGet(smbus) == SMBUS_TRANSFER_IN_PROGRESS) {
          vTaskDelayUntil( &xLastWakeTime, pdMS_TO_TICKS( 10 )); // wait
        }
        if ( *p_status != SMBUS_OK ) {
          snprintf(tmp, 64, "FIF: %s: Error %d, break loop (ps=%d,c=%d) ...\r\n", __func__, *p_status, ff,c);
          DPRINT(tmp);
          ff_temp[ff] =-55;
          break;
        }
#ifdef DEBUG_FIF
        snprintf(tmp, 64, "FIF: %d %s is 0x%02x\r\n", index, ff_i2c_addrs[index].name, data[0]);
        DPRINT(tmp);
#endif // DEBUG_FIF
        typedef union {
          uint8_t us;
          int8_t s;
        } convert_8_t;
        convert_8_t tmp1; tmp1.us = data[0]; // change from uint_8 to int8_t, preserving bit pattern
        ff_temp[ff] = tmp1.s;

      } // loop over commands

      // clear the I2C mux
      data[0] = 0x0;
      snprintf(tmp, 64, "FIF: Output of mux set to 0x%02x (clear)\r\n", data[0]);
      DPRINT(tmp);
      r = SMBusMasterI2CWrite(smbus, ff_i2c_addrs[ff].mux_addr, data, 1);
      if ( r != SMBUS_OK ) {
        Print("FIF: I2CBus command failed  (clearing mux)\r\n");
        continue;
      }
      while ( SMBusStatusGet(smbus) == SMBUS_TRANSFER_IN_PROGRESS) {
        vTaskDelayUntil( &xLastWakeTime, pdMS_TO_TICKS( 10 )); // wait
      }
      if ( *p_status != SMBUS_OK ) {
        snprintf(tmp, 64, "FIF: Mux clearing error %d, break out of loop (ps=%d) ...\r\n", *p_status, ff);
        Print(tmp);
        break;
      }
    } // loop over firefly modules
#ifdef DEBUG_FIF
    update_max(); update_min();
#endif // DEBUG_FIF
    vTaskDelayUntil( &xLastWakeTime, pdMS_TO_TICKS( 250 ) );
  } // infinite loop for task

}
