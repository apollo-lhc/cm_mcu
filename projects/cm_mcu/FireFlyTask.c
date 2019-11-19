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

#define NFIREFLIES_KU15P 11
#define NFIREFLIES_VU7P 14
#define NFIREFLIES (NFIREFLIES_KU15P+NFIREFLIES_VU7P)
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

// 8 bit 2's complement signed int, valid from 0-80 C, LSB is 1 deg C
// Same address for 4 XCVR and 12 Tx/Rx devices
#define FF_TEMP_COMMAND_REG 0x16
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
  configASSERT(i>=0&&i<NFIREFLIES);
  return ff_i2c_addrs[i].name;
}

int8_t getFFvalue(const uint8_t i)
{
  configASSERT(i>=0&&i<NFIREFLIES);
  return ff_temp[i];
}

static TickType_t ff_updateTick = 0;
TickType_t getFFupdateTick()
{
  return ff_updateTick;
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

#define ECU0_14G_TX_DISABLE_REG      0x34
#define ECU0_25G_XVCR_TX_DISABLE_REG 0x56
#define ECU0_25G_XVCR_CDR_REG        0x62

static
int disable_transmit(bool disable)
{
  int ret = 0;
  uint16_t value = 0x3ff;
  if ( disable == false )
    value = 0x0;
  for ( int i = 0; i < NFIREFLIES; ++ i) {
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
int set_xcvr_cdr(uint8_t value)
{
  int ret = 0;
  for ( int i = 0; i < NFIREFLIES; ++ i) {
    if ( strstr(ff_i2c_addrs[i].name, "XCVR") != NULL ) {
      //Print(ff_i2c_addrs[i].name); Print("\r\n");
      ret += write_ff_register(ff_i2c_addrs[i].name, ECU0_25G_XVCR_CDR_REG,
          value, 1);
    }
  }
  return ret;
}

// Todo: use semaphores to control access to these I2C controllers.
//extern SemaphoreHandle_t xI2C3Mutex;
//extern SemaphoreHandle_t xI2C4Mutex;

QueueHandle_t xFFlyQueue = NULL;


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



  for (;;) {
    tSMBus *smbus;
    tSMBusStatus *p_status;
    bool good = false;
    // loop over FireFly modules
    for ( uint8_t ff = 0; ff < NFIREFLIES; ++ ff ) {
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
      if ( xQueueReceive(xFFlyQueue, &message, 0) ) { // TODO: what if I receive more than one message
        switch (message ) {
        case FFLY_ENABLE_CDR:
          set_xcvr_cdr(0xff);
          break;
        case FFLY_DISABLE_CDR:
          set_xcvr_cdr(0x00);
          break;
        case FFLY_DISABLE_TRANSMITTERS:
          disable_transmit(true);
          break;
        case FFLY_ENABLE_TRANSMITTERS:
          disable_transmit(false);
          break;
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
