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

struct ff_i2c_addr_t {
  char *name;
  uint8_t mux_addr; // I2C address of the Mux
  uint8_t mux_bit;  // port of the mux; write value 0x1U<<mux_bit to the mux register
  uint8_t dev_addr; // I2C address of device. Either 0x50 or 0x54
};

struct ff_i2c_addr_t ff_i2c_addrs[NFIREFLIES] = {
    {"K01 12 Tx GTH", 0x70, 0, 0x54},
    {"K01 12 Rx GTH", 0x70, 1, 0x50},
    {"K02 12 Tx GTH", 0x70, 2, 0x54},
    {"K02 12 Rx GTH", 0x70, 3, 0x50},
    {"K03 12 Tx GTH", 0x70, 4, 0x54},
    {"K03 12 Rx GTH", 0x70, 5, 0x50},
    {"K04 4 XCVR GTY", 0x71, 0, 0x50},
    {"K05 4 XCVR GTY", 0x71, 1, 0x50},
    {"K06 4 XCVR GTY", 0x71, 2, 0x50},
    {"K07 12 Tx GTY", 0x71, 3, 0x54},
    {"K07 12 Rx GTY", 0x71, 4, 0x50},
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
    {"V11 12 Tx GTY", 0x70, 6, 0x54},
    {"V11 12 Rx GTY", 0x70, 7, 0x50},
    {"V12 12 Tx GTY", 0x71, 4, 0x54},
    {"V12 12 Rx GTY", 0x71, 5, 0x50},
};

#define FF_TEMP_COMMAND_REG 0x16 // 8 bit 2's complement int, valid from 0-80 C, LSB is 1 deg C

// I2C for VU7P optics
extern tSMBus g_sMaster3;
extern tSMBusStatus eStatus3 ;
// I2C for KU15P optics
extern tSMBus g_sMaster4;
extern tSMBusStatus eStatus4 ;


static int8_t ff_temp[NFIREFLIES*NPAGES_FF*NCOMMANDS_FF];
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


// FireFly temperatures, voltages, currents, via I2C/PMBUS
void FireFlyTask(void *parameters)
{
  // initialize to the current tick time
  TickType_t xLastWakeTime = xTaskGetTickCount();
  uint8_t data[2];

  for ( uint8_t i = 0; i < NFIREFLIES*NPAGES_FF*NCOMMANDS_FF; ++i ) {
    ff_temp_max[i] = -99;
    ff_temp_min[i] = +99;
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
          Print("FIF: 3V3 died. Skipping I2C monitoring.\n");
          good = false;
        }
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(500));
        continue;
      }
      else {
        good = true;
      }

      char tmp[64];
      // select the appropriate output for the mux
      data[0] = 0x1U << ff_i2c_addrs[ff].mux_bit;
      snprintf(tmp, 64, "FIF: Output of mux set to 0x%02x\n", data[0]);
      DPRINT(tmp);
      tSMBusStatus r = SMBusMasterI2CWrite(smbus, ff_i2c_addrs[ff].mux_addr, data, 1);
      if ( r != SMBUS_OK ) {
        Print("FIF: I2CBus command failed  (setting mux)\n");
        continue;
      }
      while ( SMBusStatusGet(smbus) == SMBUS_TRANSFER_IN_PROGRESS) {
        vTaskDelayUntil( &xLastWakeTime, pdMS_TO_TICKS( 10 )); // wait
      }
      if ( *p_status != SMBUS_OK ) {
        snprintf(tmp, 64, "FIF: Mux writing error %d, break out of loop (ps=%d) ...\n", *p_status, ff);
        Print(tmp);
        break;
      }

#ifdef DEBUG_FIF
      data[0] = 0xAAU;
      r = SMBusMasterI2CRead(smbus, ff_i2c_addrs[index].mux_addr, data, 1);
      if ( r != SMBUS_OK ) {
        Print("FIF: Read of MUX output failed\n");
      }
      while ( SMBusStatusGet(smbus) == SMBUS_TRANSFER_IN_PROGRESS) {
        vTaskDelayUntil( &xLastWakeTime, pdMS_TO_TICKS( 10 )); // wait
      }
      if ( *p_status != SMBUS_OK ) {
        snprintf(tmp, 64, "FIF: Mux read error %d, break out of loop (ps=%d) ...\n", *p_status, index);
        Print(tmp);
        break;
      }
      else {
        snprintf(tmp, 64, "FIF: read back register on mux to be %02x\n", data[0]);
        DPRINT(tmp);
      }
#endif // DEBUG_FIF      

      // loop over commands. Currently just one command.
      for (int c = 0; c < NCOMMANDS_FF; ++c ) {

        data[0] = 0x0U; data[1] = 0x0U;
        uint8_t reg_addr = FF_TEMP_COMMAND_REG;
        r = SMBusMasterI2CWriteRead(smbus, ff_i2c_addrs[ff].dev_addr, &reg_addr, 1, data, 1);

        if ( r != SMBUS_OK ) {
          snprintf(tmp, 64, "FIF: %s: SMBUS failed (master/bus busy, ps=%d,c=%d)\n", __func__, ff,c);
          DPRINT(tmp);
          continue; // abort reading this register
        }
        while ( SMBusStatusGet(smbus) == SMBUS_TRANSFER_IN_PROGRESS) {
          vTaskDelayUntil( &xLastWakeTime, pdMS_TO_TICKS( 10 )); // wait
        }
        if ( *p_status != SMBUS_OK ) {
          snprintf(tmp, 64, "FIF: %s: Error %d, break out of loop (ps=%d,c=%d) ...\n", __func__, *p_status, ff,c);
          DPRINT(tmp);
          break;
        }
#ifdef DEBUG_FIF
        snprintf(tmp, 64, "FIF: %d %s is 0x%02x\n", index, ff_i2c_addrs[index].name, data[0]);
        DPRINT(tmp);
#endif // DEBUG_FIF
        typedef union {
           uint8_t us;
           int8_t s;
         } convert_8_t;
        convert_8_t tmp; tmp.us = data[0]; // change from uint_8 to int8_t, preserving bit pattern
        ff_temp[ff] = tmp.s;


        // wait here for the x msec, where x is 2nd argument below.
        vTaskDelayUntil( &xLastWakeTime, pdMS_TO_TICKS( 10 ) );
      } // loop over commands
    } // loop over firefly modules
    update_max(); update_min();
    vTaskDelayUntil( &xLastWakeTime, pdMS_TO_TICKS( 250 ) );
  } // infinite loop for task

}
