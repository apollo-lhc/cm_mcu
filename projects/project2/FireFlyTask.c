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


#define NFIREFLIES 8
#define NPAGES_FF 2
#define NCOMMANDS_FF 2

// local prototype
void Print(const char* str);

//#define DEBUG_FIF
#ifdef DEBUG_FIF
// prototype of mutex'd print
# define DPRINT(x) Print(x)
#else // DEBUG_FIF
# define DPRINT(x)
#endif // DEBUG_FIF



// Beware: you need to update NCOMMANDS_FF if you change
// the number of entries in this array.
struct pm_list pm_command_ff[] = {
  { 0x8d, 2, "READ_TEMPERATURE_1", "C", PM_LINEAR11 },
  { 0x8f, 2, "READ_TEMPERATURE_3", "C", PM_LINEAR11 },
  { 0x88, 2, "READ_VIN", "V", PM_LINEAR11 },
  { 0x8B, 2, "READ_VOUT", "V", PM_LINEAR16U },
  { 0x8c, 2, "READ_IOUT", "A", PM_LINEAR11 },
  //{ 0x4F, 2, "OT_FAULT_LIMIT", "C", PM_LINEAR11},
  { 0x79, 2, "STATUS_WORD", "", PM_STATUS },
  //{ 0xE7, 2, "IOUT_AVG_OC_FAULT_LIMIT", "A", PM_LINEAR11 },
  { 0x95, 2, "READ_FREQUENCY", "Hz", PM_LINEAR11},
};

// I2C for VU7P optics
extern tSMBus g_sMaster3;
extern tSMBusStatus eStatus3 ;
// I2C for KU15P optics
extern tSMBus g_sMaster4;
extern tSMBusStatus eStatus4 ;

// pointers to controller info for the I2C masters
static tSMBus      *masters[2] = {g_sMaster3, g_sMaster4};
static tSMBusStatus *status[2] = {eStatus3,   eStatus4};

float pm_values[NFIREFLIES*NPAGES_FF*NCOMMANDS_FF];
static float pm_values_max[NFIREFLIES*NPAGES_FF*NCOMMANDS_FF];
static float pm_values_min[NFIREFLIES*NPAGES_FF*NCOMMANDS_FF];

static
void update_max() {
  for (int i = 0; i < NFIREFLIES*NPAGES_FF*NCOMMANDS_FF; ++i ) {
    if ( pm_values_max[i] < pm_values[i])
      pm_values_max[i] = pm_values[i];
  }
}
static
void update_min() {
  for (int i = 0; i < NFIREFLIES*NPAGES_FF*NCOMMANDS_FF; ++i ) {
    if ( pm_values_min[i] > pm_values[i])
      pm_values_min[i] = pm_values[i];
  }
}

// FireFly temperatures, voltages, currents, via I2C/PMBUS
void FireFlyTask(void *parameters)
{
  // initialize to the current tick time
  TickType_t xLastWakeTime = xTaskGetTickCount();
  uint8_t data[2];

  for ( int i = 0; i < NFIREFLIES*NPAGES_FF*NCOMMANDS_FF; ++i ) {
    pm_values_max[i] = -99;
    pm_values_min[i] = +99;
  }

  vTaskDelayUntil( &xLastWakeTime, pdMS_TO_TICKS( 2500 ) );


  const uint8_t addrs[NFIREFLIES] = { 0x40, 0x44, 0x43, 0x46, 0x45};
  //const uint8_t supply_prios[NSUPPLIES] = {2, 1, 1, 1, 1};
  tSMBus *smbus = masters[0];
  tSMBusStatus *p_status = status[0];

  for (;;) {
    // loop over power supplies attached to the MUX
    for ( uint8_t ff = 0; ff < NFIREFLIES; ++ ff ) {
      char tmp[64];
      // select the appropriate output for the mux
      data[0] = 0x1U<<ff;
      snprintf(tmp, 64, "FIF: Output of mux set to 0x%02x\n", data[0]);
      DPRINT(tmp);
      tSMBusStatus r = SMBusMasterI2CWrite(smbus, 0x70U, data, 1);
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

      data[0] = 0xAAU;
      r = SMBusMasterI2CRead(smbus, 0x70U, data, 1);
      if ( r != SMBUS_OK ) {
        Print("FIF: Read of MUX output failed\n");
      }
      while ( SMBusStatusGet(smbus) == SMBUS_TRANSFER_IN_PROGRESS) {
        vTaskDelayUntil( &xLastWakeTime, pdMS_TO_TICKS( 10 )); // wait
      }
      if ( *p_status != SMBUS_OK ) {
        snprintf(tmp, 64, "FIF: Mux reading error %d, break out of loop (ps=%d) ...\n", *p_status, ff);
        Print(tmp);
        break;
      }
      else {
        snprintf(tmp, 64, "FIF: read back register on mux to be %02x\n", data[0]);
        DPRINT(tmp);
      }
      // loop over pages on the supply
      for ( uint8_t page = 0; page < NPAGES_FF; ++page ) {
#define PAGE_COMMAND 0x0
        r = SMBusMasterByteWordWrite(smbus, addrs[ff], PAGE_COMMAND,
            &page, 1);
        if ( r != SMBUS_OK ) {
          Print("SMBUS command failed  (setting page)\n");
        }
        while ( SMBusStatusGet(smbus) == SMBUS_TRANSFER_IN_PROGRESS) {
          vTaskDelayUntil( &xLastWakeTime, pdMS_TO_TICKS( 10 )); // wait
        }
        // this is checking the return from the interrupt
        if (*p_status != SMBUS_OK ) {
          snprintf(tmp, 64, "FIF: Page SMBUS ERROR: %d\n", *p_status);
          Print(tmp);
        }
        snprintf(tmp, 64, "\t\tFIF: Page %d\n", page);
        DPRINT(tmp);

        // loop over commands
        for (int c = 0; c < NCOMMANDS_FF; ++c ) {

          data[0] = 0x0U; data[1] = 0x0U;
          r = SMBusMasterByteWordRead(smbus, addrs[ff], pm_command_ff[c].command,
              data, pm_command_ff[c].size);
          if ( r != SMBUS_OK ) {
            snprintf(tmp, 64, "FIF: SMBUS COMMAND failed (master or busy busy, (ps=%d,c=%d,p=%d)\n", ff,c,page);
            Print(tmp);
            continue; // abort reading this register
          }
          while ( SMBusStatusGet(smbus) == SMBUS_TRANSFER_IN_PROGRESS) {
            vTaskDelayUntil( &xLastWakeTime, pdMS_TO_TICKS( 10 )); // wait
          }
          if (*p_status != SMBUS_OK ) {
            snprintf(tmp, 64, "FIF: SMBUS ERROR: %d\n", *p_status);
            DPRINT(tmp);
          }
          if ( *p_status != SMBUS_OK ) {
            snprintf(tmp, 64, "Error %d, break out of loop (ps=%d,c=%d,p=%d) ...\n", *p_status, ff,c,page);
        	  Print(tmp);
        	  break;
          }
          snprintf(tmp, 64, "FIF: %d %s is 0x%02x %02x\n", ff, pm_command_ff[c].name, data[1], data[0]);
          DPRINT(tmp);
          float val;
          if ( pm_command_ff[c].type == PM_LINEAR11 ) {
            linear11_val_t ii; ii.raw = (data[1] << 8) | data[0];
            val = linear11_to_float(ii);
            int tens = val;
            int fraction = ABS((val - tens)*100.0);
            snprintf(tmp, 64, "\t\t%d.%02d (linear11)\n", tens, fraction);
            DPRINT(tmp);
          }
          else if ( pm_command_ff[c].type == PM_LINEAR16U ) {
            uint16_t ii = (data[1] << 8) | data[0];
            val = linear16u_to_float(ii);
            int tens = val;
            int fraction = ABS((val - tens)*100.0);
            snprintf(tmp, 64,  "\t\t%d.%02d (linear16u)\n", tens, fraction);
            DPRINT(tmp);
          }
          else if ( pm_command_ff[c].type == PM_STATUS ) {
            val = (float)((data[1] << 8) | data[0]); // ugly is my middle name
          }
          else {
            val = -99.0; // should never get here
          }
          int index = ff*(NCOMMANDS_FF*NPAGES_FF)+page*NCOMMANDS_FF+c;
          pm_values[index] = val;
          // wait here for the x msec, where x is 2nd argument below.
          vTaskDelayUntil( &xLastWakeTime, pdMS_TO_TICKS( 10 ) );
        } // loop over commands
      } // loop over pages
    } // loop over power supplies
    update_max(); update_min();
    vTaskDelayUntil( &xLastWakeTime, pdMS_TO_TICKS( 250 ) );
  } // infinite loop

}
