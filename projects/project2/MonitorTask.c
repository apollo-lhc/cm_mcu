/*
 * MonitorTask.c
 *
 *  Created on: May 19, 2019
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
#include "common/power_ctl.h"
#include "MonitorTask.h"

void Print(const char* str);

//#define DEBUG_MON
#ifdef DEBUG_MON
// prototype of mutex'd print
# define DPRINT(x) Print(x)
#else // DEBUG_MON
# define DPRINT(x)
#endif // DEBUG_MON

// Todo: rewrite to get away from awkward/bad SMBUS implementation from TI


// Beware: you need to update NCOMMANDS in header file if you change
// the number of entries in this array.
struct pm_list pm_command_dcdc[] = {
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

extern tSMBus g_sMaster1;

volatile tSMBusStatus eStatus1 = SMBUS_OK;
volatile tSMBusStatus eStatus2 = SMBUS_OK;
volatile tSMBusStatus eStatus3 = SMBUS_OK;
volatile tSMBusStatus eStatus4 = SMBUS_OK; // TODO: move these to the right place
volatile tSMBusStatus eStatus6 = SMBUS_OK;

float pm_values[NSUPPLIES*NPAGES*NCOMMANDS];
static float pm_values_max[NSUPPLIES*NPAGES*NCOMMANDS];
static float pm_values_min[NSUPPLIES*NPAGES*NCOMMANDS];

static
void update_max() {
  for (int i = 0; i < NSUPPLIES*NPAGES*NCOMMANDS; ++i ) {
    if ( pm_values_max[i] < pm_values[i])
      pm_values_max[i] = pm_values[i];
  }
}
static
void update_min() {
  for (int i = 0; i < NSUPPLIES*NPAGES*NCOMMANDS; ++i ) {
    if ( pm_values_min[i] > pm_values[i])
      pm_values_min[i] = pm_values[i];
  }
}

// Monitor temperatures, voltages, currents, via I2C/PMBUS
void MonitorTask(void *parameters)
{
  // initialize to the current tick time
  TickType_t xLastWakeTime = xTaskGetTickCount();
  uint8_t data[2];

  for ( int i = 0; i < NSUPPLIES*NPAGES*NCOMMANDS; ++i ) {
    pm_values_max[i] = -99;
    pm_values_min[i] = +99;
  }

  //vTaskDelayUntil( &xLastWakeTime, pdMS_TO_TICKS( 500 ) );
  vTaskDelayUntil( &xLastWakeTime, pdMS_TO_TICKS( 2500 ) );

  // these I2C addresses correspond to the addresses of the power supplies hanging
  // off the TM4C PWR I2C bus
  // TODO: clean up this information and collect in one place
  // Supply Address | Voltages | Priority
  // ---------------+----------|-----------
  //       0x40     | 3.3 & 1.8|     2
  //       0x44     | KVCCINT  |     1
  //       0x43     | KVCCINT  |     1
  //       0x46     | VVCCINT  |     1
  //       0x45     | VVCCINT  |     1
  const uint8_t addrs[NSUPPLIES] = { 0x40, 0x44, 0x43, 0x46, 0x45};
  //const uint8_t supply_prios[NSUPPLIES] = {2, 1, 1, 1, 1};

  for (;;) {
    //int prio = getLowestEnabledPSPriority();
    static bool good = false;
    if ( getPSStatus(5) != PWR_ON) {
      if ( good ) {
        Print("MON: 3V3 died. Skipping I2C monitoring.\n");
        good = false;
      }
      vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(500));
      continue;
    }
    else {
      good = true;
    }
    // loop over power supplies attached to the MUX
    for ( uint8_t ps = 0; ps < NSUPPLIES; ++ ps ) {
      char tmp[64];
      // select the appropriate output for the mux
      data[0] = 0x1U<<ps;
      snprintf(tmp, 64, "MON: Output of mux set to 0x%02x\n", data[0]);
      DPRINT(tmp);
      tSMBusStatus r = SMBusMasterI2CWrite(&g_sMaster1, 0x70U, data, 1);
      if ( r != SMBUS_OK ) {
        Print("MON: I2CBus command failed  (setting mux)\n");
        continue;
      }
      while ( SMBusStatusGet(&g_sMaster1) == SMBUS_TRANSFER_IN_PROGRESS) {
        vTaskDelayUntil( &xLastWakeTime, pdMS_TO_TICKS( 10 )); // wait
      }
      if ( eStatus1 != SMBUS_OK ) {
        snprintf(tmp, 64, "MON: Mux writing error %d, break out of loop (ps=%d) ...\n", eStatus1, ps);
        Print(tmp);
        break;
      }

      data[0] = 0xAAU;
      r = SMBusMasterI2CRead(&g_sMaster1, 0x70U, data, 1);
      if ( r != SMBUS_OK ) {
        Print("MON: Read of MUX output failed\n");
      }
      while ( SMBusStatusGet(&g_sMaster1) == SMBUS_TRANSFER_IN_PROGRESS) {
        vTaskDelayUntil( &xLastWakeTime, pdMS_TO_TICKS( 10 )); // wait
      }
      if ( eStatus1 != SMBUS_OK ) {
        snprintf(tmp, 64, "MON: Mux reading error %d, break out of loop (ps=%d) ...\n", eStatus1, ps);
        Print(tmp);
        break;
      }
      else {
        snprintf(tmp, 64, "MON: read back register on mux to be %02x\n", data[0]);
        DPRINT(tmp);
      }
      // loop over pages on the supply
      for ( uint8_t page = 0; page < NPAGES; ++page ) {
#define PAGE_COMMAND 0x0
        r = SMBusMasterByteWordWrite(&g_sMaster1, addrs[ps], PAGE_COMMAND,
            &page, 1);
        if ( r != SMBUS_OK ) {
          Print("SMBUS command failed  (setting page)\n");
        }
        while ( SMBusStatusGet(&g_sMaster1) == SMBUS_TRANSFER_IN_PROGRESS) {
          vTaskDelayUntil( &xLastWakeTime, pdMS_TO_TICKS( 10 )); // wait
        }
        // this is checking the return from the interrupt
        if (eStatus1 != SMBUS_OK ) {
          snprintf(tmp, 64, "MON: Page SMBUS ERROR: %d\n", eStatus1);
          Print(tmp);
        }
        snprintf(tmp, 64, "\t\tMON: Page %d\n", page);
        DPRINT(tmp);

        // loop over commands
        for (int c = 0; c < NCOMMANDS; ++c ) {

          data[0] = 0x0U; data[1] = 0x0U;
          r = SMBusMasterByteWordRead(&g_sMaster1, addrs[ps], pm_command_dcdc[c].command,
              data, pm_command_dcdc[c].size);
          if ( r != SMBUS_OK ) {
            snprintf(tmp, 64, "MON: SMBUS COMMAND failed (master or busy busy, (ps=%d,c=%d,p=%d)\n", ps,c,page);
            Print(tmp);
            continue; // abort reading this register
          }
          while ( SMBusStatusGet(&g_sMaster1) == SMBUS_TRANSFER_IN_PROGRESS) {
            vTaskDelayUntil( &xLastWakeTime, pdMS_TO_TICKS( 10 )); // wait
          }
          if (eStatus1 != SMBUS_OK ) {
            snprintf(tmp, 64, "MON: SMBUS ERROR: %d\n", eStatus1);
            DPRINT(tmp);
          }
          if ( eStatus1 != SMBUS_OK ) {
            snprintf(tmp, 64, "Error %d, break out of loop (ps=%d,c=%d,p=%d) ...\n", eStatus1, ps,c,page);
        	  Print(tmp);
        	  break;
          }
          snprintf(tmp, 64, "MON: %d %s is 0x%02x %02x\n", ps, pm_command_dcdc[c].name, data[1], data[0]);
          DPRINT(tmp);
          float val;
          if ( pm_command_dcdc[c].type == PM_LINEAR11 ) {
            linear11_val_t ii; ii.raw = (data[1] << 8) | data[0];
            val = linear11_to_float(ii);
            int tens = val;
            int fraction = ABS((val - tens)*100.0);
            snprintf(tmp, 64, "\t\t%d.%02d (linear11)\n", tens, fraction);
            DPRINT(tmp);
          }
          else if ( pm_command_dcdc[c].type == PM_LINEAR16U ) {
            uint16_t ii = (data[1] << 8) | data[0];
            val = linear16u_to_float(ii);
            int tens = val;
            int fraction = ABS((val - tens)*100.0);
            snprintf(tmp, 64,  "\t\t%d.%02d (linear16u)\n", tens, fraction);
            DPRINT(tmp);
          }
          else if ( pm_command_dcdc[c].type == PM_STATUS ) {
            val = (float)((data[1] << 8) | data[0]); // ugly is my middle name
          }
          else {
            val = -99.0; // should never get here
          }
          int index = ps*(NCOMMANDS*NPAGES)+page*NCOMMANDS+c;
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
