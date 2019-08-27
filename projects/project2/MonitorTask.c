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

// the PAGE command is an SMBUS standard at register 0
#define PAGE_COMMAND 0x0



//float pm_values[NSUPPLIES_PS*NPAGES_PS*NCOMMANDS_PS];
//static float pm_values_max[NSUPPLIES_PS*NPAGES_PS*NCOMMANDS_PS];
//static float pm_values_min[NSUPPLIES_PS*NPAGES_PS*NCOMMANDS_PS];
//
//static
//void update_max(float pm_values_max[], float pm_values[], int nentries) {
//  for (int i = 0; i < nentries; ++i ) {
//    if ( pm_values_max[i] < pm_values[i])
//      pm_values_max[i] = pm_values[i];
//  }
//}
//static
//void update_min() {
//  for (int i = 0; i < NSUPPLIES_PS*NPAGES_PS*NCOMMANDS_PS; ++i ) {
//    if ( pm_values_min[i] > pm_values[i])
//      pm_values_min[i] = pm_values[i];
//  }
//}

// Monitor temperatures, voltages, currents, via I2C/PMBUS
void MonitorTask(void *parameters)
{
  // initialize to the current tick time
  TickType_t xLastWakeTime = xTaskGetTickCount();
  uint8_t data[2];

  struct MonitorTaskArgs_t *args = parameters;

//  for ( int i = 0; i < NSUPPLIES_PS*NPAGES_PS*NCOMMANDS_PS; ++i ) {
//    pm_values_max[i] = -99;
//    pm_values_min[i] = +99;
//  }

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
  //const uint8_t addrs[NSUPPLIES_PS] = { 0x40, 0x44, 0x43, 0x46, 0x45};
  //const uint8_t supply_prios[NSUPPLIES] = {2, 1, 1, 1, 1};

  for (;;) {
    // check if the 3.3V is there or not. If it disappears then nothing works
    // since that is the I2C pullups. This will be changed with next
    // rev of the board.
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
    // loop over devices
    for ( uint8_t ps = 0; ps < args->n_devices; ++ ps ) {
      char tmp[64];
      // select the appropriate output for the mux
      data[0] = 0x1U<<args->devices[ps].mux_bit;
      snprintf(tmp, 64, "MON: Output of mux set to 0x%02x\n", data[0]);
      DPRINT(tmp);
      tSMBusStatus r = SMBusMasterI2CWrite(args->smbus, args->devices[ps].mux_addr, data, 1);
      if ( r != SMBUS_OK ) {
        Print("MON: I2CBus command failed  (setting mux)\n");
        continue;
      }
      while ( SMBusStatusGet(args->smbus) == SMBUS_TRANSFER_IN_PROGRESS) {
        vTaskDelayUntil( &xLastWakeTime, pdMS_TO_TICKS( 10 )); // wait
      }
      if ( *args->smbus_status != SMBUS_OK ) {
        snprintf(tmp, 64, "MON: Mux writing error %d, break out of loop (ps=%d) ...\n", *args->smbus_status, ps);
        Print(tmp);
        break;
      }
#ifdef DEBUG_MON
      data[0] = 0xAAU;
      r = SMBusMasterI2CRead(args->smbus, 0x70U, data, 1);
      if ( r != SMBUS_OK ) {
        Print("MON: Read of MUX output failed\n");
      }
      while ( SMBusStatusGet(args->smbus) == SMBUS_TRANSFER_IN_PROGRESS) {
        vTaskDelayUntil( &xLastWakeTime, pdMS_TO_TICKS( 10 )); // wait
      }
      if ( *args->smbus_status != SMBUS_OK ) {
        snprintf(tmp, 64, "MON: Mux reading error %d, break out of loop (ps=%d) ...\n", *args->smbus_status, ps);
        Print(tmp);
        break;
      }
      else {
        snprintf(tmp, 64, "MON: read back register on mux to be %02x\n", data[0]);
        DPRINT(tmp);
      }
#endif  // DEBUG_MON
      // loop over pages on the supply
      for ( uint8_t page = 0; page < args->n_pages; ++page ) {
        r = SMBusMasterByteWordWrite(args->smbus, args->devices[ps].dev_addr, PAGE_COMMAND,
            &page, 1);
        if ( r != SMBUS_OK ) {
          Print("SMBUS command failed  (setting page)\n");
        }
        while ( SMBusStatusGet(args->smbus) == SMBUS_TRANSFER_IN_PROGRESS) {
          vTaskDelayUntil( &xLastWakeTime, pdMS_TO_TICKS( 10 )); // wait
        }
        // this is checking the return from the interrupt
        if (*args->smbus_status != SMBUS_OK ) {
          snprintf(tmp, 64, "MON: Page SMBUS ERROR: %d\n", *args->smbus_status);
          Print(tmp);
        }
        snprintf(tmp, 64, "\t\tMON: Page %d\n", page);
        DPRINT(tmp);

        // loop over commands
        for (int c = 0; c < args->n_commands; ++c ) {

          data[0] = 0x0U; data[1] = 0x0U;
          r = SMBusMasterByteWordRead(args->smbus, args->devices[ps].dev_addr, args->commands[c].command,
              data, args->commands[c].size);
          if ( r != SMBUS_OK ) {
            snprintf(tmp, 64, "MON: SMBUS failed (master/bus busy, (ps=%d,c=%d,p=%d)\n", ps,c,page);
            Print(tmp);
            continue; // abort reading this register
          }
          while ( SMBusStatusGet(args->smbus) == SMBUS_TRANSFER_IN_PROGRESS) {
            vTaskDelayUntil( &xLastWakeTime, pdMS_TO_TICKS( 10 )); // wait
          }
          if (*args->smbus_status != SMBUS_OK ) {
            snprintf(tmp, 64, "MON: SMBUS ERROR: %d\n", *args->smbus_status);
            DPRINT(tmp);
          }
          if ( *args->smbus_status != SMBUS_OK ) {
            snprintf(tmp, 64, "Error %d, break out of loop (ps=%d,c=%d,p=%d) ...\n", *args->smbus_status, ps,c,page);
        	  Print(tmp);
        	  break;
          }
          snprintf(tmp, 64, "MON: %d %s is 0x%02x %02x\n", ps, args->commands[c].name, data[1], data[0]);
          DPRINT(tmp);
          float val;
          if ( args->commands[c].type == PM_LINEAR11 ) {
            linear11_val_t ii; ii.raw = (data[1] << 8) | data[0];
            val = linear11_to_float(ii);
            int tens = val;
            int fraction = ABS((val - tens)*100.0);
            snprintf(tmp, 64, "\t\t%d.%02d (linear11)\n", tens, fraction);
            DPRINT(tmp);
          }
          else if ( args->commands[c].type == PM_LINEAR16U ) {
            uint16_t ii = (data[1] << 8) | data[0];
            val = linear16u_to_float(ii);
            int tens = val;
            int fraction = ABS((val - tens)*100.0);
            snprintf(tmp, 64,  "\t\t%d.%02d (linear16u)\n", tens, fraction);
            DPRINT(tmp);
          }
          else if ( args->commands[c].type == PM_STATUS ) {
            // todo: this assumes 2 byte xfer and endianness and converts and int to a float
            val = (float)((data[1] << 8) | data[0]); // ugly is my middle name
          }
          else {
            val = -99.0; // should never get here
          }
          int index = ps*(args->n_commands*args->n_pages)+page*args->n_commands+c;
          args->pm_values[index] = val;
          // wait here for the x msec, where x is 2nd argument below.
          vTaskDelayUntil( &xLastWakeTime, pdMS_TO_TICKS( 10 ) );
        } // loop over commands
      } // loop over pages
    } // loop over power supplies
    //update_max(pm_values_max, args->pm_values, args->n_values); update_min();
    vTaskDelayUntil( &xLastWakeTime, pdMS_TO_TICKS( 250 ) );
  } // infinite loop

}
