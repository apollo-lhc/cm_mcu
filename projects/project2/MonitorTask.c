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
#include "MonitorTask.h"


#ifdef DEBUG_MON
// prototype of mutex'd print
void Print(const char* str);
# define DPRINT(x) Print(x)
#else // DEBUG_MON
# define DPRINT(x)
#endif // DEBUG_MON

// Todo: rewrite to get away from awkward/bad SMBUS implementation from TI



struct pm_list pm_common[] = {
  { 0x8d, 2, "READ_TEMPERATURE_1", "C", PM_LINEAR11 },
  { 0x8f, 2, "READ_TEMPERATURE_3", "C", PM_LINEAR11 },
  { 0x88, 2, "READ_VIN", "V", PM_LINEAR11 },
  { 0x8B, 2, "READ_VOUT", "V", PM_LINEAR16U },
  { 0x8c, 2, "READ_IOUT", "A", PM_LINEAR11 }
};

extern tSMBus g_sMaster;

volatile tSMBusStatus eStatus = SMBUS_OK;

float pm_values[NSUPPLIES*NPAGES*NCOMMANDS];

// Monitor temperatures, voltages, currents, usually via I2C/PMBUS
void MonitorTask(void *parameters)
{
  // initialize to the current tick time
  TickType_t xLastWakeTime = xTaskGetTickCount();
  uint8_t data[2];

  vTaskDelayUntil( &xLastWakeTime, pdMS_TO_TICKS( 500 ) );

  // these I2C addresses correspond to the addresses of the power supplies hanging
  // off the TM4C PWR I2C bus
  const uint8_t addrs[NSUPPLIES] = { 0x40, 0x44, 0x43, 0x46, 0x45};

  for (;;) {
    // loop over power supplies attached to the MUX
    for ( uint8_t ps = 0; ps < NSUPPLIES; ++ ps ) {
      char tmp[64];
      // select the appropriate output for the mux
      data[0] = 0x1U<<ps;
      snprintf(tmp, 64, "MON: Output of mux set to 0x%02x\n", data[0]);
      DPRINT(tmp);
      bool success = writeI2C(I2C1_BASE, 0x70U, data, 1);
      if ( ! success ) {
        DPRINT("MON: Write of MUX output failed\n");
      }
      vTaskDelayUntil( &xLastWakeTime, pdMS_TO_TICKS( 10 ));
      data[0] = 0xAAU;
      success = readI2C(I2C1_BASE, 0x70U, data, 1);
      if ( ! success ) {
        DPRINT("MON: Read of MUX output failed\n");
      }
      else {
        snprintf(tmp, 64, "MON: read back register on mux to be %02x\n", data[0]);
        DPRINT(tmp);
      }
      // loop over pages on the supply
      for ( uint8_t page = 0; page < NPAGES; ++page ) {
#define PAGE_COMMAND 0x0
        tSMBusStatus r = SMBusMasterByteWordWrite(&g_sMaster, addrs[ps], PAGE_COMMAND,
            &page, 1);
        if ( r != SMBUS_OK ) {
          DPRINT("SMBUS command failed\n");
        }
        while ( SMBusStatusGet(&g_sMaster) == SMBUS_TRANSFER_IN_PROGRESS) {
          vTaskDelayUntil( &xLastWakeTime, pdMS_TO_TICKS( 10 )); // wait
        }
        // this is checking the return from the interrupt
        if (eStatus != SMBUS_OK ) {
          snprintf(tmp, 64, "MON: Page SMBUS ERROR: %d\n", eStatus);
          DPRINT(tmp);
        }
        snprintf(tmp, 64, "\t\tMON: Page %d\n", page);
        DPRINT(tmp);

        // loop over commands
        for (int c = 0; c < NCOMMANDS; ++c ) {

          data[0] = 0x0U; data[1] = 0x0U;
          r = SMBusMasterByteWordRead(&g_sMaster, addrs[ps], pm_common[c].command,
              data, pm_common[c].size);
          if ( r != SMBUS_OK ) {
            DPRINT("SMBUS command failed\n");
          }
          while ( SMBusStatusGet(&g_sMaster) == SMBUS_TRANSFER_IN_PROGRESS) {
            vTaskDelayUntil( &xLastWakeTime, pdMS_TO_TICKS( 10 )); // wait
          }
          if (eStatus != SMBUS_OK ) {
            snprintf(tmp, 64, "MON: SMBUS ERROR: %d\n", eStatus);
            DPRINT(tmp);
          }
          snprintf(tmp, 64, "MON: %d %s is 0x%02x %02x\n", ps, pm_common[c].name, data[1], data[0]);
          DPRINT(tmp);
          float val;
          if ( pm_common[c].type == PM_LINEAR11 ) {
            linear11_val_t ii; ii.raw = (data[1] << 8) | data[0];
            val = linear11_to_float(ii);
            int tens = val;
            int fraction = ABS((val - tens)*100.0);
            snprintf(tmp, 64, "\t\t%d.%02d (linear11)\n", tens, fraction);
            DPRINT(tmp);
          }
          else if ( pm_common[c].type == PM_LINEAR16U ) {
            uint16_t ii = (data[1] << 8) | data[0];
            val = linear16u_to_float(ii);
            int tens = val;
            int fraction = ABS((val - tens)*100.0);
            snprintf(tmp, 64,  "\t\t%d.%02d (linear16u)\n", tens, fraction);
            DPRINT(tmp);
          }
          pm_values[ps*(NCOMMANDS*NPAGES)+page*NCOMMANDS+c] = val;
          // wait here for the x msec, where x is 2nd argument below.
          vTaskDelayUntil( &xLastWakeTime, pdMS_TO_TICKS( 50 ) );
        } // loop over commands
      } // loop over pages
      vTaskDelayUntil( &xLastWakeTime, pdMS_TO_TICKS( 1000 ) );
    }
  }

}
