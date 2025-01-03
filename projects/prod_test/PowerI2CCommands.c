/*
 * PowerI2CCommands.c
 *
 *  Created on: January 1, 2025
 *      Author: mcoshiro
 *
 * Contains code for MCU to DC-DC I2C tests used for Apollo CM production 
 * tests
 *
 * Currently uses the same SMBUS setup as the central MCU MonitorTask code
 */

// includes for types
#include <stdint.h>

// FreeRTOS
#include "FreeRTOS.h"
#include "task.h"

// local includes
#include "common/smbus.h"
#include "common/smbus_units.h"
#include "common/printf.h"
#include "common/utils.h"
#include "commands.h"
#include "InterruptHandlers.h"
#include "PowerI2CCommands.h"

// TODO implement and uncomment semaphores
//// release semaphore if we have it
//#define release_sempahore()                                                  
//  {                                                                          
//    if (xSemaphoreGetMutexHolder(args->xSem) == xTaskGetCurrentTaskHandle()) 
//      xSemaphoreGive(args->xSem);                                            
//  }

// TODO move this somewhere else
struct dev_i2c_addr_t pm_addrs_dcdc[N_PM_ADDRS_DCDC] = {
    {"3V3/1V8", 0x70, 0, 0x40},   // Dual supply 1.8 / 3.3 V
    {"F1VCCINT1", 0x70, 1, 0x44}, // first vccint, F1
    {"F1VCCINT2", 0x70, 2, 0x43}, // second vccint, F1
    {"F2VCCINT1", 0x70, 3, 0x44}, // first vccint, F2
    {"F2VCCINT2", 0x70, 4, 0x43}, // second vccint, F2
    {"F1AVTT/CC", 0x70, 5, 0x40}, // AVCC/AVTT for F1
    {"F2AVTT/CC", 0x70, 6, 0x40}, // AVCC/AVTT for F2
};

// Run DCDC I2C production test
// TODO add test for I2C_RESET_PWR
BaseType_t run_dcdc_i2ctest(int argc, char **argv, char *m)
{
  // initialize to the current tick time
  TickType_t xLastWakeTime = xTaskGetTickCount();
  uint8_t data[2];

  //// attempt to grab semaphore
  //// TODO define semaphores including i2c1 somewhere
  //if (acquireI2CSemaphore(i2c1_sem) == pdFAIL) {
  //  snprintf(m, SCRATCH_SIZE, "Could not get I2C1 semaphore in time\r\n");
  //  return pdFALSE;
  //}

  // do two passes, write the first time and read the second
  for (uint8_t rw = 0; rw < 2; ++rw) {

    // loop over devices
    for (uint8_t ps = 0; ps < N_PM_ADDRS_DCDC; ++ps) {

      // select the appropriate output for the mux
      data[0] = 0x1U << pm_addrs_dcdc[ps].mux_bit;
      tSMBusStatus r = SMBusMasterI2CWrite(&g_sMaster1, 
                                           pm_addrs_dcdc[ps].mux_addr, data, 
                                           1);
      if (r != SMBUS_OK) {
        snprintf(m, SCRATCH_SIZE, 
                 "ERROR: Could not select dev=%d on MUX\r\n", ps);
        //release_semaphore();
        return pdFALSE;
      }
      int tries = 0;
      while (SMBusStatusGet(&g_sMaster1) == SMBUS_TRANSFER_IN_PROGRESS) {
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(10)); // wait
        if (++tries > 500) {
          snprintf(m, SCRATCH_SIZE, 
                   "ERROR: MUX select timed out (dev=%d)\r\n", ps);
          //release_semaphore();
          return pdFALSE;
        }
      }
      if (eStatus1 != SMBUS_OK) {
        snprintf(m, SCRATCH_SIZE, 
                 "ERROR: SM bad status on MUX select (dev=%d)\r\n", 
                 ps);
        //release_semaphore();
        return pdFALSE;
      }

      // loop over pages on the supply
      // do we actually need this? If our goal is just to test the hardware I2C
      // connections, we could just use the first controller on each LGA80D
      for (uint8_t page = 0; page < NPAGES_PS; ++page) {
        // select page
        r = SMBusMasterByteWordWrite(&g_sMaster1, pm_addrs_dcdc[ps].dev_addr, 
                                     PAGE_COMMAND, &page, 1);
        if (r != SMBUS_OK) {
          snprintf(m, SCRATCH_SIZE, 
                   "ERROR: Could not select page=%d on dev=%d\r\n", page, ps);
          //release_semaphore();
          return pdFALSE;
        }
        while (SMBusStatusGet(&g_sMaster1) == SMBUS_TRANSFER_IN_PROGRESS) {
          vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(10)); // wait
        }
        if (eStatus1 != SMBUS_OK) {
          snprintf(m, SCRATCH_SIZE, 
                   "ERROR: SM bad status on page=%d select (dev=%d)\r\n", page,
                   ps);
          //release_semaphore();
          return pdFALSE;
        }

        // Use B0 (USER_DATA_00)
        if (rw == 0) { // write on first pass
          data[0] = page+ps*NPAGES_PS+1;
          data[1] = 0x3CU;
          r = SMBusMasterByteWordWrite(&g_sMaster1, pm_addrs_dcdc[ps].dev_addr, 
                                       0xb0, data, 2);
        }
        else { // read on second pass
          data[0] = 0x0U;
          data[1] = 0x0U;
          r = SMBusMasterByteWordRead(&g_sMaster1, pm_addrs_dcdc[ps].dev_addr,
                                      0xb0, data, 2);
        }
        if (r != SMBUS_OK) {
          snprintf(m, SCRATCH_SIZE, 
                   "ERROR: Read failed (page=%d dev=%d)\r\n", page, ps);
          //release_semaphore();
          return pdFALSE;
        }
        tries = 0;
        while (SMBusStatusGet(&g_sMaster1) == SMBUS_TRANSFER_IN_PROGRESS) {
          vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(10)); // wait
          if (++tries > 500) {
            snprintf(m, SCRATCH_SIZE, 
                     "ERROR: Read timed out (page=%d dev=%d)\r\n", page, ps);
            //release_semaphore();
            return pdFALSE;
          }
        }
        if (eStatus1 != SMBUS_OK) {
          snprintf(m, SCRATCH_SIZE, 
                   "ERROR: SM bad status after read (page=%d dev=%d)\r\n", 
                   page, ps);
          //release_semaphore();
          return pdFALSE;
        }
        if (rw == 0) { 
          // check read value
          if (data[0] != page+ps*NPAGES_PS+1) {
            snprintf(m, SCRATCH_SIZE, 
                     "ERROR: Bad bit 0 on dev %d-%d (expected %d, got %d)\r\n", 
                     ps, page, page+ps*NPAGES_PS+1, data[0]);
            //release_semaphore();
            return pdFALSE;
          }
          if (data[1] != 0x3CU) {
            snprintf(m, SCRATCH_SIZE, 
                     "ERROR: Bad bit 1 on dev %d-%d (expected 60, got %d)\r\n", 
                     ps, page, data[1]);
            //release_semaphore();
            return pdFALSE;
          }
        }

        // wait here for the 10 msec
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(10));
      } // loop over pages on the supply
    } // loop over devices
  } // read/write passes

  //print output
  snprintf(m, SCRATCH_SIZE, "Test success.\r\n");
  //release_semaphore();
  return pdFALSE;
}   
