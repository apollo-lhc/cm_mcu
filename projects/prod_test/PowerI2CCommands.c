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
  float vals[N_PM_ADDRS_DCDC*NPAGES_PS];

  //// attempt to grab semaphore
  //// TODO define semaphores including i2c1 somewhere
  //if (acquireI2CSemaphore(i2c1_sem) == pdFAIL) {
  //  snprintf(m, SCRATCH_SIZE, "Could not get I2C1 semaphore in time\r\n");
  //  return pdFALSE;
  //}

  // loop over devices
  for (int ps = 0; ps < N_PM_ADDRS_DCDC; ++ps) {

    // select the appropriate output for the mux
    data[0] = 0x1U << pm_addrs_dcdc[ps].mux_bit;
    tSMBusStatus r = SMBusMasterI2CWrite(&g_sMaster1, 
                                         pm_addrs_dcdc[ps].mux_addr, data, 1);
    if (r != SMBUS_OK) {
      snprintf(m, SCRATCH_SIZE, "Could not select dev=%d on MUX\r\n", ps);
      //release_semaphore();
      return pdFALSE;
    }
    int tries = 0;
    while (SMBusStatusGet(&g_sMaster1) == SMBUS_TRANSFER_IN_PROGRESS) {
      vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(10)); // wait
      if (++tries > 500) {
        snprintf(m, SCRATCH_SIZE, "MUX select timed out (dev=%d)\r\n", ps);
        //release_semaphore();
        return pdFALSE;
      }
    }
    if (eStatus1 != SMBUS_OK) {
      snprintf(m, SCRATCH_SIZE, "SM bad status on MUX select (dev=%d)\r\n", 
               ps);
      //release_semaphore();
      return pdFALSE;
    }

    // loop over pages on the supply
    for (uint8_t page = 0; page < NPAGES_PS; ++page) {
      // select page
      r = SMBusMasterByteWordWrite(&g_sMaster1, pm_addrs_dcdc[ps].dev_addr, 
                                   PAGE_COMMAND, &page, 1);
      if (r != SMBUS_OK) {
        snprintf(m, SCRATCH_SIZE, "Could not select page=%d on dev=%d\r\n", 
                 page, ps);
        //release_semaphore();
        return pdFALSE;
      }
      while (SMBusStatusGet(&g_sMaster1) == SMBUS_TRANSFER_IN_PROGRESS) {
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(10)); // wait
      }
      if (eStatus1 != SMBUS_OK) {
        snprintf(m, SCRATCH_SIZE, 
                 "SM bad status on page=%d select (dev=%d)\r\n", page, ps);
        //release_semaphore();
        return pdFALSE;
      }

      // For now, just do run read to test
      // TODO find a register than we can use for read/write test
      // Use 8D (Internal temperature)
      data[0] = 0x0U;
      data[1] = 0x0U;
      r = SMBusMasterByteWordRead(&g_sMaster1, pm_addrs_dcdc[ps].dev_addr,
                                  0x8d, data, 2);
      if (r != SMBUS_OK) {
        snprintf(m, SCRATCH_SIZE, 
                 "Read failed (page=%d dev=%d)\r\n", page, ps);
        //release_semaphore();
        return pdFALSE;
      }
      tries = 0;
      while (SMBusStatusGet(&g_sMaster1) == SMBUS_TRANSFER_IN_PROGRESS) {
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(10)); // wait
        if (++tries > 500) {
          snprintf(m, SCRATCH_SIZE, 
                   "Read timed out (page=%d dev=%d)\r\n", page, ps);
          //release_semaphore();
          return pdFALSE;
        }
      }
      if (eStatus1 != SMBUS_OK) {
        snprintf(m, SCRATCH_SIZE, 
                 "SM bad status after read (page=%d dev=%d)\r\n", page, ps);
        //release_semaphore();
        return pdFALSE;
      }
      linear11_val_t ii;
      ii.raw = (data[1] << 8) | data[0];
      vals[page+ps*NPAGES_PS] = linear11_to_float(ii);

      // wait here for the 10 msec
      vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(10));
    } // loop over pages on the supply
  } // loop over devices

  //print output
  int tens[N_PM_ADDRS_DCDC*NPAGES_PS];
  int frac[N_PM_ADDRS_DCDC*NPAGES_PS];
  for (uint8_t idevpg = 0; idevpg < N_PM_ADDRS_DCDC*NPAGES_PS; ++idevpg) {
    float_to_ints(vals[idevpg], &tens[idevpg], &frac[idevpg]);
  }
  snprintf(m, SCRATCH_SIZE, 
           "Test success. read temperatures(C): (%02d.%02d, %02d.%02d, %02d.%02d, %02d.%02d, %02d.%02d %02d.%02d, %02d.%02d, %02d.%02d, %02d.%02d, %02d.%02d, %02d.%02d, %02d.%02d, %02d.%02d, %02d.%02d)\r\n", 
           tens[0], frac[0], tens[1], frac[1], tens[2], frac[2],
           tens[3], frac[3], tens[4], frac[4], tens[5], frac[5],
           tens[6], frac[6], tens[7], frac[7], tens[8], frac[8],
           tens[9], frac[9], tens[10], frac[10], tens[11], frac[11],
           tens[12], frac[12], tens[13], frac[13]);
  //release_semaphore();
  return pdFALSE;
}   
