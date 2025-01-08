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
#include <stdbool.h>
#include <stdint.h>

// FreeRTOS
#include "FreeRTOS.h"
#include "task.h"

// local includes
#include "common/smbus.h"
#include "common/smbus_units.h"
#include "common/pinsel.h"
#include "common/printf.h"
#include "common/utils.h"
#include "commands.h"
#include "InterruptHandlers.h"
#include "PowerI2CCommands.h"

// DC-DC device info
// TODO move this somewhere else?
struct dev_i2c_addr_t pm_addrs_dcdc[N_PM_ADDRS_DCDC] = {
    {"3V3/1V8", 0x70, 0, 0x40},   // Dual supply 1.8 / 3.3 V
    {"F1VCCINT1", 0x70, 1, 0x44}, // first vccint, F1
    {"F1VCCINT2", 0x70, 2, 0x43}, // second vccint, F1
    {"F2VCCINT1", 0x70, 3, 0x44}, // first vccint, F2
    {"F2VCCINT2", 0x70, 4, 0x43}, // second vccint, F2
    {"F1AVTT/CC", 0x70, 5, 0x40}, // AVCC/AVTT for F1
    {"F2AVTT/CC", 0x70, 6, 0x40}, // AVCC/AVTT for F2
};

/**
 * @brief helper function to verify I2C/SMBUS transactions sucecssful
 * @details
 * To verify transaction, make a call to this function after calling an SMBUS
 * method such as SMBusMasterI2CWrite. It verifies no errors were generated
 * and that the transaction finishes in specified time
 *
 * @param [in] r  tSMBUSStatus returned by SMBUS call to check
 * @param [in] timeout  time until timeout in units of 10ms, <0 disables check
 * @param [in] no_message  if true, disables printing message
 * @param [out] m  output string
 * @return 0 if no errors encountered, otherwise, size of m buffer used by
 *         error message (no_message false) or 1 (no_message true)
 */
int check_i2c_transaction(tSMBusStatus r, int timeout, bool no_message,
                          char *m)
{
  int copied = 0;
  if (r != SMBUS_OK) {
    if (!no_message) {
      copied = snprintf(m, SCRATCH_SIZE, "ERROR: SMBUS command not OK");
    }
    else {
      copied = 1;
    }
    return copied;
  }
  int tries = 0;
  while (SMBusStatusGet(&g_sMaster1) == SMBUS_TRANSFER_IN_PROGRESS) {
    // vTaskDelayUntil(xLastWakeTime, pdMS_TO_TICKS(10)); // wait
    vTaskDelay(pdMS_TO_TICKS(10));
    if (timeout > 0) {
      if (++tries > timeout) {
        if (!no_message) {
          copied = snprintf(m, SCRATCH_SIZE, "ERROR: SMBUS timeout");
        }
        else {
          copied = 1;
        }
        return copied;
      }
    }
  }
  if (eStatus1 != SMBUS_OK) {
    if (!no_message) {
      copied = snprintf(m, SCRATCH_SIZE, "ERROR: SMBUS not OK");
    }
    else {
      copied = 1;
    }
    return copied;
  }
  return copied;
}

/**
 * @brief Tests I2C communication to DC-DC converters
 * @details
 * Tests I2C communication to the DC-DC converters by first performing a loop
 * where some (distinct) data is written to the B0 (USER_DATA_00) register of
 * each DC-DC converter, then a second loop reads the data and verifies that it
 * matches what was written. Finally, the MUX reset signal is tested by
 * checking a read attempt fails following a MUX reset
 *
 * @param [in] argc  number of CLI arguments
 * @param [in] argv  CLI arguments
 * @param [out] m  output string
 * @return pdFALSE
 */
BaseType_t run_dcdc_i2ctest(int argc, char **argv, char *m)
{
  // initialize to the current tick time
  // TickType_t xLastWakeTime = xTaskGetTickCount();
  uint8_t data[2];
  tSMBusStatus r;
  int copied = 0;

  // do two passes, write the first time and read the second
  for (uint8_t rw = 0; rw < 2; ++rw) {

    // loop over devices
    for (uint8_t ps = 0; ps < N_PM_ADDRS_DCDC; ++ps) {

      // select the appropriate output for the mux
      data[0] = 0x1U << pm_addrs_dcdc[ps].mux_bit;
      r = SMBusMasterI2CWrite(&g_sMaster1, pm_addrs_dcdc[ps].mux_addr, data,
                              1);
      copied = check_i2c_transaction(r, 500, false, m);
      if (copied != 0) {
        snprintf(m + copied, SCRATCH_SIZE - copied,
                 "(selecting dev %d on MUX)\r\n", ps);
        return pdFALSE;
      }

      // loop over pages on the supply
      // do we actually need this? If our goal is just to test the hardware I2C
      // connections, we could just use the first controller on each LGA80D
      for (uint8_t page = 0; page < NPAGES_PS; ++page) {
        // select page
        r = SMBusMasterByteWordWrite(&g_sMaster1, pm_addrs_dcdc[ps].dev_addr,
                                     PAGE_COMMAND, &page, 1);
        copied = check_i2c_transaction(r, -1, false, m);
        if (copied != 0) {
          snprintf(m + copied, SCRATCH_SIZE - copied,
                   "(selecting page %d on dev %d)\r\n", page, ps);
          return pdFALSE;
        }

        // Use B0 (USER_DATA_00)
        if (rw == 0) { // write on first pass
          data[0] = page + ps * NPAGES_PS + 1;
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
        copied = check_i2c_transaction(r, 500, false, m);
        if (copied != 0) {
          snprintf(m + copied, SCRATCH_SIZE - copied,
                   "(read/write %d, page %d, dev %d)\r\n", rw, page, ps);
          return pdFALSE;
        }

        if (rw == 0) {
          // check read value
          if (data[0] != page + ps * NPAGES_PS + 1) {
            snprintf(m, SCRATCH_SIZE,
                     "ERROR: Bad bit 0 on dev %d-%d (expected %d, got %d)\r\n",
                     ps, page, page + ps * NPAGES_PS + 1, data[0]);
            return pdFALSE;
          }
          if (data[1] != 0x3CU) {
            snprintf(m, SCRATCH_SIZE,
                     "ERROR: Bad bit 1 on dev %d-%d (expected 60, got %d)\r\n",
                     ps, page, data[1]);
            return pdFALSE;
          }
        }

        // wait here for 10 msec
        // vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(10));
        vTaskDelay(pdMS_TO_TICKS(10));
      } // loop over pages on the supply
    }   // loop over devices
  }     // read/write passes

  // test reset by attempting read; as long as we don't use an address 0xAX,
  // we shouldn't accidentally address the MUX
  write_gpio_pin(_PWR_I2C_RESET, 0x0);
  vTaskDelay(pdMS_TO_TICKS(1));
  write_gpio_pin(_PWR_I2C_RESET, 0x1);

  bool read_fail = false;
  // read register 0x98 (PMBUS_REVISION), which should be 0x22
  data[0] = 0x0U;
  r = SMBusMasterByteWordRead(&g_sMaster1,
                              pm_addrs_dcdc[N_PM_ADDRS_DCDC - 1].dev_addr, 0x98,
                              data, 1);
  copied = check_i2c_transaction(r, 500, true, m);
  if (copied == 1) {
    read_fail = true;
  }
  if (!read_fail) {
    if (data[0] != 0x22) {
      read_fail = true;
    }
  }
  if (!read_fail) {
    snprintf(m, SCRATCH_SIZE, "ERROR: I2C MUX reset did not work.\r\n");
    return pdFALSE;
  }

  // print output
  snprintf(m, SCRATCH_SIZE, "Test success.\r\n");
  return pdFALSE;
}
