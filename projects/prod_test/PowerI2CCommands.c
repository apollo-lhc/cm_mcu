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
#include "PowerI2CCommands.h"
#include "I2CCommunication.h"

// DC-DC device info
struct dev_i2c_addr_t pm_addrs_dcdc[N_PM_ADDRS_DCDC] = {
    {"3V3/1V8", U103_ADDR, POW3V31V8_MUX_BIT, POW3V31V8_ADDR},
    {"F1VCCINT1", U103_ADDR, F1VCCINT1_MUX_BIT, F1VCCINT1_ADDR},
    {"F1VCCINT2", U103_ADDR, F1VCCINT2_MUX_BIT, F1VCCINT2_ADDR},
    {"F2VCCINT1", U103_ADDR, F2VCCINT1_MUX_BIT, F2VCCINT1_ADDR},
    {"F2VCCINT2", U103_ADDR, F2VCCINT2_MUX_BIT, F2VCCINT2_ADDR},
    {"F1AVTT/CC", U103_ADDR, F1AVTTVCC_MUX_BIT, F1AVTTVCC_ADDR},
    {"F2AVTT/CC", U103_ADDR, F2AVTTVCC_MUX_BIT, F2AVTTVCC_ADDR},
};

/**
 * @details
 * Tests I2C communication to the DC-DC converters by first performing a loop
 * where some (distinct) data is written to the B0 (USER_DATA_00) register of
 * each DC-DC converter, then a second loop reads the data and verifies that it
 * matches what was written. Finally, the MUX reset signal is tested by
 * checking a read attempt fails following a MUX reset
 */
BaseType_t run_dcdc_i2ctest(int argc, char **argv, char *m)
{
  uint8_t data[2];
  int r;
  uint8_t page = 0;

  // do two passes, write the first time and read the second
  for (uint8_t rw = 0; rw < 2; ++rw) {

    // loop over devices
    for (uint8_t ps = 0; ps < N_PM_ADDRS_DCDC; ++ps) {

      // select the appropriate output for the mux
      if (apollo_i2c_ctl_w(POWER_I2C_BASE, pm_addrs_dcdc[ps].mux_addr, 1,
                           0x1U << pm_addrs_dcdc[ps].mux_bit)) {
        snprintf(m, SCRATCH_SIZE, "ERROR: Failed to select dev %d on MUX)\r\n",
                 ps);
        return pdFALSE;
      }

      // select page
      if (apollo_pmbus_rw(POWER_I2C_BASE, false, pm_addrs_dcdc[ps].dev_addr,
                          PAGE_COMMAND, &page, 1)) {
        snprintf(m, SCRATCH_SIZE,
                 "ERROR: Failed to select page 0 on dev %d\r\n", ps);
        return pdFALSE;
      }

      // write on first pass
      if (rw == 0) {
        data[0] = ps + 1;
        data[1] = LGA80D_TEST_CONST;
        r = apollo_pmbus_rw(POWER_I2C_BASE, false, pm_addrs_dcdc[ps].dev_addr,
                            LGA80D_ADDR_USER_DATA_00, data, 2);
      }
      // read on second pass
      else {
        data[0] = 0x0U;
        data[1] = 0x0U;
        r = apollo_pmbus_rw(POWER_I2C_BASE, false, pm_addrs_dcdc[ps].dev_addr,
                            LGA80D_ADDR_USER_DATA_00, data, 2);
      }
      if (r) {
        snprintf(m, SCRATCH_SIZE,
                 "ERROR: Failed read/write %d, dev %d\r\n", rw, ps);
        return pdFALSE;
      }

      if (rw == 0) {
        // check read value
        if (data[0] != (ps + 1)) {
          snprintf(m, SCRATCH_SIZE,
                   "ERROR: Bad bit 0 on dev %d (expected %d, got %d)\r\n",
                   ps, ps + 1, data[0]);
          return pdFALSE;
        }
        if (data[1] != LGA80D_TEST_CONST) {
          snprintf(m, SCRATCH_SIZE,
                   "ERROR: Bad bit 1 on dev %d (expected 60, got %d)\r\n",
                   ps, data[1]);
          return pdFALSE;
        }
      }

      // wait here for 10 msec
      vTaskDelay(pdMS_TO_TICKS(10));
    } // loop over devices
  }   // read/write passes

  // test reset by attempting read; as long as we don't use an address 0xAX,
  // we shouldn't accidentally address the MUX
  write_gpio_pin(_PWR_I2C_RESET, 0x0);
  vTaskDelay(pdMS_TO_TICKS(1));
  write_gpio_pin(_PWR_I2C_RESET, 0x1);

  bool read_fail = false;
  data[0] = 0x0U;
  if (apollo_pmbus_rw(POWER_I2C_BASE, true,
                      pm_addrs_dcdc[N_PM_ADDRS_DCDC - 1].dev_addr,
                      LGA80D_ADDR_PMBUS_REVISION, data, 1)) {
    read_fail = true;
  }
  if (!read_fail) {
    if (data[0] != LGA80D_PMBUS_VER) {
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
