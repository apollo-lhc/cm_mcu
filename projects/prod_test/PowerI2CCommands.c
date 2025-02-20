/*
 * PowerI2CCommands.c
 *
 *  Created on: January 1, 2025
 *      Author: mcoshiro
 *
 * Contains code for MCU to DC-DC I2C and power-on tests used for Apollo CM
 * production testing
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
#include "common/power_ctl.h"
#include "common/printf.h"
#include "common/utils.h"
#include "ADCMonitorTask.h"
#include "commands.h"
#include "PowerI2CCommands.h"
#include "I2CCommunication.h"

// DC-DC device info
struct dev_i2c_addr_t pm_addrs_dcdc[N_PM_ADDRS_DCDC] = {
    {"3V3/1V8", POWER_I2C_MUX_ADDR, POWER_I2C_POW3V31V8_MUX_BIT,
     POWER_I2C_POW3V31V8_ADDR},
    {"F1VCCINT1", POWER_I2C_MUX_ADDR, POWER_I2C_F1VCCINT1_MUX_BIT,
     POWER_I2C_F1VCCINT1_ADDR},
    {"F1VCCINT2", POWER_I2C_MUX_ADDR, POWER_I2C_F1VCCINT2_MUX_BIT,
     POWER_I2C_F1VCCINT2_ADDR},
    {"F2VCCINT1", POWER_I2C_MUX_ADDR, POWER_I2C_F2VCCINT1_MUX_BIT,
     POWER_I2C_F2VCCINT1_ADDR},
    {"F2VCCINT2", POWER_I2C_MUX_ADDR, POWER_I2C_F2VCCINT2_MUX_BIT,
     POWER_I2C_F2VCCINT2_ADDR},
    {"F1AVTT/CC", POWER_I2C_MUX_ADDR, POWER_I2C_F1AVTTVCC_MUX_BIT,
     POWER_I2C_F1AVTTVCC_ADDR},
    {"F2AVTT/CC", POWER_I2C_MUX_ADDR, POWER_I2C_F2AVTTVCC_MUX_BIT,
     POWER_I2C_F2AVTTVCC_ADDR},
};

/**
 * @details
 * CLI command that tests I2C communication to the DC-DC converters by first
 * performing a loop where some (distinct) data is written to the B0
 * (USER_DATA_00) register of each DC-DC converter, then a second loop reads
 * the data and verifies that it matches what was written. Finally, the MUX
 * reset signal is tested by checking a read attempt fails following a MUX
 * reset
 */
BaseType_t dcdc_i2ctest_ctl(int argc, char **argv, char *m)
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
                          LGA80D_PAGE_COMMAND, &page, 1)) {
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
        r = apollo_pmbus_rw(POWER_I2C_BASE, true, pm_addrs_dcdc[ps].dev_addr,
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
  } // read/write passes

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

// poweron check constants
struct power_monparams_t power_monparams[N_POWERON_CHECKS] = {
    {1, 1, 1, 0.85}, // FPGA VCCINT - F1
    {1, 1, 0, 0.85}, // FPGA VCCINT - F1
    {1, 2, 1, 0.85}, // FPGA VCCINT - F1
    {1, 2, 0, 0.85}, // FPGA VCCINT - F1
    {1, 3, 1, 0.85}, // FPGA VCCINT - F2
    {1, 3, 0, 0.85}, // FPGA VCCINT - F2
    {1, 4, 1, 0.85}, // FPGA VCCINT - F2
    {1, 4, 0, 0.85}, // FPGA VCCINT - F2
    {2, 0, 1, 3.3},  // 3V3/1V8 - 3v3
    {2, 0, 0, 1.8},  // 3V3/1V8 - 1v8
    {4, 5, 1, 0.9},  // AVCC - F1
    {4, 6, 1, 0.9},  // AVCC - F2
    {5, 5, 0, 1.2},  // AVTT - F1
    {5, 6, 0, 1.2},  // AVTT - F2
};

/**
 * @details
 * Loops over power-on levels, turns the power on, then checks both the MCU
 * ADCs as well as LGA80D internal reading to confirm voltage is as expected
 */
BaseType_t dcdc_poweron_ctl(int argc, char **argv, char *m)
{
  float delta;
  int copied = 0;
  uint8_t data[2];
  for (int32_t level = 1; level <= N_POWER_LEVELS; level++) {
    turn_on_ps_at_prio(true, true, level);
    vTaskDelay(pdMS_TO_TICKS(1000)); // let ADC catch up
    if (check_ps_at_prio(level, true, true, &delta)) {
      copied += snprintf(m + copied, SCRATCH_SIZE - copied,
                         "ERROR: Failed poweron at level %d, delta=%f\r\n",
                         level, (double)delta);
      return pdFALSE;
    }
    for (uint8_t icheck = 0; icheck < N_POWERON_CHECKS; icheck++) {
      if (power_monparams[icheck].level == level) {
        uint8_t ps = power_monparams[icheck].ps_index;
        uint8_t page = power_monparams[icheck].page;
        float nominal_voltage = power_monparams[icheck].nominal_voltage;
        // select MUX
        if (apollo_i2c_ctl_w(POWER_I2C_BASE, pm_addrs_dcdc[ps].mux_addr, 1,
                             0x1U << pm_addrs_dcdc[ps].mux_bit)) {
          copied += snprintf(m + copied, SCRATCH_SIZE - copied,
                             "ERROR: Failed to select dev %d on MUX\r\n", ps);
          return pdFALSE;
        }
        // select page
        if (apollo_pmbus_rw(POWER_I2C_BASE, false, pm_addrs_dcdc[ps].dev_addr,
                            LGA80D_PAGE_COMMAND, &page, 1)) {
          copied += snprintf(m + copied, SCRATCH_SIZE - copied,
                             "ERROR: Failed to select page 0 on %d\r\n", ps);
          return pdFALSE;
        }
        // read voltage
        data[0] = 0x0U;
        data[1] = 0x0U;
        if (apollo_pmbus_rw(POWER_I2C_BASE, true, pm_addrs_dcdc[ps].dev_addr,
                            LGA80D_ADDR_READ_VOUT, data, 2)) {
          copied += snprintf(m + copied, SCRATCH_SIZE - copied,
                             "ERROR: Failed to read voltage from %d\r\n", ps);
          return pdFALSE;
        }
        uint16_t data_full = (data[1] << 8) | data[0];
        float read_voltage = linear16u_to_float(data_full);
        delta = (read_voltage - nominal_voltage) / nominal_voltage;
        if (fabs(delta) > POWER_DELTA_TOLERANCE) {
          copied += snprintf(m + copied, SCRATCH_SIZE - copied,
                             "ERROR: LGA80D check %d voltage out of tolerance."
                             "Expected %f, got %f.\r\n",
                             icheck,
                             (double)nominal_voltage, (double)read_voltage);
          return pdFALSE;
        }
      }
    } // loop over checks
  } // loop over levels

  copied += snprintf(m + copied, SCRATCH_SIZE - copied,
                     "Power on test: success.\r\n");
  return pdFALSE;
}
