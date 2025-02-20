/*
 * FireflyI2CCommands.c
 *
 *  Created on: February 10, 2025
 *      Author: mcoshiro
 *
 * Contains code for MCU to FPGA I2C tests for Apollo CM production tests
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
#include "FPGAI2CCommands.h"
#include "I2CCommunication.h"

/**
 * @details
 * Puts data into format required by I2C DRP interface. See Xilinx SYSMON guide
 */
uint32_t gen_sysmon_i2cword(uint32_t addr, uint32_t data, bool read)
{
  uint32_t cmd = 0x2;
  if (read) {
    cmd = 0x1;
  }
  uint32_t i2c_word = (cmd << 26) | ((addr & 0x3FF) << 16) | data;
  // fix endianness
  i2c_word = (((i2c_word & 0xFF000000) >> 24) |
              ((i2c_word & 0x00FF0000) >> 8) |
              ((i2c_word & 0x0000FF00) << 8) |
              ((i2c_word & 0x000000FF) << 24));
  return i2c_word;
}

/**
 * @details
 * CLI function that tests I2C communication to FPGAs. A custom word is
 * written to each sysmon upper temperature alarm register and the read back
 * to confirm reading and writing work. The MUX reset is then tested by
 * checking a read fails after the reset is asserted
 */
bool fpga_i2ctest(char *m, int32_t *copied)
{

  uint8_t mux_bits[N_FPGAS] = {FPGA_I2C_F1_SYSMON_MUXBIT,
                               FPGA_I2C_F2_SYSMON_MUXBIT};
  uint32_t data;
  double temperature;

  for (uint8_t idev = 0; idev < N_FPGAS; idev++) {

    uint8_t mux_data = 0x1U << mux_bits[idev];
    if (apollo_i2c_ctl_w(FPGA_I2C_BASE, FPGA_I2C_MUX_ADDR, 1,
                         mux_data)) {
      (*copied) += snprintf(m + (*copied), SCRATCH_SIZE - (*copied),
                            "ERROR: selecting dev %d on MUX\r\n", idev);
      return false;
    }

    data = 0x0;
    if (apollo_i2c_ctl_reg_r(FPGA_I2C_BASE, FPGA_I2C_SYSMON_ADDR, 4,
                             gen_sysmon_i2cword(
                                 VU13P_TEMPERATURE_ADDR, 0x0, true),
                             2, &data)) {
      (*copied) += snprintf(m + (*copied), SCRATCH_SIZE - (*copied),
                            "ERROR: Failed to read from FPGA.\r\n");
      return false;
    }
    // magic formula from SYSMON guide
    temperature = ((double)data) * 509.314 / 65536.0 - 280.23;
    if (temperature < MIN_FPGA_TEMPERATURE || temperature > MAX_FPGA_TEMPERATURE) {
      (*copied) += snprintf(m + (*copied), SCRATCH_SIZE - (*copied),
                            "ERROR: FPGA temperature out of bounds (%f C)\r\n",
                            temperature);
      return false;
    }
  } // loop over devices

  // test MUX reset by repeating last read
  write_gpio_pin(_FPGA_I2C_RESET, 0x0);
  vTaskDelay(pdMS_TO_TICKS(1));
  write_gpio_pin(_FPGA_I2C_RESET, 0x1);

  bool fail = false;
  if (apollo_i2c_ctl_reg_r(FPGA_I2C_BASE, FPGA_I2C_SYSMON_ADDR, 4,
                           gen_sysmon_i2cword(
                               VU13P_TEMPERATURE_ADDR, 0x0, true),
                           2, &data)) {
    fail = true;
  }
  if (!fail) {
    (*copied) += snprintf(m + (*copied), SCRATCH_SIZE - (*copied),
                          "ERROR: MUX reset failed.\r\n");
    return false;
  }

  (*copied) += snprintf(m + (*copied), SCRATCH_SIZE - (*copied),
                        "FPGA I2C test: success.\r\n");
  return true;
}

/**
 * @details
 * Wrapper around fpga_i2ctest
 */
BaseType_t fpga_i2ctest_ctl(int argc, char **argv, char *m)
{
  int32_t copied = 0;
  fpga_i2ctest(m, &copied);
  return pdFALSE;
}
