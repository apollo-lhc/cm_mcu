#pragma once

#include "FreeRTOS.h" // IWYU pragma: keep
#include <stdbool.h>

#define POWER_I2C_BASE              1
#define POWER_I2C_MUX_ADDR          0x70
#define POWER_I2C_POW3V31V8_MUX_BIT 0
#define POWER_I2C_F1VCCINT1_MUX_BIT 1
#define POWER_I2C_F1VCCINT2_MUX_BIT 2
#define POWER_I2C_F2VCCINT1_MUX_BIT 3
#define POWER_I2C_F2VCCINT2_MUX_BIT 4
#define POWER_I2C_F1AVTTVCC_MUX_BIT 5
#define POWER_I2C_F2AVTTVCC_MUX_BIT 6
#define POWER_I2C_POW3V31V8_ADDR    0x40
#define POWER_I2C_F1VCCINT1_ADDR    0x44
#define POWER_I2C_F1VCCINT2_ADDR    0x43
#define POWER_I2C_F2VCCINT1_ADDR    0x44
#define POWER_I2C_F2VCCINT2_ADDR    0x43
#define POWER_I2C_F1AVTTVCC_ADDR    0x40
#define POWER_I2C_F2AVTTVCC_ADDR    0x40
#define LGA80D_PAGE_COMMAND         0x0
#define LGA80D_ADDR_USER_DATA_00    0xB0
#define LGA80D_ADDR_PMBUS_REVISION  0x98
#define LGA80D_ADDR_READ_VOUT       0x8B
#define LGA80D_PMBUS_VER            0x22
#define LGA80D_TEST_CONST           0x3CU
#define LGA80D_MAX_ATTEMPTS         500
#define N_PM_ADDRS_DCDC             7
#define NPAGES_PS                   2
#define N_POWER_LEVELS              5
#define N_POWERON_CHECKS            14
#define POWER_DELTA_TOLERANCE       0.05 // relative

// information on to find an I2C device, with a mux in front of it
struct dev_i2c_addr_t {
  char *name;
  uint8_t mux_addr; // I2C address of the Mux
  uint8_t mux_bit;  // port of the mux; write value 0x1U<<mux_bit to the mux register
  uint8_t dev_addr; // I2C address of device.
};

// information for performing voltage checks
struct power_monparams_t {
  uint8_t level;         // power level for check
  uint8_t ps_index;      // power supply index (to dev_i2c_addr_t)
  uint8_t page;          // page
  float nominal_voltage; // expected voltages
};

/**
 * @brief Tests I2C communication to DC-DC converters
 *
 * @param [out] m  output string
 * @param [inout] copied  output already used in buffer
 * @return true if test succeeds, false otherwise
 */
bool dcdc_i2ctest(char *m, int32_t *copied);

/**
 * @brief CLI wrapper around DC-DC I2C converter test. See above
 *
 * @param [in] argc  number of CLI arguments
 * @param [in] argv  CLI arguments
 * @param [out] m  output string
 * @return pdFALSE
 */
BaseType_t dcdc_i2ctest_ctl(int argc, char **argv, char *m);

/**
 * @brief Tries to enable DC-DC converters and checks output
 *
 * @param [out] m  output string
 * @param [inout] copied  output already used in buffer
 * @return true if test succeeds, false otherwise
 */
bool dcdc_powerontest(char *m, int32_t *copied);

/**
 * @brief CLI wrapper around DC-DC power-on test. See above
 *
 * @param [in] argc  number of CLI arguments
 * @param [in] argv  CLI arguments
 * @param [out] m  output string
 * @return pdFALSE
 */
BaseType_t dcdc_powerontest_ctl(int argc, char **argv, char *m);
