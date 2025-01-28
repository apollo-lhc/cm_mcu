#pragma once

#include "FreeRTOS.h" // IWYU pragma: keep

#define POWER_I2C_BASE             1
#define U103_ADDR                  0x70
#define POW3V31V8_MUX_BIT          0
#define F1VCCINT1_MUX_BIT          1
#define F1VCCINT2_MUX_BIT          2
#define F2VCCINT1_MUX_BIT          3
#define F2VCCINT2_MUX_BIT          4
#define F1AVTTVCC_MUX_BIT          5
#define F2AVTTVCC_MUX_BIT          6
#define POW3V31V8_ADDR             0x40
#define F1VCCINT1_ADDR             0x44
#define F1VCCINT2_ADDR             0x43
#define F2VCCINT1_ADDR             0x44
#define F2VCCINT2_ADDR             0x43
#define F1AVTTVCC_ADDR             0x40
#define F2AVTTVCC_ADDR             0x40
#define PAGE_COMMAND               0x0
#define LGA80D_ADDR_USER_DATA_00   0xB0
#define LGA80D_ADDR_PMBUS_REVISION 0x98
#define LGA80D_PMBUS_VER           0x22
#define LGA80D_TEST_CONST          0x3CU
#define LGA80D_MAX_ATTEMPTS        500
#define N_PM_ADDRS_DCDC            7
#define NPAGES_PS                  2

// how to find an I2C device, with a mux infront of it.
struct dev_i2c_addr_t {
  char *name;
  uint8_t mux_addr; // I2C address of the Mux
  uint8_t mux_bit;  // port of the mux; write value 0x1U<<mux_bit to the mux register
  uint8_t dev_addr; // I2C address of device.
};

/**
 * @brief Tests I2C communication to DC-DC converters
 *
 * @param [in] argc  number of CLI arguments
 * @param [in] argv  CLI arguments
 * @param [out] m  output string
 * @return pdFALSE
 */
BaseType_t run_dcdc_i2ctest(int argc, char **argv, char *m);
