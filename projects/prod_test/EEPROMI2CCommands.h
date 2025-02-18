#pragma once

#include "FreeRTOS.h" // IWYU pragma: keep

#define EEPROM_I2C_BASE   2
#define EEPROM_I2C_ADDR   0x50
#define EEPROM_TEST_ADDR  0x0
#define N_TEST_DATA       2
#define EEPROM_TEST_DATA  0xEE
#define EEPROM_TEST_DATA2 0xEF
#define EEPROM_TEST_DATA3 0xDE

/**
 * @brief CLI function that tests I2C communication to EEPROM
 *
 * @param [in] argc  number of CLI arguments
 * @param [in] argv  CLI arguments
 * @param [out] m  output string
 * @return pdFALSE
 */
BaseType_t eeprom_i2ctest_ctl(int argc, char **argv, char *m);
