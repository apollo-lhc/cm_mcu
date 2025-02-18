/*
 * FireflyI2CCommands.c
 *
 *  Created on: February 10, 2025
 *      Author: mcoshiro
 *
 * Contains code for MCU to EEPROM I2C tests for Apollo CM production tests
 */

// includes for types
#include <stdbool.h>
#include <stdint.h>

// FreeRTOS
#include "FreeRTOS.h"
#include "task.h"

// local includes
#include "common/smbus.h"
#include "common/pinsel.h"
#include "common/printf.h"
#include "common/utils.h"
#include "commands.h"
#include "InterruptHandlers.h"
#include "EEPROMI2CCommands.h"
#include "I2CCommunication.h"

// Should we program the board ID into the configuration register here or wait?

/**
 * @details
 * CLI function that tests I2C communication to EEPROM. First, it writes
 * a word to a location in memory and verifies it can be read back. Then,
 * the write protect is enabled and it is confirmed that the same process
 * does not succeed
 */
BaseType_t eeprom_i2ctest_ctl(int argc, char **argv, char *m)
{

  // disable write protect
  write_gpio_pin(ID_EEPROM_WP, 0x0);
  vTaskDelay(pdMS_TO_TICKS(50));

  uint32_t read_data = 0x0;
  uint8_t test_data[N_TEST_DATA] = {EEPROM_TEST_DATA, EEPROM_TEST_DATA2};

  for (uint8_t idata = 0; idata < N_TEST_DATA; idata++) {

    if (apollo_i2c_ctl_reg_w(EEPROM_I2C_BASE, EEPROM_I2C_ADDR, 2,
                             EEPROM_TEST_ADDR, 1, test_data[idata])) {
      snprintf(m, SCRATCH_SIZE, "ERROR: Failed to write to EEPROM.\r\n");
      return pdFALSE;
    }

    if (apollo_i2c_ctl_reg_r(EEPROM_I2C_BASE, EEPROM_I2C_ADDR, 2,
                             EEPROM_TEST_ADDR, 1, &read_data)) {
      snprintf(m, SCRATCH_SIZE, "ERROR: Failed to read from EEPROM.\r\n");
      return pdFALSE;
    }

    if (read_data != test_data[idata]) {
      snprintf(m, SCRATCH_SIZE,
               "ERROR: Incorrect data from EEPROM (expected %d, got %d)\r\n",
               test_data[idata], read_data);
      return pdFALSE;
    }
  }

  // enable write protect
  write_gpio_pin(ID_EEPROM_WP, 0x1);
  vTaskDelay(pdMS_TO_TICKS(50));

  if (apollo_i2c_ctl_reg_w(EEPROM_I2C_BASE, EEPROM_I2C_ADDR, 2,
                           EEPROM_TEST_ADDR, 1, EEPROM_TEST_DATA3)) {
    snprintf(m, SCRATCH_SIZE, "ERROR: Failed to write to EEPROM.\r\n");
    return pdFALSE;
  }

  if (apollo_i2c_ctl_reg_r(EEPROM_I2C_BASE, EEPROM_I2C_ADDR, 2,
                           EEPROM_TEST_ADDR, 1, &read_data)) {
    snprintf(m, SCRATCH_SIZE, "ERROR: Failed to read from EEPROM.\r\n");
    return pdFALSE;
  }

  if (read_data != EEPROM_TEST_DATA2) {
    snprintf(m, SCRATCH_SIZE,
             "ERROR: Write protect failed (expected %d, got %d)\r\n",
             EEPROM_TEST_DATA2, read_data);
    return pdFALSE;
  }

  // disable write protect
  write_gpio_pin(ID_EEPROM_WP, 0x0);
  vTaskDelay(pdMS_TO_TICKS(50));

  snprintf(m, SCRATCH_SIZE, "Test success.\r\n");
  return pdFALSE;
}
