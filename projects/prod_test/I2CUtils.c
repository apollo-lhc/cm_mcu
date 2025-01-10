/*
 * I2CUtils.c
 *
 *  Created on: January 10, 2025
 *      Author: mcoshiro
 *
 * Contains common I2C code for production tests to avoid duplication
 */

// includes for types
#include <stdbool.h>

// FreeRTOS
#include "FreeRTOS.h"
#include "task.h"

// local includes
#include "common/smbus.h"
#include "common/printf.h"
#include "commands.h"
#include "I2CUtils.h"

/**
 * @details
 * To verify transaction, make a call to this function after calling an SMBUS
 * method such as SMBusMasterI2CWrite. It verifies no errors were generated
 * and that the transaction finishes in specified time
 */
int check_i2c_transaction(tSMBusStatus r, int timeout, bool no_message,
                          tSMBus *g_sMaster, tSMBusStatus eStatus, char *m)
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
  while (SMBusStatusGet(g_sMaster) == SMBUS_TRANSFER_IN_PROGRESS) {
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
  if (eStatus != SMBUS_OK) {
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
