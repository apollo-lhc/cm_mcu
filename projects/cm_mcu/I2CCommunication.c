/*
 * I2CCommunication.c
 *
 *  Created on: Sep 3, 2020
 *      Author: rzou
 */

#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_types.h"
#include "inc/hw_memmap.h"
#include "inc/hw_nvic.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "driverlib/systick.h"
#include "driverlib/sysctl.h"
#include "driverlib/eeprom.h"

// local includes
#include "common/i2c_reg.h"
#include "common/uart.h"
#include "common/power_ctl.h"
#include "common/pinsel.h"
#include "common/smbus.h"
#include "common/utils.h"
#include "common/microrl.h"

// FreeRTOS includes
#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "stream_buffer.h"
#include "queue.h"

// strlen, strtol, and strncpy
#include <string.h>
#include <stdlib.h>

// TivaWare includes
#include "driverlib/uart.h"

//#include "MonitorTask.h"
#include "Tasks.h"
#include "I2CCommunication.h"

#ifdef DEBUG_CON
// prototype of mutex'd print
#define DPRINT(x) Print(x)
#else // DEBUG_CON
#define DPRINT(x)
#endif // DEBUG_CON

void Print(const char *str);

extern tSMBus g_sMaster1;
extern tSMBusStatus eStatus1;
extern tSMBus g_sMaster2;
extern tSMBusStatus eStatus2;
extern tSMBus g_sMaster3;
extern tSMBusStatus eStatus3;
extern tSMBus g_sMaster4;
extern tSMBusStatus eStatus4;
extern tSMBus g_sMaster6;
extern tSMBusStatus eStatus6;

static tSMBus *p_sMaster = &g_sMaster4;
static tSMBusStatus *p_eStatus = &eStatus4;

// Ugly hack for now -- I don't understand how to reconcile these
// two parts of the FreeRTOS-Plus code w/o casts-o-plenty
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wformat" // because of our mini-sprintf

int apollo_i2c_ctl_set_dev(uint8_t base)
{
  if (!((base == 1) || (base == 2) || (base == 3) || (base == 4) || (base == 6))) {
    return -1;
  }
  switch (base) {
    case 1:
      p_sMaster = &g_sMaster1;
      p_eStatus = &eStatus1;
      break;
    case 2:
      p_sMaster = &g_sMaster2;
      p_eStatus = &eStatus2;
      break;
    case 3:
      p_sMaster = &g_sMaster3;
      p_eStatus = &eStatus3;
      break;
    case 4:
      p_sMaster = &g_sMaster4;
      p_eStatus = &eStatus4;
      break;
    case 6:
      p_sMaster = &g_sMaster6;
      p_eStatus = &eStatus6;
      break;
    default:
      return -2;
      break;
  }
  return 0;
}

int apollo_i2c_ctl_r(uint8_t address, uint8_t nbytes, uint8_t data[4])
{
  const int MAX_BYTES = 4;
  memset(data, 0, MAX_BYTES * sizeof(data[0]));
  if (nbytes > MAX_BYTES)
    nbytes = MAX_BYTES;

  tSMBusStatus r = SMBusMasterI2CRead(p_sMaster, address, data, nbytes);
  if (r != SMBUS_OK) {
    return -1;
  }
  while (SMBusStatusGet(p_sMaster) == SMBUS_TRANSFER_IN_PROGRESS) {
    vTaskDelay(pdMS_TO_TICKS(10));
  }
  if (*p_eStatus != SMBUS_OK) {
    return -2;
  }

  return 0;
}

int apollo_i2c_ctl_reg_r(uint8_t address, uint8_t reg_address, uint8_t nbytes, uint8_t data[4])
{
  const int MAX_BYTES = 4;
  memset(data, 0, MAX_BYTES * sizeof(data[0]));
  if (nbytes > MAX_BYTES)
    nbytes = MAX_BYTES;
  tSMBusStatus r = SMBusMasterI2CWriteRead(p_sMaster, address, &reg_address, 1, data, nbytes);
  if (r != SMBUS_OK) {
    return -1;
  }
  while (SMBusStatusGet(p_sMaster) == SMBUS_TRANSFER_IN_PROGRESS) {
    vTaskDelay(pdMS_TO_TICKS(10));
  }
  if (*p_eStatus != SMBUS_OK) {
    return -2;
  }

  return 0;
}

#define MAX_BYTES 4
int apollo_i2c_ctl_reg_w(uint8_t address, uint8_t reg_address, uint8_t nbytes, uint8_t packed_data)
{
  // first byte is the register, others are the data
  uint8_t data[MAX_BYTES+1];
  data[0] = reg_address;
  // pack the bytes into the data array, offset by
  // one due to the address
  for (int i = 1; i < MAX_BYTES + 1; ++i) {
    data[i] = (packed_data >> (i - 1) * 8) & 0xFFUL;
  }
  nbytes++; // to account for the register address
  if (nbytes > MAX_BYTES)
    nbytes = MAX_BYTES;

  tSMBusStatus r = SMBusMasterI2CWrite(p_sMaster, address, data, nbytes);
  if (r != SMBUS_OK) {
    return -1;
  }
  while (SMBusStatusGet(p_sMaster) == SMBUS_TRANSFER_IN_PROGRESS) {
    vTaskDelay(pdMS_TO_TICKS(10));
  }
  if (*p_eStatus != SMBUS_OK) {
    return -2;
  }

  return 0;
}

int apollo_i2c_ctl_w(uint8_t address, uint8_t nbytes, uint8_t value)
{
  uint8_t data[MAX_BYTES];
  for (int i = 0; i < MAX_BYTES; ++i) {
    data[i] = (value >> i * 8) & 0xFFUL;
  }
  if (nbytes > MAX_BYTES)
    nbytes = MAX_BYTES;

  tSMBusStatus r = SMBusMasterI2CWrite(p_sMaster, address, data, nbytes);
  if (r != SMBUS_OK) {
    return -1;
  }
  while (SMBusStatusGet(p_sMaster) == SMBUS_TRANSFER_IN_PROGRESS) {
    vTaskDelay(pdMS_TO_TICKS(10));
  }
  if (*p_eStatus != SMBUS_OK) {
    return -2;
  }

  return 0;
}
