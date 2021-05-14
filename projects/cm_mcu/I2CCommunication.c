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

//static tSMBus *p_sMaster = &g_sMaster4;
//static tSMBusStatus *p_eStatus = &eStatus4;

// Ugly hack for now -- I don't understand how to reconcile these
// two parts of the FreeRTOS-Plus code w/o casts-o-plenty
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wformat" // because of our mini-sprintf

tSMBus* pSMBus[10] = {NULL, &g_sMaster1, &g_sMaster2, &g_sMaster3, &g_sMaster4, NULL, &g_sMaster6, NULL, NULL, NULL};
tSMBusStatus* eStatus[10] = {NULL, &eStatus1, &eStatus2, &eStatus3, &eStatus4, NULL, &eStatus6, NULL, NULL, NULL};

#define MAX_BYTES 4
int apollo_i2c_ctl_r(uint8_t device, uint8_t address, uint8_t nbytes, uint8_t data[MAX_BYTES])
{
  if (!((device == 1) || (device == 2) || (device == 3) || (device == 4) || (device == 6))) {
    return -1;
  }
  tSMBus* p_sMaster = pSMBus[device];
  tSMBusStatus* p_eStatus = eStatus[device];

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

int apollo_i2c_ctl_reg_r(uint8_t device, uint8_t address, uint8_t reg_address, uint8_t nbytes, uint8_t data[MAX_BYTES])
{
  if (!((device == 1) || (device == 2) || (device == 3) || (device == 4) || (device == 6))) {
    return -1;
  }
  tSMBus* smbus = pSMBus[device];
  tSMBusStatus* p_status = eStatus[device];

  memset(data, 0, MAX_BYTES * sizeof(data[0]));
  if (nbytes > MAX_BYTES)
    nbytes = MAX_BYTES;
  tSMBusStatus r = SMBusMasterI2CWriteRead(smbus, address, &reg_address, 1, data, nbytes);
  if (r != SMBUS_OK) {
    return -1;
  }
  while (SMBusStatusGet(smbus) == SMBUS_TRANSFER_IN_PROGRESS) {
    vTaskDelay(pdMS_TO_TICKS(10));
  }
  return *p_status;
}

int apollo_i2c_ctl_reg_w(uint8_t device, uint8_t address, uint8_t reg_address, uint8_t nbytes, int packed_data)
{
  if (!((device == 1) || (device == 2) || (device == 3) || (device == 4) || (device == 6))) {
    return -1;
  }
  tSMBus* p_sMaster = pSMBus[device];
  tSMBusStatus* p_eStatus = eStatus[device];
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

int apollo_i2c_ctl_w(uint8_t device, uint8_t address, uint8_t nbytes, int value)
{
  if (!((device == 1) || (device == 2) || (device == 3) || (device == 4) || (device == 6))) {
    return -1;
  }
  tSMBus* p_sMaster = pSMBus[device];
  tSMBusStatus* p_eStatus = eStatus[device];
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

int apollo_pmbus_rw(tSMBus *smbus, volatile tSMBusStatus *smbus_status, bool read,
                    struct dev_i2c_addr_t *add, struct pm_command_t *cmd, uint8_t *value)
{
  // write to the I2C mux
  uint8_t data;
  // select the appropriate output for the mux
  data = 0x1U << add->mux_bit;
  tSMBusStatus r = SMBusMasterI2CWrite(smbus, add->mux_addr, &data, 1);
  if (r != SMBUS_OK) {
    return -1;
  }
  while (SMBusStatusGet(smbus) == SMBUS_TRANSFER_IN_PROGRESS) {
    vTaskDelay(pdMS_TO_TICKS(10)); // wait
  }
  if (*smbus_status != SMBUS_OK) {
    return -2;
  }
  // read/write to the device itself
  if (read) {
    r = SMBusMasterByteWordRead(smbus, add->dev_addr, cmd->command, value, cmd->size);
  }
  else { // write
    r = SMBusMasterByteWordWrite(smbus, add->dev_addr, cmd->command, value, cmd->size);
  }
  if (r != SMBUS_OK) {
    return -3;
  }
  while (SMBusStatusGet(smbus) == SMBUS_TRANSFER_IN_PROGRESS) {
    vTaskDelay(pdMS_TO_TICKS(10)); // wait
  }
  // this is checking the return from the interrupt
  if (*smbus_status != SMBUS_OK) {
    return -4;
  }
  // if we get here, a successful read/write command

  return 0;
}
