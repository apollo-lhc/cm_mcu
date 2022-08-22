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
#include "common/LocalUart.h"
#include "common/power_ctl.h"
#include "common/pinsel.h"
#include "common/smbus.h"
#include "common/utils.h"
#include "common/microrl.h"
#include "common/log.h"

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

#include "Tasks.h"
#include "I2CCommunication.h"
#include "common/smbus_helper.h"

extern tSMBus g_sMaster1;
extern tSMBusStatus eStatus1;
extern tSMBus g_sMaster2;
extern tSMBusStatus eStatus2;
extern tSMBus g_sMaster3;
extern tSMBusStatus eStatus3;
extern tSMBus g_sMaster4;
extern tSMBusStatus eStatus4;
extern tSMBus g_sMaster5;
extern tSMBusStatus eStatus5;
extern tSMBus g_sMaster6;
extern tSMBusStatus eStatus6;

tSMBus *const pSMBus[10] = {NULL, &g_sMaster1, &g_sMaster2, &g_sMaster3, &g_sMaster4, &g_sMaster5, &g_sMaster6, NULL, NULL, NULL};
tSMBusStatus *const eStatus[10] = {NULL, &eStatus1, &eStatus2, &eStatus3, &eStatus4, &eStatus5, &eStatus6, NULL, NULL, NULL};

#define MAX_BYTES_ADDR 2
#define MAX_BYTES      4
int apollo_i2c_ctl_r(uint8_t device, uint8_t address, uint8_t nbytes, uint8_t data[MAX_BYTES])
{
  tSMBus *p_sMaster = pSMBus[device];
  tSMBusStatus *p_eStatus = eStatus[device];

  configASSERT(p_sMaster != NULL);

  memset(data, 0, nbytes * sizeof(data[0]));
  if (nbytes > MAX_BYTES)
    nbytes = MAX_BYTES;

  tSMBusStatus r = SMBusMasterI2CRead(p_sMaster, address, data, nbytes);
  if (r == SMBUS_OK) { // the read was successfully initiated
    int tries = 0;
    while (SMBusStatusGet(p_sMaster) == SMBUS_TRANSFER_IN_PROGRESS) {
      vTaskDelay(pdMS_TO_TICKS(10));
      if (tries++ > 100) {
        log_warn(LOG_I2C, "transfer stuck\r\n");
        break;
      }
    }
    r = *p_eStatus;
  }
  else {
    log_error(LOG_I2C, "read fail %s\r\n", SMBUS_get_error(r));
  }
  return r;
}

int apollo_i2c_ctl_reg_r(uint8_t device, uint8_t address, uint8_t nbytes_addr,
                         uint16_t packed_reg_address, uint8_t nbytes,
                         uint32_t *packed_data)
{
  tSMBus *smbus = pSMBus[device];
  tSMBusStatus *p_status = eStatus[device];

  configASSERT(smbus != NULL);
  uint8_t reg_address[MAX_BYTES_ADDR];
  for (int i = 0; i < nbytes_addr; ++i) {
    reg_address[i] = (packed_reg_address >> (nbytes_addr - 1 - i) * 8) & 0xFF; // the first byte is high byte in EEPROM's two-byte reg address
  }
  uint8_t data[MAX_BYTES];

  tSMBusStatus r = SMBusMasterI2CWriteRead(smbus, address, reg_address, nbytes_addr, data, nbytes);
  if (r == SMBUS_OK) { // the WriteRead was successfully initiated
    int tries = 0;
    while (SMBusStatusGet(smbus) == SMBUS_TRANSFER_IN_PROGRESS) {
      vTaskDelay(pdMS_TO_TICKS(10));
      if (tries++ > 100) {
        log_warn(LOG_I2C, "transfer stuck\r\n");
        break;
      }
    }
    r = *p_status;
  }
  else {
    log_error(LOG_I2C, "read fail %s\r\n", SMBUS_get_error(r));
  }
  // pack the data for return to the caller
  *packed_data = 0UL;
  nbytes = (nbytes > MAX_BYTES) ? MAX_BYTES : nbytes;
  for (int i = 0; i < nbytes; ++i) {
    *packed_data |= data[i] << (i * 8);
  }
  return r;
}

int apollo_i2c_ctl_reg_w(uint8_t device, uint8_t address, uint8_t nbytes_addr, uint16_t packed_reg_address, uint8_t nbytes, uint32_t packed_data)
{
  tSMBus *p_sMaster = pSMBus[device];
  tSMBusStatus *p_eStatus = eStatus[device];

  configASSERT(p_sMaster != NULL);

  // first byte (if write to one of five clock chips) or two bytes (if write to EEPROM) is the register, others are the data
  uint8_t data[MAX_BYTES_ADDR + MAX_BYTES];
  for (int i = 0; i < nbytes_addr; ++i) {
    data[i] = (packed_reg_address >> (nbytes_addr - 1 - i) * 8) & 0xFF; // the first byte is high byte in EEPROM's two-byte reg address
  }
  nbytes += nbytes_addr;
  // pack the bytes into the data array, offset by
  // one or two due to the address
  for (int i = nbytes_addr; i < MAX_BYTES + nbytes_addr; ++i) {
    data[i] = (packed_data >> (i - nbytes_addr) * 8) & 0xFF;
  }

  if (nbytes > MAX_BYTES + nbytes_addr)
    nbytes = MAX_BYTES + nbytes_addr;

  tSMBusStatus r = SMBusMasterI2CWrite(p_sMaster, address, data, nbytes);
  if (r == SMBUS_OK) { // the write was successfully initiated
    int tries = 0;
    while (SMBusStatusGet(p_sMaster) == SMBUS_TRANSFER_IN_PROGRESS) {
      vTaskDelay(pdMS_TO_TICKS(10));
      if (tries++ > 100) {
        log_warn(LOG_I2C, "transfer stuck\r\n");
        break;
      }
    }
    r = *p_eStatus;
  }
  else {
    log_error(LOG_I2C, "write fail %s\r\n", SMBUS_get_error(r));
  }

  return r;
}

int apollo_i2c_ctl_w(uint8_t device, uint8_t address, uint8_t nbytes, int value)
{
  tSMBus *p_sMaster = pSMBus[device];
  tSMBusStatus *p_eStatus = eStatus[device];
  configASSERT(p_sMaster != NULL);

  uint8_t data[MAX_BYTES];
  for (int i = 0; i < MAX_BYTES; ++i) {
    data[i] = (value >> i * 8) & 0xFFUL;
  }
  if (nbytes > MAX_BYTES)
    nbytes = MAX_BYTES;

  tSMBusStatus r = SMBusMasterI2CWrite(p_sMaster, address, data, nbytes);
  if (r == SMBUS_OK) { // the write was successfully initiated
    int tries = 0;
    while (SMBusStatusGet(p_sMaster) == SMBUS_TRANSFER_IN_PROGRESS) {
      vTaskDelay(pdMS_TO_TICKS(10));
      if (tries++ > 100) {
        log_warn(LOG_I2C, "transfer stuck\r\n");
        break;
      }
    }
    r = *p_eStatus;
  }
  else {
    log_error(LOG_I2C, "write fail %s\r\n", SMBUS_get_error(r));
  }

  return r;
}
// for PMBUS commands
int apollo_pmbus_rw(tSMBus *smbus, volatile tSMBusStatus *const smbus_status, bool read,
                    struct dev_i2c_addr_t *add, struct pm_command_t *cmd, uint8_t *value)
{
  // select the appropriate output for the mux
  uint8_t data = 0x1U << add->mux_bit;
  tSMBusStatus r = SMBusMasterI2CWrite(smbus, add->mux_addr, &data, 1);
  if (r != SMBUS_OK) {
    log_error(LOG_I2C, "PMBUS write fail %s\r\n", SMBUS_get_error(r));
    return r;
  }
  int tries = 0;
  while (SMBusStatusGet(smbus) == SMBUS_TRANSFER_IN_PROGRESS) {
    vTaskDelay(pdMS_TO_TICKS(10));
    if (tries++ > 100) {
      log_warn(LOG_I2C, "transfer stuck\r\n");
      break;
    }
  }
  if (*smbus_status != SMBUS_OK) {
    return *smbus_status;
  }
  // read/write to the device itself
  if (read) {
    r = SMBusMasterByteWordRead(smbus, add->dev_addr, cmd->command, value, cmd->size);
  }
  else { // write
    r = SMBusMasterByteWordWrite(smbus, add->dev_addr, cmd->command, value, cmd->size);
  }
  if (r != SMBUS_OK) {
    log_error(LOG_I2C, "PMBUS write/read fail %s\r\n", SMBUS_get_error(r));
    return r;
  }
  tries = 0;
  while (SMBusStatusGet(smbus) == SMBUS_TRANSFER_IN_PROGRESS) {
    vTaskDelay(pdMS_TO_TICKS(10));
    if (tries++ > 100) {
      log_warn(LOG_I2C, "transfer stuck\r\n");
      break;
    }
  }

  return r;
}
