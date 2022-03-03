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

tSMBus* pSMBus[10] = {NULL, &g_sMaster1, &g_sMaster2, &g_sMaster3, &g_sMaster4, &g_sMaster5, &g_sMaster6, NULL, NULL, NULL};
tSMBusStatus* eStatus[10] = {NULL, &eStatus1, &eStatus2, &eStatus3, &eStatus4, &eStatus5, &eStatus6, NULL, NULL, NULL};

// array of function pointers to access to the semaphores to control access to the
// I2C controller
// PMBUS commands are currently not covered
SemaphoreHandle_t (*getSemaphore[7])(void) = {
    NULL,
    NULL,
    NULL,
    NULL,
    getFFMutex,
    NULL,
    NULL,
};

#define MAX_BYTES 4
#define MAX_BYTES_ADDR 2
int apollo_i2c_ctl_r(uint8_t device, uint8_t address, uint8_t nbytes, uint8_t data[MAX_BYTES])
{
  tSMBus* p_sMaster = pSMBus[device];
  tSMBusStatus* p_eStatus = eStatus[device];

  configASSERT(p_sMaster != NULL);

  memset(data, 0, nbytes * sizeof(data[0]));
  if (nbytes > MAX_BYTES)
    nbytes = MAX_BYTES;

  // get the semaphore
  SemaphoreHandle_t s = NULL;
  if ( getSemaphore[device] != NULL ) {
    s = (*getSemaphore[device])();
    xSemaphoreTake(s, portMAX_DELAY);
  }
  int retval = 0;
  tSMBusStatus r = SMBusMasterI2CRead(p_sMaster, address, data, nbytes);
  if (r != SMBUS_OK) {
    retval = -1;
  }
  if ( ! retval ) {
    while (SMBusStatusGet(p_sMaster) == SMBUS_TRANSFER_IN_PROGRESS) {
      vTaskDelay(pdMS_TO_TICKS(10));
    }
    if (*p_eStatus != SMBUS_OK) {
      retval = -2;
    }
  }
  if ( s )
    xSemaphoreGive(s);
  return retval;
}

int apollo_i2c_ctl_reg_r(uint8_t device, uint8_t address, uint8_t nbytes_addr, uint16_t packed_reg_address, uint8_t nbytes, uint32_t packed_data)
{
  tSMBus* smbus = pSMBus[device];
  tSMBusStatus* p_status = eStatus[device];

  configASSERT(smbus != NULL);
  uint8_t reg_address[MAX_BYTES_ADDR];
    for (int i = 0; i < MAX_BYTES_ADDR; ++i) {
        reg_address[i] = (packed_reg_address >> (i) * 8) & 0xFFUL;
      }
  uint8_t data[MAX_BYTES];
  for (int i = 0; i < MAX_BYTES; ++i) {
      data[i] = (packed_data >> (i) * 8) & 0xFFUL;
    }
  // get the semaphore
  SemaphoreHandle_t s = NULL;
  if ( getSemaphore[device] != NULL ) {
    s = (*getSemaphore[device])();
    xSemaphoreTake(s, portMAX_DELAY);
  }
  int retval = 0;

  tSMBusStatus r = SMBusMasterI2CWriteRead(smbus, address, reg_address, nbytes_addr, data, nbytes);
  if (r != SMBUS_OK) {
    retval = -1;
  }
  if ( ! retval ) {
    while (SMBusStatusGet(smbus) == SMBUS_TRANSFER_IN_PROGRESS) {
      vTaskDelay(pdMS_TO_TICKS(10));
    }
  }
  if (s)
    xSemaphoreGive(s);
  if ( retval )
    return retval;
  else
    return *p_status;
}

int apollo_i2c_ctl_reg_w(uint8_t device, uint8_t address, uint8_t nbytes_addr, uint16_t packed_reg_address, uint8_t nbytes, uint32_t packed_data)
{
  tSMBus* p_sMaster = pSMBus[device];
  tSMBusStatus* p_eStatus = eStatus[device];

  configASSERT(p_sMaster != NULL);

  // first byte (if write to one of five clcok chips) or two bytes (if write to EEPROM) is the register, others are the data
  uint8_t data[MAX_BYTES_ADDR + MAX_BYTES];
  for (int i = 0; i < MAX_BYTES_ADDR; ++i){
	data[i] = (packed_reg_address >> (i) * 8) & 0xFFUL;
    if (data[i] != 0) ++nbytes; // to account for the register address
  }
  // pack the bytes into the data array, offset by
  // one or two due to the address
  for (int i = MAX_BYTES_ADDR; i < MAX_BYTES + MAX_BYTES_ADDR; ++i) {
    data[i] = (packed_data >> (i) * 8) & 0xFFUL;
  }
  
  if (nbytes > MAX_BYTES+MAX_BYTES_ADDR)
    nbytes = MAX_BYTES+MAX_BYTES_ADDR;
  // get the semaphore
  SemaphoreHandle_t s = NULL;
  if ( getSemaphore[device] != NULL ) {
    s = (*getSemaphore[device])();
    xSemaphoreTake(s, portMAX_DELAY);
  }
  int retval = 0;

  tSMBusStatus r = SMBusMasterI2CWrite(p_sMaster, address, data, nbytes);
  if (r != SMBUS_OK) {
    retval = -1;
  }
  if ( ! retval ) {
    while (SMBusStatusGet(p_sMaster) == SMBUS_TRANSFER_IN_PROGRESS) {
      vTaskDelay(pdMS_TO_TICKS(10));
    }
    if (*p_eStatus != SMBUS_OK) {
      retval= -2;
    }
  }
  if (s)
    xSemaphoreGive(s);

  return retval;
}

int apollo_i2c_ctl_w(uint8_t device, uint8_t address, uint8_t nbytes, int value)
{
  tSMBus* p_sMaster = pSMBus[device];
  tSMBusStatus* p_eStatus = eStatus[device];
  configASSERT(p_sMaster != NULL);
  
  uint8_t data[MAX_BYTES];
  for (int i = 0; i < MAX_BYTES; ++i) {
    data[i] = (value >> i * 8) & 0xFFUL;
  }
  if (nbytes > MAX_BYTES)
    nbytes = MAX_BYTES;

  // get the semaphore
  SemaphoreHandle_t s = NULL;
  if ( getSemaphore[device] != NULL ) {
    s = (*getSemaphore[device])();
    xSemaphoreTake(s, portMAX_DELAY);
  }
  int retval = 0;

  tSMBusStatus r = SMBusMasterI2CWrite(p_sMaster, address, data, nbytes);
  if (r != SMBUS_OK) {
    retval = -1;
  }
  if (! retval ) {
    while (SMBusStatusGet(p_sMaster) == SMBUS_TRANSFER_IN_PROGRESS) {
      vTaskDelay(pdMS_TO_TICKS(10));
    }
    if (*p_eStatus != SMBUS_OK) {
      retval = -2;
    }
  }
  if (s)
    xSemaphoreGive(s);
  return retval;
}
// for PMBUS commands 
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
