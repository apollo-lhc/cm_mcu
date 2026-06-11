/*
 * I2CCommunication.c
 *
 *  Created on: Sep 3, 2020
 *      Author: rzou
 */

#include <stdint.h>
#include <stdbool.h>

// local includes
#include "common/smbus.h"
#include "common/log.h"

// FreeRTOS includes
#include "FreeRTOSConfig.h"
#include "FreeRTOS.h" // IWYU pragma: keep
#include "stream_buffer.h"
#include "queue.h"

// strlen, strtol, and strncpy
#include <string.h>
#include <stdlib.h>

#include "Tasks.h"
#include "I2CCommunication.h"
#include "common/smbus_helper.h"
#include "common/utils.h" // DELAY_US

#include "InterruptHandlers.h"

// TI hardware register access for I2C master state diagnostics
#include "inc/hw_types.h"      // HWREG
#include "inc/hw_i2c.h"        // I2C_O_MCS, I2C_MCS_BUSY, I2C_MCS_BUSBSY
#include "driverlib/i2c.h"     // I2CMasterBusy
#include "driverlib/rom.h"     // ROM_SysCtlDelay (used by DELAY_US)
#include "driverlib/rom_map.h" // MAP_I2CMasterBusy

extern tSMBus g_sMaster1;
extern tSMBus g_sMaster2;
extern tSMBus g_sMaster3;
extern tSMBus g_sMaster4;
extern tSMBus g_sMaster5;
extern tSMBus g_sMaster6;

tSMBus *const pSMBus[10] = {NULL, &g_sMaster1, &g_sMaster2, &g_sMaster3, &g_sMaster4, &g_sMaster5, &g_sMaster6, NULL, NULL, NULL};
volatile tSMBusStatus *const eStatus[10] = {NULL, &eStatus1, &eStatus2, &eStatus3, &eStatus4, &eStatus5, &eStatus6, NULL, NULL, NULL};

#define MAX_BYTES_ADDR 2
#define MAX_BYTES      4

// Per-bus scratch buffers for I2C transfers, indexed by device the same as pSMBus[].
// The TI SMBus driver keeps a *pointer* (pui8Rx/TxBuffer) to the caller's buffer and its
// ISR reads/writes it as bytes arrive. Passing these persistent .bss buffers instead of a
// stack local means a late ISR completing after the wrapper has returned writes here, not
// into a freed/reused stack frame (the memory-corruption root cause; see i2c_lockup_notes.md).
//
// OWNERSHIP CONTRACT: these buffers are shared by every caller of a given bus and are NOT
// internally locked. They are safe ONLY because each bus is serialized by its own mutex
// (i2c1_sem..i2c6_sem) and the caller is required to hold that mutex across the whole
// transaction sequence. If two callers touch the same bus without that mutex (e.g. the
// deliberately semaphore-free CLI exploration helpers -- readFFpresentSignals(false),
// ff_present -- running concurrently with a MonitorTaskI2C on the same bus), they race on
// these buffers AND on TaskNotifySMBus[bus]. Because the storage is static .bss the worst
// case is a wrong value or a 250 ms SMBUS_TIMEOUT, never memory corruption -- but the result
// is unreliable. See README.md "Higher-level semaphore-free helpers and unprogrammed
// exploration" for the safe workflow (manual sem_ctl lock) and hardening options.
static uint8_t i2c_rxbuf[sizeof(pSMBus) / sizeof(pSMBus[0])][MAX_BYTES];
static uint8_t i2c_txbuf[sizeof(pSMBus) / sizeof(pSMBus[0])][MAX_BYTES_ADDR + MAX_BYTES];

#define I2C_TIMEOUT_MS 250

// Bounded wait (microseconds) for the on-chip I2C master FSM to finish a prior
// transaction's STOP before we initiate a new one. This targets the suspected
// PERIPHERAL_BUSY race where a task is notified and resumes before the hardware
// auto-STOP completes. See i2c_lockup_notes.md.
#define I2C_IDLE_WAIT_US 250

static void i2c_arm_notify_slot(uint8_t device)
{
  // Bounded spin until the master FSM is idle, so a late-completing previous
  // STOP cannot make this transaction's initiation return PERIPHERAL_BUSY.
  uint32_t base = pSMBus[device]->ui32I2CBase;
  uint32_t us = 0;
  while (MAP_I2CMasterBusy(base) && us < I2C_IDLE_WAIT_US) {
    DELAY_US(1);
    ++us;
  }
  if (us) {
    log_debug(LOG_I2C, "dev %d waited %u us for master idle\r\n", device, us);
  }

  TaskNotifySMBus[device] = xTaskGetCurrentTaskHandle();
}

// Diagnostic: log the raw I2C master control/status register at the moment an
// initiation returns PERIPHERAL_BUSY (or other non-OK). Must be called BEFORE
// any recovery (e.g. mux reset) so it captures the true on-chip state.
//   BUSY set, BUSBSY clear  -> on-chip master still finishing its STOP (FSM race)
//   BUSBSY set              -> bus line held externally (device/mux)
static void i2c_log_busy(uint8_t device, const char *op, tSMBusStatus r)
{
  uint32_t mcs = HWREG(pSMBus[device]->ui32I2CBase + I2C_O_MCS);
  log_warn(LOG_I2C, "dev %d %s %s MCS=0x%02lx BUSY=%d BUSBSY=%d\r\n", device, op,
           SMBUS_get_error(r), (unsigned long)mcs, !!(mcs & I2C_MCS_BUSY),
           !!(mcs & I2C_MCS_BUSBSY));
}

static void i2c_wait_for_transfer(uint8_t device)
{
  // Caller must have already set TaskNotifySMBus[device]
  if (ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(I2C_TIMEOUT_MS)) == 0) {
    // timeout — no notification arrived
    *eStatus[device] = SMBUS_TIMEOUT;
    TaskNotifySMBus[device] = NULL;
    log_warn(LOG_I2C, "transfer stuck, dev %d\r\n", device);
  }
}

int apollo_i2c_ctl_r(uint8_t device, uint8_t address, uint8_t nbytes, uint8_t data[MAX_BYTES])
{
  tSMBus *p_sMaster = pSMBus[device];
  volatile tSMBusStatus *p_eStatus = eStatus[device];

  configASSERT(p_sMaster != NULL);

  if (nbytes > MAX_BYTES)
    nbytes = MAX_BYTES;

  // Read into the persistent per-bus buffer rather than the caller's (possibly stack) buffer,
  // so a late ISR completing after this function returns can't write into a freed/reused frame.
  uint8_t *rxbuf = i2c_rxbuf[device];
  memset(rxbuf, 0, nbytes * sizeof(rxbuf[0]));

  i2c_arm_notify_slot(device);

  tSMBusStatus r = SMBusMasterI2CRead(p_sMaster, address, rxbuf, nbytes);
  if (r == SMBUS_OK) { // the read was successfully initiated
    i2c_wait_for_transfer(device);
    r = *p_eStatus;
  }
  else {
    TaskNotifySMBus[device] = NULL; // clean up if initiation failed
    i2c_log_busy(device, "read", r);
    memcpy(data, rxbuf, nbytes); // hand back zeros (rxbuf was memset)
    return r;
  }
  memcpy(data, rxbuf, nbytes); // copy results to the caller before returning
  if (r != SMBUS_OK) {
    log_error(LOG_I2C, "dev %d read fail %s\r\n", device, SMBUS_get_error(r));
  }
  else {
    log_debug(LOG_I2C, "dev %d read success 0x%0*X\r\n", device, nbytes * 2, data[0] | (data[1] << 8) | (data[2] << 16) | (data[3] << 24));
  }
  return r;
}

int apollo_i2c_ctl_reg_r(uint8_t device, uint8_t address, uint8_t nbytes_addr,
                         uint16_t packed_reg_address, uint8_t nbytes,
                         uint32_t *packed_data)
{
  tSMBus *smbus = pSMBus[device];
  volatile tSMBusStatus *p_status = eStatus[device];

  configASSERT(smbus != NULL);
  uint8_t *reg_address = i2c_txbuf[device]; // persistent per-bus TX buffer (see note at top)
  for (int i = 0; i < nbytes_addr; ++i) {
    reg_address[i] = (packed_reg_address >> (nbytes_addr - 1 - i) * 8) & 0xFF; // the first byte is high byte in EEPROM's two-byte reg address
  }
  uint8_t *data = i2c_rxbuf[device]; // persistent per-bus RX buffer

  if (nbytes > MAX_BYTES)
    nbytes = MAX_BYTES;

  i2c_arm_notify_slot(device);

  tSMBusStatus r = SMBusMasterI2CWriteRead(smbus, address, reg_address, nbytes_addr, data, nbytes);
  if (r == SMBUS_OK) { // the WriteRead was successfully initiated
    i2c_wait_for_transfer(device);
    r = *p_status;
  }
  else {
    TaskNotifySMBus[device] = NULL; // clean up if initiation failed
    i2c_log_busy(device, "reg read", r);
    return r;
  }
  // pack the data for return to the caller
  *packed_data = 0UL;
  nbytes = (nbytes > MAX_BYTES) ? MAX_BYTES : nbytes;
  for (int i = 0; i < nbytes; ++i) {
    *packed_data |= data[i] << (i * 8);
  }
  if (r != SMBUS_OK) {
    log_error(LOG_I2C, "dev %d reg read fail %s\r\n", device, SMBUS_get_error(r));
  }
  else {
    log_debug(LOG_I2C, "dev %d reg read success 0x%0*X\r\n", device, nbytes * 2, *packed_data);
  }
  return r;
}

int apollo_i2c_ctl_reg_w(uint8_t device, uint8_t address, uint8_t nbytes_addr, uint16_t packed_reg_address, uint8_t nbytes, uint32_t packed_data)
{
  tSMBus *p_sMaster = pSMBus[device];
  volatile tSMBusStatus *p_eStatus = eStatus[device];

  configASSERT(p_sMaster != NULL);

  // first byte (if write to one of five clock chips) or two bytes (if write to EEPROM) is the register, others are the data
  uint8_t *data = i2c_txbuf[device]; // persistent per-bus TX buffer (see note at top)
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

  i2c_arm_notify_slot(device);

  tSMBusStatus r = SMBusMasterI2CWrite(p_sMaster, address, data, nbytes);
  if (r == SMBUS_OK) { // the write was successfully initiated
    i2c_wait_for_transfer(device);
    r = *p_eStatus;
  }
  else {
    TaskNotifySMBus[device] = NULL; // clean up if initiation failed
    i2c_log_busy(device, "reg write", r);
    return r;
  }

  if (r != SMBUS_OK) {
    log_error(LOG_I2C, "dev %d reg write fail %s\r\n", device, SMBUS_get_error(r));
  }
  else {
    log_debug(LOG_I2C, "dev %d reg write success 0x%0*X\r\n", device, nbytes * 2, packed_data);
  }
  return r;
}

int apollo_i2c_ctl_w(uint8_t device, uint8_t address, uint8_t nbytes, unsigned int value)
{
  tSMBus *p_sMaster = pSMBus[device];
  volatile tSMBusStatus *p_eStatus = eStatus[device];
  configASSERT(p_sMaster != NULL);

  uint8_t *data = i2c_txbuf[device]; // persistent per-bus TX buffer (see note at top)
  for (int i = 0; i < MAX_BYTES; ++i) {
    data[i] = (value >> i * 8) & 0xFFUL;
  }
  if (nbytes > MAX_BYTES)
    nbytes = MAX_BYTES;

  i2c_arm_notify_slot(device);

  tSMBusStatus r = SMBusMasterI2CWrite(p_sMaster, address, data, nbytes);
  if (r == SMBUS_OK) { // the write was successfully initiated
    i2c_wait_for_transfer(device);
    r = *p_eStatus;
  }
  else {
    TaskNotifySMBus[device] = NULL; // clean up if initiation failed
    i2c_log_busy(device, "write", r);
    return r;
  }

  if (r != SMBUS_OK) {
    log_error(LOG_I2C, "dev %d write fail %s\r\n", device, SMBUS_get_error(r));
  }
  else {
    log_debug(LOG_I2C, "dev %d write success 0x%0*X\r\n", device, nbytes * 2, value);
  }
  return r;
}
// for PMBUS commands
static uint8_t smbus_get_device_index(tSMBus *smbus)
{
  for (int i = 1; i <= 6; i++) {
    if (pSMBus[i] == smbus) {
      return i;
    }
  }
  return 0; // error/not found
}

tSMBusStatus apollo_pmbus_rw(tSMBus *smbus, volatile tSMBusStatus *const smbus_status, bool read,
                             struct dev_i2c_addr_t *add, struct pm_command_t *cmd, uint8_t *value)
{
  uint8_t device = smbus_get_device_index(smbus);
  if (device == 0) {
    log_error(LOG_I2C, "PMBUS invalid device\r\n");
    return SMBUS_PERIPHERAL_BUSY;
  }

  // TRANSACTION 1: mux selection via existing helper
  uint8_t data = 0x1U << add->mux_bit;
  tSMBusStatus r = apollo_i2c_ctl_w(device, add->mux_addr, 1, data);
  if (r != SMBUS_OK) {
    log_error(LOG_I2C, "PMBUS mux write fail %s\r\n", SMBUS_get_error(r));
    return r;
  }

  // TRANSACTION 2: device read/write. PMBus/SMBus command.
  i2c_arm_notify_slot(device);
  if (read) {
    r = SMBusMasterByteWordRead(smbus, add->dev_addr, cmd->command, value, cmd->size);
  }
  else {
    r = SMBusMasterByteWordWrite(smbus, add->dev_addr, cmd->command, value, cmd->size);
  }
  if (r != SMBUS_OK) {
    i2c_log_busy(device, read ? "pmbus read" : "pmbus write", r);
    TaskNotifySMBus[device] = NULL;
    return r;
  }
  i2c_wait_for_transfer(device);
  // ISR writes the per-bus status (SMBUS_OK or specific error).
  r = *smbus_status;

  return r;
}
