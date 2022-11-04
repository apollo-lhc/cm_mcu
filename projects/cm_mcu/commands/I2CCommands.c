/*
 * I2CCommands.c
 *
 *  Created on: Jan 14, 2021
 *      Author: fatimayousuf
 */

#include "I2CCommands.h"
#include "commands/parameters.h"
#include "common/smbus_helper.h"
#include "Semaphore.h"
#include "projdefs.h"

SemaphoreHandle_t getSemaphore(int number)
{
  SemaphoreHandle_t s;
  switch (number) {
    case 1:
      s = i2c1_sem;
      break;
    case 2:
      s = i2c2_sem;
      break;
    case 3:
      s = i2c3_sem;
      break;
    case 4:
      s = i2c4_sem;
      break;
    case 5:
      s = i2c5_sem;
      break;
    case 6:
      s = i2c6_sem;
      break;
    default:
      s = 0;
      break;
  }
  return s;
}
#define MAX_TRIES 10
static int acquireI2CSemaphore(SemaphoreHandle_t s)
{
  int retval = pdTRUE;
  if ( s == NULL ) {
    return pdFAIL;
  }
  int tries = 0;
  while (xSemaphoreTake(s, (TickType_t)10) == pdFALSE) {
    ++tries;
    if (tries > MAX_TRIES ) {
      retval = pdFAIL;
      break;
    }
  }
  return retval;
}

static bool isValidDevice(int device)
{
#ifdef REV1
  bool isValidDevice = (device >= 0 && device <= 4) || (device == 6);
#elif (REV2)
  bool isValidDevice = (device >= 0 && device <= 5);
#endif
  return isValidDevice;
}

BaseType_t i2c_ctl_r(int argc, char **argv, char *m)
{
  BaseType_t device = strtol(argv[1], NULL, 10);
  BaseType_t address = strtol(argv[2], NULL, 16);
  BaseType_t nbytes = strtol(argv[3], NULL, 10);
  uint8_t data[I2C_CTL_MAX_BYTES] = {0, 0, 0, 0};
  if (nbytes == 0) {
    snprintf(m, SCRATCH_SIZE, "%s: cannot read 0 bytes\r\n", argv[0]);
    return pdFALSE;
  }
  if (!isValidDevice(device)) {
    snprintf(m, SCRATCH_SIZE, "%s: invalid device %ld\r\n", argv[0], device);
    return pdFALSE;
  }
  SemaphoreHandle_t s = getSemaphore(device);
  if ( s == NULL ) {
    snprintf(m, SCRATCH_SIZE, "%s: could not get semaphore\r\n", argv[0]);
    return pdFALSE;
  }
  if ( acquireI2CSemaphore(s) == pdFAIL ) {
    snprintf(m, SCRATCH_SIZE, "%s: could not get semaphore in time\r\n", argv[0]);
    return pdFALSE;
  }
  int status = apollo_i2c_ctl_r(device, address, nbytes, data);
  if (status == 0) {
    snprintf(m, SCRATCH_SIZE, "%s: dev %ld, addr 0x%02lx: val=0x%02x %02x %02x %02x\r\n", argv[0],
             device, address, data[3], data[2], data[1], data[0]);
  }
  else {
    snprintf(m, SCRATCH_SIZE, "%s: failure %d (%s)\r\n", argv[0], status, SMBUS_get_error(status));
  }
  xSemaphoreGive(s);
  return pdFALSE;
}

BaseType_t i2c_ctl_reg_r(int argc, char **argv, char *m)
{
  UBaseType_t address, packed_reg_address;
  BaseType_t device;
  uint32_t packed_data = 0U;
  BaseType_t nbytes_addr, nbytes;
  device = strtol(argv[1], NULL, 10); // i2c device
  if (!isValidDevice((int)device)) {
    snprintf(m, SCRATCH_SIZE, "%s: invalid device %lu\r\n", argv[0], device);
    return pdFALSE;
  }
  address = strtol(argv[2], NULL, 16);
  nbytes_addr = strtol(argv[3], NULL, 10);
  packed_reg_address = strtol(argv[4], NULL, 16);
  nbytes = strtol(argv[5], NULL, 10);

  if (nbytes == 0 || nbytes_addr == 0) {
    snprintf(m, SCRATCH_SIZE, "%s: nbytes or nbytes_addr is zero\r\n", argv[0]);
    return pdFALSE;
  }
  SemaphoreHandle_t s = getSemaphore(device);
  if (s == NULL) {
    snprintf(m, SCRATCH_SIZE, "%s: could not get semaphore\r\n", argv[0]);
    return pdFALSE;
  }
  if (acquireI2CSemaphore(s) == pdFAIL) {
    snprintf(m, SCRATCH_SIZE, "%s: could not get semaphore in time\r\n", argv[0]);
    return pdFALSE;
  }

  int status = apollo_i2c_ctl_reg_r(device, address, nbytes_addr, packed_reg_address,
                                    nbytes, &packed_data);
  if (status == 0) {
    snprintf(m, SCRATCH_SIZE, "i2cr: add: 0x%02lx, reg 0x%02lx: value 0x%08lx (%ld bytes)\r\n",
             address, packed_reg_address, packed_data, nbytes);
  }
  else {
    int copied = snprintf(m, SCRATCH_SIZE, "i2cr: add: 0x%02lx, reg 0x%02lx: value 0x%08lx (%ld bytes)\r\n",
                          address, packed_reg_address, packed_data, nbytes);
    snprintf(m + copied, SCRATCH_SIZE - copied, "%s: failure %d (%s)\r\n", argv[0], status,
             SMBUS_get_error(status));
  }
  xSemaphoreGive(s);
  return pdFALSE;
}

BaseType_t i2c_ctl_reg_w(int argc, char **argv, char *m)
{
  // first byte is the register, others are the data
  UBaseType_t address, packed_reg_address, packed_data;
  BaseType_t device, nbytes_addr, nbytes;
  device = strtol(argv[1], NULL, 10); // i2c device
  if (!isValidDevice(device)) {
    snprintf(m, SCRATCH_SIZE, "%s: invalid device %lu\r\n", argv[0], device);
    return pdFALSE;
  }
  address = strtoul(argv[2], NULL, 16); // address
  nbytes_addr = strtol(argv[3], NULL, 10);
  packed_reg_address = strtoul(argv[4], NULL, 16); // register
  nbytes = strtol(argv[5], NULL, 16);              // number of bytes
  packed_data = strtoul(argv[6], NULL, 16);        // data
  if (nbytes == 0 || nbytes_addr == 0) {
    snprintf(m, SCRATCH_SIZE, "%s: nbytes or nbytes_addr is zero\r\n", argv[0]);
    return pdFALSE;
  }

  SemaphoreHandle_t s = getSemaphore(device);
  if (s == NULL) {
    snprintf(m, SCRATCH_SIZE, "%s: could not get semaphore\r\n", argv[0]);
    return pdFALSE;
  }
  if (acquireI2CSemaphore(s) == pdFAIL) {
    snprintf(m, SCRATCH_SIZE, "%s: could not get semaphore in time\r\n", argv[0]);
    return pdFALSE;
  }

  int status = apollo_i2c_ctl_reg_w(device, address, nbytes_addr, packed_reg_address, nbytes, packed_data);
  if (status == 0) {
    snprintf(m, SCRATCH_SIZE, "%s: W to addr 0x%lx, reg 0x%lx, val=0x%08lx (%ld bytes)\r\n", argv[0],
             address, packed_reg_address, packed_data, nbytes);
  }
  else {
    snprintf(m, SCRATCH_SIZE, "%s: failure %d (%s)\r\n", argv[0], status, SMBUS_get_error(status));
  }

  xSemaphoreGive(s);
  return pdFALSE;
}

BaseType_t i2c_ctl_w(int argc, char **argv, char *m)
{
  UBaseType_t address, value;
  UBaseType_t nbytes;
  BaseType_t device;
  device = strtol(argv[1], NULL, 10);
  if (!isValidDevice(device)) {
    snprintf(m, SCRATCH_SIZE, "%s: invalid device %lu\r\n", argv[0], device);
    return pdFALSE;
  }
  address = strtoul(argv[2], NULL, 16);
  nbytes = strtoul(argv[3], NULL, 16);
  value = strtoul(argv[4], NULL, 16);

  if (nbytes == 0) {
    snprintf(m, SCRATCH_SIZE, "%s: cannot write 0 bytes\r\n", argv[0]);
    return pdFALSE;
  }

  SemaphoreHandle_t s = getSemaphore(device);
  if (s == NULL) {
    snprintf(m, SCRATCH_SIZE, "%s: could not get semaphore\r\n", argv[0]);
    return pdFALSE;
  }
  if (acquireI2CSemaphore(s) == pdFAIL) {
    snprintf(m, SCRATCH_SIZE, "%s: could not get semaphore in time\r\n", argv[0]);
    return pdFALSE;
  }

  int status = apollo_i2c_ctl_w(device, address, nbytes, value);
  if (status == 0) {
    snprintf(m, SCRATCH_SIZE, "i2cwr: Wrote to addr 0x%lx, val=0x%08lx (%ld bytes)\r\n", address,
             value, nbytes);
  }
  else {
    snprintf(m, SCRATCH_SIZE, "%s: failure %d (%s)\r\n", argv[0], status, SMBUS_get_error(status));
  }
  xSemaphoreGive(s);
  return pdFALSE;
}

BaseType_t i2c_scan(int argc, char **argv, char *m)
{
  // takes one argument
  int device = strtol(argv[1], NULL, 10); // i2c device
  if (!isValidDevice(device)) {
    snprintf(m, SCRATCH_SIZE, "%s: invalid device %d\r\n", argv[0], device);
    return pdFALSE;
  }
  SemaphoreHandle_t s = getSemaphore(device);
  if (s == NULL) {
    snprintf(m, SCRATCH_SIZE, "%s: could not get semaphore\r\n", argv[0]);
    return pdFALSE;
  }
  if (acquireI2CSemaphore(s) == pdFAIL) {
    snprintf(m, SCRATCH_SIZE, "%s: could not get semaphore in time\r\n", argv[0]);
    return pdFALSE;
  }

  int copied = 0;
  copied += snprintf(m, SCRATCH_SIZE - copied, "i2c bus scan, device %d\r\n", device);
  copied += snprintf(m + copied, SCRATCH_SIZE - copied,
                     "     0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f\r\n00:         ");
  for (uint8_t i = 0x3; i < 0x78; ++i) {
    uint8_t data;
    if (i % 16 == 0) {
      copied += snprintf(m + copied, SCRATCH_SIZE - copied, "\r\n%2x:", i);
    }
    // try to read one byte from current address
    int status = apollo_i2c_ctl_r(device, i, 1, &data);
    if (status == 0)
      copied += snprintf(m + copied, SCRATCH_SIZE - copied, " %2x", i);
    else
      copied += snprintf(m + copied, SCRATCH_SIZE - copied, " --");
    configASSERT(copied < SCRATCH_SIZE);
  }
  copied += snprintf(m + copied, SCRATCH_SIZE - copied, "\r\n");
  configASSERT(copied < SCRATCH_SIZE);

  xSemaphoreGive(s);
  return pdFALSE;
}
