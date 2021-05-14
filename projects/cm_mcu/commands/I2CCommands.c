/*
 * I2CCommands.c
 *
 *  Created on: Jan 14, 2021
 *      Author: fatimayousuf
 */

#include "I2CCommands.h"

static tSMBus *p_sMaster = &g_sMaster4;
static tSMBusStatus *p_eStatus = &eStatus4;

//BaseType_t i2c_ctl_set_dev(int argc, char **argv, char* m)
//{
//  int s = SCRATCH_SIZE;
//  BaseType_t i = strtol(argv[1], NULL, 10); // device number
//
//  int status = apollo_i2c_ctl_set_dev(i);
//  if (status == 0) {
//    snprintf(m, s, "Setting i2c device to %d\r\n", i);
//  }
//  else if (status == -1) {
//    snprintf(m, s, "Invalid i2c device %d (%s), only 1,2,3, 4 and 6 supported\r\n", i, argv[1]);
//  }
//  else if (status == -2) {
//    snprintf(m, s, "%s: huh? line %d\r\n", argv[0], __LINE__);
//  }
//  else {
//    snprintf(m, s, "%s: invalid return value. line %d\r\n", argv[0], __LINE__);
//  }
//  return pdFALSE;
//}

BaseType_t i2c_ctl_r(int argc, char **argv, char* m)
{
  int s = SCRATCH_SIZE;
  BaseType_t device, address, nbytes;
  device = strtol(argv[1], NULL, 16);
  address = strtol(argv[2], NULL, 16);
  nbytes = strtol(argv[3], NULL, 10);
  uint8_t data[I2C_CTL_MAX_BYTES];

  int status = apollo_i2c_ctl_r(device, address, nbytes, data);
  if (status == 0) {
    snprintf(m, s, "%s: add: 0x%02x: value 0x%02x %02x %02x %02x\r\n", argv[0], address, data[3],
             data[2], data[1], data[0]);
  }
  else if (status == -1) {
    snprintf(m, s, "%s: operation failed (1)\r\n", argv[0]);
  }
  else if (status == -2) {
    snprintf(m, s, "%s: operation failed (2, value=%d)\r\n", argv[0], *p_eStatus);
  }
  else {
    snprintf(m, s, "%s: invalid return value. line %d\r\n", argv[0], __LINE__);
  }
  return pdFALSE;
}

BaseType_t i2c_ctl_reg_r(int argc, char **argv, char* m)
{
  int s = SCRATCH_SIZE;
  BaseType_t address, reg_address, nbytes;
  address = strtol(argv[1], NULL, 16);
  reg_address = strtol(argv[2], NULL, 16);
  nbytes = strtol(argv[3], NULL, 10);
  uint8_t data[I2C_CTL_MAX_BYTES];
  uint8_t txdata = reg_address;

  int status = apollo_i2c_ctl_reg_r(address, reg_address, txdata, nbytes, data);
  if (status == 0) {
    snprintf(m, s, "i2cr: add: 0x%02x, reg 0x%02x: value 0x%02x %02x %02x %02x\r\n", address,
             reg_address, data[3], data[2], data[1], data[0]);
  }
  else if (status == -1) {
    snprintf(m, s, "%s: operation failed (1)\r\n", argv[0]);
  }
  else if (status == -2) {
    snprintf(m, s, "%s: operation failed (2, value=%d)\r\n", argv[0], *p_eStatus);
  }
  else {
    snprintf(m, s, "%s: invalid return value. line %d\r\n", argv[0], __LINE__);
  }
  return pdFALSE;
}

BaseType_t i2c_ctl_reg_w(int argc, char **argv, char* m)
{
  int s = SCRATCH_SIZE;
  // first byte is the register, others are the data
  BaseType_t device, address, reg_address, nbytes, packed_data;
  device = strtol(argv[1], NULL, 16);      // i2c device
  address = strtol(argv[2], NULL, 16);     // address
  reg_address = strtol(argv[3], NULL, 16); // register
  nbytes = strtol(argv[4], NULL, 16);      // number of bytes
  packed_data = strtol(argv[5], NULL, 16); // data

  int status = apollo_i2c_ctl_reg_w(device, address, reg_address, nbytes, packed_data);
  if (status == 0) {
    snprintf(m, s, "%s: Wrote to address 0x%x, register 0x%x, value 0x%08x (%d bytes)\r\n", argv[0],
             address, reg_address, packed_data, nbytes - 1);
  }
  else if (status == -1) {
    snprintf(m, s, "%s: operation failed (1)\r\n", argv[0]);
  }
  else if (status == -2) {
    snprintf(m, s, "%s: operation failed (2)\r\n", argv[0]);
  }
  else {
    snprintf(m, s, "%s: invalid return value. line %d\r\n", argv[0], __LINE__);
  }

  return pdFALSE;
}

BaseType_t i2c_ctl_w(int argc, char **argv, char* m)
{
  int s = SCRATCH_SIZE;
  BaseType_t device, address, nbytes, value;
  device = strtol(argv[1], NULL, 16);
  address = strtol(argv[2], NULL, 16);
  nbytes = strtol(argv[3], NULL, 16);
  value = strtol(argv[4], NULL, 16);

  int status = apollo_i2c_ctl_w(device, address, nbytes, value);
  if (status == 0) {
    snprintf(m, s, "i2cwr: Wrote to address 0x%x, value 0x%08x (%d bytes)\r\n", address, value,
             nbytes);
  }
  else if (status == -1) {
    snprintf(m, s, "%s: write failed (1)\r\n", argv[0]);
  }
  else if (status == -2) {
    snprintf(m, s, "%s: write failed (2)\r\n", argv[0]);
  }
  else {
    snprintf(m, s, "%s: invalid return value. line %d\r\n", argv[0], __LINE__);
  }
  return pdFALSE;
}

BaseType_t i2c_scan(int argc, char **argv, char* m)
{
  // takes no arguments
  int copied = 0;
  copied += snprintf(m, SCRATCH_SIZE - copied, "i2c bus scan\r\n");
  copied += snprintf(m + copied, SCRATCH_SIZE - copied,
                     "     0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f\r\n00:         ");
  for (uint8_t i = 0x3; i < 0x78; ++i) {
    uint8_t data;
    if (i % 16 == 0)
      copied += snprintf(m + copied, SCRATCH_SIZE - copied, "\r\n%2x:", i);
    tSMBusStatus r = SMBusMasterI2CRead(p_sMaster, i, &data, 1);
    if (r != SMBUS_OK) {
      Print("i2c_scan: Probe failed 1\r\n");
    }
    while (SMBusStatusGet(p_sMaster) == SMBUS_TRANSFER_IN_PROGRESS) {
      vTaskDelay(pdMS_TO_TICKS(10)); // wait
    }
    if (*p_eStatus == SMBUS_OK)
      copied += snprintf(m + copied, SCRATCH_SIZE - copied, " %2x", i);
    else
      copied += snprintf(m + copied, SCRATCH_SIZE - copied, " --");
    configASSERT(copied < SCRATCH_SIZE);
  }
  copied += snprintf(m + copied, SCRATCH_SIZE - copied, "\r\n");
  configASSERT(copied < SCRATCH_SIZE);

  return pdFALSE;
}
