/*
 * I2CCommands.c
 *
 *  Created on: Jan 14, 2021
 *      Author: fatimayousuf
 */

#include "I2CCommands.h"

BaseType_t i2c_ctl_r(int argc, char **argv, char *m)
{
  int s = SCRATCH_SIZE;
  BaseType_t device, address, nbytes;
  device = strtol(argv[1], NULL, 16);
  address = strtol(argv[2], NULL, 16);
  nbytes = strtol(argv[3], NULL, 10);
  uint8_t data[I2C_CTL_MAX_BYTES];

  int status = apollo_i2c_ctl_r(device, address, nbytes, data);
  if (status == 0) {
    snprintf(m, s, "%s: dev %ld, add 0x%02lx: value 0x%02x %02x %02x %02x\r\n", argv[0], device, address, data[3],
             data[2], data[1], data[0]);
  }
  else {
    snprintf(m, s, "%s: failure %d\r\n", argv[0], status);
  }
  return pdFALSE;
}

BaseType_t i2c_ctl_reg_r(int argc, char **argv, char *m)
{
  int s = SCRATCH_SIZE;
  UBaseType_t device, address, packed_reg_address;
  uint32_t packed_data;
    BaseType_t nbytes_addr, nbytes;
  device = strtol(argv[1], NULL, 16); // i2c device
  address = strtol(argv[2], NULL, 16);
  nbytes_addr = strtol(argv[3], NULL, 10);
  packed_reg_address = strtol(argv[4], NULL, 16);
  nbytes = strtol(argv[5], NULL, 10);
  packed_data = strtoul(argv[6], NULL, 16); // data;

  int status = apollo_i2c_ctl_reg_r(device, address, nbytes_addr, packed_reg_address,
                                    nbytes, &packed_data);
  if (status == 0) {
    snprintf(m, s, "i2cr: add: 0x%02lx, reg 0x%02lx: value 0x%08lx (%ld bytes)\r\n",
             address, packed_reg_address, packed_data, nbytes);
  }
  else {
    snprintf(m, s, "%s: failure %d\r\n", argv[0], status);
  }
  return pdFALSE;
}

BaseType_t i2c_ctl_reg_w(int argc, char **argv, char *m)
{
  int s = SCRATCH_SIZE;
  // first byte is the register, others are the data
  UBaseType_t device, address, packed_reg_address, packed_data;
  BaseType_t nbytes_addr, nbytes;
  device = strtoul(argv[1], NULL, 16);     // i2c device
  address = strtoul(argv[2], NULL, 16);     // address
  nbytes_addr = strtol(argv[3], NULL, 10);
  packed_reg_address = strtoul(argv[4], NULL, 16); // register
  nbytes = strtol(argv[5], NULL, 16);      // number of bytes
  packed_data = strtoul(argv[6], NULL, 16); // data

  int status = apollo_i2c_ctl_reg_w(device, address, nbytes_addr, packed_reg_address, nbytes, packed_data);
  if (status == 0) {
    snprintf(m, s, "%s: Wrote to address 0x%lx, register 0x%lx, value 0x%08lx (%ld bytes)\r\n", argv[0], address,
             packed_reg_address, packed_data, nbytes);
  }
  else {
    snprintf(m, s, "%s: failure %d\r\n", argv[0], status);
  }

  return pdFALSE;
}

BaseType_t i2c_ctl_w(int argc, char **argv, char *m)
{
  int s = SCRATCH_SIZE;
  UBaseType_t device, address, value;
  BaseType_t nbytes;
  device = strtol(argv[1], NULL, 16);
  address = strtol(argv[2], NULL, 16);
  nbytes = strtol(argv[3], NULL, 16);
  value = strtol(argv[4], NULL, 16);

  int status = apollo_i2c_ctl_w(device, address, nbytes, value);
  if (status == 0) {
    snprintf(m, s, "i2cwr: Wrote to address 0x%lx, value 0x%08lx (%ld bytes)\r\n", address, value, nbytes);
  }
  else {
    snprintf(m, s, "%s: failure %d\r\n", argv[0], status);
  }
  return pdFALSE;
}

BaseType_t i2c_scan(int argc, char **argv, char *m)
{
  // takes one argument
  int device = strtol(argv[1], NULL, 16); // i2c device

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

  return pdFALSE;
}
