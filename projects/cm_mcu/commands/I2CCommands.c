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
#include "inc/hw_i2c.h" // I2C_MBMON_SCL/SDA

static bool isValidDevice(int device)
{
#ifdef REV1
  bool isValidDevice = (device >= 0 && device <= 4) || (device == 6);
#elif defined(REV2) || defined(REV3)
  bool isValidDevice = (device > 0 && device <= 5);
#endif
  return isValidDevice;
}

// Check widths on BaseType_t, before they narrow to uint8_t at the
// apollo_i2c_ctl_reg_* call sites. Returns true if OK.
#define STR_(x) #x
#define STR(x)  STR_(x)
__attribute__((noinline)) static bool checkWidths(char *m, BaseType_t nbytes_addr, BaseType_t nbytes)
{
  if (nbytes_addr < 1 || nbytes_addr > MAX_BYTES_ADDR || nbytes < 1 || nbytes > MAX_BYTES) {
    snprintf(m, SCRATCH_SIZE,
             "nbytes_addr must be 1-" STR(MAX_BYTES_ADDR) ", nbytes 1-" STR(MAX_BYTES) "\r\n");
    return false;
  }
  return true;
}

BaseType_t i2c_ctl_r(int argc, char **argv, char *m)
{
  BaseType_t device = strtol(argv[1], NULL, 10);
  BaseType_t address = strtol(argv[2], NULL, 16);
  BaseType_t nbytes = strtol(argv[3], NULL, 10);
  uint8_t data[MAX_BYTES] = {0, 0, 0, 0};
  if (nbytes == 0) {
    snprintf(m, SCRATCH_SIZE, "%s: cannot read 0 bytes\r\n", argv[0]);
    return pdFALSE;
  }
  if (!isValidDevice(device)) {
    snprintf(m, SCRATCH_SIZE, "%s: invalid device %ld\r\n", argv[0], device);
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

  if (!checkWidths(m, nbytes_addr, nbytes)) {
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
  if (!checkWidths(m, nbytes_addr, nbytes)) {
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

  int status = apollo_i2c_ctl_w(device, address, nbytes, value);
  if (status == 0) {
    snprintf(m, SCRATCH_SIZE, "i2cwr: Wrote to addr 0x%lx, val=0x%08lx (%ld bytes)\r\n", address,
             value, nbytes);
  }
  else {
    snprintf(m, SCRATCH_SIZE, "%s: failure %d (%s)\r\n", argv[0], status, SMBUS_get_error(status));
  }
  return pdFALSE;
}

// you should grab the semaphore before calling this function
BaseType_t i2c_scan(int argc, char **argv, char *m)
{
  // takes one argument
  int device = strtol(argv[1], NULL, 10); // i2c device
  if (!isValidDevice(device)) {
    snprintf(m, SCRATCH_SIZE, "%s: invalid device %d\r\n", argv[0], device);
    return pdFALSE;
  }

  int copied = 0;
  copied += snprintf(m, SCRATCH_SIZE, "(semaphore?)\r\n");
  copied += snprintf(m + copied, SCRATCH_SIZE - copied, "i2c bus scan, device %d\r\n", device);
  copied += snprintf(m + copied, SCRATCH_SIZE - copied,
                     "     0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f\r\n00:         ");
  for (uint8_t i = 0x3; i < 0x78; ++i) {
    uint8_t data[4];
    if (i % 16 == 0) {
      copied += snprintf(m + copied, SCRATCH_SIZE - copied, "\r\n%2x:", i);
    }
    // try to read one byte from current address
    int status = apollo_i2c_ctl_r(device, i, 1, data);
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

static int print_recover(char *m, int copied, const char *what, const struct i2c_recover_info *info)
{
  copied += snprintf(m + copied, SCRATCH_SIZE - copied,
                     "%s: %s, %d clocks, SCL/SDA %d%d -> %d%d\r\n", what,
                     i2c_recover_status_str(info->status), info->nclocks, !!(info->pre & I2C_MBMON_SCL),
                     !!(info->pre & I2C_MBMON_SDA), !!(info->post & I2C_MBMON_SCL),
                     !!(info->post & I2C_MBMON_SDA));
  return copied;
}

// Recover bus <dev>: SCL clocks + STOP, then controller reset. Like the other raw
// i2c commands it does not take the bus semaphore; the user holds it (and sets
// the mux) by hand.
BaseType_t i2c_reset(int argc, char **argv, char *m)
{
  BaseType_t device = strtol(argv[1], NULL, 10);
  if (device == 0 || !isValidDevice(device)) {
    snprintf(m, SCRATCH_SIZE, "%s: invalid device %ld\r\n", argv[0], device);
    return pdFALSE;
  }
  struct i2c_recover_info info;
  apollo_i2c_bus_reset(device, &info);
  print_recover(m, 0, argv[0], &info);
  return pdFALSE;
}

extern const struct dev_i2c_addr_t pm_addrs_dcdc[N_PM_ADDRS_DCDC]; // LocalTasks.c

// Recover the bus 1 mux channel of LGA80D <n> with apollo_i2c_bus_reset_mux(),
// holding i2c1_sem. No mux read-back: on a stuck bus it would only add a failed
// transfer to the counters.
BaseType_t i2c_reset_dcdc(int argc, char **argv, char *m)
{
  int copied = 0;
  if (argc == 1) {
    for (int i = 0; i < N_PM_ADDRS_DCDC; ++i) {
      copied += snprintf(m + copied, SCRATCH_SIZE - copied, "%d: %-10s mux 0x%02x bit %d addr 0x%02x\r\n",
                         i, pm_addrs_dcdc[i].name, pm_addrs_dcdc[i].mux_addr,
                         pm_addrs_dcdc[i].mux_bit, pm_addrs_dcdc[i].dev_addr);
    }
    return pdFALSE;
  }
  if (argc != 2) {
    snprintf(m, SCRATCH_SIZE, "%s: expect 0 or 1 arguments\r\n", argv[0]);
    return pdFALSE;
  }
  BaseType_t n = strtol(argv[1], NULL, 10);
  if (n < 0 || n >= N_PM_ADDRS_DCDC) {
    snprintf(m, SCRATCH_SIZE, "%s: device must be 0-%d\r\n", argv[0], N_PM_ADDRS_DCDC - 1);
    return pdFALSE;
  }
  const struct dev_i2c_addr_t *dev = &pm_addrs_dcdc[n];
  const uint8_t bus = 1;

  bool took = false;
  if (xSemaphoreGetMutexHolder(i2c1_sem) != xTaskGetCurrentTaskHandle()) {
    if (xSemaphoreTake(i2c1_sem, pdMS_TO_TICKS(5000)) != pdTRUE) {
      snprintf(m, SCRATCH_SIZE, "%s: could not take semaphore %d\r\n", argv[0], bus);
      return pdFALSE;
    }
    took = true;
  }

  copied += snprintf(m + copied, SCRATCH_SIZE - copied, "%s: %s (mux 0x%02x bit %d)\r\n",
                     argv[0], dev->name, dev->mux_addr, dev->mux_bit);
  struct i2c_recover_info info;
  int status = apollo_i2c_bus_reset_mux(bus, dev->mux_addr, dev->mux_bit, &info);
  copied = print_recover(m, copied, "channel", &info);
  copied += snprintf(m + copied, SCRATCH_SIZE - copied, "result (mux deselected): %s\r\n",
                     i2c_recover_status_str(status));

  if (took && xSemaphoreGetMutexHolder(i2c1_sem) == xTaskGetCurrentTaskHandle())
    xSemaphoreGive(i2c1_sem);
  configASSERT(copied < SCRATCH_SIZE);
  return pdFALSE;
}
