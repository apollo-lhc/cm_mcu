/*
 * FireflyCommands.c
 *
 * Firefly CLI command handlers, extracted from SensorControl.c.
 */

#include <stdlib.h>
#include <strings.h>
#include <string.h>
#include <assert.h>

#include "commands/FireflyCommands.h"
#include "commands/parameters.h"
#include "FireflyUtils.h"
#include "I2CCommunication.h"
#include "MonI2C_addresses.h"
#include "MonUtils.h"
#include "common/log.h"
#include "common/utils.h"
#include "common/smbus_helper.h"
#include "Semaphore.h"
#include "Tasks.h"
#include "projdefs.h"

// A NOTE ABOUT THE FIREFLY REGISTER ADDRESSES
// If you look at the memory maps in the data sheets for the firefly devices, you see that
// the listed addresses for the internal registers are grouped into pages.
// The lowest 7 bits of address are always the same, regardless of the value stored in the
// page select register.
// the page select register is always accessible at byte 0x7f (127).
// Bytes above 127 (128 and following) vary in their meaning depending on the value set in the
// page register. that is to say, byte 128, e.g., is not uniquely defined.
// not all page register values are valid.
// we therefore encode the page number in the register address as follows:
// lowest 8 bits: register address, 0-255.
// if the register address is > 127, the next 8 bits are the page number.
// next 8 bits: page register number.
// This is used in both the read and write functions.
int read_ff_register(const char *name, uint16_t packed_reg_addr, uint8_t *value, size_t size, int i2c_device)
{
  memset(value, 0, size);
  // find the appropriate information for this FF device
  int ff;
  for (ff = 0; ff < NFIREFLIES; ++ff) {
    if (strncmp(ff_moni2c_addrs[ff].name, name, 10) == 0)
      break;
  }
  if (ff == NFIREFLIES) {
    return -2; // no match found
  }

  int res;
  SemaphoreHandle_t s = i2c4_sem;
  if (i2c_device == I2C_DEVICE_F2) {
    s = i2c3_sem;
  }

  if (acquireI2CSemaphore(s) == pdFAIL) {
    log_warn(LOG_SERVICE, "could not get semaphore in time\r\n");
    return SEM_ACCESS_ERROR;
  }

  // write to the mux
  // select the appropriate output for the mux
  uint8_t muxmask = 0x1U << ff_moni2c_addrs[ff].mux_bit;
  res = apollo_i2c_ctl_w(i2c_device, ff_moni2c_addrs[ff].mux_addr, 1, muxmask);
  if (res != 0) {
    log_warn(LOG_SERVICE, "%s: Mux writing error %d (%s) (ff=%s) ...\r\n", __func__, res,
             SMBUS_get_error(res), ff_moni2c_addrs[ff].name);
  }

  if (!res) {
    // Read from register.  if the register number is > FF_PAGE_SELECT_BYTE (0x7FU), we
    // must first write the page number to page select byte.
    if ((packed_reg_addr & 0xFFU) > FF_PAGE_SELECT_BYTE) {
      uint8_t page = (packed_reg_addr >> 8) & 0xFFU;
      res = apollo_i2c_ctl_reg_w(i2c_device, ff_moni2c_addrs[ff].dev_addr, 1,
                                 FF_PAGE_SELECT_BYTE, 1, page);
      if (res)
        log_warn(LOG_SERVICE, "%s: FF page write error %d (%s) (ff=%s) ...\r\n", __func__, res,
                 SMBUS_get_error(res), ff_moni2c_addrs[ff].name);
    }
    packed_reg_addr &= 0x00FFU; // select out the register number, bottom 8 bits
    uint32_t uidata;
    res += apollo_i2c_ctl_reg_r(i2c_device, ff_moni2c_addrs[ff].dev_addr, 1,
                                packed_reg_addr, size, &uidata);
    for (int i = 0; i < size; ++i) {
      value[i] = (uint8_t)((uidata >> (i * 8)) & 0xFFU);
    }
    if (res != 0) {
      log_warn(LOG_SERVICE, "%s: FF Regread error %d (%s) (ff=%s) ...\r\n", __func__, res,
               SMBUS_get_error(res), ff_moni2c_addrs[ff].name);
    }
  }
  if (!res) { // clear the mux
    muxmask = 0x0U;
    res = apollo_i2c_ctl_w(i2c_device, ff_moni2c_addrs[ff].mux_addr, 1, muxmask);
    if (res != 0) {
      log_warn(LOG_SERVICE, "%s: Mux clear error %d (%s) (ff=%s) ...\r\n", __func__, res,
               SMBUS_get_error(res), ff_moni2c_addrs[ff].name);
    }
  }

  // release the semaphore
  if (xSemaphoreGetMutexHolder(s) == xTaskGetCurrentTaskHandle())
    xSemaphoreGive(s);
  return res;
}

// see comments above read_ff_register
static int write_ff_register(const char *name, uint16_t reg, uint16_t value, int size, int i2c_device)
{
  configASSERT(size <= 2);
  // find the appropriate information for this FF device
  int ff;
  for (ff = 0; ff < NFIREFLIES; ++ff) {
    if (strncmp(ff_moni2c_addrs[ff].name, name, 10) == 0)
      break;
  }
  if (ff == NFIREFLIES) {
    return -2; // no match found
  }

  int res;
  SemaphoreHandle_t s = getSemaphore(4);
  if (i2c_device == I2C_DEVICE_F2) {
    s = getSemaphore(3);
  }

  if (acquireI2CSemaphore(s) == pdFAIL) {
    log_warn(LOG_SERVICE, "could not get semaphore in time\r\n");
    return SEM_ACCESS_ERROR;
  }

  // write to the mux
  // select the appropriate output for the mux
  uint8_t muxmask = 0x1U << ff_moni2c_addrs[ff].mux_bit;
  res = apollo_i2c_ctl_w(i2c_device, ff_moni2c_addrs[ff].mux_addr, 1, muxmask);
  if (res != 0) {
    log_warn(LOG_SERVICE, "%s: Mux writing error %d (%s) (ff=%s) ...\r\n", __func__, res,
             SMBUS_get_error(res), ff_moni2c_addrs[ff].name);
  }

  // write to register. First word is reg address, then the data.
  // increment size to account for the register address
  if (!res) {
    // If the register number is > FF_PAGE_SELECT_BYTE (0x7FU), we
    // must first write the page number to page select byte.
    if ((reg & 0xFFU) > FF_PAGE_SELECT_BYTE) {
      uint8_t page = (reg >> 8) & 0xFFU;
      res = apollo_i2c_ctl_reg_w(i2c_device, ff_moni2c_addrs[ff].dev_addr, 1,
                                 FF_PAGE_SELECT_BYTE, 1, page);
      if (res)
        log_warn(LOG_SERVICE, "%s: FF page write error %d (%s) (ff=%s) ...\r\n", __func__, res,
                 SMBUS_get_error(res), ff_moni2c_addrs[ff].name);
    }
    reg &= 0x00FFU; // select out the register number, bottom 8 bits

    res = apollo_i2c_ctl_reg_w(i2c_device, ff_moni2c_addrs[ff].dev_addr, 1, reg, size, (uint32_t)value);
    if (res != 0) {
      log_warn(LOG_SERVICE, "%s: FF writing error %d (%s) (ff=%s) ...\r\n", __func__, res,
               SMBUS_get_error(res), ff_moni2c_addrs[ff].name);
    }
  }
  if (!res) { // clear the mux
    muxmask = 0x0U;
    res = apollo_i2c_ctl_w(i2c_device, ff_moni2c_addrs[ff].mux_addr, 1, muxmask);
    if (res != 0) {
      log_warn(LOG_SERVICE, "%s: Mux clear error %d (%s) (ff=%s) ...\r\n", __func__, res,
               SMBUS_get_error(res), ff_moni2c_addrs[ff].name);
    }
  }

  // release the semaphore
  if (xSemaphoreGetMutexHolder(s) == xTaskGetCurrentTaskHandle())
    xSemaphoreGive(s);
  return res;
}

static int disable_transmit(bool disable, int num_ff)
{
  int ret = 0, i = num_ff, imax = num_ff + 1;
  // i and imax are used as limits for the loop below. By default, only iterate once, with i=num_ff.
  uint16_t value = 0xffffU; // see data sheet re how bits are arranged; do not set this to 0xfffU!
  if (disable == false)
    value = 0x0U;
  if (num_ff == NFIREFLIES) { // if NFIREFLIES is given for num_ff, loop over ALL transmitters.
    i = 0;
    imax = NFIREFLIES;
  }
  int i2c_dev;
  for (; i < imax; ++i) {
    if (!isEnabledFF(i)) // skip the FF if it's not enabled via the FF config
      continue;
    if (i < NFIREFLIES_F1) {
      i2c_dev = I2C_DEVICE_F1;
    }
    else {
      i2c_dev = I2C_DEVICE_F2;
    }

    if (strstr(ff_moni2c_addrs[i].name, "XCVR") != NULL) {
      // only 4 LSB matter, so mask out others.
      ret += write_ff_register(ff_moni2c_addrs[i].name, ECU0_25G_XVCR_TX_DISABLE_REG, value & 0xFU, 1, i2c_dev);
    }
    else if (strstr(ff_moni2c_addrs[i].name, "Tx") != NULL) { // same for all 12 channel parts
      ret += write_ff_register(ff_moni2c_addrs[i].name, ECU0_14G_TX_DISABLE_REG, value, 2, i2c_dev);
    }
  }
  return ret;
}

static int disable_receivers(bool disable, int num_ff)
{
  int ret = 0, i = num_ff, imax = num_ff + 1;
  // i and imax are used as limits for the loop below. By default, only iterate once, with i=num_ff.
  uint16_t value = 0xfff;
  if (disable == false)
    value = 0x0;
  if (num_ff == NFIREFLIES) { // if NFIREFLIES is given for num_ff, loop over ALL transmitters.
    i = 0;
    imax = NFIREFLIES;
  }
  int i2c_dev;
  for (; i < imax; ++i) {
    if (!isEnabledFF(i)) // skip the FF if it's not enabled via the FF config
      continue;
    if (i < NFIREFLIES_F1) {
      i2c_dev = I2C_DEVICE_F1;
    }
    else {
      i2c_dev = I2C_DEVICE_F2;
    }
    if (strstr(ff_moni2c_addrs[i].name, "XCVR") != NULL) {
      value &= 0x000fU; // only 4 LSB matter, so mask out others
      ret += write_ff_register(ff_moni2c_addrs[i].name, ECU0_25G_XVCR_RX_DISABLE_REG, value, 1, i2c_dev);
    }
    else if (strstr(ff_moni2c_addrs[i].name, "Rx") != NULL) { // Same for CERNB vs 25G
      ret += write_ff_register(ff_moni2c_addrs[i].name, ECU0_14G_RX_DISABLE_REG, value, 2, i2c_dev);
    }
  }
  return ret;
}

static int set_xcvr_cdr(uint8_t value, int num_ff)
{
  int ret = 0, i = num_ff, imax = num_ff + 1;
  // i and imax are used as limits for the loop below. By default, only iterate once, with i=num_ff.
  if (num_ff == NFIREFLIES) { // if NFIREFLIES is given for num_ff, loop over ALL transmitters.
    i = 0;
    imax = NFIREFLIES;
  }
  int i2c_dev;
  for (; i < imax; ++i) {
    if (!isEnabledFF(i)) // skip the FF if it's not enabled via the FF config
      continue;
    if (i < NFIREFLIES_F1) {
      i2c_dev = I2C_DEVICE_F1;
    }
    else {
      i2c_dev = I2C_DEVICE_F2;
    }
    if (strstr(ff_moni2c_addrs[i].name, "XCVR") != NULL) {
      ret += write_ff_register(ff_moni2c_addrs[i].name, ECU0_25G_XVCR_CDR_REG, value, 1, i2c_dev);
    }
    else {                                          // Tx/Rx
      uint16_t value16 = value == 0 ? 0U : 0xffffU; // hack
      ret += write_ff_register(ff_moni2c_addrs[i].name, ECU0_25G_TXRX_CDR_REG, value16, 2, i2c_dev);
    }
  }
  return ret;
}

static int write_arbitrary_ff_register(uint16_t regnumber, uint8_t value, int num_ff)
{
  int ret = 0, i = num_ff, imax = num_ff + 1;
  // i and imax are used as limits for the loop below. By default, only iterate once, with i=num_ff.
  if (num_ff == NFIREFLIES) { // if NFIREFLIES is given for num_ff, loop over ALL transmitters.
    i = 0;
    imax = NFIREFLIES;
  }
  int i2c_dev;
  for (; i < imax; ++i) {

    if (num_ff == i)
      break;
  }

  if (!isEnabledFF(i)) { // skip the FF if it's not enabled via the FF config
    log_warn(LOG_SERVICE, "Skip writing to disabled FF %d\r\n", i);
  }
  if (i < NFIREFLIES_F1) {
    i2c_dev = I2C_DEVICE_F1;
  }
  else {
    i2c_dev = I2C_DEVICE_F2;
  }
  int ret1 = write_ff_register(ff_moni2c_addrs[i].name, regnumber, value, 1, i2c_dev);
  if (ret1) {
    log_warn(LOG_SERVICE, "%s: error %s\r\n", __func__, SMBUS_get_error(ret1));
    ret += ret1;
  }
  return ret;
}

// read a SINGLE firefly register, size bytes (up to 4 bytes)
uint16_t read_arbitrary_ff_register(uint16_t regnumber, int num_ff, uint8_t *value, uint8_t size)
{
  if (num_ff >= NFIREFLIES) {
    return -1;
  }
  int i2c_dev;
  if (num_ff < NFIREFLIES_F1) {
    i2c_dev = I2C_DEVICE_F1;
  }
  else {
    i2c_dev = I2C_DEVICE_F2;
  }
  int ret = read_ff_register(ff_moni2c_addrs[num_ff].name, regnumber, value, size, i2c_dev);
  return ret;
}

#if defined(REV2) || defined(REV3)
// reset firefly devices. The resets are ganged together,
// so you can only reset all of them at once, for those
// attached to F1 or F2
// I/O expanders are different in Rev2 and Rev3 for optics, see schematic pages 4.05 and 4.06
#define FF_RESET_MUX_ADDR     0x71
#define FF_RESET_MUX_BIT_MASK (0x1 << 6)
#define FF_RESET_IOEXP_ADDR   0x21
#if defined(REV2)
#define FF_RESET_IOEXP_REG_ADDR 0x3        // output port 1
#define FF_RESET_IOEXP_REG_BIT  (0x1 << 0) // bit P10, i.e., bit 0 of output port 1
#elif defined(REV3)
#define FF_RESET_IOEXP_REG_ADDR 0x2        // output port 0
#define FF_RESET_IOEXP_REG_BIT  (0x1 << 7) // bit P07, i.e., bit 7 of output port 0
#endif                                     // REV
BaseType_t ff_reset(int argc, char **argv, char *m)
{
  int copied = 0;
  BaseType_t which_fpga = strtol(argv[1], NULL, 10);
  if (which_fpga != 1 && which_fpga != 2) {
    copied += snprintf(m + copied, SCRATCH_SIZE - copied, "%s: arg must 1 or 2\r\n", argv[0]);
    return pdFALSE;
  }
  // grab semaphore
  // grab the semaphore to ensure unique access to I2C controller
  // otherwise, block its operations indefinitely until it's available
  SemaphoreHandle_t s = i2c4_sem;
  uint8_t i2c_dev = 4;
  if (which_fpga == 2) {
    s = i2c3_sem;
    i2c_dev = 3;
  }
  if (acquireI2CSemaphore(s) == pdFAIL) {
    log_warn(LOG_SERVICE, "could not get semaphore in time\r\n");
    return pdFALSE;
  }
  // select the appropriate output for the mux
  apollo_i2c_ctl_w(i2c_dev, FF_RESET_MUX_ADDR, 1, FF_RESET_MUX_BIT_MASK);
  // read/modify/write the reset register
  uint32_t reset_reg;
  int ret = apollo_i2c_ctl_reg_r(i2c_dev, FF_RESET_IOEXP_ADDR, 1, FF_RESET_IOEXP_REG_ADDR, 1, &reset_reg);
  // set reset bit 0 which is active low
  reset_reg &= ~FF_RESET_IOEXP_REG_BIT;
  ret += apollo_i2c_ctl_reg_w(i2c_dev, FF_RESET_IOEXP_ADDR, 1, FF_RESET_IOEXP_REG_ADDR, 1, reset_reg);
  // wait a tick
  vTaskDelay(pdMS_TO_TICKS(1));
  // clear the active low reset bit
  reset_reg |= FF_RESET_IOEXP_REG_BIT;
  ret += apollo_i2c_ctl_reg_w(i2c_dev, FF_RESET_IOEXP_ADDR, 1, FF_RESET_IOEXP_REG_ADDR, 1, reset_reg);
  // release the semaphore
  if (xSemaphoreGetMutexHolder(s) == xTaskGetCurrentTaskHandle())
    xSemaphoreGive(s);
  if (ret) {
    copied += snprintf(m + copied, SCRATCH_SIZE - copied, "%s: error %d\r\n", argv[0], ret);
  }
  else {
    copied += snprintf(m + copied, SCRATCH_SIZE - copied, "%s: reset complete F%ld\r\n", argv[0], which_fpga);
  }
  return pdFALSE;
}
// reset the muxes for the firefly devices
BaseType_t ff_mux_reset(int argc, char **argv, char *m)
{
  int copied = 0;
  BaseType_t which_fpga = strtol(argv[1], NULL, 10);
  if (which_fpga != 1 && which_fpga != 2) {
    copied += snprintf(m + copied, SCRATCH_SIZE - copied, "%s: arg must 1 or 2\r\n", argv[0]);
    return pdFALSE;
  }
  // grab semaphore
  // grab the semaphore to ensure unique access to I2C controller
  // otherwise, block its operations indefinitely until it's available
  SemaphoreHandle_t s = i2c4_sem;
  if (which_fpga == 2) {
    s = i2c3_sem;
  }
  if (acquireI2CSemaphore(s) == pdFAIL) {
    log_warn(LOG_SERVICE, "could not get semaphore in time\r\n");
    return pdFALSE;
  }
  // select the appropriate output for the mux
  int pin = _F1_OPTICS_I2C_RESET;
  if (which_fpga == 2)
    pin = _F2_OPTICS_I2C_RESET;
  // toggle the pin
  write_gpio_pin(pin, 0); // active low signal
  vTaskDelay(pdMS_TO_TICKS(1));
  write_gpio_pin(pin, 1); // active low signal

  if (xSemaphoreGetMutexHolder(s) == xTaskGetCurrentTaskHandle())
    xSemaphoreGive(s);
  copied += snprintf(m + copied, SCRATCH_SIZE - copied, "%s: mux reset complete F%ld\r\n", argv[0], which_fpga);
  return pdFALSE;
}
#endif // REV2

BaseType_t ff_status(int argc, char **argv, char *m)
{
  // argument handling
  int copied = 0;

  static int whichff = 0;

  if (whichff == 0) {
    // check for stale data
    TickType_t now = pdTICKS_TO_S(xTaskGetTickCount());

    if (isFFStale()) {
      TickType_t last = pdTICKS_TO_S(getFFupdateTick(isFFStale()));
      int mins = (now - last) / 60;
      copied += snprintf(m + copied, SCRATCH_SIZE - copied,
                         "%s: stale data, last update %d minutes ago\r\n", argv[0], mins);
    }
    copied += snprintf(m + copied, SCRATCH_SIZE - copied, "FF STATUS:\r\n");
  }

#if defined(REV2) || defined(REV3)
  int nTx = -1; // order of Tx ch
  // print out the "present" bits on first pass
  if (whichff == 0) {
    copied += snprintf(m + copied, SCRATCH_SIZE - copied, "PRESENT:\r\n");
    char *ff_bitmask_names[4] = {"1_12", "1_4 ", "2_12", "2_4 "};
    for (int i = 0; i < 4; ++i) {
      copied += snprintf(m + copied, SCRATCH_SIZE - copied, "F%s: 0x%02lx\r\n", ff_bitmask_names[i],
                         getFFpresentbit(i));
    }
  }
#endif // REV2
  for (; whichff < NFIREFLIES; ++whichff) {
    if (isEnabledFF(whichff)) {
      uint8_t val = get_FF_STATUS_REG_data(whichff);
      copied += snprintf(m + copied, SCRATCH_SIZE - copied, "%17s: 0x%02x", ff_moni2c_addrs[whichff].name, val);
    }
    else { // dummy value
      copied += snprintf(m + copied, SCRATCH_SIZE - copied, "%17s: %4s", ff_moni2c_addrs[whichff].name, "--");
    }
    bool isTx = (strstr(ff_moni2c_addrs[whichff].name, "Tx") != NULL);
    if (isTx) {
#ifdef REV1
      copied += snprintf(m + copied, SCRATCH_SIZE - copied, "\t");
#elif defined(REV2) || defined(REV3)
      nTx += 1;
      uint8_t ff_4v0_sel = 1 << (nTx % (NFIREFLIES_IT_F1 / 2));
      if (nTx < (NFIREFLIES_IT_F1 / 2))
        ff_4v0_sel &= f1_ff12xmit_4v0_sel;
      else
        ff_4v0_sel &= f2_ff12xmit_4v0_sel;
      copied += snprintf(m + copied, SCRATCH_SIZE - copied, " 3v8?(%x) \t", ff_4v0_sel >> (nTx % (NFIREFLIES_IT_F1 / 2)));
#endif // REV2
    }
    else {
      copied += snprintf(m + copied, SCRATCH_SIZE - copied, "\r\n");
    }
    if ((SCRATCH_SIZE - copied) < 20) {
      ++whichff;
      return pdTRUE;
    }
  }
  if (whichff % 2 == 1) {
    m[copied++] = '\r';
    m[copied++] = '\n';
    m[copied] = '\0';
  }
  whichff = 0;

  return pdFALSE;
}

BaseType_t ff_los_alarm(int argc, char **argv, char *m)
{
  int copied = 0;

  static int whichff = 0;
  // static int n = 0;

  if (whichff == 0) {
    // check for stale data
    if (isFFStale()) {
      TickType_t now = pdTICKS_TO_S(xTaskGetTickCount());
      TickType_t last = pdTICKS_TO_S(getFFupdateTick(isFFStale()));
      int mins = (now - last) / 60;
      copied += snprintf(m + copied, SCRATCH_SIZE - copied,
                         "%s: stale data, last update %d minutes ago\r\n", argv[0], mins);
    }
    copied += snprintf(m + copied, SCRATCH_SIZE - copied, "FIREFLY LOS ALARM:\r\n");
  }

  for (; whichff < NFIREFLIES; ++whichff) {
    uint16_t val = get_FF_LOS_ALARM_data(whichff);
    copied += snprintf(m + copied, SCRATCH_SIZE - copied, "%17s: 0x%04x",
                       ff_moni2c_addrs[whichff].name, val);
    bool isTx = (strstr(ff_moni2c_addrs[whichff].name, "Tx") != NULL);
    if (isTx)
      copied += snprintf(m + copied, SCRATCH_SIZE - copied, "\t");
    else
      copied += snprintf(m + copied, SCRATCH_SIZE - copied, "\r\n");
    if ((SCRATCH_SIZE - copied) < 20) {
      ++whichff;
      return pdTRUE;
    }
  }

  if (whichff % 2 == 1) {
    m[copied++] = '\r';
    m[copied++] = '\n';
    m[copied] = '\0';
  }
  whichff = 0;

  return pdFALSE;
}

BaseType_t ff_ch_disable_status(int argc, char **argv, char *m)
{
  int copied = 0;

  static int whichff = 0;

  if (whichff == 0) {
    // check for stale data
    if (isFFStale()) {
      TickType_t now = pdTICKS_TO_S(xTaskGetTickCount());
      TickType_t last = pdTICKS_TO_S(getFFupdateTick(isFFStale()));
      int mins = (now - last) / 60;
      copied += snprintf(m + copied, SCRATCH_SIZE - copied,
                         "%s: stale data, last update %d minutes ago\r\n", argv[0], mins);
    }
    copied += snprintf(m + copied, SCRATCH_SIZE - copied, "FIREFLY LOS ALARM:\r\n");
  }

  for (; whichff < NFIREFLIES; ++whichff) {
    copied += snprintf(m + copied, SCRATCH_SIZE - copied, "%17s: ",
                       ff_moni2c_addrs[whichff].name);
    if (isEnabledFF(whichff)) {
      uint16_t val = get_FF_CHANNEL_DISABLE_data(whichff);
      copied += snprintf(m + copied, SCRATCH_SIZE - copied, "0x%04x", val);
    }
    else {
      copied += snprintf(m + copied, SCRATCH_SIZE - copied, "  --  ");
    }
    bool isTx = (strstr(ff_moni2c_addrs[whichff].name, "Tx") != NULL);
    if (isTx)
      copied += snprintf(m + copied, SCRATCH_SIZE - copied, "\t");
    else
      copied += snprintf(m + copied, SCRATCH_SIZE - copied, "\r\n");
    if ((SCRATCH_SIZE - copied) < 20) {
      ++whichff;
      return pdTRUE;
    }
  }

  if (whichff % 2 == 1) {
    m[copied++] = '\r';
    m[copied++] = '\n';
    m[copied] = '\0';
  }
  whichff = 0;

  return pdFALSE;
}

BaseType_t ff_cdr_lol_alarm(int argc, char **argv, char *m)
{
  int copied = 0;

  static int whichff = 0;

  if (whichff == 0) {
    // check for stale data
    if (isFFStale()) {
      TickType_t now = pdTICKS_TO_S(xTaskGetTickCount());
      TickType_t last = pdTICKS_TO_S(getFFupdateTick(isFFStale()));
      int mins = (now - last) / 60;
      copied += snprintf(m + copied, SCRATCH_SIZE - copied,
                         "%s: stale data, last update %d minutes ago\r\n", argv[0], mins);
    }
    copied += snprintf(m + copied, SCRATCH_SIZE - copied, "FIREFLY CDR LOL ALARM:\r\n");
  }

  for (; whichff < NFIREFLIES; ++whichff) {
    copied += snprintf(m + copied, SCRATCH_SIZE - copied, "%17s: ",
                       ff_moni2c_addrs[whichff].name);
    if (isEnabledFF(whichff) && (FireflyType(whichff) == DEVICE_25G12 || FireflyType(whichff) == DEVICE_25G4)) {
      uint16_t val = get_FF_CDR_LOL_ALARM_data(whichff);
      copied += snprintf(m + copied, SCRATCH_SIZE - copied, "0x%04x", val);
    }
    else {
      copied += snprintf(m + copied, SCRATCH_SIZE - copied, "  --  ");
    }
    bool isTx = (strstr(ff_moni2c_addrs[whichff].name, "Tx") != NULL);
    if (isTx)
      copied += snprintf(m + copied, SCRATCH_SIZE - copied, "\t");
    else
      copied += snprintf(m + copied, SCRATCH_SIZE - copied, "\r\n");
    if ((SCRATCH_SIZE - copied) < 30) {
      ++whichff;
      return pdTRUE;
    }
  }
  if (whichff % 2 == 1) {
    m[copied++] = '\r';
    m[copied++] = '\n';
    m[copied] = '\0';
  }
  whichff = 0;
  // n = 0;

  return pdFALSE;
}
BaseType_t ff_cdr_enable_status(int argc, char **argv, char *m)
{
  int copied = 0;

  static int whichff = 0;

  if (whichff == 0) {
    // check for stale data
    if (isFFStale()) {
      TickType_t now = pdTICKS_TO_S(xTaskGetTickCount());
      TickType_t last = pdTICKS_TO_S(getFFupdateTick(isFFStale()));
      int mins = (now - last) / 60;
      copied += snprintf(m + copied, SCRATCH_SIZE - copied,
                         "%s: stale data, last update %d minutes ago\r\n", argv[0], mins);
    }
    copied += snprintf(m + copied, SCRATCH_SIZE - copied, "FF CDR Enable:\r\n");
  }

  for (; whichff < NFIREFLIES; ++whichff) {
    uint16_t val = get_FF_CDR_ENABLE_data(whichff);
    copied += snprintf(m + copied, SCRATCH_SIZE - copied, "%17s: 0x%04x",
                       ff_moni2c_addrs[whichff].name, val);
    bool isTx = (strstr(ff_moni2c_addrs[whichff].name, "Tx") != NULL);
    if (isTx)
      copied += snprintf(m + copied, SCRATCH_SIZE - copied, "\t");
    else
      copied += snprintf(m + copied, SCRATCH_SIZE - copied, "\r\n");
    if ((SCRATCH_SIZE - copied) < 20) {
      ++whichff;
      return pdTRUE;
    }
  }

  if (whichff % 2 == 1) {
    m[copied++] = '\r';
    m[copied++] = '\n';
    m[copied] = '\0';
  }
  whichff = 0;

  return pdFALSE;
}

BaseType_t ff_temp(int argc, char **argv, char *m)
{
  // argument handling
  int copied = 0;

  static int nn = 0;

  if (nn == 0) {
    // check for stale data
    TickType_t now = pdTICKS_TO_S(xTaskGetTickCount());

    if (isFFStale()) {
      TickType_t last = pdTICKS_TO_S(getFFupdateTick(isFFStale()));
      int mins = (now - last) / 60;
      copied += snprintf(m + copied, SCRATCH_SIZE - copied,
                         "%s: stale data, last update %d minutes ago\r\n", argv[0], mins);
    }
    copied += snprintf(m + copied, SCRATCH_SIZE - copied, "FF Temperature:\r\n");
  }

  for (; nn < NFIREFLIES; ++nn) {
    if (isEnabledFF(nn)) {
      uint8_t val = get_FF_TEMPERATURE_data(nn);
      copied += snprintf(m + copied, SCRATCH_SIZE - copied, "%17s: %2d", ff_moni2c_addrs[nn].name, val);
    }
    else // dummy value
      copied += snprintf(m + copied, SCRATCH_SIZE - copied, "%17s: %2s", ff_moni2c_addrs[nn].name, "--");
    if ((SCRATCH_SIZE - copied) < 20) {
      //++whichff;
      return pdTRUE;
    }
    bool isTx = (strstr(ff_moni2c_addrs[nn].name, "Tx") != NULL);
    if (isTx)
      copied += snprintf(m + copied, SCRATCH_SIZE - copied, "\t");
    else
      copied += snprintf(m + copied, SCRATCH_SIZE - copied, "\r\n");
  }
  nn = 0;

  if (nn % 2 == 1) {
    m[copied++] = '\r';
    m[copied++] = '\n';
    m[copied] = '\0';
  }

  return pdFALSE;
}

// loop over all channels on all devices and show optical power
BaseType_t ff_optpow(int argc, char **argv, char *m)
{
  // takes no arguments
  static int i = 0;
  int copied = 0;
  if (i == 0) {
    copied += snprintf(m, SCRATCH_SIZE, "FF average Optical Power (uW)\r\n");
  }
  for (; i < NFIREFLIES; ++i) {
    bool isTx = (strstr(ff_moni2c_addrs[i].name, "Tx") != NULL);
    if (isEnabledFF(i) && !isTx) {
      float val = getFFavgoptpow(i);
      int tens, frac;
      float_to_ints(val, &tens, &frac);
      copied += snprintf(m + copied, SCRATCH_SIZE - copied, "%17s: % 5d.%02d",
                         ff_moni2c_addrs[i].name, tens, frac);
    }
    else {
      copied += snprintf(m + copied, SCRATCH_SIZE - copied, "%17s:     ---",
                         ff_moni2c_addrs[i].name);
    }
    if (isTx) {
      copied += snprintf(m + copied, SCRATCH_SIZE - copied, "\t\t");
    }
    else {
      copied += snprintf(m + copied, SCRATCH_SIZE - copied, "\r\n");
    }
    if ((SCRATCH_SIZE - copied) < 40) {
      ++i;
      return pdTRUE;
    }
  } // loop over NFIREFLIES
  i = 0;
  return pdFALSE;
}

// show optical power for all channels in a single device
BaseType_t ff_optpow_dev(int argc, char **argv, char *m)
{
  // takes one argument
  BaseType_t whichFF = strtol(argv[1], NULL, 10);
  int copied = 0;
  if (whichFF >= NFIREFLIES) {
    copied += snprintf(m, SCRATCH_SIZE, "%s: choose ff number less than %d\r\n",
                       argv[0], NFIREFLIES);
    return pdFALSE;
  }
  copied += snprintf(m, SCRATCH_SIZE, "FF %s Optical Power (uW)\r\n", ff_moni2c_addrs[whichFF].name);
  int nchannels = FireflyType(whichFF) == DEVICE_25G4 ? 4 : 12;
  for (int i = 0; i < nchannels; ++i) {
    float val = getFFoptpow(whichFF, i);
    int tens, frac;
    float_to_ints(val, &tens, &frac);
    copied += snprintf(m + copied, SCRATCH_SIZE - copied, "Ch %02d: % 5d.%02d\r\n", i, tens, frac);
  }
  return pdFALSE;
}

// this command takes up to two arguments. control firefly devices
BaseType_t ff_ctl(int argc, char **argv, char *m)
{
  // argument handling
  int copied = 0;

  if (argc == 2) {
    snprintf(m + copied, SCRATCH_SIZE - copied, "%s: %s not understood", argv[0], argv[1]);
    return pdFALSE;
  }
  else {
    int whichFF = 0;
    // handle the channel number first
    if (strncmp(argv[argc - 1], "all", 3) == 0) {
      whichFF = NFIREFLIES;
    }
    else { // commands with arguments. The last argument is always which FF module.
      whichFF = strtol(argv[argc - 1], NULL, 10);
      if (whichFF >= NFIREFLIES || (whichFF == 0 && strncmp(argv[argc - 1], "0", 1) != 0)) {
        snprintf(m + copied, SCRATCH_SIZE - copied, "%s: choose ff number less than %d\r\n",
                 argv[0], NFIREFLIES);
        return pdFALSE;
      }
    }
    // now process various commands.
    if (argc == 4) { // command + three arguments
      int channel = whichFF;
      if (channel > NFIREFLIES)
        channel = NFIREFLIES;
      if (strncmp(argv[1], "cdr", 3) == 0) {
        uint8_t val = 0x00U; // default to off
        if (strncmp(argv[2], "on", 2) == 0) {
          val = 0xFF;
        }
        set_xcvr_cdr(val, channel);
        return pdFALSE;
      }
      else if (strncmp(argv[1], "xmit", 4) == 0) {
        bool disable = true;
        if (strncmp(argv[2], "on", 2) == 0) {
          disable = false;
        }
        disable_transmit(disable, channel);
        return pdFALSE;
      }
      else if (strncmp(argv[1], "rcvr", 4) == 0) {
        bool disable = true;
        if (strncmp(argv[2], "on", 2) == 0) {
          disable = false;
        }
        disable_receivers(disable, channel);
        return pdFALSE;
      }
      // Add here
      else if (strncmp(argv[1], "regr", 4) == 0) {
        if (whichFF == NFIREFLIES) {
          copied += snprintf(m + copied, SCRATCH_SIZE - copied, "%s: cannot read all registers\r\n",
                             argv[0]);
          return pdFALSE;
        }
        // register number
        uint16_t regnum = strtol(argv[2], NULL, 16);
        copied +=
            snprintf(m + copied, SCRATCH_SIZE - copied, "%s: reading FF %s, register 0x%x\r\n",
                     argv[0], ff_moni2c_addrs[whichFF].name, regnum);
        uint8_t value;
        int ret = read_arbitrary_ff_register(regnum, channel, &value, 1);
        if (ret != 0) {
          copied += snprintf(m + copied, SCRATCH_SIZE - copied, "read_ff_reg failed with %d\r\n", ret);
          if (ret == SEM_ACCESS_ERROR) {
            snprintf(m + copied, SCRATCH_SIZE - copied, "please release semaphore \r\n");
          }
          return pdFALSE;
        }
        copied += snprintf(m + copied, SCRATCH_SIZE - copied,
                           "%s: Command returned 0x%x (ret %d - \"%s\").\r\n", argv[0], value,
                           ret, SMBUS_get_error(ret));
      }
      else {
        snprintf(m + copied, SCRATCH_SIZE - copied, "%s: command %s not recognized\r\n", argv[0],
                 argv[1]);
        return pdFALSE;
      }

    }                     // argc == 4
    else if (argc == 5) { // command + five arguments
      // register write. model:
      // ff regw reg# val (0-23|all)
      // register read/write commands
      if (strncmp(argv[1], "regw", 4) == 0) {
        // the two additional arguments
        // register number
        // value to be written
        uint16_t regnum = strtol(argv[2], NULL, 16);
        uint16_t value = strtol(argv[3], NULL, 16);
        uint8_t channel = whichFF;
        if (channel == NFIREFLIES) {
          channel = 0; // silently fall back to first channel
        }
        int ret = write_arbitrary_ff_register(regnum, value, channel);
        if (ret != 0) {
          copied += snprintf(m + copied, SCRATCH_SIZE - copied, "write_ff_reg failed with %d\r\n", ret);
          if (ret == SEM_ACCESS_ERROR) {
            snprintf(m + copied, SCRATCH_SIZE - copied, "please release semaphore \r\n");
          }
          return pdFALSE;
        }
        snprintf(m + copied, SCRATCH_SIZE - copied,
                 "%s: write val 0x%x to register 0x%x, FF %d.\r\n", argv[0], value, regnum,
                 channel);
        whichFF = 0;
        return pdFALSE;
      }
      // end regw
#ifdef FF_TEST_DEBUG
      else if (strncmp(argv[1], "test", 4) == 0) { // test code
        if (whichFF == NFIREFLIES) {
          copied += snprintf(m + copied, SCRATCH_SIZE - copied,
                             "%s: cannot test read all registers\r\n", argv[0]);
          return pdFALSE;
        }
        uint8_t regnum = strtol(argv[2], NULL, 16);   // which register
        uint8_t charsize = strtol(argv[3], NULL, 10); // how big
#define CHARLENGTH 64
        if (charsize > CHARLENGTH) {
          charsize = CHARLENGTH;
        }
        if (whichFF > NFIREFLIES) {
          whichFF = 0;
        }
        char tmp[CHARLENGTH];
        snprintf(tmp, CHARLENGTH, "FF %s (%d)\r\n", ff_moni2c_addrs[whichFF].name, whichFF);
        Print(tmp);
        snprintf(tmp, CHARLENGTH, "Register %d (size %d)\r\n", regnum, charsize);
        Print(tmp);
        uint8_t regdata[CHARLENGTH];
        memset(regdata, 'x', CHARLENGTH);
        regdata[charsize - 1] = '\0';
        int i2c_dev;
        if (whichFF < NFIREFLIES_F1) {
          i2c_dev = I2C_DEVICE_F1;
        }
        else {
          i2c_dev = I2C_DEVICE_F2;
        }
        int ret = read_ff_register(ff_moni2c_addrs[whichFF].name, regnum, &regdata[0], charsize, i2c_dev);
        if (ret != 0) {
          snprintf(tmp, CHARLENGTH, "read_ff_reg failed with %d\r\n", ret);
          Print(tmp);
          return pdFALSE;
        }
        regdata[CHARLENGTH - 1] = '\0'; // Sanity check
        Print((char *)regdata);
        Print("\r\n");
        whichFF = 0;
        return pdFALSE;
      }
      else {
        snprintf(m + copied, SCRATCH_SIZE - copied, "%s: command %s not understood\r\n", argv[0],
                 argv[1]);
        return pdFALSE;
      }
#endif // FF_TEST_DEBUG
    }  // argc == 5
    else {
      snprintf(m + copied, SCRATCH_SIZE - copied, "%s: command %s not understood\r\n", argv[0],
               argv[1]);
      return pdFALSE;
    }
  }
  return pdFALSE;
}

// firefly 3.3V monitor dumper
BaseType_t ff_v3v3(int argc, char **argv, char *m)
{
  // argument handling
  int copied = 0;

  static int nn = 0;

  if (nn == 0) {
    // check for stale data
    TickType_t now = pdTICKS_TO_S(xTaskGetTickCount());

    if (isFFStale()) {
      TickType_t last = pdTICKS_TO_S(getFFupdateTick(isFFStale()));
      int mins = (now - last) / 60;
      copied += snprintf(m + copied, SCRATCH_SIZE - copied,
                         "%s: stale data, last update %d minutes ago\r\n", argv[0], mins);
    }
    copied += snprintf(m + copied, SCRATCH_SIZE - copied, "FF 3V3 Mon:\r\n");
  }

  for (; nn < NFIREFLIES; ++nn) {
    copied += snprintf(m + copied, SCRATCH_SIZE - copied, "%17s: ", ff_moni2c_addrs[nn].name);
    if (isEnabledFF(nn)) {
      float val = (float)__builtin_bswap16(get_FF_VCC3V3_data(nn)) * 100e-6f; // LSB is 100uV
      int tens, frac;
      float_to_ints(val, &tens, &frac);
      copied += snprintf(m + copied, SCRATCH_SIZE - copied, "% 2d.%02d", tens, frac);
    }
    else // dummy value
      copied += snprintf(m + copied, SCRATCH_SIZE - copied, " --- ");
    bool isTx = (strstr(ff_moni2c_addrs[nn].name, "Tx") != NULL);
    if (isTx)
      copied += snprintf(m + copied, SCRATCH_SIZE - copied, "\t");
    else
      copied += snprintf(m + copied, SCRATCH_SIZE - copied, "\r\n");
    if ((SCRATCH_SIZE - copied) < 50) {
      return pdTRUE;
    }
  }
  nn = 0;

  if (nn % 2 == 1) {
    m[copied++] = '\r';
    m[copied++] = '\n';
    m[copied] = '\0';
  }

  return pdFALSE;
}

// this command takes no arguments
BaseType_t ff_dump_names(int argc, char **argv, char *m)
{
  // ensure at compile-time that the vendor count is the same
  static_assert(FF_VENDOR_COUNT_FF12 == FF_VENDOR_COUNT_FFDAQ, "Vendor count mismatch");
  static int i = 0;
  int copied = 0;
  if (i == 0) { // not if we are on 2nd iteration
    copied += snprintf(m, SCRATCH_SIZE, "%s: ID registers\r\n", argv[0]);
  }
  for (; i < NFIREFLIES; ++i) {
    copied += snprintf(m + copied, SCRATCH_SIZE - copied, "%17s: ", ff_moni2c_addrs[i].name);
    if (isEnabledFF(i)) { // process if enabled

      char name[FF_VENDOR_COUNT_FF12 + 1];
      memset(name, '\0', FF_VENDOR_COUNT_FF12 + 1);
      int type = FireflyType(i);
      int startReg = FF_VENDOR_START_BIT_FFDAQ;
      if (type == DEVICE_CERNB || type == DEVICE_25G12) {
        startReg = FF_VENDOR_START_BIT_FF12;
      }
      int ret = 0;
      for (unsigned char c = 0; c < FF_VENDOR_COUNT_FF12 / 4; ++c) { // read name 4 chars at a time
        uint8_t v[4];
        ret += read_arbitrary_ff_register(startReg + 4 * c, i, v, 4);
        name[4 * c] = v[0];
        name[4 * c + 1] = v[1];
        name[4 * c + 2] = v[2];
        name[4 * c + 3] = v[3];
      }
      if (ret != 0) {
        snprintf(m + copied, SCRATCH_SIZE - copied, "%s: read failed\r\n", argv[0]);
        return pdFALSE;
      }
      copied += snprintf(m + copied, SCRATCH_SIZE - copied, "%s", name);
    }
    else {
      copied += snprintf(m + copied, SCRATCH_SIZE - copied, "--------------");
    }
    bool isTx = (strstr(ff_moni2c_addrs[i].name, "Tx") != NULL);
    if (isTx)
      copied += snprintf(m + copied, SCRATCH_SIZE - copied, "\t");
    else
      copied += snprintf(m + copied, SCRATCH_SIZE - copied, "\r\n");
    if ((SCRATCH_SIZE - copied) < 45 && (i < NFIREFLIES)) {
      ++i;
      return pdTRUE;
    }
  }
  i = 0;
  return pdFALSE;
}
