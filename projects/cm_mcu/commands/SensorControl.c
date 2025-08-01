/*
 * SensorControl.c
 *
 *  Created on: Jan 14, 2021
 *      Author: fatimayousuf
 */

#include <strings.h>
#include <sys/_types.h>
#include <assert.h>

#include "AlarmUtilities.h"
#include "FireflyUtils.h"
#include "I2CCommunication.h"
#include "MonI2C_addresses.h"
#include "MonitorTaskI2C.h"
#include "common/log.h"
#include "common/utils.h"
#include "parameters.h"
#include "SensorControl.h"
#include "Semaphore.h"
#include "common/smbus_helper.h"
#include "Tasks.h"
#include "projdefs.h"
#include "MonUtils.h"
#include "common/power_ctl.h"

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

// dump monitor information
BaseType_t psmon_ctl(int argc, char **argv, char *m)
{
  BaseType_t i1 = strtol(argv[1], NULL, 10);

  if (i1 < 0 || i1 >= dcdc_args.n_commands) {
    snprintf(m, SCRATCH_SIZE, "%s: Invalid argument, must be between 0 and %d\r\n", argv[0],
             dcdc_args.n_commands - 1);
    return pdFALSE;
  }
  // update times, in seconds
  TickType_t now = pdTICKS_TO_MS(xTaskGetTickCount()) / 1000;
  TickType_t last = pdTICKS_TO_MS(dcdc_args.updateTick) / 1000;
  int copied = 0;
  if (checkStale(last, now)) {
    int mins = (now - last) / 60;
    copied += snprintf(m + copied, SCRATCH_SIZE - copied,
                       "%s: stale data, last update %d minutes ago\r\n", argv[0], mins);
  }
  copied += snprintf(m + copied, SCRATCH_SIZE - copied, "%s (0x%02x)\r\n",
                     dcdc_args.commands[i1].name, dcdc_args.commands[i1].command);
  for (int ps = 0; ps < dcdc_args.n_devices; ++ps) {
    copied +=
        snprintf(m + copied, SCRATCH_SIZE - copied, "SUPPLY %s\r\n", dcdc_args.devices[ps].name);
    for (int page = 0; page < dcdc_args.n_pages; ++page) {
      float val = dcdc_args.pm_values[ps * (dcdc_args.n_commands * dcdc_args.n_pages) +
                                      page * dcdc_args.n_commands + i1];
      int tens, frac;
      float_to_ints(val, &tens, &frac);
      copied += snprintf(m + copied, SCRATCH_SIZE - copied, "VALUE %02d.%02d\t", tens, frac);
    }
    copied += snprintf(m + copied, SCRATCH_SIZE - copied, "\r\n");
  }

  return pdFALSE;
}

// send power control commands

// power control state names
static const char *power_control_state_names[] = {
#define X(name) #name,
    X_MACRO_PS_STATES
#undef X
};

BaseType_t power_ctl(int argc, char **argv, char *m)
{
  int s = SCRATCH_SIZE;

  uint32_t message;
  if (strncmp(argv[1], "on", 2) == 0) {
    message = PS_ON; // turn on power supply
  }
  else if (strncmp(argv[1], "off", 3) == 0) {
    message = PS_OFF; // turn off power supply
  }
  else if (strncmp(argv[1], "clearfail", 9) == 0) {
    message = PS_ANYFAIL_ALARM_CLEAR;
  }
  else if (strncmp(argv[1], "status", 5) == 0) { // report status to UART
    int copied = 0;
    bool f1_enable = (isFPGAF1_PRESENT());
    bool f2_enable = (isFPGAF2_PRESENT());
    static int i = 0;
    if (i == 0) {
      copied += snprintf(m + copied, SCRATCH_SIZE - copied,
                         "%s:\r\nF1_ENABLE:\t%d\r\n"
                         "F2_ENABLE:\t%d\r\n",
                         argv[0], f1_enable, f2_enable);
      copied += snprintf(m + copied, SCRATCH_SIZE - copied, "State machine state: %s\r\n",
                         getPowerControlStateName(getPowerControlState()));
      copied += snprintf(m + copied, SCRATCH_SIZE - copied, "External Alarm: %d\r\n",
                         getPowerControlExternalAlarmState());
    }
    for (; i < N_PS_OKS; ++i) {
      enum ps_state j = getPSStatus(i);
      const char *c = power_control_state_names[j];

      copied +=
          snprintf(m + copied, SCRATCH_SIZE - copied, "%16s: %s\r\n", oks[i].name, c);
      if ((SCRATCH_SIZE - copied) < 20 && (i < N_PS_OKS)) {
        ++i;
        return pdTRUE;
      }
    }
    i = 0;
    return pdFALSE;
  }
  else if (strncmp(argv[1], "igmask", 6) == 0) { // report ignore mask to UART
    int copied = 0;
    static int i = 0;
    uint16_t ignore_mask = getPowerControlIgnoreMask();
    for (; i < N_PS_OKS; ++i) {
      BaseType_t ignored = (ignore_mask & (0x1U << i)) != 0;
      copied += snprintf(m + copied, SCRATCH_SIZE - copied, "%-16s: %ld\r\n", oks[i].name, ignored);
      if ((SCRATCH_SIZE - copied) < 20 && (i < N_PS_OKS)) {
        ++i;
        return pdTRUE;
      }
    }
    i = 0;
    return pdFALSE;
  }
  else {
    snprintf(m, s, "power_ctl: invalid argument %s received\r\n", argv[1]);
    return pdFALSE;
  }
  // Send a message to the power supply task, if needed
  xQueueSendToBack(xPwrQueue, &message, pdMS_TO_TICKS(10));
  m[0] = '\0'; // no output from this command

  return pdFALSE;
}

// takes 1-2 arguments
BaseType_t alarm_ctl(int argc, char **argv, char *m)
{
  int s = SCRATCH_SIZE;
  if (argc < 2) {
    snprintf(m, s, "%s: need one or more arguments\r\n", argv[0]);
    return pdFALSE;
  }

  uint32_t message;
  if (strncmp(argv[1], "clear", 4) == 0) {
    message = ALM_CLEAR_ALL; // clear all alarms
    xQueueSendToBack(tempAlarmTask.xAlmQueue, &message, pdMS_TO_TICKS(10));
    xQueueSendToBack(voltAlarmTask.xAlmQueue, &message, pdMS_TO_TICKS(10));
    m[0] = '\0'; // no output from this command

    return pdFALSE;
  }
  else if (strncmp(argv[1], "status", 5) == 0) { // report status to UART
    int copied = 0;
    copied += snprintf(m + copied, SCRATCH_SIZE - copied, "%s: TALARM status\r\n", argv[0]);
    uint32_t stat = getTempAlarmStatus();
    copied += snprintf(m + copied, SCRATCH_SIZE - copied, "Raw: 0x%08lx\r\n", stat);

    float ff_val = getAlarmTemperature(FF);
    int tens, frac;
    float_to_ints(ff_val, &tens, &frac);
    copied +=
        snprintf(m + copied, SCRATCH_SIZE - copied, "TEMP FFLY: %s \t Threshold: %02d.%02d\r\n",
                 (stat & ALM_STAT_FIREFLY_OVERTEMP) ? "ALARM" : "GOOD", tens, frac);

    float fpga_val = getAlarmTemperature(FPGA);
    float_to_ints(fpga_val, &tens, &frac);
    copied +=
        snprintf(m + copied, SCRATCH_SIZE - copied, "TEMP FPGA: %s \t Threshold: %02d.%02d\r\n",
                 (stat & ALM_STAT_FPGA_OVERTEMP) ? "ALARM" : "GOOD", tens, frac);

    float dcdc_val = getAlarmTemperature(DCDC);
    float_to_ints(dcdc_val, &tens, &frac);
    copied +=
        snprintf(m + copied, SCRATCH_SIZE - copied, "TEMP DCDC: %s \t Threshold: %02d.%02d\r\n",
                 (stat & ALM_STAT_DCDC_OVERTEMP) ? "ALARM" : "GOOD", tens, frac);

    float tm4c_val = getAlarmTemperature(TM4C);
    float_to_ints(tm4c_val, &tens, &frac);
    copied +=
        snprintf(m + copied, SCRATCH_SIZE - copied, "TEMP TM4C: %s \t Threshold: %02d.%02d\r\n",
                 (stat & ALM_STAT_TM4C_OVERTEMP) ? "ALARM" : "GOOD", tens, frac);

    uint32_t adc_volt_stat = getVoltAlarmStatus();
    float voltthres = getAlarmVoltageThres() * 100;
    float_to_ints(voltthres, &tens, &frac);
    copied +=
        snprintf(m + copied, SCRATCH_SIZE - copied, "VOLT ADC: %s (for FPGAs) \t Threshold: +/-%02d.%02d %%\r\n",
                 (adc_volt_stat) ? "ALARM" : "GOOD", tens, frac);

    configASSERT(copied < SCRATCH_SIZE);

    return pdFALSE;
  }
  else if (strcmp(argv[1], "settemp") == 0) {
    if (argc != 4) {
      snprintf(m, s, "Invalid command\r\n");
      return pdFALSE;
    }
    float newtemp = (float)strtol(argv[3], NULL, 10);
    char *device = argv[2];
    if (!strncasecmp(device, "ff", 2)) {
      setAlarmTemperature(FF, newtemp);
      snprintf(m, s, "%s: set Firefly alarm temperature to %s\r\n", argv[0], argv[3]);
      return pdFALSE;
    }
    if (!strncasecmp(device, "fpga", 4)) {
      setAlarmTemperature(FPGA, newtemp);
      snprintf(m, s, "%s: set FPGA alarm temperature to %s\r\n", argv[0], argv[3]);
      return pdFALSE;
    }
    if (!strncasecmp(device, "dcdc", 4)) {
      setAlarmTemperature(DCDC, newtemp);
      snprintf(m, s, "%s: set DCDC alarm temperature to %s\r\n", argv[0], argv[3]);
      return pdFALSE;
    }
    if (!strncasecmp(device, "tm4c", 4)) {
      setAlarmTemperature(TM4C, newtemp);
      snprintf(m, s, "%s: set TM4C alarm temperature to %s\r\n", argv[0], argv[3]);
      return pdFALSE;
    }
    else {
      snprintf(m, s, "%s is not a valid device.\r\n", argv[2]);
      return pdFALSE;
    }
  }
  else if (strcmp(argv[1], "setvoltthres") == 0) {
    if (argc != 3) {
      snprintf(m, s, "Invalid command\r\n");
      return pdFALSE;
    }
    float voltthres = (float)strtol(argv[2], NULL, 10);
    setAlarmVoltageThres(voltthres);
    snprintf(m, s, "alarm voltages are set their threshold by +/-%s %% \r\n", argv[2]);
    return pdFALSE;
  }
  else {
    snprintf(m, s, "%s: invalid argument %s received\r\n", argv[0], argv[1]);
    return pdFALSE;
  }
  return pdFALSE;
}

// send LED commands
BaseType_t led_ctl(int argc, char **argv, char *m)
{

  BaseType_t i1 = strtol(argv[1], NULL, 10);

  BaseType_t ones = i1 % 10;
  BaseType_t tens = i1 / 10; // integer truncation

  uint32_t message = HUH; // default: message not understood
  if (ones < 5 && tens > 0 && tens < 4) {
    message = i1;
  }
  // Send a message to the LED task
  xQueueSendToBack(xLedQueue, &message, pdMS_TO_TICKS(10));
  m[0] = '\0'; // no output from this command

  return pdFALSE;
}

// this command takes no arguments
BaseType_t adc_ctl(int argc, char **argv, char *m)
{
  int copied = 0;

  static int whichadc = 0;
  if (whichadc == 0) {
    copied += snprintf(m + copied, SCRATCH_SIZE - copied, "ADC outputs\r\n");
  }
  for (; whichadc < 21; ++whichadc) {
    float val = getADCvalue(whichadc);
    int tens, frac;
    float_to_ints(val, &tens, &frac);
    copied += snprintf(m + copied, SCRATCH_SIZE - copied, "%14s: %02d.%02d\r\n",
                       getADCname(whichadc), tens, frac);
    if ((SCRATCH_SIZE - copied) < 20 && (whichadc < 20)) {
      ++whichadc;
      return pdTRUE;
    }
  }
  whichadc = 0;
  return pdFALSE;
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
      }                                            // end regw
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
    } // argc == 5
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

// dump clock monitor information
BaseType_t clkmon_ctl(int argc, char **argv, char *m)
{
  int copied = 0;
  static int c = 0;
  BaseType_t i = strtol(argv[1], NULL, 10);

  if (i < 0 || i > 4) {
    snprintf(m, SCRATCH_SIZE, "%s: Invalid argument %s\r\n", argv[0], argv[1]);
    return pdFALSE;
  }
  // print out header once
  if (c == 0) {
    const char *clk_ids[5] = {"0A", "0B", "1A", "1B", "1C"};
    copied += snprintf(m + copied, SCRATCH_SIZE - copied, "Clock R%s\r\n",
                       clk_ids[i]);
    char *header = "REG_TABLE";
    copied += snprintf(m + copied, SCRATCH_SIZE - copied, "%-15s REG_ADDR BIT_MASK  VALUE \r\n", header);
  }
  // update times, in seconds
  TickType_t now = pdTICKS_TO_MS(xTaskGetTickCount()) / 1000;
  TickType_t last = pdTICKS_TO_MS(clk_args.updateTick) / 1000;

  if (checkStale(last, now)) {
    unsigned mins = (now - last) / 60;
    copied += snprintf(m + copied, SCRATCH_SIZE - copied,
                       "%s: stale data, last update %u minutes ago\r\n", argv[0], mins);
  }

  for (; c < clk_args.n_commands; ++c) {
    // check if device i has this command
    if (!(clk_args.commands[c].devicelist() & ClockType(i))) {
      continue;
    }
    uint16_t val = clk_args.commands[c].retrieveData(i);
    int devtype = 31 - __builtin_clz(ClockType(i));
    copied += snprintf(m + copied, SCRATCH_SIZE - copied, "%-15s : 0x%04x   0x%04x    0x%04x\r\n",
                       clk_args.commands[c].name, clk_args.commands[c].command[devtype],
                       clk_args.commands[c].bit_mask, val);
    if ((SCRATCH_SIZE - copied) < 20) {
      ++c;
      return pdTRUE;
    }
  }
  if (c % 2 == 1) {
    m[copied++] = '\r';
    m[copied++] = '\n';
    m[copied] = '\0';
  }
  c = 0;
#if defined(REV2) || defined(REV3)
  copied += snprintf(m + copied, SCRATCH_SIZE - copied, "Program (read from clock chip): %s", clkprog_args[i].progname_clkdesgid);
  if (strncmp(clkprog_args[i].progname_clkdesgid, "5395ABP1", 3) == 0 || strncmp(clkprog_args[i].progname_clkdesgid, "5341ABP1", 3) == 0) {
    copied += snprintf(m + copied, SCRATCH_SIZE - copied, " (not found)");
  }

  snprintf(m + copied, SCRATCH_SIZE - copied, "\r\nProgram (read from eeprom): %s\r\n", clkprog_args[i].progname_eeprom);
#endif

  return pdFALSE;
}

// read out clock frequency measurements via special i2c port on FPGAs.
// implemented in production test FPGA bit file.
BaseType_t clk_freq_fpga_cmd(int argc, char **argv, char *m)
{
  int copied = 0;
  // check if we are looking for FPGA1 or two based on the command argument
  int fpga = strtol(argv[1], NULL, 10) - 1;
  if (fpga < 0 || fpga > 1) {
    snprintf(m, SCRATCH_SIZE, "FPGA should be 1 or 2 (got %s)\r\n", argv[1]);
    return pdFALSE;
  }

  char *names[] = {
      "rw0", "rw1", "clk_200_ext", "lhc_clk", "tcds40_clk", "rt_x4_r0_clk",
      "rt_x12_r0_clk", "lf_x4_r0_clk", "lf_x12_r0_clk", "clk_100", "clk_325",
      "rt_r0_p", "rt_r0_n", "rt_r0_l", "rt_r0_i", "rt_r0_g", "rt_r0_e", "rt_r0_b",
      "lf_r0_y", "lf_r0_w", "lf_r0_u", "lf_r0_r", "lf_r0_af", "lf_r0_ad", "lf_r0_ab",
      "rt_r1_p", "rt_r1_n", "rt_r1_l", "rt_r1_i", "rt_r1_g", "rt_r1_e", "rt_r1_b",
      "lf_r1_y", "lf_r1_w", "lf_r1_u", "lf_r1_r", "lf_r1_af", "lf_r1_ad", "lf_r1_ab"};

  int name_size = sizeof(names) / sizeof(names[0]);

  const int EXPECTED_FREQ[] = {
      0, 0, 200000000, 40000000, 55000000, 140000000, 320000000,
      130000000, 260000000, 100000000, 325000000, 140000000, 320000000,
      200000000, 320000000, 140000000, 320000000, 320000000, 260000000,
      130000000, 260000000, 260000000, 130000000, 260000000, 40000000,
      226000000, 340000000, 136000000, 174000000, 232000000, 348000000,
      310000000, 134000000, 336000000, 326000000, 268000000, 156000000,
      148000000, 110000000};

  const float TOLERANCE = .001f;

  float actual_freq[name_size];
  char *matches[name_size];

  SemaphoreHandle_t s = getSemaphore(5);
  static int i = 0;
  if (i == 0) {
    if (acquireI2CSemaphoreTime(s, 1) != pdTRUE) {
      snprintf(m, SCRATCH_SIZE, "Failed to acquire I2C semaphore\r\n");
      return pdFALSE;
    }

    unsigned mux_val = 0x4; // default to F1
    if (fpga == 1) {
      mux_val = 0x1;
    }
    int r = apollo_i2c_ctl_w(5, 0x70, 1, mux_val);
    if (r != 0) {
      snprintf(m, SCRATCH_SIZE, "Failed to set mux (%d, %s)\r\n", r, SMBUS_get_error(r));
      xSemaphoreGive(s);
      return pdFALSE;
    }
  }

  const int NUM_REGISTERS = 39;

  for (; i < NUM_REGISTERS; ++i) {
    uint32_t data;
    uint16_t reg_addr = i << 2;
    int r = apollo_i2c_ctl_reg_r(5, 0x2b, 1, reg_addr, 4, &data);
    if (r != 0) {
      snprintf(m + copied, SCRATCH_SIZE - copied, "Failed to read FPGA registers %d (%s)\r\n",
               i, SMBUS_get_error(r));
      xSemaphoreGive(s);
      i = 0;
      return pdFALSE;
    }

    actual_freq[i] = data;
    float diff = ABS(EXPECTED_FREQ[i] - actual_freq[i]);
    if ((EXPECTED_FREQ[i] + actual_freq[i]) != 0) {
      diff = diff / ((EXPECTED_FREQ[i] + actual_freq[i]) / 2.0f);
    }

    if (i < 2) {
      matches[i] = "N/A";
    }
    else if (diff < TOLERANCE || (EXPECTED_FREQ[i] + actual_freq[i]) == 0.0f) {
      matches[i] = "MATCH";
    }
    else {
      matches[i] = "NON-MATCH";
    }

    copied += snprintf(m + copied, SCRATCH_SIZE - copied,
                       "F%d r%02d: % 10d (0x%08x) %s %s\r\n ",
                       fpga + 1, i, data, data, names[i], matches[i]);

    if ((SCRATCH_SIZE - copied) < 50) {
      ++i;
      return pdTRUE;
    }
  }

  int r = apollo_i2c_ctl_w(5, 0x70, 1, 0);
  if (r != 0) {
    snprintf(m + copied, SCRATCH_SIZE - copied,
             "Failed to clear mux %s\r\n", SMBUS_get_error(r));
  }

  xSemaphoreGive(s);
  i = 0;
  return pdFALSE;
}

// read clock program names from clock chips
BaseType_t clk_prog_name(int argc, char **argv, char *m)
{
  // argument is which clock chip to read. Should be in range 0-4.
  int i = strtol(argv[1], NULL, 10);
  if (i < 0 || i > 4) {
    snprintf(m, SCRATCH_SIZE, "Clock chip should be in range 0-4 (got %s)\r\n", argv[1]);
    return pdFALSE;
  }
  char from_chip[10];
  char from_eeprom[10];
  char *names[5] = {"0A", "0B", "1A", "1B", "1C"};
  SemaphoreHandle_t s = getSemaphore(2);
  if (acquireI2CSemaphoreTime(s, 1) != pdTRUE) {
    snprintf(m, SCRATCH_SIZE, "Failed to acquire I2C semaphore\r\n");
    return pdFALSE;
  }
  getClockProgram(i, from_chip, from_eeprom);
  xSemaphoreGive(s);

  snprintf(m, SCRATCH_SIZE, "CLK%d (R%s): %s %s\r\n", i, names[i], from_chip, from_eeprom);
  return pdFALSE;
}

BaseType_t fpga_ctl(int argc, char **argv, char *m)
{
  if (argc == 2) {
    if (strncmp(argv[1], "done", 4) == 0) { // print out value of done pins
      int f1_done_ = read_gpio_pin(_F1_FPGA_DONE);
      int f2_done_ = read_gpio_pin(_F2_FPGA_DONE);
      snprintf(m, SCRATCH_SIZE, "F1_DONE* = %d\r\nF2_DONE* = %d\r\n", f1_done_, f2_done_);
      return pdFALSE;
    }
    else {
      snprintf(m, SCRATCH_SIZE, "%s: invalid command %s\r\n", argv[0], argv[1]);
      return pdFALSE;
    }
  }
  else if (argc != 1) {
    // error, invalid
    snprintf(m, SCRATCH_SIZE, "%s: invalid argument count %d\r\n", argv[0], argc);
    return pdFALSE;
  }
  else {
    int copied = 0;
    static int whichfpga = 0;
    int howmany = fpga_args.n_devices * fpga_args.n_pages;
    if (whichfpga == 0) {
      TickType_t now = pdTICKS_TO_S(xTaskGetTickCount());

      if (isFFStale()) {
        TickType_t last = pdTICKS_TO_S(getFFupdateTick(isFFStale()));
        int mins = (now - last) / 60;
        copied += snprintf(m + copied, SCRATCH_SIZE - copied,
                           "%s: stale data, last update %d minutes ago\r\n", argv[0], mins);
      }

      copied += snprintf(m + copied, SCRATCH_SIZE - copied, "FPGA monitors\r\n");
      copied += snprintf(m + copied, SCRATCH_SIZE - copied, "%s\r\n", fpga_args.commands[0].name);
    }

    for (; whichfpga < howmany; ++whichfpga) {
      float val = fpga_args.pm_values[whichfpga];
      int tens, frac;
      float_to_ints(val, &tens, &frac);

      copied += snprintf(m + copied, SCRATCH_SIZE - copied, "%5s: %02d.%02d",
                         fpga_args.devices[whichfpga].name, tens, frac);
      if (whichfpga % 2 == 1)
        copied += snprintf(m + copied, SCRATCH_SIZE - copied, "\r\n");
      else
        copied += snprintf(m + copied, SCRATCH_SIZE - copied, "\t");
      if ((SCRATCH_SIZE - copied) < 20) {
        ++whichfpga;
        return pdTRUE;
      }
    }
    if (whichfpga % 2 == 1) {
      m[copied++] = '\r';
      m[copied++] = '\n';
      m[copied] = '\0';
    }
    whichfpga = 0;
    return pdFALSE;
  }
}

// This command takes 1 argument, either f1 or f2
#if defined(REV2) || defined(REV3)
BaseType_t fpga_flash(int argc, char **argv, char *m)
{
  const TickType_t kDELAY = 1 / portTICK_PERIOD_MS; // 1 ms delay
  char *which = NULL;
  if (strcmp(argv[1], "f2") == 0) {
    write_gpio_pin(FPGA_CFG_FROM_FLASH, 0x1);
    write_gpio_pin(F2_FPGA_PROGRAM, 0x0);
    vTaskDelay(kDELAY);
    write_gpio_pin(F2_FPGA_PROGRAM, 0x1);
    vTaskDelay(kDELAY);
    write_gpio_pin(F2_FPGA_PROGRAM, 0x0);
    which = "F2";
  }
  if (strcmp(argv[1], "f1") == 0) {
    write_gpio_pin(FPGA_CFG_FROM_FLASH, 0x1);
    write_gpio_pin(F1_FPGA_PROGRAM, 0x0);
    vTaskDelay(kDELAY);
    write_gpio_pin(F1_FPGA_PROGRAM, 0x1);
    vTaskDelay(kDELAY);
    write_gpio_pin(F1_FPGA_PROGRAM, 0x0);
    which = "F1";
  }
  snprintf(m, SCRATCH_SIZE, "%s programmed via flash\r\n", which);
  return pdFALSE;
}
#endif

// This command takes 1 argument, either f1 or f2
BaseType_t fpga_reset(int argc, char **argv, char *m)
{
  const TickType_t delay = 1 / portTICK_PERIOD_MS; // 1 ms delay
  char *which = NULL;
  if (strcmp(argv[1], "f2") == 0) {
    write_gpio_pin(F2_FPGA_PROGRAM, 0x1);
    vTaskDelay(delay);
    write_gpio_pin(F2_FPGA_PROGRAM, 0x0);
    which = "F2";
  }
  if (strcmp(argv[1], "f1") == 0) {
    write_gpio_pin(F1_FPGA_PROGRAM, 0x1);
    vTaskDelay(delay);
    write_gpio_pin(F1_FPGA_PROGRAM, 0x0);
    which = "F1";
  }
  snprintf(m, SCRATCH_SIZE, "%s has been reset\r\n", which);
  return pdFALSE;
}

extern struct MonitorTaskArgs_t dcdc_args;
extern struct dev_i2c_addr_t pm_addrs_dcdc[N_PM_ADDRS_DCDC];
extern struct pm_command_t extra_cmds[N_EXTRA_CMDS]; // LocalTasks.c

// Read out registers from LGA80D
BaseType_t psmon_reg(int argc, char **argv, char *m)
{
  int copied = 0;
  int page = strtol(argv[1], NULL, 10); // which supply within the LGA08D
  int which = page / 10;
  page = page % 10;
  if (page < 0 || page > 1) {
    copied += snprintf(m + copied, SCRATCH_SIZE - copied, "%s: page %d must be between 0-1\r\n",
                       argv[0], page);
    return pdFALSE;
  }
  if (which < 0 || which > (NSUPPLIES_PS - 1)) {
    copied += snprintf(m + copied, SCRATCH_SIZE - copied, "%s: device %d must be between 0-%d\r\n",
                       argv[0], which, (NSUPPLIES_PS - 1));
    return pdFALSE;
  }
  UBaseType_t regAddress = strtoul(argv[2], NULL, 16);
  copied += snprintf(m + copied, SCRATCH_SIZE - copied, "%s: page %d of device %s, reg 0x%02lx\r\n", argv[0],
                     page, pm_addrs_dcdc[which].name, regAddress);

  // acquire the semaphore
  if (acquireI2CSemaphore(dcdc_args.xSem) == pdFAIL) {
    snprintf(m + copied, SCRATCH_SIZE - copied, "%s: could not get semaphore in time\r\n", argv[0]);
    return pdFALSE;
  }
  uint8_t ui8page = page;
  // page register
  int r = apollo_pmbus_rw(&g_sMaster1, &eStatus1, false, &pm_addrs_dcdc[which], &extra_cmds[0], &ui8page);
  if (r) {
    Print("error in psmon_reg (page)\r\n");
  }
  // read register, 2 bytes
  uint8_t thevalue[2] = {0, 0};
  struct pm_command_t thecmd = {regAddress, 2, "dummy", "", PM_STATUS};
  r = apollo_pmbus_rw(&g_sMaster1, &eStatus1, true, &pm_addrs_dcdc[which], &thecmd, thevalue);
  if (r) {
    Print("error in psmon_reg (regr)\r\n");
  }
  uint16_t vv = (thevalue[0] | (thevalue[1] << 8));
  copied += snprintf(m + copied, SCRATCH_SIZE - copied, "%s: read value 0x%04x\r\n",
                     argv[0], vv);

  // release the semaphore
  if (xSemaphoreGetMutexHolder(dcdc_args.xSem) == xTaskGetCurrentTaskHandle()) {
    xSemaphoreGive(dcdc_args.xSem);
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
