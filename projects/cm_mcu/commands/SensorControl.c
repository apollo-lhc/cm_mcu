/*
 * SensorControl.c
 *
 *  Created on: Jan 14, 2021
 *      Author: fatimayousuf
 */

#include <strings.h>
#include "parameters.h"
#include "SensorControl.h"
#include "common/smbus_helper.h"

// Register definitions
// -------------------------------------------------
// 8 bit 2's complement signed int, valid from 0-80 C, LSB is 1 deg C
// Same address for 4 XCVR and 12 Tx/Rx devices

// two bytes, 12 FF to be disabled
#define ECU0_14G_TX_DISABLE_REG 0x34U
// one byte, 4 FF to be enabled/disabled (only 4 LSB are used)
#define ECU0_25G_XVCR_TX_DISABLE_REG 0x56U
// two bytes, 12 FF to be disabled
#define ECU0_14G_RX_DISABLE_REG 0x34U
// one byte, 4 FF to be enabled/disabled (only 4 LSB are used)
#define ECU0_25G_XVCR_RX_DISABLE_REG 0x35U
// one byte, 4 FF to be enabled/disabled (4 LSB are Rx, 4 LSB are Tx)
#define ECU0_25G_XVCR_CDR_REG 0x62U
// two bytes, 12 FF to be enabled/disabled. The byte layout
// is a bit weird -- 0-3 on byte 4a, 4-11 on byte 4b
#define ECU0_25G_TXRX_CDR_REG 0x4AU

static int read_ff_register(const char *name, uint16_t packed_reg_addr, uint8_t *value, size_t size, int i2c_device)
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
  SemaphoreHandle_t s = ffldaq_f1_args.xSem;
  if (i2c_device == I2C_DEVICE_F2) {
    s = ffldaq_f2_args.xSem;
  }
  while (xSemaphoreTake(s, (TickType_t)10) == pdFALSE)
    ;

  // write to the mux
  // select the appropriate output for the mux
  uint8_t muxmask = 0x1U << ff_moni2c_addrs[ff].mux_bit;
  res = apollo_i2c_ctl_w(i2c_device, ff_moni2c_addrs[ff].mux_addr, 1, muxmask);
  if (res != 0) {
    log_warn(LOG_SERVICE, "%s: Mux writing error %d (%s) (ff=%s) ...\r\n", __func__, res,
             SMBUS_get_error(res), ff_moni2c_addrs[ff].name);
  }

  if (!res) {
    // Read from register.
    uint32_t uidata;
    res = apollo_i2c_ctl_reg_r(i2c_device, ff_moni2c_addrs[ff].dev_addr, 1,
                               packed_reg_addr, size, &uidata);
    for (int i = 0; i < size; ++i) {
      value[i] = (uint8_t)((uidata >> (i * 8)) & 0xFFU);
    }
    if (res != 0) {
      log_warn(LOG_SERVICE, "%s: FF Regread error %d (%s) (ff=%s) ...\r\n", __func__, res,
               SMBUS_get_error(res), ff_moni2c_addrs[ff].name);
    }
  }

  xSemaphoreGive(s);
  return res;
}

static int write_ff_register(const char *name, uint8_t reg, uint16_t value, int size, int i2c_device)
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
  SemaphoreHandle_t s = ffldaq_f1_args.xSem;
  if (i2c_device == I2C_DEVICE_F2) {
    s = ffldaq_f2_args.xSem;
  }
  while (xSemaphoreTake(s, (TickType_t)10) == pdFALSE)
    ;

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
    res = apollo_i2c_ctl_reg_w(i2c_device, ff_moni2c_addrs[ff].dev_addr, 1, reg, size, (uint32_t)value);
    if (res != 0) {
      log_warn(LOG_SERVICE, "%s: FF writing error %d (%s) (ff=%s) ...\r\n", __func__, res,
               SMBUS_get_error(res), ff_moni2c_addrs[ff].name);
    }
  }

  xSemaphoreGive(s);
  return res;
}

static int disable_transmit(bool disable, int num_ff)
{
  int ret = 0, i = num_ff, imax = num_ff + 1;
  // i and imax are used as limits for the loop below. By default, only iterate once, with i=num_ff.
  uint16_t value = 0x3ff;
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
      ret += write_ff_register(ff_moni2c_addrs[i].name, ECU0_25G_XVCR_TX_DISABLE_REG, value, 1, i2c_dev);
    }
    else if (strstr(ff_moni2c_addrs[i].name, "Tx") != NULL) {
      ret += write_ff_register(ff_moni2c_addrs[i].name, ECU0_14G_TX_DISABLE_REG, value, 2, i2c_dev);
    }
  }
  return ret;
}

static int disable_receivers(bool disable, int num_ff)
{
  int ret = 0, i = num_ff, imax = num_ff + 1;
  // i and imax are used as limits for the loop below. By default, only iterate once, with i=num_ff.
  uint16_t value = 0x3ff;
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
      ret += write_ff_register(ff_moni2c_addrs[i].name, ECU0_25G_XVCR_RX_DISABLE_REG, value, 1, i2c_dev);
    }
    else if (strstr(ff_moni2c_addrs[i].name, "Rx") != NULL) {
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

    if (!isEnabledFF(i)) { // skip the FF if it's not enabled via the FF config
      log_warn(LOG_SERVICE, "Skip writing to disabled FF %d\r\n", i);
      continue;
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
  }
  return ret;
}

// read a SINGLE firefly register, one byte only
static uint16_t read_arbitrary_ff_register(uint16_t regnumber, int num_ff, uint8_t *value, uint8_t size)
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
  int ret = read_ff_register(ff_moni2c_addrs[num_ff].name, regnumber, value, 1, i2c_dev);
  return ret;
}

// this command takes no arguments since there is only one command
// right now.
BaseType_t sensor_summary(int argc, char **argv, char *m)
{
  int copied = 0;
  // collect all sensor information
  // highest temperature for each
  // Firefly
  // FPGA
  // DCDC
  // TM4C
  float tm4c_temp = getADCvalue(ADC_INFO_TEMP_ENTRY);
  int tens, frac;
  float_to_ints(tm4c_temp, &tens, &frac);
  copied += snprintf(m + copied, SCRATCH_SIZE - copied, "MCU %02d.%02d\r\n", tens, frac);
  // Fireflies. These are reported as ints but we are asked
  // to report a float.
  int8_t imax_temp = -99;
  for (int i = 0; i < NFIREFLIES; ++i) {
    int8_t v = getFFtemp(i);
    if (v > imax_temp)
      imax_temp = v;
  }
  copied += snprintf(m + copied, SCRATCH_SIZE - copied, "FIREFLY %02d.0\r\n", imax_temp);
  // FPGAs.
  float max_fpga;
  if (fpga_args.n_devices == 2)
    max_fpga = MAX(fpga_args.pm_values[0], fpga_args.pm_values[1]);
  else
    max_fpga = fpga_args.pm_values[0];
  float_to_ints(max_fpga, &tens, &frac);
  copied += snprintf(m + copied, SCRATCH_SIZE - copied, "FPGA %02d.%02d\r\n", tens, frac);

  // DCDC. The first command is READ_TEMPERATURE_1.
  // I am assuming it stays that way!!!!!!!!
  float max_temp = -99.0f;
  for (int ps = 0; ps < dcdc_args.n_devices; ++ps) {
    for (int page = 0; page < dcdc_args.n_pages; ++page) {
      float thistemp = dcdc_args.pm_values[ps * (dcdc_args.n_commands * dcdc_args.n_pages) +
                                           page * dcdc_args.n_commands + 0];
      if (thistemp > max_temp)
        max_temp = thistemp;
    }
  }
  float_to_ints(max_temp, &tens, &frac);
  copied += snprintf(m + copied, SCRATCH_SIZE - copied, "REG %02d.%02d\r\n", tens, frac);

  return pdFALSE;
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
extern struct gpio_pin_t oks[N_PS_OKS];
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
      char *c;
      switch (j) {
        case PWR_UNKNOWN:
          c = "PWR_UNKNOWN";
          break;
        case PWR_ON:
          c = "PWR_ON";
          break;
        case PWR_OFF:
          c = "PWR_OFF";
          break;
        case PWR_DISABLED:
          c = "PWR_DISABLED";
          break;
        case PWR_FAILED:
          c = "PWR_FAILED";
          break;
        default:
          c = "UNKNOWN";
          break;
      }

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
    m[0] = '\0'; // no output from this command

    return pdFALSE;
  }
  else if (strncmp(argv[1], "status", 5) == 0) { // report status to UART
    int copied = 0;
    copied += snprintf(m + copied, SCRATCH_SIZE - copied, "%s: ALARM status\r\n", argv[0]);
    uint32_t stat = getAlarmStatus();
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

BaseType_t ff_status(int argc, char **argv, char *m)
{
  // argument handling
  int copied = 0;

  static int whichff = 0;
  static int n = 0;

  if (whichff == 0) {
    // check for stale data
    TickType_t now = pdTICKS_TO_MS(xTaskGetTickCount()) / 1000;

    if (getFFcheckStale() == 0) {
      TickType_t last = pdTICKS_TO_MS(getFFupdateTick(getFFcheckStale())) / 1000;
      int mins = (now - last) / 60;
      copied += snprintf(m + copied, SCRATCH_SIZE - copied,
                         "%s: stale data, last update %d minutes ago\r\n", argv[0], mins);
    }
    copied += snprintf(m + copied, SCRATCH_SIZE - copied, "FF STATUS:\r\n");
  }

  int i1 = 0; // 0 for status

  for (; n < NFIREFLY_ARG; ++n) {
    struct MonitorI2CTaskArgs_t *ff_arg = ff_moni2c_arg[n].arg;
    for (; whichff < ff_moni2c_arg[n].int_idx + ff_moni2c_arg[n].num_dev; ++whichff) {
      int dev = whichff - ff_moni2c_arg[n].int_idx + ff_moni2c_arg[n].dev_int_idx;
      if (isEnabledFF(ff_moni2c_arg[n].int_idx + dev)) {
        int index = dev * (ff_arg->n_commands * ff_arg->n_pages) + i1;
        uint8_t val = ffldaq_f1_args.sm_values[index];
        copied += snprintf(m + copied, SCRATCH_SIZE - copied, "%17s: %02d", ff_moni2c_addrs[whichff].name, val);
      }
      else // dummy value
        copied += snprintf(m + copied, SCRATCH_SIZE - copied, "%17s: %2s", ff_moni2c_addrs[whichff].name, "--");

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
  }

  if (whichff % 2 == 1) {
    m[copied++] = '\r';
    m[copied++] = '\n';
    m[copied] = '\0';
  }
  whichff = 0;
  n = 0;

  return pdFALSE;
}

BaseType_t ff_los_alarm(int argc, char **argv, char *m)
{
  int copied = 0;

  static int whichff = 0;
  static int n = 0;

  if (whichff == 0) {
    // check for stale data
    TickType_t now = pdTICKS_TO_MS(xTaskGetTickCount()) / 1000;

    if (getFFcheckStale() == 0) {
      TickType_t last = pdTICKS_TO_MS(getFFupdateTick(getFFcheckStale())) / 1000;
      int mins = (now - last) / 60;
      copied += snprintf(m + copied, SCRATCH_SIZE - copied,
                         "%s: stale data, last update %d minutes ago\r\n", argv[0], mins);
    }
    copied += snprintf(m + copied, SCRATCH_SIZE - copied, "FIREFLY LOS ALARM:\r\n");
  }

  int i1 = 2; // 2 for los_alarm
  uint8_t i2cdata[2];

  for (; n < NFIREFLY_ARG; ++n) {
    struct MonitorI2CTaskArgs_t *ff_arg = ff_moni2c_arg[n].arg;

    for (; whichff < ff_moni2c_arg[n].int_idx + ff_moni2c_arg[n].num_dev; ++whichff) {
      int dev = whichff - ff_moni2c_arg[n].int_idx + ff_moni2c_arg[n].dev_int_idx;
      copied += snprintf(m + copied, SCRATCH_SIZE - copied, "%s ", ff_moni2c_addrs[whichff].name);
      if (!isEnabledFF(ff_moni2c_arg[n].int_idx + dev)) {
        copied += snprintf(m + copied, SCRATCH_SIZE - copied, "------------");
      }
      else {
        int index = dev * (ff_arg->n_commands * ff_arg->n_pages) + i1;
        for (int i = 0; i < 2; ++i) {
          i2cdata[1 - i] = (ff_arg->sm_values[index] >> (1 - i) * 8) & 0xFFU;
        }
        for (size_t i = 0; i < 8; i++) {
          int alarm = getFFch_low(i2cdata[0], i) ? 1 : 0;
          copied += snprintf(m + copied, SCRATCH_SIZE - copied, "%d", alarm);
        }
        if (strstr(ff_moni2c_arg[n].ff_part, "FFL12") != NULL) {
          for (size_t i = 8; i < 12; i++) {
            int alarm = getFFch_high(i2cdata[1], i) ? 1 : 0;
            copied += snprintf(m + copied, SCRATCH_SIZE - copied, "%d", alarm);
          }
        }
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
  }
  if (whichff % 2 == 1) {
    m[copied++] = '\r';
    m[copied++] = '\n';
    m[copied] = '\0';
  }
  whichff = 0;
  n = 0;

  return pdFALSE;
}

BaseType_t ff_cdr_lol_alarm(int argc, char **argv, char *m)
{
  int copied = 0;

  static int whichff = 0;
  static int n = 0;

  if (whichff == 0) {
    // check for stale data
    TickType_t now = pdTICKS_TO_MS(xTaskGetTickCount()) / 1000;

    if (getFFcheckStale() == 0) {
      TickType_t last = pdTICKS_TO_MS(getFFupdateTick(getFFcheckStale())) / 1000;
      int mins = (now - last) / 60;
      copied += snprintf(m + copied, SCRATCH_SIZE - copied,
                         "%s: stale data, last update %d minutes ago\r\n", argv[0], mins);
    }
    copied += snprintf(m + copied, SCRATCH_SIZE - copied, "FIREFLY CDR LOL ALARM:\r\n");
  }

  int i1 = 3; // 2 for cdr_lol_alarm
  uint8_t i2cdata[2];

  for (; n < NFIREFLY_ARG; ++n) {
    struct MonitorI2CTaskArgs_t *ff_arg = ff_moni2c_arg[n].arg;

    for (; whichff < ff_moni2c_arg[n].int_idx + ff_moni2c_arg[n].num_dev; ++whichff) {
      int dev = whichff - ff_moni2c_arg[n].int_idx + ff_moni2c_arg[n].dev_int_idx;
      copied += snprintf(m + copied, SCRATCH_SIZE - copied, "%s ", ff_moni2c_addrs[whichff].name);
      if (!isEnabledFF(ff_moni2c_arg[n].int_idx + dev)) {
        copied += snprintf(m + copied, SCRATCH_SIZE - copied, "------------");
      }
      else {
        int index = dev * (ff_arg->n_commands * ff_arg->n_pages) + i1;
        for (int i = 0; i < 2; ++i) {
          i2cdata[1 - i] = (ff_arg->sm_values[index] >> (1 - i) * 8) & 0xFFU;
        }
        for (size_t i = 0; i < 8; i++) {
          int alarm = getFFch_low(i2cdata[0], i) ? 1 : 0;
          copied += snprintf(m + copied, SCRATCH_SIZE - copied, "%d", alarm);
        }
        if (strstr(ff_moni2c_arg[n].ff_part, "FFL12") != NULL) {
          for (size_t i = 8; i < 12; i++) {
            int alarm = getFFch_high(i2cdata[1], i) ? 1 : 0;
            copied += snprintf(m + copied, SCRATCH_SIZE - copied, "%d", alarm);
          }
        }
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
  }
  if (whichff % 2 == 1) {
    m[copied++] = '\r';
    m[copied++] = '\n';
    m[copied] = '\0';
  }
  whichff = 0;
  n = 0;

  return pdFALSE;
}

BaseType_t ff_temp(int argc, char **argv, char *m)
{
  // argument handling
  int copied = 0;

  static int whichff = 0;
  static int n = 0;

  if (whichff == 0) {
    // check for stale data
    TickType_t now = pdTICKS_TO_MS(xTaskGetTickCount()) / 1000;

    if (getFFcheckStale() == 0) {
      TickType_t last = pdTICKS_TO_MS(getFFupdateTick(getFFcheckStale())) / 1000;
      int mins = (now - last) / 60;
      copied += snprintf(m + copied, SCRATCH_SIZE - copied,
                         "%s: stale data, last update %d minutes ago\r\n", argv[0], mins);
    }
    copied += snprintf(m + copied, SCRATCH_SIZE - copied, "FF Temperature:\r\n");
  }

  int i1 = 1; // 1 for temperature

  for (; n < NFIREFLY_ARG; ++n) {
    struct MonitorI2CTaskArgs_t *ff_arg = ff_moni2c_arg[n].arg;
    for (; whichff < ff_moni2c_arg[n].int_idx + ff_moni2c_arg[n].num_dev; ++whichff) {
      int dev = whichff - ff_moni2c_arg[n].int_idx + ff_moni2c_arg[n].dev_int_idx;
      if (isEnabledFF(ff_moni2c_arg[n].int_idx + dev)) {
        int index = dev * (ff_arg->n_commands * ff_arg->n_pages) + i1;
        uint8_t val = ffldaq_f1_args.sm_values[index];
        copied += snprintf(m + copied, SCRATCH_SIZE - copied, "%17s: %02d", ff_moni2c_addrs[whichff].name, val);
      }
      else // dummy value
        copied += snprintf(m + copied, SCRATCH_SIZE - copied, "%17s: %2s", ff_moni2c_addrs[whichff].name, "--");

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
  }

  if (whichff % 2 == 1) {
    m[copied++] = '\r';
    m[copied++] = '\n';
    m[copied] = '\0';
  }
  whichff = 0;
  n = 0;

  return pdFALSE;
}

// this command takes up to two arguments
BaseType_t ff_ctl(int argc, char **argv, char *m)
{
  // argument handling
  int copied = 0;
  // check for stale data
  TickType_t now = pdTICKS_TO_MS(xTaskGetTickCount()) / 1000;

  if (getFFcheckStale() == 0) {
    TickType_t last = pdTICKS_TO_MS(getFFupdateTick(getFFcheckStale())) / 1000;
    int mins = (now - last) / 60;
    copied += snprintf(m + copied, SCRATCH_SIZE - copied,
                       "%s: stale data, last update %d minutes ago\r\n", argv[0], mins);
  }
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
        set_xcvr_cdr(0x00, channel); // default: disable
        return pdFALSE;
        if (strncmp(argv[2], "on", 2) == 0) {
          set_xcvr_cdr(0xff, channel);
          return pdFALSE;
        }
      }
      else if (strncmp(argv[1], "xmit", 4) == 0) {
        disable_transmit(true, channel);
        return pdFALSE;
        if (strncmp(argv[2], "on", 2) == 0) {
          disable_transmit(false, channel);
          return pdFALSE;
        }
      }
      else if (strncmp(argv[1], "rcvr", 4) == 0) {
        disable_receivers(true, channel);
        return pdFALSE;
        if (strncmp(argv[2], "on", 2) == 0) {
          disable_receivers(false, channel);
          return pdFALSE;
        }
      }
      // Add here
      else if (strncmp(argv[1], "regr", 4) == 0) {
        if (whichFF == NFIREFLIES) {
          copied += snprintf(m + copied, SCRATCH_SIZE - copied, "%s: cannot read all registers\r\n",
                             argv[0]);
          return pdFALSE;
        }
        // register number
        uint8_t regnum = strtol(argv[2], NULL, 16);
        copied +=
            snprintf(m + copied, SCRATCH_SIZE - copied, "%s: reading FF %s, register 0x%x\r\n",
                     argv[0], ff_moni2c_addrs[whichFF].name, regnum);
        uint8_t value;
        uint16_t ret = read_arbitrary_ff_register(regnum, channel, &value, 1);
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
        uint8_t regnum = strtol(argv[2], NULL, 16);
        uint16_t value = strtol(argv[3], NULL, 16);
        uint8_t channel = whichFF;
        if (channel == NFIREFLIES) {
          channel = 0; // silently fall back to first channel
        }
        write_arbitrary_ff_register(regnum, value, channel);
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

extern struct dev_moni2c_addr_t ff_moni2c_addrs[NFIREFLIES];

extern struct arg_moni2c_ff_t ff_moni2c_arg[NFIREFLY_ARG];
extern struct MonitorI2CTaskArgs_t ffldaq_f1_args;
extern struct MonitorI2CTaskArgs_t ffl12_f1_args;
extern struct MonitorI2CTaskArgs_t ffldaq_f2_args;
extern struct MonitorI2CTaskArgs_t ffl12_f2_args;

// dump clock monitor information
BaseType_t clkmon_ctl(int argc, char **argv, char *m)
{
  int copied = 0;
  static int c = 0;
  BaseType_t i = strtol(argv[1], NULL, 10);
  if (c == 0) {
    char *clk_ids[5] = {"r0a", "r0b", "r1a", "r1b", "r1c"};
    copied += snprintf(m + copied, SCRATCH_SIZE - copied, "Monitoring SI clock with id : %s \r\n",
                       clk_ids[i]);
    char *header = "REG_TABLE";
    copied += snprintf(m + copied, SCRATCH_SIZE - copied, "%-15s REG_ADDR BIT_MASK  VALUE \r\n", header);
  }
  if (i < 0 || i > 4) {
    snprintf(m + copied, SCRATCH_SIZE - copied,
             "Invalid clock chip %ld , the clock id options are r0a:0, r0b:1, r1a:2, "
             "r1b:3 and r1c:4 \r\n",
             i);
    return pdFALSE;
  }

  if (i == 0) {

    // update times, in seconds
    TickType_t now = pdTICKS_TO_MS(xTaskGetTickCount()) / 1000;
    TickType_t last = pdTICKS_TO_MS(clockr0a_args.updateTick) / 1000;

    if (checkStale(last, now)) {
      int mins = (now - last) / 60;
      copied += snprintf(m + copied, SCRATCH_SIZE - copied,
                         "%s: stale data, last update %d minutes ago\r\n", argv[0], mins);
    }

    for (; c < clockr0a_args.n_commands; ++c) {
      uint8_t val = clockr0a_args.sm_values[c];
      copied += snprintf(m + copied, SCRATCH_SIZE - copied, "%-15s : 0x%04x   0x%02x    0x%04x\r\n", clockr0a_args.commands[c].name, clockr0a_args.commands[c].command, clockr0a_args.commands[c].bit_mask, val);

      // copied += snprintf(m + copied, SCRATCH_SIZE - copied, "\r\n");
      if ((SCRATCH_SIZE - copied) < 50) {
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
  }
  else {

    // update times, in seconds
    TickType_t now = pdTICKS_TO_MS(xTaskGetTickCount()) / 1000;
    TickType_t last = pdTICKS_TO_MS(clock_args.updateTick) / 1000;

    if (checkStale(last, now)) {
      int mins = (now - last) / 60;
      copied += snprintf(m + copied, SCRATCH_SIZE - copied,
                         "%s: stale data, last update %d minutes ago\r\n", argv[0], mins);
    }

    for (; c < clock_args.n_commands; ++c) {
      uint8_t val = clock_args.sm_values[(i - 1) * (clock_args.n_commands * clock_args.n_pages) + c];
      copied += snprintf(m + copied, SCRATCH_SIZE - copied, "%-15s : 0x%04x   0x%02x    0x%04x\r\n", clock_args.commands[c].name, clock_args.commands[c].command, clock_args.commands[c].bit_mask, val);

      // copied += snprintf(m + copied, SCRATCH_SIZE - copied, "\r\n");
      if ((SCRATCH_SIZE - copied) < 50) {
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
  }

  return pdFALSE;
}

extern struct MonitorI2CTaskArgs_t clock_args;
extern struct MonitorI2CTaskArgs_t clockr0a_args;

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
      TickType_t now = pdTICKS_TO_MS(xTaskGetTickCount()) / 1000;
      if (getFFcheckStale() == 0) {
        TickType_t last = pdTICKS_TO_MS(getFFupdateTick(getFFcheckStale())) / 1000;
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

// This command takes 1 argument, either k or v
BaseType_t fpga_reset(int argc, char **argv, char *m)
{
  int copied = 0;
  const TickType_t delay = 1 / portTICK_PERIOD_MS; // 1 ms delay

  if (strcmp(argv[1], "f2") == 0) {
    write_gpio_pin(F2_FPGA_PROGRAM, 0x1);
    vTaskDelay(delay);
    write_gpio_pin(F2_FPGA_PROGRAM, 0x0);
    copied += snprintf(m + copied, SCRATCH_SIZE - copied, "VU7P has been reset\r\n");
  }
  if (strcmp(argv[1], "f1") == 0) {
    write_gpio_pin(F1_FPGA_PROGRAM, 0x1);
    vTaskDelay(delay);
    write_gpio_pin(F1_FPGA_PROGRAM, 0x0);
    copied += snprintf(m + copied, SCRATCH_SIZE - copied, "KU15P has been reset\r\n");
  }
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
  while (xSemaphoreTake(dcdc_args.xSem, (TickType_t)10) == pdFALSE)
    ;
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
  xSemaphoreGive(dcdc_args.xSem);
  return pdFALSE;
}
