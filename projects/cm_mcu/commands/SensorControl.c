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

  int copied = 0;

  static int whichff = 0;
  if (whichff == 0) {
    // check for stale data
    TickType_t now = pdTICKS_TO_MS(xTaskGetTickCount()) / 1000;
    TickType_t last = pdTICKS_TO_MS(getFFupdateTick()) / 1000;
    if (checkStale(last, now)) {
      int mins = (now - last) / 60;
      copied += snprintf(m + copied, SCRATCH_SIZE - copied,
                         "%s: stale data, last update %d minutes ago\r\n", argv[0], mins);
    }
    copied += snprintf(m + copied, SCRATCH_SIZE - copied, "FIREFLY STATUS:\r\n");
  }

  for (; whichff < NFIREFLIES; ++whichff) {
    if (!isEnabledFF(whichff)) {
      copied += snprintf(m + copied, SCRATCH_SIZE - copied, "%17s   --", ff_moni2c_addrs[whichff].name);
    }
    else {
      int i1 = 0; // 0 for status
      if (0 <= whichff && whichff < NFIREFLIES_IT_F1) {
        int index = whichff * (ffl12_f1_args.n_commands * ffl12_f1_args.n_pages) + i1;
        uint16_t val = ffl12_f1_args.sm_values[index];
        copied += snprintf(m + copied, SCRATCH_SIZE - copied, "%17s 0x%02x ", ff_moni2c_addrs[whichff].name, val);
      }

      else if (NFIREFLIES_IT_F1 <= whichff && whichff < NFIREFLIES_IT_F1 + NFIREFLIES_DAQ_F1) {
        int index = (whichff - NFIREFLIES_IT_F1) * (ffldaq_f1_args.n_commands * ffldaq_f1_args.n_pages) + i1;
        uint16_t val = ffldaq_f1_args.sm_values[index];
        copied += snprintf(m + copied, SCRATCH_SIZE - copied, "%17s 0x%02x ", ff_moni2c_addrs[whichff].name, val);
      }
      else if (NFIREFLIES_F1 <= whichff && whichff < NFIREFLIES_F1 + NFIREFLIES_IT_F2) {
        int index = (whichff - NFIREFLIES_F1) * (ffl12_f2_args.n_commands * ffl12_f2_args.n_pages) + i1;
        uint16_t val = ffl12_f2_args.sm_values[index];
        copied += snprintf(m + copied, SCRATCH_SIZE - copied, "%17s: 0x%02x", ff_moni2c_addrs[whichff].name, val);
      }
      else {
        int index = (whichff - NFIREFLIES_F1 - NFIREFLIES_IT_F2) * (ffldaq_f2_args.n_commands * ffldaq_f2_args.n_pages) + i1;
        uint16_t val = ffldaq_f2_args.sm_values[index];
        copied += snprintf(m + copied, SCRATCH_SIZE - copied, "%17s: 0x%02x", ff_moni2c_addrs[whichff].name, val);
      }
    }
    bool isTx = (strstr(ff_moni2c_addrs[whichff].name, "Tx") != NULL);
    if (isTx)
      copied += snprintf(m + copied, SCRATCH_SIZE - copied, "\t");
    else
      copied += snprintf(m + copied, SCRATCH_SIZE - copied, "\r\n");

    if ((SCRATCH_SIZE - copied) < 20 && (whichff < 25)) {
      ++whichff;
      return pdTRUE;
    }
  }
  whichff = 0;
  return pdFALSE;
}

BaseType_t ff_los_alarm(int argc, char **argv, char *m)
{
  int copied = 0;

  static int whichff = 0;
  if (whichff == 0) {
    // check for stale data
    TickType_t now = pdTICKS_TO_MS(xTaskGetTickCount()) / 1000;
    TickType_t last = pdTICKS_TO_MS(getFFupdateTick()) / 1000;
    if (checkStale(last, now)) {
      int mins = (now - last) / 60;
      copied += snprintf(m + copied, SCRATCH_SIZE - copied,
                         "%s: stale data, last update %d minutes ago\r\n", argv[0], mins);
    }
    copied += snprintf(m + copied, SCRATCH_SIZE - copied, "FIREFLY LOS ALARM:\r\n");
  }

  for (; whichff < NFIREFLIES; ++whichff) {
    copied += snprintf(m + copied, SCRATCH_SIZE - copied, "%s ", ff_moni2c_addrs[whichff].name);
    if (!isEnabledFF(whichff)) {
      copied += snprintf(m + copied, SCRATCH_SIZE - copied, "------------");
    }
    else {
      int i1 = 2; // 2 for los_alarm
      uint8_t i2cdata[2];
      if (0 <= whichff && whichff < NFIREFLIES_IT_F1) {
        int index = whichff * (ffl12_f1_args.n_commands * ffl12_f1_args.n_pages) + i1;
        for (int i = 0; i < 2; ++i) {
          i2cdata[1 - i] = (ffl12_f1_args.sm_values[index] >> (1 - i) * 8) & 0xFFU;
        }
        for (size_t i = 0; i < 8; i++) {
          int alarm = getFFch_low(i2cdata[0], i) ? 1 : 0;
          copied += snprintf(m + copied, SCRATCH_SIZE - copied, "%d", alarm);
        }
        for (size_t i = 8; i < 12; i++) {
          int alarm = getFFch_high(i2cdata[1], i) ? 1 : 0;
          copied += snprintf(m + copied, SCRATCH_SIZE - copied, "%d", alarm);
        }
      }
      else if (NFIREFLIES_IT_F1 <= whichff && whichff < NFIREFLIES_IT_F1 + NFIREFLIES_DAQ_F1) {
        int index = (whichff - NFIREFLIES_IT_F1) * (ffldaq_f1_args.n_commands * ffldaq_f1_args.n_pages) + i1;
        for (int i = 0; i < 2; ++i) {
          i2cdata[1 - i] = (ffldaq_f1_args.sm_values[index] >> (1 - i) * 8) & 0xFFU;
        }
        for (size_t i = 0; i < 8; i++) {
          int alarm = getFFch_low(i2cdata[0], i) ? 1 : 0;
          copied += snprintf(m + copied, SCRATCH_SIZE - copied, "%d", alarm);
        }
      }
      else if (NFIREFLIES_F1 <= whichff && whichff < NFIREFLIES_F1 + NFIREFLIES_IT_F2) {
        int index = (whichff - NFIREFLIES_F1) * (ffl12_f2_args.n_commands * ffl12_f2_args.n_pages) + i1;
        for (int i = 0; i < 2; ++i) {
          i2cdata[1 - i] = (ffl12_f2_args.sm_values[index] >> (1 - i) * 8) & 0xFFU;
        }
        for (size_t i = 0; i < 8; i++) {
          int alarm = getFFch_low(i2cdata[0], i) ? 1 : 0;
          copied += snprintf(m + copied, SCRATCH_SIZE - copied, "%d", alarm);
        }
        for (size_t i = 8; i < 12; i++) {
          int alarm = getFFch_high(i2cdata[1], i) ? 1 : 0;
          copied += snprintf(m + copied, SCRATCH_SIZE - copied, "%d", alarm);
        }
      }
      else {
        int index = (whichff - NFIREFLIES_F1 - NFIREFLIES_IT_F2) * (ffldaq_f2_args.n_commands * ffldaq_f2_args.n_pages) + i1;
        for (int i = 0; i < 2; ++i) {
          i2cdata[1 - i] = (ffldaq_f2_args.sm_values[index] >> (1 - i) * 8) & 0xFFU;
        }
        for (size_t i = 0; i < 8; i++) {
          int alarm = getFFch_low(i2cdata[0], i) ? 1 : 0;
          copied += snprintf(m + copied, SCRATCH_SIZE - copied, "%d", alarm);
        }
      }
    }
    bool isTx = (strstr(ff_moni2c_addrs[whichff].name, "Tx") != NULL);
    if (isTx)
      copied += snprintf(m + copied, SCRATCH_SIZE - copied, "\t");
    else
      copied += snprintf(m + copied, SCRATCH_SIZE - copied, "\r\n");

    if ((SCRATCH_SIZE - copied) < 20 && (whichff < 25)) {
      ++whichff;
      return pdTRUE;
    }
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
    TickType_t now = pdTICKS_TO_MS(xTaskGetTickCount()) / 1000;
    TickType_t last = pdTICKS_TO_MS(getFFupdateTick()) / 1000;
    if (checkStale(last, now)) {
      int mins = (now - last) / 60;
      copied += snprintf(m + copied, SCRATCH_SIZE - copied,
                         "%s: stale data, last update %d minutes ago\r\n", argv[0], mins);
    }
    copied += snprintf(m + copied, SCRATCH_SIZE - copied, "FIREFLY LOS ALARM:\r\n");
  }

  for (; whichff < NFIREFLIES; ++whichff) {
    copied += snprintf(m + copied, SCRATCH_SIZE - copied, "%s ", ff_moni2c_addrs[whichff].name);
    if (!isEnabledFF(whichff)) {
      copied += snprintf(m + copied, SCRATCH_SIZE - copied, "------------");
    }
    else {
      int i1 = 3; // 3 for cdr_lol_alarm
      uint8_t i2cdata[2];
      if (0 <= whichff && whichff < NFIREFLIES_IT_F1) {
        int index = whichff * (ffl12_f1_args.n_commands * ffl12_f1_args.n_pages) + i1;
        for (int i = 0; i < 2; ++i) {
          i2cdata[1 - i] = (ffl12_f1_args.sm_values[index] >> (1 - i) * 8) & 0xFFU;
        }
        for (size_t i = 0; i < 8; i++) {
          int alarm = getFFch_low(i2cdata[0], i) ? 1 : 0;
          copied += snprintf(m + copied, SCRATCH_SIZE - copied, "%d", alarm);
        }
        for (size_t i = 8; i < 12; i++) {
          int alarm = getFFch_high(i2cdata[1], i) ? 1 : 0;
          copied += snprintf(m + copied, SCRATCH_SIZE - copied, "%d", alarm);
        }
      }

      else if (NFIREFLIES_IT_F1 <= whichff && whichff < NFIREFLIES_IT_F1 + NFIREFLIES_DAQ_F1) {
        int index = (whichff - NFIREFLIES_IT_F1) * (ffldaq_f1_args.n_commands * ffldaq_f1_args.n_pages) + i1;
        for (int i = 0; i < 2; ++i) {
          i2cdata[1 - i] = (ffldaq_f1_args.sm_values[index] >> (1 - i) * 8) & 0xFFU;
        }
        for (size_t i = 0; i < 8; i++) {
          int alarm = getFFch_low(i2cdata[0], i) ? 1 : 0;
          copied += snprintf(m + copied, SCRATCH_SIZE - copied, "%d", alarm);
        }
      }
      else if (NFIREFLIES_F1 <= whichff && whichff < NFIREFLIES_F1 + NFIREFLIES_IT_F2) {
        int index = (whichff - NFIREFLIES_F1) * (ffl12_f2_args.n_commands * ffl12_f2_args.n_pages) + i1;
        for (int i = 0; i < 2; ++i) {
          i2cdata[1 - i] = (ffl12_f2_args.sm_values[index] >> (1 - i) * 8) & 0xFFU;
        }
        for (size_t i = 0; i < 8; i++) {
          int alarm = getFFch_low(i2cdata[0], i) ? 1 : 0;
          copied += snprintf(m + copied, SCRATCH_SIZE - copied, "%d", alarm);
        }
        for (size_t i = 8; i < 12; i++) {
          int alarm = getFFch_high(i2cdata[1], i) ? 1 : 0;
          copied += snprintf(m + copied, SCRATCH_SIZE - copied, "%d", alarm);
        }
      }
      else {
        int index = (whichff - NFIREFLIES_F1 - NFIREFLIES_IT_F2) * (ffldaq_f2_args.n_commands * ffldaq_f2_args.n_pages) + i1;
        for (int i = 0; i < 2; ++i) {
          i2cdata[1 - i] = (ffldaq_f2_args.sm_values[index] >> (1 - i) * 8) & 0xFFU;
        }
        for (size_t i = 0; i < 8; i++) {
          int alarm = getFFch_low(i2cdata[0], i) ? 1 : 0;
          copied += snprintf(m + copied, SCRATCH_SIZE - copied, "%d", alarm);
        }
      }
    }
    bool isTx = (strstr(ff_moni2c_addrs[whichff].name, "Tx") != NULL);
    if (isTx)
      copied += snprintf(m + copied, SCRATCH_SIZE - copied, "\t");
    else
      copied += snprintf(m + copied, SCRATCH_SIZE - copied, "\r\n");

    if ((SCRATCH_SIZE - copied) < 20 && (whichff < 25)) {
      ++whichff;
      return pdTRUE;
    }
  }
  whichff = 0;
  return pdFALSE;
}

BaseType_t ff_temp(int argc, char **argv, char *m)
{
  // argument handling
  int copied = 0;
  static int whichff = 0;

  if (whichff == 0) {
    // check for stale data
    TickType_t now = pdTICKS_TO_MS(xTaskGetTickCount()) / 1000;
    TickType_t last = pdTICKS_TO_MS(getFFupdateTick()) / 1000;
    if (checkStale(last, now)) {
      int mins = (now - last) / 60;
      copied += snprintf(m + copied, SCRATCH_SIZE - copied,
                         "%s: stale data, last update %d minutes ago\r\n", argv[0], mins);
    }
  }
  // parse command based on how many arguments it has
  if (argc == 1) { // default command: temps
    if (whichff == 0) {
      copied += snprintf(m + copied, SCRATCH_SIZE - copied, "FF Temperature\r\n");
    }
    for (; whichff < NFIREFLIES; ++whichff) {
      if (isEnabledFF(whichff)) {
        int i1 = 1; // 1 for temperature
        if (0 <= whichff && whichff < NFIREFLIES_IT_F1) {
          int index = whichff * (ffl12_f1_args.n_commands * ffl12_f1_args.n_pages) + i1;
          uint8_t val = ffl12_f1_args.sm_values[index];
          copied += snprintf(m + copied, SCRATCH_SIZE - copied, "%17s: %02d", ff_moni2c_addrs[whichff].name, val);
        }

        else if (NFIREFLIES_IT_F1 <= whichff && whichff < NFIREFLIES_IT_F1 + NFIREFLIES_DAQ_F1) {
          int index = (whichff - NFIREFLIES_IT_F1) * (ffldaq_f1_args.n_commands * ffldaq_f1_args.n_pages) + i1;
          uint8_t val = ffldaq_f1_args.sm_values[index];
          copied += snprintf(m + copied, SCRATCH_SIZE - copied, "%17s: %02d", ff_moni2c_addrs[whichff].name, val);
        }

        else if (NFIREFLIES_F1 <= whichff && whichff < NFIREFLIES_F1 + NFIREFLIES_IT_F2) {
          int index = (whichff - NFIREFLIES_F1) * (ffl12_f2_args.n_commands * ffl12_f2_args.n_pages) + i1;
          uint8_t val = ffl12_f2_args.sm_values[index];
          copied += snprintf(m + copied, SCRATCH_SIZE - copied, "%17s: %02d", ff_moni2c_addrs[whichff].name, val);
        }
        else {
          int index = (whichff - NFIREFLIES_F1 - NFIREFLIES_IT_F2) * (ffldaq_f2_args.n_commands * ffldaq_f2_args.n_pages) + i1;
          uint8_t val = ffldaq_f2_args.sm_values[index];
          copied += snprintf(m + copied, SCRATCH_SIZE - copied, "%17s: %02d", ff_moni2c_addrs[whichff].name, val);
        }
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
    if (whichff % 2 == 1) {
      m[copied++] = '\r';
      m[copied++] = '\n';
      m[copied] = '\0';
    }
    whichff = 0;
  }

  return pdFALSE;
}

extern struct dev_moni2c_addr_t ff_moni2c_addrs[NFIREFLIES];
extern struct MonitorI2CTaskArgs_t ffldaq_f1_args;
extern struct MonitorI2CTaskArgs_t ffl12_f1_args;

// dump clock monitor information
BaseType_t clkmon_ctl(int argc, char **argv, char *m)
{
  int copied = 0;
  char *clk_ids[5] = {"r0a", "r0b", "r1a", "r1b", "r1c"};
  BaseType_t i = strtol(argv[1], NULL, 10);

  if (i < 0 || i > 4) {
    snprintf(m + copied, SCRATCH_SIZE - copied,
             "Invalid clock chip %ld , the clock id options are r0a:0, r0b:1, r1a:2, "
             "r1b:3 and r1c:4 \r\n",
             i);
    return pdFALSE;
  }

  if (i == 0) {
    copied += snprintf(m + copied, SCRATCH_SIZE - copied, "Monitoring SI5341 with id : %s \r\n",
        clk_ids[i]);
    // update times, in seconds
    TickType_t now = pdTICKS_TO_MS(xTaskGetTickCount()) / 1000;
    TickType_t last = pdTICKS_TO_MS(clockr0a_args.updateTick) / 1000;

    if (checkStale(last, now)) {
      int mins = (now - last) / 60;
      copied += snprintf(m + copied, SCRATCH_SIZE - copied,
          "%s: stale data, last update %d minutes ago\r\n", argv[0], mins);
    }
    for (int c = 0; c < clockr0a_args.n_commands; ++c) {
      uint8_t val = clockr0a_args.sm_values[c];
      copied += snprintf(m + copied, SCRATCH_SIZE - copied, "%-15s : REG_ADDR 0x%04x BIT_MASK 0x%02x VALUE 0x%04x\t", clockr0a_args.commands[c].name, clockr0a_args.commands[c].command, clockr0a_args.commands[c].bit_mask, val);

      copied += snprintf(m + copied, SCRATCH_SIZE - copied, "\r\n");
    }
  }
  else {
    copied += snprintf(m + copied, SCRATCH_SIZE - copied, "Monitoring SI5395 with id : %s \r\n",
        clk_ids[i]);
    // update times, in seconds
    TickType_t now = pdTICKS_TO_MS(xTaskGetTickCount()) / 1000;
    TickType_t last = pdTICKS_TO_MS(clock_args.updateTick) / 1000;

    if (checkStale(last, now)) {
      int mins = (now - last) / 60;
      copied += snprintf(m + copied, SCRATCH_SIZE - copied,
          "%s: stale data, last update %d minutes ago\r\n", argv[0], mins);
    }
    for (int c = 0; c < clock_args.n_commands; ++c) {
      uint8_t val = clock_args.sm_values[(i - 1) * (clock_args.n_commands * clock_args.n_pages) + c];
      copied += snprintf(m + copied, SCRATCH_SIZE - copied, "%-15s : REG_ADDR 0x%04x BIT_MASK 0x%02x VALUE 0x%04x\t", clock_args.commands[c].name, clock_args.commands[c].command, clock_args.commands[c].bit_mask, val);

      copied += snprintf(m + copied, SCRATCH_SIZE - copied, "\r\n");
    }
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
      TickType_t last = pdTICKS_TO_MS(getFFupdateTick()) / 1000;
      if (checkStale(last, now)) {
        int mins = (now - last) / 60;
        copied += snprintf(m + copied, SCRATCH_SIZE - copied,
                           "%s: stale data, last update %d minutes ago (%ld, %ld)\r\n", argv[0], mins,
                           now, last);
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
