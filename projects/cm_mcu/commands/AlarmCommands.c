/*
 * AlarmCommands.c
 *
 * Alarm CLI command handlers, extracted from SensorControl.c.
 */

#include <string.h>
#include <strings.h>
#include <errno.h>
#include <limits.h>

#include "AlarmUtilities.h"
#include "commands/AlarmCommands.h"
#include "commands/parameters.h"
#include "common/utils.h"
#include "Tasks.h"
#include "projdefs.h"

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

    copied +=
        snprintf(m + copied, SCRATCH_SIZE - copied, "TEMP FFLY: %s \t Threshold: %d\r\n",
                 (stat & ALM_STAT_FIREFLY_OVERTEMP) ? "ALARM" : "GOOD", getAlarmTemperature(FF));

    copied +=
        snprintf(m + copied, SCRATCH_SIZE - copied, "TEMP FPGA: %s \t Threshold: %d\r\n",
                 (stat & ALM_STAT_FPGA_OVERTEMP) ? "ALARM" : "GOOD", getAlarmTemperature(FPGA));

    copied +=
        snprintf(m + copied, SCRATCH_SIZE - copied, "TEMP DCDC: %s \t Threshold: %d\r\n",
                 (stat & ALM_STAT_DCDC_OVERTEMP) ? "ALARM" : "GOOD", getAlarmTemperature(DCDC));

    copied +=
        snprintf(m + copied, SCRATCH_SIZE - copied, "TEMP TM4C: %s \t Threshold: %d\r\n",
                 (stat & ALM_STAT_TM4C_OVERTEMP) ? "ALARM" : "GOOD", getAlarmTemperature(TM4C));

    uint32_t adc_volt_stat = getVoltAlarmStatus();
    float voltthres = getAlarmVoltageThres() * 100;
    int tens, frac;
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
    errno = 0;
    char *endptr = NULL;
    long tmp = strtol(argv[3], &endptr, 10);
    if (endptr == argv[3] || *endptr != '\0' || errno == ERANGE || tmp < INT16_MIN || tmp > INT16_MAX) {
      snprintf(m, s, "Invalid temp '%s'; must be a signed 16-bit int\r\n",
               argv[3]);
      return pdFALSE;
    }
    int16_t newtemp = (int16_t)tmp;
    char *device = argv[2];
    if (!strncasecmp(device, "ff", 2)) {
      setAlarmTemperature(FF, newtemp);
      snprintf(m, s, "%s: set Firefly temp to %s (saved)\r\n", argv[0], argv[3]);
      return pdFALSE;
    }
    else if (!strncasecmp(device, "fpga", 4)) {
      setAlarmTemperature(FPGA, newtemp);
      snprintf(m, s, "%s: set FPGA temp to %s (saved)\r\n", argv[0], argv[3]);
      return pdFALSE;
    }
    else if (!strncasecmp(device, "dcdc", 4)) {
      setAlarmTemperature(DCDC, newtemp);
      snprintf(m, s, "%s: set DCDC temp to %s (saved)\r\n", argv[0], argv[3]);
      return pdFALSE;
    }
    else if (!strncasecmp(device, "tm4c", 4)) {
      setAlarmTemperature(TM4C, newtemp);
      snprintf(m, s, "%s: set TM4C temp to %s (saved)\r\n", argv[0], argv[3]);
      return pdFALSE;
    }
    else {
      snprintf(m, s, "%s is not a valid device.\r\n", argv[2]);
      return pdFALSE;
    }
  }
  else if (strcmp(argv[1], "resettemp") == 0) {
    if (argc != 3) {
      snprintf(m, s, "Usage: %s resettemp [ff|fpga|dcdc|tm4c|all]\r\n", argv[0]);
      return pdFALSE;
    }
    char *device = argv[2];
    int handled = 0;
    if (!strncasecmp(device, "ff", 2) || !strncasecmp(device, "all", 3)) {
      setAlarmTemperature(FF, INITIAL_ALARM_TEMP_FF);
      handled = 1;
    }
    if (!strncasecmp(device, "dcdc", 4) || !strncasecmp(device, "all", 3)) {
      setAlarmTemperature(DCDC, INITIAL_ALARM_TEMP_DCDC);
      handled = 1;
    }
    if (!strncasecmp(device, "tm4c", 4) || !strncasecmp(device, "all", 3)) {
      setAlarmTemperature(TM4C, INITIAL_ALARM_TEMP_TM4C);
      handled = 1;
    }
    if (!strncasecmp(device, "fpga", 4) || !strncasecmp(device, "all", 3)) {
      setAlarmTemperature(FPGA, INITIAL_ALARM_TEMP_FPGA);
      handled = 1;
    }
    if (!handled) {
      snprintf(m,
               s,
               "%s: %s is not a valid device. Usage: %s resettemp [ff|fpga|dcdc|tm4c|all]\r\n",
               argv[0],
               argv[2],
               argv[0]);
      return pdFALSE;
    }
    snprintf(m, s, "%s: reset %s alarm temps to defaults\r\n", argv[0], argv[2]);
    return pdFALSE;
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
