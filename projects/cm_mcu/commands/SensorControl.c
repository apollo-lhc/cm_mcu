/*
 * SensorControl.c
 *
 *  Created on: Jan 14, 2021
 *      Author: fatimayousuf
 */

#include <string.h>

#include "commands/SensorControl.h"
#include "commands/parameters.h"
#include "common/utils.h"
#include "Tasks.h"
#include "projdefs.h"

// send LED status commands
BaseType_t led_ctl(int argc, char **argv, char *m)
{
  static const struct {
    const char *name;
    const LedMsg_t *msg;
  } statuses[] = {
      {"init", &LED_STATUS_INIT},
      {"normal", &LED_STATUS_NORMAL},
      {"load", &LED_STATUS_PS_LOADING},
      {"warn", &LED_STATUS_WARN},
      {"alarm", &LED_STATUS_ALARM},
      {"psfault", &LED_STATUS_PS_FAULT},
      {"fwfault", &LED_STATUS_FW_FAULT},
  };
  for (size_t i = 0; i < sizeof(statuses) / sizeof(statuses[0]); ++i) {
    if (strcmp(argv[1], statuses[i].name) == 0) {
      xQueueSendToBack(xLedQueue, statuses[i].msg, pdMS_TO_TICKS(10));
      m[0] = '\0';
      return pdFALSE;
    }
  }
  snprintf(m, SCRATCH_SIZE,
           "Unknown LED status '%s'.\r\nOptions: init normal load warn alarm psfault fwfault\r\n",
           argv[1]);
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
