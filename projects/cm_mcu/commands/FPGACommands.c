/*
 * FPGACommands.c
 *
 * FPGA CLI command handlers, extracted from SensorControl.c.
 */

#include <string.h>

#include "commands/FPGACommands.h"
#include "commands/parameters.h"
#include "common/utils.h"
#include "FireflyUtils.h"
#include "MonitorTaskI2C.h"
#include "Tasks.h"
#include "projdefs.h"

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
