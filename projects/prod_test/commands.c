#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "commands.h"
#include "FreeRTOS.h"
#include "common/printf.h"
#include "common/power_ctl.h"
#include "common/pinsel.h"
#include "common/utils.h"

// #define NUM_COMMANDS (sizeof(commands) / sizeof(commands[0]))

#include "driverlib/rom.h"
#include "inc/hw_types.h"
#include "inc/hw_nvic.h"
#include "portmacro.h"
#include "task.h"
// #include "inc/hw_memmap.h"

#include "ADCMonitorTask.h"
#include "ClockI2CCommands.h"
#include "EEPROMI2CCommands.h"
#include "FireflyI2CCommands.h"
#include "FPGAI2CCommands.h"
#include "PowerI2CCommands.h"

void Print(const char *str);

// this command takes no arguments and never returns.
__attribute__((noreturn)) BaseType_t bl_ctl(int argc, char **argv, char *m)
{
  Print("Jumping to bootloader\r\n");
  ROM_SysCtlDelay(100000);
  // this code is copied from the JumpToBootLoader()
  // stack from the boot_demo1 application in the
  // ek-tm4c129exl part of tiva ware.
  //
  // We must make sure we turn off SysTick and its interrupt before entering
  // the boot loader!
  //
  ROM_SysTickIntDisable();
  ROM_SysTickDisable();

  //
  // Disable all processor interrupts.  Instead of disabling them
  // one at a time, a direct write to NVIC is done to disable all
  // peripheral interrupts.
  //
  HWREG(NVIC_DIS0) = 0xffffffff;
  HWREG(NVIC_DIS1) = 0xffffffff;
  HWREG(NVIC_DIS2) = 0xffffffff;
  HWREG(NVIC_DIS3) = 0xffffffff;

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Warray-bounds"

  //
  // Return control to the boot loader.  This is a call to the SVC
  // handler in the boot loader.
  //
  (*((void (*)(void))(*(uint32_t *)0x2c)))();

  // the above points to a memory location in flash.
#pragma GCC diagnostic pop
  __builtin_unreachable();
}

// turn on power at the specified level
BaseType_t power_ctl(int argc, char **argv, char *m)
{
  // parse argv[1] as a number
  // if it is not a number, return an error
  // if it is a number, turn on power at that level
  // return success
  int32_t level = strtol(argv[1], NULL, 16);
  if (level < 0 || level > PS_NUM_PRIORITIES) {
    snprintf(m, SCRATCH_SIZE, "Invalid power level %s\r\n", argv[1]);
    return pdFALSE;
  }
  // 0 is automatic
  if (level > 0)
    turn_on_ps_at_prio(true, true, level);
  vTaskDelay(pdMS_TO_TICKS(1000)); // let ADC catch up
  float delta;
  int r = check_ps_at_prio(level, true, true, &delta);
  int copied = snprintf(m, SCRATCH_SIZE, "volt compare at level %d: %s\r\n",
                        level, (r == 0 ? "good" : "bad"));
  copied += snprintf(m + copied, SCRATCH_SIZE - copied, "delta: %f%%\r\n",
                     (double)delta);

  return pdFALSE;
}

BaseType_t power_off_ctl(int argc, char **argv, char *m)
{
  disable_ps();
  snprintf(m, SCRATCH_SIZE, "Power off\r\n");
  return pdFALSE;
}

// This command takes no arguments
BaseType_t restart_mcu(int argc, char **argv, char *m)
{
  disable_ps();
  snprintf(m, SCRATCH_SIZE, "Restarting MCU\r\n");
  ROM_SysCtlReset(); // This function does not return
  __builtin_unreachable();
  return pdFALSE;
}

// Display ADC measurements
BaseType_t adc_ctl(int argc, char **argv, char *m)
{
  int copied = 0;

  static int whichadc = 0;
  if (whichadc == 0) {
    copied += snprintf(m + copied, SCRATCH_SIZE - copied, "ADC outputs\r\n");
  }
  for (; whichadc < 21; ++whichadc) {
    float val = getADCvalue(whichadc);
    copied += snprintf(m + copied, SCRATCH_SIZE - copied, "%14s: %5.2f",
                       getADCname(whichadc), (double)val);
    if (whichadc < ADC_INFO_CUR_INIT_CH) { // for voltage vals, compare to expected
      float target_val = getADCtargetValue(whichadc);
      float diff = (target_val - val) / val;
      if (ABS(diff) > ADC_DIFF_TOLERANCE) {
        copied += snprintf(m + copied, SCRATCH_SIZE - copied, "\tBAD");
      }
    }
    m[copied++] = '\r';
    m[copied++] = '\n';
    m[copied] = '\0';
    if ((SCRATCH_SIZE - copied) < 50 && (whichadc < 20)) {
      ++whichadc;
      return pdTRUE;
    }
  }
  whichadc = 0;
  return pdFALSE;
}

/**
 * @details
 * Runs all first step production tests
 */
BaseType_t prodtest_firststep_ctl(int argc, char **argv, char *m)
{
  int32_t copied = 0;
  int r;
  if (!dcdc_i2ctest(m, &copied))
    return pdFALSE;
  if (!dcdc_powerontest(m, &copied))
    return pdFALSE;
  r = init_registers_clk();
  if (r) {
    copied += snprintf(m + copied, SCRATCH_SIZE - copied,
                       "ERROR: Failed to initialize clk IO expanders.\r\n");
    return pdFALSE;
  }
  if (!clock_i2ctest(m, &copied))
    return pdFALSE;
  if (!fpga_i2ctest(m, &copied))
    return pdFALSE;
  r = init_registers_firefly();
  if (r) {
    copied += snprintf(m + copied, SCRATCH_SIZE - copied,
                       "ERROR: Failed to initialize FF IO expanders.\r\n");
    return pdFALSE;
  }
  if (!firefly_i2ctest(m, &copied))
    return pdFALSE;
  if (!eeprom_i2ctest(m, &copied))
    return pdFALSE;
  copied += snprintf(m + copied, SCRATCH_SIZE - copied,
                     "All tests successful.\r\n");
  disable_ps();
  return pdFALSE;
}
