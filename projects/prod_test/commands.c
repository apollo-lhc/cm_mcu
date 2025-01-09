#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "commands.h"
#include "FreeRTOS.h"
#include "common/printf.h"
#include "common/power_ctl.h"
#include "common/pinsel.h"

// #define NUM_COMMANDS (sizeof(commands) / sizeof(commands[0]))

#include "driverlib/rom.h"
#include "inc/hw_types.h"
#include "inc/hw_nvic.h"
#include "portmacro.h"
// #include "inc/hw_memmap.h"

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

// power control state names
static const char *power_control_state_names[] = {
#define X(name) #name,
    X_MACRO_PS_STATES
#undef X
};

// turn on power at the specified level
BaseType_t power_ctl(int argc, char **argv, char *m)
{
  int copied = 0;
  // parse argv[1] as a number
  // if it is not a number, return an error
  // if it is a number, turn on power at that level
  // return success
  int32_t level = strtol(argv[1], NULL, 16);
  if (level <= 0 || level > N_PS_OKS) {
    snprintf(m, SCRATCH_SIZE, "Invalid power level %s\r\n", argv[1]);
    return pdFALSE;
  }
  turn_on_ps_at_prio(true, true, level);
  static int i = 0;
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

BaseType_t power_off_ctl(int argc, char **argv, char *m)
{
  disable_ps();
  snprintf(m, SCRATCH_SIZE, "Power off\r\n");
  return pdTRUE;
}

// direct copy paste from other project?
// This command takes no arguments
BaseType_t restart_mcu(int argc, char **argv, char *m)
{
  disable_ps();
  snprintf(m, SCRATCH_SIZE, "Restarting MCU\r\n");
  ROM_SysCtlReset(); // This function does not return
  __builtin_unreachable();
  return pdFALSE;
}
