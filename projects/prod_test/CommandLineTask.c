/*
 * CommandLineTask.c
 *
 *  Created on: Apr 7, 2019
 *      Author: wittich, rzou
 */

// Include commands
#include <strings.h>
#include <string.h>
// includes for types
#include <stdint.h>
#include <stdbool.h>

#include "FreeRTOS.h" // IWYU pragma: keep
#include "portmacro.h"
#include "stream_buffer.h"
#include "common/LocalUart.h"
#include "common/printf.h"
#include "common/microrl.h"

typedef struct {
  StreamBufferHandle_t UartStreamBuffer;
  uint32_t uart_base;
  UBaseType_t stack_size;
} CommandLineTaskArgs_t;

void Print(const char *str);

#include "driverlib/rom.h"
#include "inc/hw_types.h"
#include "inc/hw_nvic.h"
#include "inc/hw_memmap.h"

// #include "Semaphore.h"
#define SCRATCH_SIZE 512
static char m[SCRATCH_SIZE];

// this command takes no arguments and never returns.
__attribute__((noreturn)) static BaseType_t bl_ctl(int argc, char **argv, char *m)
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

static BaseType_t help_command_fcn(int argc, char **, char *m);

////////////////////////////////////////////////////////////////////////

static const char *const pcWelcomeMessage =
    "CLI based on microrl.\r\nType \"help\" to view a list of registered commands.\r\n";

struct command_t {
  const char *commandstr;
  BaseType_t (*interpreter)(int argc, char **, char *m);
  const char *helpstr;
  const int num_args;
};

#define NUM_COMMANDS (sizeof(commands) / sizeof(commands[0]))
static struct command_t commands[] = {
    {"bootloader", bl_ctl, "Call bootloader\r\n", 0},
    {"help", help_command_fcn, "This help command\r\n", -1},
};

static void U0Print(const char *str)
{
  UARTPrint(UART0_BASE, str);
}

struct microrl_user_data_t {
  uint32_t uart_base;
};

static BaseType_t help_command_fcn(int argc, char **argv, char *m)
{
  int copied = 0;
  if (argc == 1) {
    static int i = 0;
    for (; i < NUM_COMMANDS; ++i) {
      int left = SCRATCH_SIZE - copied;
      // need room for command string, help string, newlines, etc, and trailing \0
      unsigned int len = strlen(commands[i].helpstr) + strlen(commands[i].commandstr) + 7;
      if (left < len) {
        return pdTRUE;
      }
      copied += snprintf(m + copied, SCRATCH_SIZE - copied, "%s:\r\n %s",
                         commands[i].commandstr, commands[i].helpstr);
    }
    i = 0;
    return pdFALSE;
  }
  else { // help on a specific command.
    // help for any command that matches the entered command
    static int j = 0;
    for (; j < NUM_COMMANDS; ++j) {
      if (strncmp(commands[j].commandstr, argv[1], strlen(argv[1])) == 0) {
        int left = SCRATCH_SIZE - copied;
        // need room for command string, help string, newlines, etc, and trailing \0
        unsigned int len = strlen(commands[j].helpstr) + strlen(commands[j].commandstr) + 7;
        if (left < len) {
          return pdTRUE;
        }
        copied += snprintf(m + copied, SCRATCH_SIZE - copied, "%s:\r\n %s",
                           commands[j].commandstr, commands[j].helpstr);
      }
    }
    j = 0;
    if (copied == 0) {
      snprintf(m + copied, SCRATCH_SIZE - copied,
               "%s: No command starting with %s found\r\n", argv[0], argv[1]);
    }
    return pdFALSE;
  }
}

static int execute(void *p, int argc, char **argv)
{
  struct microrl_user_data_t *userdata = p;
  uint32_t base = userdata->uart_base;

  UARTPrint(base, "\r\n"); // the microrl does not terminate the active command

  // find the command in the list
  // argc here includes the actual command itself, so the
  // number of supplied arguments is argc-1
  for (int i = 0; i < NUM_COMMANDS; ++i) {
    if (strncmp(commands[i].commandstr, argv[0], 256) == 0) {
      if ((argc == commands[i].num_args + 1) || commands[i].num_args < 0) {
        int retval = commands[i].interpreter(argc, argv, m);
        if (m[0] != '\0')
          UARTPrint(base, m);
        while (retval == pdTRUE) {
          retval = commands[i].interpreter(argc, argv, m);
          if (m[0] != '\0')
            UARTPrint(base, m);
        }
        m[0] = '\0';
        return 0;
      }
      else {
        snprintf(m, SCRATCH_SIZE,
                 "Wrong number of arguments for command %s: %d expected, got %d\r\n", argv[0],
                 commands[i].num_args, argc - 1);
        UARTPrint(base, m);
        return 0;
      }
    }
  }
  UARTPrint(base, "Command unknown: ");
  UARTPrint(base, argv[0]);
  UARTPrint(base, "\r\n");

  return 0;
}

// The actual task
void vCommandLineTask(void *pvParameters)
{
  uint8_t cRxedChar;

  CommandLineTaskArgs_t *args = pvParameters;
  StreamBufferHandle_t uartStreamBuffer = args->UartStreamBuffer;
  uint32_t uart_base = args->uart_base;

  UARTPrint(uart_base, pcWelcomeMessage);
  struct microrl_user_data_t rl_userdata = {
      .uart_base = uart_base,
  };

  void (*printer)(const char *) = U0Print;

  struct microrl_config rl_config = {
      .print = printer, // default to front panel
      // set callback for execute
      .execute = execute,
      .prompt_str = "% ",
      .prompt_length = 2,
      .userdata = &rl_userdata,
  };
  microrl_t rl;
  microrl_init(&rl, &rl_config);
  microrl_set_execute_callback(&rl, execute);
  microrl_insert_char(&rl, ' '); // this seems to be necessary?

  for (;;) {
    /* This implementation reads a single character at a time.  Wait in the
       Blocked state until a character is received. */
    xStreamBufferReceive(uartStreamBuffer, &cRxedChar, 1, portMAX_DELAY);
    microrl_insert_char(&rl, cRxedChar);

    // monitor stack usage for this task
    // CHECK_TASK_STACK_USAGE(args->stack_size);
  }
}
