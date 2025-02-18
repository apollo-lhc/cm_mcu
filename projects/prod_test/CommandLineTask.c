/*
 * CommandLineTask.c
 *
 *  Created on: Apr 7, 2019
 *      Author: wittich, rzou
 */

// Include commands
#include <string.h>
#include <strings.h>
// includes for types
#include <stdint.h>
#include <stdbool.h>

#include "FreeRTOS.h" // IWYU pragma: keep
#include "portmacro.h"
#include "stream_buffer.h"
#include "common/LocalUart.h"
#include "common/printf.h"
#include "common/microrl.h"
#include "commands.h"
#include "PowerI2CCommands.h"
#include "ClockI2CCommands.h"
#include "FireflyI2CCommands.h"
#include "EEPROMI2CCommands.h"
#include "FPGAI2CCommands.h"

typedef struct {
  StreamBufferHandle_t UartStreamBuffer;
  uint32_t uart_base;
  UBaseType_t stack_size;
} CommandLineTaskArgs_t;

#define NUM_COMMANDS (sizeof(commands) / sizeof(commands[0]))

struct command_t commands[] = {
    {"adc", adc_ctl, "Display ADC measurements", 0},
    {"bootloader", bl_ctl, "Call bootloader", 0},
    {"help", help_command_fcn, "This help command", -1},
    {"dcdci2ctest", dcdc_i2ctest_ctl, "Test I2C to DC-DC converters", 0},
    {"clocki2ctest", clock_i2ctest_ctl, "Test I2C to clock synths", 0},
    {"ffi2ctest", firefly_i2ctest_ctl, "Test I2C to optics", 0},
    {"eepromi2ctest", eeprom_i2ctest_ctl, "Test I2C to EEPROM", 0},
    {"fpgai2ctest", fpga_i2ctest_ctl, "Test I2C to FPGAs", 0},
    {"initclockreg", clock_ioexpanders_init_ctl, "Initialize IO expanders", 0},
    {"initffreg", firefly_ioexpanders_init_ctl, "Initialize IO expanders", 0},
    {"poweron", power_ctl, "power on at level n", 1},
    {"poweroff", power_off_ctl, "power off", 0},
    {"restart", restart_mcu, "restart the MCU", 0},
};

////////////////////////////////////////////////////////////////////////

static char m[SCRATCH_SIZE];

static const char *const pcWelcomeMessage =
    "CLI based on microrl.\r\nType \"help\" to view a list of registered commands.\r\n";

BaseType_t help_command_fcn(int argc, char **argv, char *m)
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
      copied += snprintf(m + copied, SCRATCH_SIZE - copied, "%s:\r\n %s\r\n",
                         commands[i].commandstr, commands[i].helpstr);
    }
    i = 0;
    return pdFALSE;
  }
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
      copied += snprintf(m + copied, SCRATCH_SIZE - copied, "%s:\r\n %s\r\n",
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
static void U0Print(const char *str)
{
  UARTPrint(UART0_BASE, str);
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
