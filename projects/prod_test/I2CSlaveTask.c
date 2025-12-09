/*
 * I2CSlaveTask.c
 *
 *  Created on: Nov 6, 2019
 *      Author: pw94 (Peter Wittich)
 */
// includes for types
#include <stdint.h>
#include <stdbool.h>

#include <string.h>

#include "inc/hw_memmap.h"
#include "driverlib/rom.h" // to be removed
#include "common/printf.h"
#include "InterruptHandlers.h"

#if defined(REV1) || defined(REV2) || defined(REV3) // why is this here
#define SLAVE_I2C_BASE I2C0_BASE
#endif


#define REG_MAP_SIZE 256
extern volatile uint8_t g_ui8SlaveRegisters[REG_MAP_SIZE];

#include "commands.h"

void Print(const char *str);
// Parse space-separated args in-place. Returns argc.
static int parse_args(char *line, char **argv, int max_argv)
{
  int argc = 0;
  bool in_tok = false;

  for (char *p = line; *p && argc < max_argv; ++p) {
    if (*p == ' ' || *p == '\t' || *p == '\r' || *p == '\n') {
      *p = '\0';
      in_tok = false;
    } else if (!in_tok) {
      argv[argc++] = p;
      in_tok = true;
    }
  }
  return argc;
}


extern struct command_t commands[];
extern const int NUM_COMMANDS;
char m[SCRATCH_SIZE];

void I2CSlaveTask(void *parameters)
{
  TaskNotifyI2CSlave = xTaskGetCurrentTaskHandle();
  const uint8_t CMD_REG_ADDR = 0x10;
  const uint8_t CMD_RETURN_ADDR = 0x11;
  const uint8_t CMD_ARGS_ADDR = 0x20;
  memset((void*)g_ui8SlaveRegisters, '\0', REG_MAP_SIZE); // zero out registers
  snprintf((char *)g_ui8SlaveRegisters+CMD_ARGS_ADDR, REG_MAP_SIZE - CMD_ARGS_ADDR, "I2C Slave Ready");

  ROM_I2CSlaveEnable(SLAVE_I2C_BASE);

  uint32_t registerAddress; // This will hold the register address

  // Save this task's handle to the global variable so the ISR can find us
  TaskNotifyI2CSlave = xTaskGetCurrentTaskHandle();

  // loop forever
  for (;;) {
    // Block indefinitely (portMAX_DELAY) until the ISR sends a notification
    // 0x00, 0xFFffffff = Clear bits on entry/exit (standard usage)
    if (xTaskNotifyWait(0x00, 0xFFFFFFFF, &registerAddress, portMAX_DELAY) == pdTRUE) {
      // ui32NotificationValue now contains the Register Index that changed
      snprintf(m, SCRATCH_SIZE, "Received a notification for register 0x%02x\r\n", registerAddress);
      Print(m);
      // Check if the modified register was our Command Register
      if (registerAddress == CMD_REG_ADDR) {
        uint8_t cmd = g_ui8SlaveRegisters[CMD_REG_ADDR];

        // Process Command
        if (cmd >= NUM_COMMANDS) {
          snprintf(m,SCRATCH_SIZE, "Received invalid command 0x%02x\r\n", cmd);
          Print(m);
          continue;
        }
        struct command_t c = commands[cmd];
        snprintf(m, SCRATCH_SIZE, "Processing command 0x%02x: %s\r\n", cmd, c.commandstr);
        Print(m);
        // if the command requires arguments, they should be stored in the CMD_ARGS_ADDR onwards
        // argc parsing is done by the interpreter
        char args[80];
        char *argv[10]; int argc;
        if ( c.num_args != 0 ) {
          memcpy((uint8_t *)g_ui8SlaveRegisters + CMD_ARGS_ADDR, args, 80);
          args[79] = '\0'; // ensure null termination
          // parse the args into argv
          argc = parse_args(args, argv, 10);
        }
        else {
          argc = 0;
          args[0] = '\0';
        }


        // Execute the command
        char response[512] = {0};
        int ret = c.interpreter(argc, argv, response);
        if (ret != 0) {
          snprintf(m, SCRATCH_SIZE, "Command 0x%02x returned error %d\r\n", cmd, ret);
          Print(m);
        }
        if (response[0] != '\0') {
          snprintf(m, SCRATCH_SIZE, "Command 0x%02x response: %s\r\n", cmd, response);
          Print(m);
        }
        // Copy response to the register map starting at 0x20
        size_t resp_len = strlen(response);
        size_t copy_len = (resp_len < (REG_MAP_SIZE - 0x20)) ? resp_len : (REG_MAP_SIZE - 0x20);
        memcpy((char *)g_ui8SlaveRegisters + 0x20, response, copy_len);
        // set the return value in register CMD_RETURN_ADDR
        g_ui8SlaveRegisters[CMD_RETURN_ADDR] = (uint8_t)ret;

        // Optional: Clear the register to "acknowledge" the command
        g_ui8SlaveRegisters[CMD_REG_ADDR] = 0x00;
      }
    }
  }
}
