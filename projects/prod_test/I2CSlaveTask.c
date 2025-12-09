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
#include "InterruptHandlers.h"
#include "common/log.h"

#if defined(REV1) || defined(REV2) || defined(REV3) // why is this here
#define SLAVE_I2C_BASE I2C0_BASE
#endif


#define REG_MAP_SIZE 256
extern volatile uint8_t g_ui8SlaveRegisters[REG_MAP_SIZE];

#include "commands.h"

extern struct command_t commands[];
extern const int NUM_COMMANDS;

void I2CSlaveTask(void *parameters)
{
  TaskNotifyI2CSlave = xTaskGetCurrentTaskHandle();
  const uint8_t CMD_REG_ADDR = 0x10;
  const uint8_t CMD_RETURN_ADDR = 0x11;
  const uint8_t CMD_ARGS_ADDR = 0x20;

  snprintf((char *)g_ui8SlaveRegisters+CMD_ARGS_ADDR, REG_MAP_SIZE - CMD_ARGS_ADDR, "I2C Slave Ready");

  ROM_I2CSlaveEnable(SLAVE_I2C_BASE);

  // loop forever
  uint32_t registerAddress; // This will hold the register address

  // Save this task's handle to the global variable so the ISR can find us
  TaskNotifyI2CSlave = xTaskGetCurrentTaskHandle();

  // loop forever
  for (;;) {
    // Block indefinitely (portMAX_DELAY) until the ISR sends a notification
    // 0x00, 0xFFffffff = Clear bits on entry/exit (standard usage)
    if (xTaskNotifyWait(0x00, 0xFFFFFFFF, &registerAddress, portMAX_DELAY) == pdTRUE) {
      // ui32NotificationValue now contains the Register Index that changed
      log_info(LOG_I2C, "Received a notifcation for register 0x%02x\r\n", registerAddress);

      // Check if the modified register was our Command Register
      if (registerAddress == CMD_REG_ADDR) {
        uint8_t cmd = g_ui8SlaveRegisters[CMD_REG_ADDR];

        // Process Command
        if (cmd >= NUM_COMMANDS) {
          log_warn(LOG_I2C, "Received invalid command 0x%02x\r\n", cmd);
          continue;
        }
        struct command_t c = commands[cmd];
        log_info(LOG_I2C, "Processing command 0x%02x: %s\r\n", cmd, c.commandstr);
        // if the command requires arguments, they should be stored in the CMD_ARGS_ADDR onwards
        // argc parsing is done by the interpreter
        char args[80];
        if ( c.num_args != 0 ) {
          memcpy((uint8_t *)g_ui8SlaveRegisters + CMD_ARGS_ADDR, args, 80);
          args[79] = '\0'; // ensure null termination
        }
        else {
          args[0] = '\0';
        }
        // parse the args into argv
        char *argv[10];
        int argc = 0;
        char *token = strtok(args, " ");
        while (token != NULL && argc < 10) {
          argv[argc++] = token;
          token = strtok(NULL, " ");
        }

        // Execute the command
        char response[256] = {0};
        int ret = c.interpreter(0, argv, response);
        if (ret != 0) {
          log_warn(LOG_I2C, "Command 0x%02x returned error %d\r\n", cmd, ret);
        }
        if (response[0] != '\0') {
          log_info(LOG_I2C, "Command 0x%02x response: %s\r\n", cmd, response);
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
