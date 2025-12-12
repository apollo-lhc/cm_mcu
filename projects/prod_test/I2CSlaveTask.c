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
extern volatile uint8_t slaveRegistersData[REG_MAP_SIZE];

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


extern const struct command_t commands[];
extern const int NUM_COMMANDS;
static char mm[SCRATCH_SIZE];
static char response[512]; // interpreter assumes the buffer has 512 bytes
static char args[80];

void I2CSlaveTask(void *parameters)
{
  TaskNotifyI2CSlave = xTaskGetCurrentTaskHandle();
  const uint8_t CMD_REG_ADDR = 0x10;
  const uint8_t CMD_RETURN_ADDR = 0x11;
  const uint8_t CMD_ARGS_ADDR = 0x20;
  const uint8_t CMD_MAX_ARGS_SIZE = 80;
  const uint8_t CMD_RETURN_MSG_ADDR = 0x70;
  const uint8_t CMD_RETURN_MSG_SIZE = 128;

  memset((void*)slaveRegistersData, '\0', REG_MAP_SIZE); // zero out registers
  snprintf((char *)slaveRegistersData+CMD_ARGS_ADDR, 16, "I2C Slave Ready");

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
      snprintf(mm, SCRATCH_SIZE, "Received a notification for register 0x%02x\r\n", registerAddress);
      Print(mm);
      // Check if the modified register was our Command Register
      if (registerAddress == CMD_REG_ADDR) {
        uint8_t cmd = slaveRegistersData[CMD_REG_ADDR];

        // Process Command
        if (cmd >= NUM_COMMANDS) {
          snprintf(mm,SCRATCH_SIZE, "Received invalid command 0x%02x\r\n", cmd);
          Print(mm);
          slaveRegistersData[CMD_RETURN_ADDR] = 0xffU; // invalid command
          continue;
        }
        struct command_t c = commands[cmd];
        snprintf(mm, SCRATCH_SIZE, "Processing command 0x%02x: %s\r\n", cmd, c.commandstr);
        Print(mm);
        // if the command requires arguments, they should be stored in the CMD_ARGS_ADDR onwards
        char *argv[10] = {0};
        argv[0] = (char*) c.commandstr; // first arg is the command itself
        int argc; // number of args excluding command
        if ( c.num_args != 0 ) {
          memset(args, 0, sizeof(args));
          memcpy(args, (uint8_t *)slaveRegistersData + CMD_ARGS_ADDR, CMD_MAX_ARGS_SIZE);
          args[CMD_MAX_ARGS_SIZE - 1] = '\0'; // ensure null termination
          snprintf(mm, SCRATCH_SIZE, "Command args: '%s'\r\n", args);
          Print(mm);
          // parse the args into argv
          argc = parse_args(args, argv+1, 9);
          if ( c.num_args > 0 && argc != c.num_args ) { // check for correct number of args. -1 means variable number of args
            snprintf(mm, SCRATCH_SIZE, "Wrong # args for cmd %s: %d expected, got %d\r\n", c.commandstr,
                     c.num_args, argc);
            Print(mm);
            // copy message to response buffer
            memset(response, 0, sizeof(response));
            size_t len = snprintf(response, sizeof(response), "Wrong # args for cmd %s: %d expected, got %d\r\n", c.commandstr,
                                 c.num_args, argc);
            len = (len < CMD_RETURN_MSG_SIZE) ? len : CMD_RETURN_MSG_SIZE;
            memcpy((uint8_t *)slaveRegistersData + CMD_RETURN_MSG_ADDR, response, len);
            slaveRegistersData[CMD_RETURN_MSG_ADDR + len] = '\0'; // null terminate
            // set return value
            slaveRegistersData[CMD_RETURN_ADDR] = (uint8_t)CLI_ERROR;
            // Clear the register to "acknowledge" the command
            slaveRegistersData[CMD_REG_ADDR] = 0x00;
            continue;
          }
          else if ( c.num_args < 0 && argc == 0 ) {
            args[1]='\0';
            argc = 0;
          }
        }
        else {
          argc = 0;
          args[1] = '\0';
        }


        // Execute the command
        memset(response, 0, sizeof(response));
        int retval;
        while ( (retval = c.interpreter(argc, argv, response)) == CLI_MORE ) {
          // keep calling while it returns MORE
          if ( response[0] != '\0' ) { // intermediate response
            Print(response); 
          }
        }
        if ( response[0] != '\0' ) { // final response
          Print(response); 
        }
        snprintf(mm, SCRATCH_SIZE, "retval: %s\r\n", (retval == CLI_OK) ? "OK" : "ERROR");
        Print(mm);
        
        // get the return value
        // Copy response to the register map starting at CMD_RETURN_MSG_ADDR
        // zero out the response area first
        memset((uint8_t *)slaveRegistersData + CMD_RETURN_MSG_ADDR, 0, CMD_RETURN_MSG_SIZE);
        size_t resp_len = strlen(response);
        size_t copy_len = (resp_len < CMD_RETURN_MSG_SIZE) ? resp_len : CMD_RETURN_MSG_SIZE;
        memcpy((char *)slaveRegistersData + CMD_RETURN_MSG_ADDR, response, copy_len);
        slaveRegistersData[CMD_RETURN_MSG_ADDR + copy_len] = '\0'; // null terminate
        // set the return value in register CMD_RETURN_ADDR
        slaveRegistersData[CMD_RETURN_ADDR] = (uint8_t)retval;

        // Clear the register to "acknowledge" the command
        slaveRegistersData[CMD_REG_ADDR] = 0x00;
      }
    }
  }
}
