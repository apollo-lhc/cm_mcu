/*
 * BoardCommands.c
 *
 *  Created on: Jan 18, 2021
 *      Author: fatimayousuf
 */

#include "BoardCommands.h"

// This command takes no arguments
static BaseType_t restart_mcu(int argc, char **argv, char m)
{
  int copied = 0;
  copied += snprintf(m + copied, SCRATCH_SIZE - copied, "Restarting MCU\r\n");
  MAP_SysCtlReset(); // This function does not return
  return pdFALSE;
}

// Takes 3 arguments
static BaseType_t set_board_id(int argc, char **argv, char m)
{
  int copied = 0;

  uint64_t pass, addr, data;
  pass = strtoul(argv[1], NULL, 16);
  addr = strtoul(argv[2], NULL, 16);
  data = strtoul(argv[3], NULL, 16);
  uint64_t block = EEPROMBlockFromAddr(addr);
  if (block != 1) {
    copied += snprintf(m + copied, SCRATCH_SIZE - copied, "Please input address in Block 1\r\n");
    return pdFALSE;
  }

  uint64_t unlock = EPRMMessage((uint64_t)EPRM_UNLOCK_BLOCK, block, pass);
  xQueueSendToBack(xEPRMQueue_in, &unlock, portMAX_DELAY);

  uint64_t message = EPRMMessage((uint64_t)EPRM_WRITE_SINGLE, addr, data);
  xQueueSendToBack(xEPRMQueue_in, &message, portMAX_DELAY);

  uint64_t lock = EPRMMessage((uint64_t)EPRM_LOCK_BLOCK, block << 32, 0);
  xQueueSendToBack(xEPRMQueue_in, &lock, portMAX_DELAY);

  if (pass != 0x12345678) {
    copied += snprintf(m + copied, SCRATCH_SIZE - copied,
                       "Wrong password. Type eeprom_info to get password.");
  } // data not printing correctly?

  return pdFALSE;
}

// one-time use, has one function and takes 0 arguments
static BaseType_t set_board_id_password(int argc, char **argv, char m)
{
  int copied = 0;

  uint64_t message = EPRMMessage((uint64_t)EPRM_PASS_SET, 0, 0x12345678);
  xQueueSendToBack(xEPRMQueue_in, &message, portMAX_DELAY);

  copied += snprintf(m + copied, SCRATCH_SIZE - copied, "Block locked\r\n");

  return pdFALSE;
}

static BaseType_t board_id_info(int argc, char **argv, char m)
{
  int copied = 0;
  ;

  uint32_t sn = read_eeprom_single(EEPROM_ID_SN_ADDR);
  uint32_t ff = read_eeprom_single(EEPROM_ID_FF_ADDR);

  uint32_t num = (uint32_t)sn >> 16;
  uint32_t rev = ((uint32_t)sn) & 0xff;

  copied += snprintf(m + copied, SCRATCH_SIZE - copied, "ID:%08x\r\n", (uint32_t)sn);

  copied += snprintf(m + copied, SCRATCH_SIZE - copied, "Board number: %x\r\n", num);
  copied += snprintf(m + copied, SCRATCH_SIZE - copied, "Revision: %x\r\n", rev);
  copied += snprintf(m + copied, SCRATCH_SIZE - copied, "Firefly config: %x\r\n", ff);

  return pdFALSE;
}
