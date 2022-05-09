/*
 * EEPROMCommands.c
 *
 *  Created on: Jan 14, 2021
 *      Author: fatimayousuf
 */

#include "EEPROMCommands.h"

// This command takes 1 arg, the address
BaseType_t eeprom_read(int argc, char **argv, char* m)
{
  uint32_t addr;
  addr = strtol(argv[1], NULL, 16);
  uint32_t block = EEPROMBlockFromAddr(addr);

  uint64_t data = read_eeprom_multi(addr);
  uint32_t data2 = (uint32_t)(data >> 32);
  uint32_t data1 = (uint32_t)data;
  snprintf(m, SCRATCH_SIZE, "Data read from EEPROM block %ld: %08lx %08lx\r\n", block,
           data1, data2);

  return pdFALSE;
}

// This command takes 2 args, the address and 4 bytes of data to be written
BaseType_t eeprom_write(int argc, char **argv, char* m)
{
  int copied = 0;

  uint32_t data, addr;
  data = strtoul(argv[2], NULL, 16);
  addr = strtoul(argv[1], NULL, 16);
  uint32_t block = EEPROMBlockFromAddr(addr);
  if ((block == 1) || ((EBUF_MINBLK <= block) && (block <= EBUF_MAXBLK))) {
    copied += snprintf(m + copied, SCRATCH_SIZE - copied, "Please choose available block\r\n");
    return pdFALSE;
  }
  write_eeprom(data, addr);
  copied += snprintf(m + copied, SCRATCH_SIZE - copied, "Data written to EEPROM block %ld: %08lx\r\n",
                     block, data);

  return pdFALSE;
}

// Takes 0 arguments
BaseType_t eeprom_info(int argc, char **argv, char* m)
{
  int copied = 0;

  copied +=
      snprintf(m + copied, SCRATCH_SIZE - copied, "EEPROM has 96 blocks of 64 bytes each.\r\n");
  copied += snprintf(m + copied, SCRATCH_SIZE - copied, "Block 0 \t 0x0000-0x0040 \t Free.\r\n");
  copied += snprintf(m + copied, SCRATCH_SIZE - copied,
                     "Block 1 \t 0x0040-0x007c \t Apollo ID Information. Password: 0x12345678\r\n");
  copied += snprintf(m + copied, SCRATCH_SIZE - copied,
                     "Blocks %u-%u \t 0x%04x-0x%04x \t Error buffer.\r\n", EBUF_MINBLK, EBUF_MAXBLK,
                     EEPROMAddrFromBlock(EBUF_MINBLK), EEPROMAddrFromBlock(EBUF_MAXBLK + 1) - 4);

  return pdFALSE;
}
