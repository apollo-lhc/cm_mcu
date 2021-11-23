/*
 * BoardCommands.c
 *
 *  Created on: Jan 18, 2021
 *      Author: fatimayousuf
 */
#include <time.h>
#include "BoardCommands.h"
#include "common/pinsel.h"
#include "driverlib/hibernate.h"

// This command takes no arguments
BaseType_t restart_mcu(int argc, char **argv, char* m)
{
  int copied = 0;
  copied += snprintf(m + copied, SCRATCH_SIZE - copied, "Restarting MCU\r\n");
  MAP_SysCtlReset(); // This function does not return
  return pdFALSE;
}

// Takes 3 arguments
BaseType_t set_board_id(int argc, char **argv, char* m)
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
BaseType_t set_board_id_password(int argc, char **argv, char* m)
{
  int copied = 0;

  uint64_t message = EPRMMessage((uint64_t)EPRM_PASS_SET, 0, 0x12345678);
  xQueueSendToBack(xEPRMQueue_in, &message, portMAX_DELAY);

  copied += snprintf(m + copied, SCRATCH_SIZE - copied, "Block locked\r\n");

  return pdFALSE;
}

BaseType_t board_id_info(int argc, char **argv, char* m)
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
#if defined(REV2)
BaseType_t jtag_sm_ctl(int argc, char **argv, char *m)
{
  int copied = 0;
  if ( argc == 2 ) {
    if (strncmp(argv[1], "on", 2) == 0) {
      copied += snprintf(m + copied, SCRATCH_SIZE - copied, "JTAG from SM enabled\r\n");
      write_gpio_pin(JTAG_FROM_SM, 1);
    }
    else if (strncmp(argv[1], "off", 3) == 0) {
      copied += snprintf(m + copied, SCRATCH_SIZE - copied, "JTAG from SM disabled\r\n");
      write_gpio_pin(JTAG_FROM_SM, 0);
    } 
    else {
      copied += snprintf(m + copied, SCRATCH_SIZE - copied, "Usage: jtag_sm_ctl on/off (got %s)\r\n", argv[1]);
    }
  }
  else if ( argc == 3) {
	  BaseType_t pin = -1;
	  if (strncmp(argv[1], "f1", 2) == 0) {
		  pin = _F1_JTAG_BYPASS;
	  }
	  else if (strncmp(argv[1], "f2", 2) == 0) {
		  pin = _F2_JTAG_BYPASS;
	  }
	  else {
		  copied += snprintf(m+copied, SCRATCH_SIZE-copied, "%s: did not understand argument %s\r\n",
				  argv[0], argv[1]);
		  return pdFALSE;
	  }
	  BaseType_t val = 1;
	  if (strncmp(argv[2], "on", 2) == 0) {
		  val = 1;
	  }
	  else if (strncmp(argv[2], "off", 3) == 0) {
		  val = 0;
	  }
	  write_gpio_pin(pin, val);
  }
  else { // all other cases
    copied += snprintf(m + copied, SCRATCH_SIZE - copied, "JTAG from SM: %d\r\n", read_gpio_pin(JTAG_FROM_SM));
    copied += snprintf(m + copied, SCRATCH_SIZE - copied, "F1 BYPASS*: %d\r\n", read_gpio_pin(_F1_JTAG_BYPASS));
    copied += snprintf(m + copied, SCRATCH_SIZE - copied, "F2 BYPASS*: %d\r\n", read_gpio_pin(_F2_JTAG_BYPASS));
  }
  return pdFALSE;
}

BaseType_t time_ctl(int argc, char **argv, char *m)
{
  int copied = 0;
  if (argc == 3) {
    if (strncmp(argv[1], "set", 2) == 0) {
      copied += snprintf(m + copied, SCRATCH_SIZE - copied, "Does nothing yet (TM)\r\n");
      write_gpio_pin(JTAG_FROM_SM, 1);
    }
    else {
      copied += snprintf(m + copied, SCRATCH_SIZE - copied, "Usage: jtag_sm_ctl on/off (got %s)\r\n", argv[1]);
    }
  }
  else { // all other cases
    struct tm now;
    HibernateCalendarGet(&now);
    copied += snprintf(m + copied, SCRATCH_SIZE - copied, "Time now: %02d:%02d:%02d %02d/%02d/%d\r\n", 
      now.tm_hour, now.tm_min, now.tm_sec, now.tm_mon+1, now.tm_mday, now.tm_year+1900);
  }
  return pdFALSE;
}

#endif
