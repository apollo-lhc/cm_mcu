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
#include "inc/hw_hibernate.h"
BaseType_t time_ctl(int argc, char **argv, char *m)
{
  int copied = 0;
  if (argc == 4) {
    if (strncmp(argv[1], "set", 3) == 0) {
      // we don't have access to sscanf, let alone strptime, since it requires _sbrk ...
      // convert HH:MM:SS into three strings
      char *p = argv[2];
      char *pp[3];
      pp[0] = p;
      int i = 1;
      while ( *p != '\0') {
        if ( *p == ':' ) {
          pp[i++] = p+1;
          *p = '\0';
        }
        ++p;
      }
      BaseType_t hour, min, sec;
      hour = atoi(pp[0]);
      min = atoi(pp[1]);
      sec = atoi(pp[2]);
      // now convert MM/DD/YY into three strings
      p = argv[3];
      pp[0] = p;
      i = 1;
      while ( *p != '\0') {
        if ( *p == '/' ) {
          pp[i++] = p+1;
          *p = '\0';
        }
        ++p;
      }

      BaseType_t month, year, day;

      month = atoi(pp[0]);
      day = atoi(pp[1]);
      year = atoi(pp[2]);
      struct tm t;
      t.tm_hour = hour;
      t.tm_min = min;
      t.tm_sec = sec;
      t.tm_year = year>=100?year-1900:year+1900; // years since 1900
      t.tm_mday = day;
      t.tm_mon = month-1; // month goes from 0-11

      copied += snprintf(m + copied, SCRATCH_SIZE - copied, "New time: %02d:%02d:%02d %02d/%02d/%d\r\n",
          t.tm_hour, t.tm_min, t.tm_sec, t.tm_mon+1, t.tm_mday, t.tm_year+1900);
      ROM_HibernateCalendarSet(&t);
    }
    else {
      copied += snprintf(m + copied, SCRATCH_SIZE - copied, "Usage: %s set HH:MM:SS MM:DD:YYYY\r\n", argv[0]);
    }
  }
  else { // all other cases
    uint32_t ui32Date = HWREG(HIB_CAL1);
    if ( ! (ui32Date & HIB_CAL1_VALID )) {
      copied += snprintf(m+copied, SCRATCH_SIZE-copied, "%s: RTC state invalid\r\n", argv[0]);
    }
    else {
    	struct tm now;
    	ROM_HibernateCalendarGet(&now);
    	copied += snprintf(m + copied, SCRATCH_SIZE - copied, "Time now: %02d:%02d:%02d %02d/%02d/%d\r\n",
    			now.tm_hour, now.tm_min, now.tm_sec, now.tm_mon+1, now.tm_mday, now.tm_year+1900);
    }
  }
  return pdFALSE;
}

#endif
