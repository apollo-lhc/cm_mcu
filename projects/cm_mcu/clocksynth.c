/*
 * clocksynth.c
 *
 *  Created on: Jul 30, 2020
 *      Author: rzou
 */
#include <assert.h>
#include <string.h>

#include "MonitorTaskI2C.h"
#include "common/smbus_helper.h"
#include "clocksynth.h"
#include "I2CCommunication.h"
#include "Tasks.h"

#if defined(REV2) || defined(REV3)


// must grab and release the semaphore in a larger scope when calling this function
int clear_clk_stickybits(void)
{
  static_assert(((CLOCK_I2C_BASE == 1) || (CLOCK_I2C_BASE == 2) || (CLOCK_I2C_BASE == 3) ||
                 (CLOCK_I2C_BASE == 4) || (CLOCK_I2C_BASE == 6)),
                "Invalid I2C base");

  if (getPowerControlState() != POWER_ON) {
    return 1;
  }

  for (int i = 0; i < NDEVICES_CLK; ++i) {
    // set the mux
    int res = apollo_i2c_ctl_w(CLOCK_I2C_BASE, clk_moni2c_addrs[i].mux_addr, 1,
                               1 << clk_moni2c_addrs[i].mux_bit);
    if (res != 0) {
      log_warn(LOG_SERVICE, "Mux error %s, break (instance=%s)\r\n", SMBUS_get_error(res),
               clk_moni2c_addrs[i].name);
      return res;
    }
    // clear the sticky flag
    res = apollo_i2c_ctl_reg_w(CLOCK_I2C_BASE, clk_moni2c_addrs[i].dev_addr, 1,
                               CLOCK_SYNTH_STICKY_FLAG_REGISTER, 1, 0);
    if (res != 0) {
      log_warn(LOG_SERVICE, "Sticky flag error %s, break (instance=%s)\r\n", SMBUS_get_error(res),
               clk_moni2c_addrs[i].name);
      return res;
    }
  }
  return 0;
}

// return the string that corresponds to the programmed file. If
// there is an error, an empty string is returned.
// must grab and release the semaphore in a larger scope when calling this function
void getClockProgram(int device, char progname_clkdesgid[CLOCK_PROGNAME_REG_NAME],
                     char progname_eeprom[CLOCK_EEPROM_PROGNAME_REG_NAME])
{
  // first clear out the return value
  memset(progname_clkdesgid, '\0', CLOCK_PROGNAME_REG_NAME);
  memset(progname_eeprom, '\0', CLOCK_EEPROM_PROGNAME_REG_NAME);
  // ensure that the device is in the right range 0-5
  if (device < 0 || device > 4)
    return;

  // extract info about device
  uint8_t mux_addr, mux_bit, dev_addr;
  uint16_t eeprom_progname_reg;
  // In Rev2 device 0 and devices 1-5 are different and hence are stored in different arrays
  // for monitoring purposes
  mux_addr = clk_moni2c_addrs[device].mux_addr;
  mux_bit = clk_moni2c_addrs[device].mux_bit;
  dev_addr = clk_moni2c_addrs[device].dev_addr;
  eeprom_progname_reg = clk_moni2c_addrs[device].eeprom_progname_reg;
  // set mux bit
  int status = apollo_i2c_ctl_w(CLOCK_I2C_DEV, mux_addr, 1, 1 << mux_bit);
  if (status != 0) {
    log_debug(LOG_I2C, "mux write stat=%d\r\n", status); // can't return due to semaphore
  }
  else {
    // set the page
    uint8_t page = (CLOCK_PROGNAME_REG_ADDR_START >> 8) & 0xFF;
    status = apollo_i2c_ctl_reg_w(CLOCK_I2C_DEV, dev_addr, 1,
                                  CLOCK_CHANGEPAGE_REG_ADDR, 1, page);

    // now read out the six bytes of data in two reads
    const uint8_t reg = (CLOCK_PROGNAME_REG_ADDR_START) & 0xFF;
    uint16_t init_postamble_page = 32 * (device + 1) - 1;

    // read the addresses in EEPROM that store the number of registers in Preamble-register, Register, and Postamble-register list per a clock config file
    uint32_t PreambleList_row; // the size of preamble-register list of a given clock
    status += apollo_i2c_ctl_reg_r(CLOCK_I2C_DEV, CLOCK_I2C_EEPROM_ADDR, 2, (init_postamble_page << 8) + 0x007C, 1, &PreambleList_row);
    uint32_t RegisterList_row; // the size of register list of a given clock
    status += apollo_i2c_ctl_reg_r(CLOCK_I2C_DEV, CLOCK_I2C_EEPROM_ADDR, 2, (init_postamble_page << 8) + 0x007D, 2, &RegisterList_row);
    uint32_t PostambleList_row; // the size of postamble-register list of a given clock
    status += apollo_i2c_ctl_reg_r(CLOCK_I2C_DEV, CLOCK_I2C_EEPROM_ADDR, 2, (init_postamble_page << 8) + 0x007F, 1, &PostambleList_row);

    uint32_t eepromdata[2];
    // check if a given clock has been programmed or not from above set of three register values
    if (PreambleList_row == 0xff && RegisterList_row == 0xffff && PostambleList_row == 0xff) {
      // for an unprogrammed clock, an "X" will be shown in clkmon info
      eepromdata[0] = 'X';
      eepromdata[1] = '\0';
    }
    else {
      eepromdata[0] = 0UL;
      eepromdata[1] = 0UL;
      uint32_t tempdata[2];
      for (int i = 0; i < 4; ++i) {
        // four I2C transactions to read four value bytes of DESIGN_ID0-3 and DESIGN_ID4-7 clock chip addresses
        // as eepromdata[0] and eepromdat[1],respectively

        // third byte from three addresses in EEPROM is a value byte
        apollo_i2c_ctl_reg_r(CLOCK_I2C_DEV, CLOCK_I2C_EEPROM_ADDR, 2, eeprom_progname_reg + ((i) * 3), 3, tempdata);
        eepromdata[0] |= ((tempdata[0] >> (16)) & 0xFF) << (i * 8);
        // third byte from three addresses in EEPROM is a value byte
        apollo_i2c_ctl_reg_r(CLOCK_I2C_DEV, CLOCK_I2C_EEPROM_ADDR, 2, eeprom_progname_reg + 12 + ((i) * 3), 3, tempdata);
        eepromdata[1] |= ((tempdata[0] >> (16)) & 0xFF) << (i * 8);
      }
    }
    memcpy(progname_eeprom, eepromdata, CLOCK_EEPROM_PROGNAME_REG_COUNT);

    // read out the six bytes directly from the DESIGN_ID register
    uint32_t data[2];
    status += apollo_i2c_ctl_reg_r(CLOCK_I2C_DEV, dev_addr, 1, reg, 4, data);
    status += apollo_i2c_ctl_reg_r(CLOCK_I2C_DEV, dev_addr, 1, reg + 4, 4, data + 1);
    if (!status) // only copy if there were no errors on any reads
      memcpy(progname_clkdesgid, data, CLOCK_PROGNAME_REG_COUNT);
  }
}
#endif // REV2
