/*
 * ClockI2CCommands.c
 *
 *  Created on: January 10, 2025
 *      Author: mcoshiro
 *
 * Contains code for MCU to clock synth I2C tests used for Apollo CM production
 * tests
 */

// includes for types
#include <stdbool.h>
#include <stdint.h>

// FreeRTOS
#include "FreeRTOS.h"
#include "task.h"

// local includes
#include "common/smbus.h"
#include "common/pinsel.h"
#include "common/printf.h"
#include "common/utils.h"
#include "commands.h"
#include "InterruptHandlers.h"
#include "ClockI2CCommands.h"
#include "I2CUtils.h"

// device info, should this need to be moved somewhere?
struct dev_moni2c_addr_t clk_moni2c_addrs[NDEVICES_CLK] = {
    {"r0a", 0x70, 0, 0x77, 0x45D},  // CLK R0A : Si5341-REVD with #regs = 378 (read at 0x1F7D in EEPROM) if change, addr 0x45D will have to change
    {"r0b", 0x70, 1, 0x77, 0x264E}, // CLK R0B : Si5395-REVA #regs = 587 (read at 0x1F7D in EEPROM) if change, addr 0x264E will have to change
    {"r1a", 0x70, 2, 0x77, 0x464E}, // CLK R1A : Si5395-REVA #regs = 587 (read at 0x5F7D in EEPROM) if change, addr 0x464E will have to change
    {"r1b", 0x70, 3, 0x77, 0x664E}, // CLK R1B : Si5395-REVA #regs = 584 (read at 0x7F7D in EEPROM) if change, addr 0x664E will have to change
    {"r1c", 0x70, 4, 0x77, 0x864E}, // CLK R1C : Si5395-REVA #regs = 587 (read at 0x9F7D in EEPROM) if change, addr 0x864E will have to change
};
// TODO check stuff, addresses (except r0a) were 0x6b in cm_mcu, but seem like they should be 0x77 based on Rev2 and Rev3 schematics

/**
 * @details
 * Tests I2C communication to clock synth chips by first performing a loop
 * where some (distinct) data is written to a user scratch register on each
 * clock synth, then a second loop reads the data and verifies taht it matches
 * what was written. Finally, the MUX reset signal is tested by checking a read
 * attempt fails following a MUX reset
 */
BaseType_t run_clock_i2ctest(int argc, char **argv, char *m)
{
  // TODO check power is up: currently there is no PowerSupplyTask or similar
  // so check with Peter what to do here

  uint8_t data[2];
  tSMBusStatus r;
  int copied = 0;

  // do two passes, write the first time and read the second
  for (uint8_t rw = 0; rw < 2; ++rw) {

    // loop over devices
    for (uint8_t idev = 0; idev < NDEVICES_CLK; ++idev) {

      // select the appropriate output for the mux
      data[0] = 0x1U << clk_moni2c_addrs[idev].mux_bit;
      r = SMBusMasterI2CWrite(&g_sMaster2, clk_moni2c_addrs[idev].mux_addr,
                              data, 1);
      copied = check_i2c_transaction(r, 500, false, &g_sMaster2, eStatus2, m);
      if (copied != 0) {
        snprintf(m + copied, SCRATCH_SIZE - copied,
                 "(selecting dev %d on MUX)\r\n", idev);
        return pdFALSE;
      }

      // select page
      data[0] = SI5395_ADDR_PAGESEL;
      data[1] = SI5395_ADDR_SCRATCH_UPPER;
      r = SMBusMasterI2CWrite(&g_sMaster2, clk_moni2c_addrs[idev].dev_addr,
                              data, 2);
      copied = check_i2c_transaction(r, 500, false, &g_sMaster2, eStatus2, m);
      if (copied != 0) {
        snprintf(m + copied, SCRATCH_SIZE - copied,
                 "(selecting page on dev %d)\r\n", idev);
        return pdFALSE;
      }

      // write on first pass
      if (rw == 0) {
        data[0] = SI5395_ADDR_SCRATCH_LOWER0;
        data[1] = idev + 1;
        r = SMBusMasterI2CWrite(&g_sMaster2, clk_moni2c_addrs[idev].dev_addr,
                                data, 2);
      }
      // read on second pass
      else {
        data[0] = SI5395_ADDR_SCRATCH_LOWER0;
        data[1] = 0x0U;
        r = SMBusMasterI2CWriteRead(&g_sMaster2,
                                    clk_moni2c_addrs[idev].dev_addr, &data[0],
                                    1, &data[1], 1);
      }
      copied = check_i2c_transaction(r, 500, false, &g_sMaster2, eStatus2, m);
      if (copied != 0) {
        snprintf(m + copied, SCRATCH_SIZE - copied,
                 "(read/write %d, dev %d)\r\n", rw, idev);
        return pdFALSE;
      }

      if (rw == 0) {
        // check read value
        if (data[1] != (idev + 1)) {
          snprintf(m, SCRATCH_SIZE,
                   "ERROR: Bad readback on dev %d (expected %d, got %d)\r\n",
                   idev, idev + 1, data[0]);
          return pdFALSE;
        }
      }

      // wait here for 10 msec
      vTaskDelay(pdMS_TO_TICKS(10));
    } // loop over devices
  } // read/write passes

  // DEBUG: temporarily don't reset to purposely fail
  //  test reset by attempting read; as long as we don't use an address 0x7X,
  //  we shouldn't accidentally address the MUX
  // write_gpio_pin(_CLOCKS_I2C_RESET, 0x0);
  // vTaskDelay(pdMS_TO_TICKS(1));
  // write_gpio_pin(_CLOCKS_I2C_RESET, 0x1);

  bool read_fail = false;
  // select page
  data[0] = SI5395_ADDR_PAGESEL;
  data[1] = SI5395_ADDR_OPN_UPPER;
  r = SMBusMasterI2CWrite(&g_sMaster2,
                          clk_moni2c_addrs[NDEVICES_CLK - 1].dev_addr, data, 2);
  copied = check_i2c_transaction(r, 500, true, &g_sMaster2, eStatus2, m);
  if (copied == 1) {
    read_fail = true;
  }
  // read OPN register
  data[0] = SI5395_ADDR_OPN_LOWER0;
  data[1] = 0x0U;
  r = SMBusMasterI2CWriteRead(&g_sMaster2,
                              clk_moni2c_addrs[NDEVICES_CLK - 1].dev_addr,
                              &data[0], 1, &data[1], 1);
  copied = check_i2c_transaction(r, 500, true, &g_sMaster2, eStatus2, m);
  if (copied == 1) {
    read_fail = true;
  }
  // check value read
  if (!read_fail) {
    if (data[0] != SI5395_OPN0) {
      read_fail = true;
    }
  }
  if (!read_fail) {
    snprintf(m, SCRATCH_SIZE, "ERROR: I2C MUX reset did not work.\r\n");
    return pdFALSE;
  }

  // TODO also check the TCA9555 devices and resets

  // print output
  snprintf(m, SCRATCH_SIZE, "Test success.\r\n");
  return pdFALSE;
}
