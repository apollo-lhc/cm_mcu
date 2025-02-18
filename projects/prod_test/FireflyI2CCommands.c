/*
 * FireflyI2CCommands.c
 *
 *  Created on: February 3, 2025
 *      Author: mcoshiro
 *
 * Contains code for MCU to firefly I2C tests for Apollo CM production tests
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
#include "FireflyI2CCommands.h"
#include "I2CCommunication.h"

// device info
// dev_class field represents whether device is installed, so this will need
// to be changed for the actual configuration of fireflies we decide to
// install for the production tests

// 3002 current setup
struct dev_ff_i2c_addr_t ff_addrs[NDEVICES_FF] = {
    {"F1_FF1_XMIT", F1FF_I2C_BASE, FF_I2C_MUX1_ADDR, FF_I2C_F1_FF1_T_MUX_BIT,
     0, DEV_NONE},
    {"F1_FF1_RECV", F1FF_I2C_BASE, FF_I2C_MUX1_ADDR, FF_I2C_F1_FF1_R_MUX_BIT,
     0, DEV_NONE},
    {"F1_FF5_XCVR", F1FF_I2C_BASE, FF_I2C_MUX1_ADDR, FF_I2C_F1_FF5_B_MUX_BIT,
     0, DEV_FF_B04},
    {"F1_FF2_XMIT", F1FF_I2C_BASE, FF_I2C_MUX1_ADDR, FF_I2C_F1_FF2_T_MUX_BIT,
     0, DEV_FF_TX},
    {"F1_FF2_RECV", F1FF_I2C_BASE, FF_I2C_MUX1_ADDR, FF_I2C_F1_FF2_R_MUX_BIT,
     0, DEV_FF_RX},
    {"F1_FF3_XMIT", F1FF_I2C_BASE, FF_I2C_MUX2_ADDR, FF_I2C_F1_FF3_T_MUX_BIT,
     0, DEV_NONE},
    {"F1_FF3_RECV", F1FF_I2C_BASE, FF_I2C_MUX2_ADDR, FF_I2C_F1_FF3_R_MUX_BIT,
     0, DEV_NONE},
    {"F1_FF6_RECV", F1FF_I2C_BASE, FF_I2C_MUX2_ADDR, FF_I2C_F1_FF6_B_MUX_BIT,
     0, DEV_NONE},
    {"F1_FF4_XMIT", F1FF_I2C_BASE, FF_I2C_MUX2_ADDR, FF_I2C_F1_FF4_T_MUX_BIT,
     0, DEV_NONE},
    {"F1_FF4_RECV", F1FF_I2C_BASE, FF_I2C_MUX2_ADDR, FF_I2C_F1_FF4_R_MUX_BIT,
     0, DEV_NONE},
    {"F2_FF1_XMIT", F2FF_I2C_BASE, FF_I2C_MUX1_ADDR, FF_I2C_F2_FF1_T_MUX_BIT,
     0, DEV_NONE},
    {"F2_FF1_RECV", F2FF_I2C_BASE, FF_I2C_MUX1_ADDR, FF_I2C_F2_FF1_R_MUX_BIT,
     0, DEV_NONE},
    {"F2_FF5_XCVR", F2FF_I2C_BASE, FF_I2C_MUX1_ADDR, FF_I2C_F2_FF5_B_MUX_BIT,
     0, DEV_FF_B04},
    {"F2_FF2_XMIT", F2FF_I2C_BASE, FF_I2C_MUX1_ADDR, FF_I2C_F2_FF2_T_MUX_BIT,
     0, DEV_FF_TX},
    {"F2_FF2_RECV", F2FF_I2C_BASE, FF_I2C_MUX1_ADDR, FF_I2C_F2_FF2_R_MUX_BIT,
     0, DEV_FF_RX},
    {"F2_FF3_XMIT", F2FF_I2C_BASE, FF_I2C_MUX2_ADDR, FF_I2C_F2_FF3_T_MUX_BIT,
     0, DEV_NONE},
    {"F2_FF3_RECV", F2FF_I2C_BASE, FF_I2C_MUX2_ADDR, FF_I2C_F2_FF3_R_MUX_BIT,
     0, DEV_NONE},
    {"F2_FF6_RECV", F2FF_I2C_BASE, FF_I2C_MUX2_ADDR, FF_I2C_F2_FF6_B_MUX_BIT,
     0, DEV_NONE},
    {"F2_FF4_XMIT", F2FF_I2C_BASE, FF_I2C_MUX2_ADDR, FF_I2C_F2_FF4_T_MUX_BIT,
     0, DEV_NONE},
    {"F2_FF4_RECV", F2FF_I2C_BASE, FF_I2C_MUX2_ADDR, FF_I2C_F2_FF4_R_MUX_BIT,
     0, DEV_NONE},
};

// setup with all fireflies installed
// struct dev_i2c_addr_t ff_addrs[NDEVICES_FF] = {
//    {"F1_FF1_XMIT", F1FF_I2C_BASE, FF_I2C_MUX1_ADDR, FF_I2C_F1_FF1_T_MUX_BIT,
//     0, DEV_FF_TX},
//    {"F1_FF1_RECV", F1FF_I2C_BASE, FF_I2C_MUX1_ADDR, FF_I2C_F1_FF1_R_MUX_BIT,
//     0, DEV_FF_RX},
//    {"F1_FF5_XCVR", F1FF_I2C_BASE, FF_I2C_MUX1_ADDR, FF_I2C_F1_FF5_B_MUX_BIT,
//     0, DEV_FF_B04},
//    {"F1_FF2_XMIT", F1FF_I2C_BASE, FF_I2C_MUX1_ADDR, FF_I2C_F1_FF2_T_MUX_BIT,
//     0, DEV_FF_TX},
//    {"F1_FF2_RECV", F1FF_I2C_BASE, FF_I2C_MUX1_ADDR, FF_I2C_F1_FF2_R_MUX_BIT,
//     0, DEV_FF_RX},
//    {"F1_FF3_XMIT", F1FF_I2C_BASE, FF_I2C_MUX2_ADDR, FF_I2C_F1_FF3_T_MUX_BIT,
//     0, DEV_FF_TX},
//    {"F1_FF3_RECV", F1FF_I2C_BASE, FF_I2C_MUX2_ADDR, FF_I2C_F1_FF3_R_MUX_BIT,
//     0, DEV_FF_RX},
//    {"F1_FF6_RECV", F1FF_I2C_BASE, FF_I2C_MUX2_ADDR, FF_I2C_F1_FF6_B_MUX_BIT,
//     0, DEV_FF_B04},
//    {"F1_FF4_XMIT", F1FF_I2C_BASE, FF_I2C_MUX2_ADDR, FF_I2C_F1_FF4_T_MUX_BIT,
//     0, DEV_FF_TX},
//    {"F1_FF4_RECV", F1FF_I2C_BASE, FF_I2C_MUX2_ADDR, FF_I2C_F1_FF4_R_MUX_BIT,
//     0, DEV_FF_RX},
//    {"F2_FF1_XMIT", F2FF_I2C_BASE, FF_I2C_MUX1_ADDR, FF_I2C_F2_FF1_T_MUX_BIT,
//     0, DEV_FF_TX},
//    {"F2_FF1_RECV", F2FF_I2C_BASE, FF_I2C_MUX1_ADDR, FF_I2C_F2_FF1_R_MUX_BIT,
//     0, DEV_FF_RX},
//    {"F2_FF5_XCVR", F2FF_I2C_BASE, FF_I2C_MUX1_ADDR, FF_I2C_F2_FF5_B_MUX_BIT,
//     0, DEV_FF_B04},
//    {"F2_FF2_XMIT", F2FF_I2C_BASE, FF_I2C_MUX1_ADDR, FF_I2C_F2_FF2_T_MUX_BIT,
//     0, DEV_FF_TX},
//    {"F2_FF2_RECV", F2FF_I2C_BASE, FF_I2C_MUX1_ADDR, FF_I2C_F2_FF2_R_MUX_BIT,
//     0, DEV_FF_RX},
//    {"F2_FF3_XMIT", F2FF_I2C_BASE, FF_I2C_MUX2_ADDR, FF_I2C_F2_FF3_T_MUX_BIT,
//     0, DEV_FF_TX},
//    {"F2_FF3_RECV", F2FF_I2C_BASE, FF_I2C_MUX2_ADDR, FF_I2C_F2_FF3_R_MUX_BIT,
//     0, DEV_FF_RX},
//    {"F2_FF6_RECV", F2FF_I2C_BASE, FF_I2C_MUX2_ADDR, FF_I2C_F2_FF6_B_MUX_BIT,
//     0, DEV_FF_B04},
//    {"F2_FF4_XMIT", F2FF_I2C_BASE, FF_I2C_MUX2_ADDR, FF_I2C_F2_FF4_T_MUX_BIT,
//     0, DEV_FF_TX},
//    {"F2_FF4_RECV", F2FF_I2C_BASE, FF_I2C_MUX2_ADDR, FF_I2C_F2_FF4_R_MUX_BIT,
//     0, DEV_FF_RX},
//};

struct dev_ff_i2c_addr_t ff_ioexp_addrs[NDEVICES_FF_IOEXPANDER] = {
    {"F1_IOEXP1", F1FF_I2C_BASE, FF_I2C_MUX1_ADDR, FF_I2C_F1_IOEXP1_MUX_BIT,
     IOEXP1_I2C_ADDR, DEV_IOEXP},
    {"F1_IOEXP2", F1FF_I2C_BASE, FF_I2C_MUX2_ADDR, FF_I2C_F1_IOEXP2_MUX_BIT,
     IOEXP2_I2C_ADDR, DEV_IOEXP},
    {"F2_IOEXP1", F2FF_I2C_BASE, FF_I2C_MUX1_ADDR, FF_I2C_F2_IOEXP1_MUX_BIT,
     IOEXP1_I2C_ADDR, DEV_IOEXP},
    {"F2_IOEXP2", F2FF_I2C_BASE, FF_I2C_MUX2_ADDR, FF_I2C_F2_IOEXP2_MUX_BIT,
     IOEXP2_I2C_ADDR, DEV_IOEXP},
};

// additional constants for IOexpander test, again based on which devices are
// installed

const uint8_t ioexp_present_addrs[4] = {TCA9555_ADDR_INPORT1,
                                        TCA9555_ADDR_INPORT0,
                                        TCA9555_ADDR_INPORT1,
                                        TCA9555_ADDR_INPORT0};

const uint32_t ioexp_present_mask[4] = {IOEXP1_PRESENT_MASK,
                                        IOEXP2_PRESENT_MASK,
                                        IOEXP1_PRESENT_MASK,
                                        IOEXP2_PRESENT_MASK};

const uint32_t ioexp_present_expect[4] = {IOEXP1_PRESENT_EXPECT,
                                          IOEXP2_PRESENT_EXPECT,
                                          IOEXP1_PRESENT_EXPECT,
                                          IOEXP2_PRESENT_EXPECT};

const int ioexp_mux_reset[4] = {_F1_OPTICS_I2C_RESET,
                                _F1_OPTICS_I2C_RESET,
                                _F2_OPTICS_I2C_RESET,
                                _F2_OPTICS_I2C_RESET};

/**
 * @brief Helper for transceiver portion of firefly I2C test
 *
 * @details
 * Helper for CLI firefly test that handles firefly portion. It loops over
 * fireflies, and for each connected device disables a unique set of channels.
 * It then loops over them again, checking the disabled channels match what
 * was set. Finally, all channels are re-enabled.
 *
 * @param [out] m  output string
 * @return true if test passes, false otherwise
 */
bool firefly_i2ctest_transceiver_helper(char *m)
{

  // do three passes, write the first time, read the second, and reset third
  for (uint8_t rw = 0; rw < 3; ++rw) {

    // loop over devices
    for (uint8_t idev = 0; idev < NDEVICES_FF; ++idev) {

      if (ff_addrs[idev].dev_class == DEV_NONE) {
        continue;
      }

      // select device on MUX
      uint8_t mux_data = 0x1U << ff_addrs[idev].mux_bit;
      if (apollo_i2c_ctl_w(ff_addrs[idev].i2c_ctrl, ff_addrs[idev].mux_addr, 1,
                           mux_data)) {
        snprintf(m, SCRATCH_SIZE, "ERROR: selecting dev %d on MUX\r\n", idev);
        return false;
      }

      uint8_t i2c_addr = FF_12X_TX_I2C_ADDR;
      uint8_t disable_addr = FF_12X_DISABLE_ADDR;
      if (ff_addrs[idev].dev_class == DEV_FF_TX) {
        i2c_addr = FF_12X_TX_I2C_ADDR;
      }
      else if (ff_addrs[idev].dev_class == DEV_FF_RX) {
        i2c_addr = FF_12X_RX_I2C_ADDR;
      }
      else if (ff_addrs[idev].dev_class == DEV_FF_B04) {
        i2c_addr = FF_4X_I2C_ADDR;
        disable_addr = FF_4X_DISABLE_ADDR;
      }
      uint8_t test_data = (idev + 1) % 16;

      // page select
      if (apollo_i2c_ctl_reg_w(ff_addrs[idev].i2c_ctrl, i2c_addr,
                               1, FF_PAGESEL_ADDR, 1, FF_12X_DISABLE_PAGE)) {
        snprintf(m, SCRATCH_SIZE, "ERROR: selecting page on dev %d\r\n",
                 idev);
        return false;
      }

      if (rw == 0) {
        // write
        if (apollo_i2c_ctl_reg_w(ff_addrs[idev].i2c_ctrl,
                                 i2c_addr, 1,
                                 disable_addr, 1, test_data)) {
          snprintf(m, SCRATCH_SIZE, "ERROR: writing bits to %d\r\n",
                   idev);
          return false;
        }
      }
      else if (rw == 1) {
        // read
        uint32_t data = 0;
        if (apollo_i2c_ctl_reg_r(ff_addrs[idev].i2c_ctrl,
                                 i2c_addr, 1,
                                 disable_addr, 1, &data)) {
          snprintf(m, SCRATCH_SIZE, "ERROR: reading bits from %d\r\n",
                   idev);
          return false;
        }
        if (data != test_data) {
          snprintf(m, SCRATCH_SIZE,
                   "ERROR: incorrect readback on dev %d (expected %d, got %d"
                   ")\r\n",
                   idev, test_data, data);
          return false;
        }
      }
      else {
        // reset to default
        if (apollo_i2c_ctl_reg_w(ff_addrs[idev].i2c_ctrl,
                                 i2c_addr, 1,
                                 disable_addr, 1, 0x0)) {
          snprintf(m, SCRATCH_SIZE, "ERROR: resetting bits on %d\r\n",
                   idev);
          return false;
        }
      }

    } // loop over devices
  } // read/write loop

  snprintf(m, SCRATCH_SIZE, "Test success");
  return true;
}

/**
 * @brief Helper function for IO expander/MUX portion of firefly I2C test
 *
 * @details
 * Helper for CLI firefly test that handles IOexpander portion. It has two
 * modes: when mux_reset is false, it just reads the firefly present signals
 * and confirms they match the expected values; while when mux_reset is true,
 * it attempts to do the same thing but is expected to fail due to resetting
 * the mux before addressing the IO expanders
 *
 * @param [in] mux_reset
 * @param [out] m  output string
 * @return true if test passes, false otherwise
 */
bool firefly_i2ctest_ioexpandermux_helper(bool mux_reset, char *m)
{
  // loop over devices
  for (uint8_t idev = 0; idev < NDEVICES_FF_IOEXPANDER; ++idev) {

    // select device on MUX
    uint8_t mux_data = 0x1U << ff_ioexp_addrs[idev].mux_bit;
    if (apollo_i2c_ctl_w(ff_ioexp_addrs[idev].i2c_ctrl,
                         ff_ioexp_addrs[idev].mux_addr, 1,
                         mux_data)) {
      snprintf(m, SCRATCH_SIZE, "ERROR: selecting dev %d on MUX\r\n", idev);
      return false;
    }

    // if performing the MUX version of the test, reset the MUX here
    if (mux_reset) {
      write_gpio_pin(ioexp_mux_reset[idev], 0x0);
      vTaskDelay(pdMS_TO_TICKS(1));
      write_gpio_pin(ioexp_mux_reset[idev], 0x1);
    }

    // read present bits
    uint32_t data;
    bool fail = false;
    if (apollo_i2c_ctl_reg_r(ff_ioexp_addrs[idev].i2c_ctrl,
                             ff_ioexp_addrs[idev].dev_addr, 1,
                             ioexp_present_addrs[idev], 1, &data)) {
      if (!mux_reset) {
        snprintf(m, SCRATCH_SIZE, "ERROR: reading from IOexp %d\r\n", idev);
        return false;
      }
      else {
        fail = true;
      }
    }
    data = data & ioexp_present_mask[idev];
    if (data != ioexp_present_expect[idev]) {
      if (!mux_reset) {
        snprintf(m, SCRATCH_SIZE,
                 "ERROR: present bits on IOexp %d (expected %d, got %d)\r\n",
                 idev, ioexp_present_expect[idev], data);
        return false;
      }
      else {
        fail = true;
      }
    }
    if (mux_reset && !fail) {
      snprintf(m, SCRATCH_SIZE, "ERROR: MUX reset failed for dev %d\r\n",
               idev);
      return false;
    }

  } // loop over devices
  return true;
}

/**
 * @details
 * CLI function that tests I2C communication to fireflies/IO expanders/MUX.
 * This is broken into 3 parts, see helper functions above for details
 */
BaseType_t firefly_i2ctest_ctl(int argc, char **argv, char *m)
{
  if (!firefly_i2ctest_transceiver_helper(m)) {
    return pdFALSE;
  }

  if (!firefly_i2ctest_ioexpandermux_helper(false, m)) {
    return pdFALSE;
  }

  if (!firefly_i2ctest_ioexpandermux_helper(true, m)) {
    return pdFALSE;
  }

  snprintf(m, SCRATCH_SIZE, "Test success.\r\n");
  return pdFALSE;
}

/**
 * @details
 * CLI function that initializes firefly IO expanders by calling
 * init_registers_firefly
 */
BaseType_t firefly_ioexpanders_init_ctl(int argc, char **argv, char *m)
{
  int r;
  r = init_registers_firefly();
  if (r) {
    snprintf(m, SCRATCH_SIZE, "ERROR in IO expander initialization.\r\n");
  }
  else {
    snprintf(m, SCRATCH_SIZE, "IO expanders initialized successfully.\r\n");
  }

  return pdFALSE;
}

/**
 * @details
 * Initializes IO expanders on the firefly I2C MUX, following what is done in
 * central cm_mcu code. This code is currently wholesale copied from
 * LocalTasks.c in cm_mcu, can be cleaned if necessary
 */
int init_registers_firefly(void)
{
  int result;
  // =====================================================
  // CMv3 Schematic 4.05 I2C FPGA#1 OPTICS

  // 3a) U15 inputs vs. outputs (I2C address 0x20 on I2C channel #4)
  // All signals are inputs.

  // # set first I2C switch on channel 4 (U14, address 0x70) to port 7
  result = apollo_i2c_ctl_w(4, 0x70, 1, 0x80);
  result += apollo_i2c_ctl_reg_w(4, 0x20, 1, 0x06, 1, 0xff); //  11111111 [P07..P00]
  result += apollo_i2c_ctl_reg_w(4, 0x20, 1, 0x07, 1, 0xff); //  11111111 [P17..P10]

  // clear first I2C switch on channel 4
  result += apollo_i2c_ctl_w(4, 0x70, 1, 0x0);

  // 3b) U15 default output values (I2C address 0x20 on I2C channel #4)
  // All signals are inputs so nothing needs to be done.

  // 4a) U18 inputs vs. outputs (I2C address 0x21 on I2C channel #4)
  // The "/F1_FF_RESET" signal on P07 is an output
  // The "EN_...3V8" signals on P11, P12, and P13 are outputs.
  // All other signals are inputs

  // # set second I2C switch on channel 4 (U17, address 0x71) to port 6
  result += apollo_i2c_ctl_w(4, 0x71, 1, 0x40);
  result += apollo_i2c_ctl_reg_w(4, 0x21, 1, 0x06, 1, 0x7f); //  01111111 [P07..P00]
  result += apollo_i2c_ctl_reg_w(4, 0x21, 1, 0x07, 1, 0xf0); //  11110000 [P17..P10]

  // 4b) U17 default output values (I2C address 0x21 on I2C channel #4)
  // The output on P07 should default to "1".
  // This negates the active-lo "RESET" input on the FPGA#1 FireFlys
  // The outputs on P10, P11, P12, and P13 should default to "0"
  // This disables the 3.8 volt power supplies on the three FireFly
  // 12-lane transmitter sites for FPGA#1.

  // # set second I2C switch on channel 4 (U16, address 0x71) to port 6
  result += apollo_i2c_ctl_w(4, 0x71, 1, 0x40);
  result += apollo_i2c_ctl_reg_w(4, 0x21, 1, 0x02, 1, 0x80); //  10000000 [P07..P00]
  result += apollo_i2c_ctl_reg_w(4, 0x21, 1, 0x03, 1, 0x00); //  00000000 [P17..P10]

  // clear 2nd I2C switch on channel 4
  result += apollo_i2c_ctl_w(4, 0x70, 1, 0x0);

  // =====================================================
  // CMv3 Schematic 4.06 I2C FPGA#2 OPTICS

  // 5a) U2 inputs vs. outputs (I2C address 0x20 on I2C channel #3)
  // All signals are inputs.

  // # set first I2C switch on channel 3 (U9, address 0x70) to port 7
  result += apollo_i2c_ctl_w(3, 0x70, 1, 0x80);
  result += apollo_i2c_ctl_reg_w(3, 0x20, 1, 0x06, 1, 0xff); //  11111111 [P07..P00]
  result += apollo_i2c_ctl_reg_w(3, 0x20, 1, 0x07, 1, 0xff); //  11111111 [P17..P10]

  // clear first I2C switch on channel 3
  result += apollo_i2c_ctl_w(3, 0x70, 1, 0x0);

  // 5b) U2 default output values (I2C address 0x20 on I2C channel #3)
  // All signals are inputs so nothing needs to be done.

  // 6a) U12 inputs vs. outputs (I2C address 0x21 on I2C channel #3)
  // The "/F2_FF_RESET" signal on P07 is an output
  // The "EN_...3V8" signals on P10, P11, P12, and P13 are outputs.
  // All other signals are inputs

  // # set second I2C switch on channel 3 (U11, address 0x71) to port 6
  result += apollo_i2c_ctl_w(3, 0x71, 1, 0x40);
  result += apollo_i2c_ctl_reg_w(3, 0x21, 1, 0x06, 1, 0x7f); //  01111111 [P07..P00]
  result += apollo_i2c_ctl_reg_w(3, 0x21, 1, 0x07, 1, 0xf0); //  11110000 [P17..P10]

  // 6b) U12 default output values (I2C address 0x21 on I2C channel #3)
  // The output on P07 should default to "1".
  // This negates the active-lo "RESET" input on the FPGA#2 FireFlys
  // The outputs on P10, P11, P12, and P13 should default to "0"
  // This disables the 3.8 volt power supplies on the three FireFly
  // 12-lane transmitter sites for FPGA#2.

  // # set second I2C switch on channel 3 (U11, address 0x71) to port 6
  result += apollo_i2c_ctl_w(3, 0x71, 1, 0x40);
  result += apollo_i2c_ctl_reg_w(3, 0x21, 1, 0x02, 1, 0x80); //  10000000 [P07..P00]
  result += apollo_i2c_ctl_reg_w(3, 0x21, 1, 0x03, 1, 0x00); //  00000000 [P17..P10]

  // clear 2nd I2C switch on channel 3
  result += apollo_i2c_ctl_w(3, 0x71, 1, 0x0);

  return result;
}
