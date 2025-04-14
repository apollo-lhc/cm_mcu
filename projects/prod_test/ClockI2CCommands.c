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
#include "I2CCommunication.h"

// device info
struct dev_moni2c_addr_t clk_moni2c_addrs[NDEVICES_CLK] = {
    {"r0a", CLOCK_I2C_MUX_ADDR, CLOCK_I2C_R0A_MUX_BIT, SI5395_I2C_ADDR},
    {"r0b", CLOCK_I2C_MUX_ADDR, CLOCK_I2C_R0B_MUX_BIT, SI5395_I2C_ADDR},
    {"r1a", CLOCK_I2C_MUX_ADDR, CLOCK_I2C_R1A_MUX_BIT, SI5395_I2C_ADDR},
    {"r1b", CLOCK_I2C_MUX_ADDR, CLOCK_I2C_R1B_MUX_BIT, SI5395_I2C_ADDR},
    {"r1c", CLOCK_I2C_MUX_ADDR, CLOCK_I2C_R1C_MUX_BIT, SI5395_I2C_ADDR},
};
struct dev_ioexpander_addr_t ioexpander_addrs[NDEVICES_CLK_IOEXPANDER] = {
    {CLOCK_I2C_MUX_ADDR, CLOCK_I2C_IOEXP_R0_MUX_BIT, IOEXPANDER_R0_I2C_ADDR},
    {CLOCK_I2C_MUX_ADDR, CLOCK_I2C_IOEXP_R1_MUX_BIT, IOEXPANDER_R1_I2C_ADDR},
};

/**
 * @brief Helper function for 1st stage of clock I2C test, testing synth ICs
 *
 * @details
 * Performs a loop over clock synth ICs where some (distinct) data is written
 * to a user scratch register on each IC, then a second loop reads the data and
 * verifies that it matches what was written
 *
 * @param [out] m  output string
 * @param [inout] m  length of output buffer already used
 * @return true if test passes, false otherwise
 */
bool clock_i2ctest_synth_helper(char *m, int32_t *copied)
{
  uint8_t mux_data;
  uint32_t data;
  int r;

  // do two passes, write the first time and read the second
  for (uint8_t rw = 0; rw < 2; ++rw) {

    // loop over devices
    for (uint8_t idev = 0; idev < NDEVICES_CLK; ++idev) {

      // select the appropriate output for the mux
      mux_data = 0x1U << clk_moni2c_addrs[idev].mux_bit;
      if (apollo_i2c_ctl_w(CLOCK_I2C_BASE, clk_moni2c_addrs[idev].mux_addr, 1,
                           mux_data)) {
        (*copied) += snprintf(m + (*copied), SCRATCH_SIZE - (*copied),
                              "ERROR: selecting dev %d on MUX\r\n", idev);
        return false;
      }

      // select page
      if (apollo_i2c_ctl_reg_w(CLOCK_I2C_BASE, clk_moni2c_addrs[idev].dev_addr,
                               1, SI5395_ADDR_PAGESEL, 1,
                               SI5395_ADDR_SCRATCH_UPPER)) {
        (*copied) += snprintf(m + (*copied), SCRATCH_SIZE - (*copied),
                              "ERROR: selecting page on dev %d\r\n", idev);
        return false;
      }

      // write on first pass
      if (rw == 0) {
        r = apollo_i2c_ctl_reg_w(CLOCK_I2C_BASE,
                                 clk_moni2c_addrs[idev].dev_addr,
                                 1, SI5395_ADDR_SCRATCH_LOWER0, 1, idev + 1);
      }
      // read on second pass
      else {
        data = 0x0U;
        r = apollo_i2c_ctl_reg_r(CLOCK_I2C_BASE,
                                 clk_moni2c_addrs[idev].dev_addr,
                                 1, SI5395_ADDR_SCRATCH_LOWER0, 1, &data);
      }
      if (r) {
        (*copied) += snprintf(m + (*copied), SCRATCH_SIZE - (*copied),
                              "ERROR: read/write %d, dev %d\r\n", rw, idev);
        return false;
      }

      if (rw == 1) {
        // check read value
        if (data != (idev + 1)) {
          (*copied) += snprintf(m + (*copied), SCRATCH_SIZE - (*copied),
                                "ERROR: Bad readback on dev %d (expected %d,"
                                " got %d)\r\n",
                                idev, idev + 1, data);
          return false;
        }
      }

      // wait here for 10 msec
      vTaskDelay(pdMS_TO_TICKS(10));
    } // loop over devices
  }   // read/write passes
  return true;
}

/**
 * @brief Writes or reads to clock MUX IO expander on port 0
 *
 * @details
 * Performs I2C transaction that allows one to read or set pins 00-07 on
 * the IO expanders for R0 and R1
 *
 * @param [in] device_index  0 or 1 to address R0 or R1 expanders respectively
 * @param [in] addr  I2C register address
 * @param [in] io_data  input data byte if write or output data byte if read
 * @param [in] read  true indicates a read operation, false indicates write
 * @param [out] m  output string. If 0, then no output
 * @param [inout] copied  length of output buffer already used
 * @return true if read/write succeeds, false otherwise
 */
bool i2c_ioexpander(int device_index, uint8_t addr, uint32_t *io_data,
                    bool read, char *m, int32_t *copied)
{
  uint8_t mux_data;
  // select device on MUX
  mux_data = 0x1U << ioexpander_addrs[device_index].mux_bit;
  if (apollo_i2c_ctl_w(CLOCK_I2C_BASE, ioexpander_addrs[device_index].mux_addr,
                       1, mux_data)) {
    (*copied) += snprintf(m + (*copied), SCRATCH_SIZE - (*copied),
                          "ERROR: selecting %d on MUX\r\n",
                          ioexpander_addrs[device_index].mux_bit);
    return false;
  }

  if (!read) {
    // write to IO expander
    if (apollo_i2c_ctl_reg_w(CLOCK_I2C_BASE,
                             ioexpander_addrs[device_index].dev_addr, 1, addr,
                             1, *io_data)) {
      (*copied) += snprintf(m + (*copied), SCRATCH_SIZE - (*copied),
                            "ERROR: writing %d to %d on IO/expander %d\r\n",
                            *io_data, addr, device_index);
      return false;
    }
  }
  else {
    // read from IO expander
    if (apollo_i2c_ctl_reg_r(CLOCK_I2C_BASE,
                             ioexpander_addrs[device_index].dev_addr, 1, addr,
                             1, io_data)) {
      (*copied) += snprintf(m + (*copied), SCRATCH_SIZE - (*copied),
                            "ERROR: reading %d from IO/expander %d\r\n",
                            addr, device_index);
      return false;
    }
  }
  return true;
}

/**
 * @brief Helper function for 2nd stage of clock I2C test, testing IO expanders
 *
 * @details
 * Writes distinct data to the output pins of the two IOexpanders and checks
 * the read values are as expected. Then the values are switched, and another
 * read confirms the switch was successful. The pins are then returned to their
 * default values
 *
 * @param [out] m  output string
 * @return true if test passes, false otherwise
 */
bool clock_i2ctest_ioexpander_helper(char *m, int32_t *copied)
{
  uint32_t data[1];

  // set pin7 on IO expander R0 to low and R1 to high and check
  data[0] = IOEXPANDER_R0_REG0_RESET_R0A;
  if (!i2c_ioexpander(0, TCA9555_ADDR_OUTPORT0, data, false, m, copied)) {
    return false;
  }
  data[0] = IOEXPANDER_R1_REG0_DEFAULT;
  if (!i2c_ioexpander(1, TCA9555_ADDR_OUTPORT0, data, false, m, copied)) {
    return false;
  }
  if (!i2c_ioexpander(0, TCA9555_ADDR_INPORT0, data, true, m, copied)) {
    return false;
  }
  if ((data[0] & IOEXPANDER_TEST_MASK) != IOEXPANDER_TEST_RESULT0) {
    (*copied) += snprintf(m + (*copied), SCRATCH_SIZE - (*copied),
                          "ERROR: Failed to assert expander R0 reset\r\n");
    return false;
  }
  if (!i2c_ioexpander(1, TCA9555_ADDR_INPORT0, data, true, m, copied)) {
    return false;
  }
  if ((data[0] & IOEXPANDER_TEST_MASK) != IOEXPANDER_TEST_RESULT1) {
    (*copied) += snprintf(m + (*copied), SCRATCH_SIZE - (*copied),
                          "ERROR: Failed to deassert expander R1 reset\r\n");
    return false;
  }

  // set P7 on expander R0 to high and R1 to low and check
  data[0] = IOEXPANDER_R0_REG0_DEFAULT;
  if (!i2c_ioexpander(0, TCA9555_ADDR_OUTPORT0, data, false, m, copied)) {
    return false;
  }
  data[0] = IOEXPANDER_R1_REG0_RESET_R1A;
  if (!i2c_ioexpander(1, TCA9555_ADDR_OUTPORT0, data, false, m, copied)) {
    return false;
  }
  if (!i2c_ioexpander(0, TCA9555_ADDR_INPORT0, data, true, m, copied)) {
    return false;
  }
  if ((data[0] & IOEXPANDER_TEST_MASK) != IOEXPANDER_TEST_RESULT1) {
    (*copied) += snprintf(m + (*copied), SCRATCH_SIZE - (*copied),
                          "ERROR: Failed to deassert expander R0 reset\r\n");
    return false;
  }
  if (!i2c_ioexpander(1, TCA9555_ADDR_INPORT0, data, true, m, copied)) {
    return false;
  }
  if ((data[0] & IOEXPANDER_TEST_MASK) != IOEXPANDER_TEST_RESULT0) {
    (*copied) += snprintf(m + (*copied), SCRATCH_SIZE - (*copied),
                          "EROR: Failed to assert expander R1 reset\r\n");
    return false;
  }

  // return to default settings by deasserting reset on expander R1
  data[0] = IOEXPANDER_R1_REG0_DEFAULT;
  if (!i2c_ioexpander(1, TCA9555_ADDR_OUTPORT0, data, false, m, copied)) {
    return false;
  }

  return true;
}

/**
 * @brief Helper function for 3rd stage of clock I2C test, testing MUX reset
 *
 * @details
 * The clock synth I2C MUX reset is tested by checking a read attempt fails
 * following a MUX reset
 *
 * @param [out] m  output string
 * @param [inout] copied  output already used in buffer
 * @return true if test passes, false otherwise
 */
bool clock_i2ctest_muxreset_helper(char *m, int32_t *copied)
{
  uint8_t mux_data;
  uint32_t data;

  mux_data = 0x1U << clk_moni2c_addrs[0].mux_bit;
  if (apollo_i2c_ctl_w(CLOCK_I2C_BASE, clk_moni2c_addrs[0].mux_addr, 1,
                       mux_data)) {
    (*copied) += snprintf(m + (*copied), SCRATCH_SIZE - (*copied),
                          "ERROR: selecting dev 0 on MUX\r\n");
    return false;
  }

  //  test reset by attempting read; as long as we don't use an address 0x7X,
  //  we shouldn't accidentally address the MUX
  write_gpio_pin(_CLOCKS_I2C_RESET, 0x0);
  vTaskDelay(pdMS_TO_TICKS(1));
  write_gpio_pin(_CLOCKS_I2C_RESET, 0x1);

  bool read_fail = false;
  // select page
  if (apollo_i2c_ctl_reg_w(CLOCK_I2C_BASE, clk_moni2c_addrs[0].dev_addr, 1,
                           SI5395_ADDR_PAGESEL, 1, SI5395_ADDR_OPN_UPPER)) {
    read_fail = true;
  }
  // read OPN register
  if (apollo_i2c_ctl_reg_r(CLOCK_I2C_BASE, clk_moni2c_addrs[0].dev_addr, 1,
                           SI5395_ADDR_OPN_LOWER0, 1, &data)) {
    read_fail = true;
  }
  // check value read
  if (!read_fail) {
    if (data != SI5395_OPN0) {
      read_fail = true;
    }
  }
  if (!read_fail) {
    (*copied) += snprintf(m + (*copied), SCRATCH_SIZE - (*copied),
                          "ERROR: I2C MUX reset did not work.\r\n");
    return false;
  }
  return true;
}

/**
 * @details
 * CLI function that tests I2C communication to clock synth chips by first
 * performing a loop where some (distinct) data is written to a user scratch
 * register on each clock synth, then a second loop reads the data and verifies
 * that it matches what was written. Then, communication to each IO expander
 * is tested similarly by writing to the internal registers for an output pin
 * and reading back. Finally, the MUX reset signal is tested by checking a
 * read attempt fails following a MUX reset.
 */
bool clock_i2ctest(char *m, int32_t *copied)
{
  if (!clock_i2ctest_synth_helper(m, copied)) {
    return false;
  }

  if (!clock_i2ctest_ioexpander_helper(m, copied)) {
    return false;
  }

  if (!clock_i2ctest_muxreset_helper(m, copied)) {
    return false;
  }

  return true;
}

/**
 * @details
 * Wrapper around clock_i2ctest
 */
BaseType_t clock_i2ctest_ctl(int argc, char **argv, char *m)
{
  int32_t copied = 0;
  if (clock_i2ctest(m, &copied))
    copied += snprintf(m + copied, SCRATCH_SIZE - copied,
                       "Clock I2C test: success.\r\n");
  return pdFALSE;
}

/**
 * @details
 * CLI function that initializes clock IO expanders by calling
 * init_registers_clk
 */
BaseType_t clock_ioexpanders_init_ctl(int argc, char **argv, char *m)
{
  int r;
  r = init_registers_clk();
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
 * Initializes IO expanders on the clock I2C MUX, following what is done in
 * central cm_mcu code. This code is currently wholesale copied from
 * LocalTasks.c in cm_mcu, can be cleaned if necessary
 */
int init_registers_clk(void)
{
  int status = 0;

  // =====================================================
  // CMv2 Schematic 4.03 I2C CLOCK CONTROL

  // 1a) U88 inputs vs. outputs (I2C address 0x20 on I2C channel #2)
  // The "/INT..." signals on P04 and P05 are inputs.
  // The unused signals on P06, P11, P16, and P17 should be inputs.
  // The remaining 10 signals are outputs.

  // # set I2C switch on channel 2 (U84, address 0x70) to port 6

  status += apollo_i2c_ctl_w(2, 0x70, 1, 0x40);
  status += apollo_i2c_ctl_reg_w(2, 0x20, 1, 0x06, 1, 0x70); //  01110000 [P07..P00]
  status += apollo_i2c_ctl_reg_w(2, 0x20, 1, 0x07, 1, 0xc2); //  11000010 [P17..P10]

  // 1b) U88 default output values (I2C address 0x20 on I2C channel #2)
  // The outputs on P00, P01, P02, and P03 should default to "0".
  // This causes the muxes on sheet 2.08 to use clocks from synth R0A.
  // The outputs on P07 and P10 should default to "1".
  // This negates the active-lo "RESET" inputs on synths R0A and R0B.
  // The outputs on P12, P13, P14, and P15 should default to "0".
  // Selection of which input clock to use on synths R0A and R0B will be
  // defined in the configuration files for these chips. They will not be
  // switchable under program control.

  // # set I2C switch on channel 2 (U84, address 0x70) to port 6
  status += apollo_i2c_ctl_w(2, 0x70, 1, 0x40);
  status += apollo_i2c_ctl_reg_w(2, 0x20, 1, 0x02, 1, 0x80); //  10000000 [P07..P00]
  status += apollo_i2c_ctl_reg_w(2, 0x20, 1, 0x03, 1, 0x01); //  00000001 [P17..P10]

  // 2a) U83 inputs vs. outputs (I2C address 0x21 on I2C channel #2)
  // The "/INT..." signals on P04, P05, and P06 are inputs.
  // There ane no unused signals.
  // The remaining 13 signals are outputs.

  // # set I2C switch on channel 2 (U84, address 0x70) to port 7
  status += apollo_i2c_ctl_w(2, 0x70, 1, 0x80);
  status += apollo_i2c_ctl_reg_w(2, 0x21, 1, 0x06, 1, 0x70); //  01110000 [P07..P00]
  status += apollo_i2c_ctl_reg_w(2, 0x21, 1, 0x07, 1, 0x00); //  00000000 [P17..P10]

  // 2b) U88 default output values (I2C address 0x21 on I2C channel #2)
  // The outputs on P00, P01, P02, and P03 should default to "0".
  // This causes the muxes on sheet 2.08 to use clocks from synth R0A.
  // The outputs on P07, P10, and P11 should default to "1".
  // This negates the active-lo "RESET" inputs on synths R1A, R1B, and R1C.
  // The outputs on P12, P13, P14, P15, P16, and P17 should default to "0".
  // Selection of which input clock to use on synths R1A, R1B, and R1C
  // will be defined in the configuration files for these chips. They will
  // not be switchable under program control.

  // # set I2C switch on channel 2 (U84, address 0x70) to port 7
  status += apollo_i2c_ctl_w(2, 0x70, 1, 0x80);
  status += apollo_i2c_ctl_reg_w(2, 0x21, 1, 0x02, 1, 0x80); //  10000000 [P07..P00]
  status += apollo_i2c_ctl_reg_w(2, 0x21, 1, 0x03, 1, 0x03); //  00000011 [P17..P10]

  status += apollo_i2c_ctl_w(2, 0x70, 1, 0x0); // reset the mux

  return status;
}
