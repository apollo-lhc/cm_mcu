/*
 * ClockCommands.c
 *
 * Clock synthesizer CLI command handlers, extracted from SensorControl.c
 * and CommandLineTask.c.
 */

#include <string.h>
#include <math.h>

#include "commands/ClockCommands.h"
#include "commands/parameters.h"
#include "common/smbus_helper.h"
#include "MonitorTaskI2C.h"
#include "MonUtils.h"
#include "Semaphore.h"
#include "Tasks.h"
#include "projdefs.h"

// names of the clock ids
static const char *const clk_ids[5] = {"0A", "0B", "1A", "1B", "1C"};

// dump clock monitor information
BaseType_t clkmon_ctl(int argc, char **argv, char *m)
{
  int copied = 0;
  static int c = 0;
  BaseType_t i = strtol(argv[1], NULL, 10);

  if (i < 0 || i > 4) {
    snprintf(m, SCRATCH_SIZE, "%s: Invalid argument %s\r\n", argv[0], argv[1]);
    return pdFALSE;
  }
  // print out header once
  if (c == 0) {
    copied += snprintf(m + copied, SCRATCH_SIZE - copied, "Clock R%s\r\n",
                       clk_ids[i]);
    char *header = "REG_TABLE";
    copied += snprintf(m + copied, SCRATCH_SIZE - copied, "%-15s REG_ADDR BIT_MASK  VALUE \r\n", header);
  }
  // update times, in seconds
  TickType_t now = pdTICKS_TO_MS(xTaskGetTickCount()) / 1000;
  TickType_t last = pdTICKS_TO_MS(clk_args.updateTick) / 1000;

  if (checkStale(last, now)) {
    unsigned mins = (now - last) / 60;
    copied += snprintf(m + copied, SCRATCH_SIZE - copied,
                       "%s: stale data, last update %u minutes ago\r\n", argv[0], mins);
  }

  for (; c < clk_args.n_commands; ++c) {
    // check if device i has this command
    if (!(clk_args.commands[c].devicelist() & ClockType(i))) {
      continue;
    }
    uint16_t val = clk_args.commands[c].retrieveData(i);
    int devtype = 31 - __builtin_clz(ClockType(i));
    copied += snprintf(m + copied, SCRATCH_SIZE - copied, "%-15s : 0x%04x   0x%04x    0x%04x\r\n",
                       clk_args.commands[c].name, clk_args.commands[c].command[devtype],
                       clk_args.commands[c].bit_mask, val);
    if ((SCRATCH_SIZE - copied) < 20) {
      ++c;
      return pdTRUE;
    }
  }
  if (c % 2 == 1) {
    m[copied++] = '\r';
    m[copied++] = '\n';
    m[copied] = '\0';
  }
  c = 0;
#if defined(REV2) || defined(REV3)
  copied += snprintf(m + copied, SCRATCH_SIZE - copied, "Program (read from clock chip): %s", clkprog_args[i].progname_clkdesgid);
  if (strncmp(clkprog_args[i].progname_clkdesgid, "5395ABP1", 3) == 0 || strncmp(clkprog_args[i].progname_clkdesgid, "5341ABP1", 3) == 0) {
    copied += snprintf(m + copied, SCRATCH_SIZE - copied, " (not found)");
  }

  snprintf(m + copied, SCRATCH_SIZE - copied, "\r\nProgram (read from eeprom): %s\r\n", clkprog_args[i].progname_eeprom);
#endif

  return pdFALSE;
}

// read out clock frequency measurements via special i2c port on FPGAs.
// implemented in production test FPGA bit file.
BaseType_t clk_freq_fpga_cmd(int argc, char **argv, char *m)
{
  static int i = 0;

  int copied = 0;
  // check if we are looking for FPGA1 or two based on the command argument
  char *c;
  int fpga = strtol(argv[1], &c, 10) - 1; // FPGA is 0 or 1 (not 1 or 2)
  if (*c != '\0' || c == argv[1] || fpga < 0 || fpga > 1) {
    snprintf(m, SCRATCH_SIZE, "FPGA should be 1 or 2 (got %s)\r\n", argv[1]);
    return pdFALSE;
  }
#define MAXTEST 2
  // which test to run
  int test = strtol(argv[2], &c, 10);
  if (*c != '\0' || c == argv[2] || test < 1 || test > MAXTEST) {
    snprintf(m, SCRATCH_SIZE, "Test should be between 1 and %d (got %s)\r\n", MAXTEST, argv[2]);
    return pdFALSE;
  }

  if (i == 0) { // say hello once
    copied += snprintf(m + copied, SCRATCH_SIZE - copied, "Testing F%d, test %d\r\n",
                       fpga + 1, test);
    // set up which input from clocks to use (R0A or R0B)
  }
  char *names[] = {
      "rw0", "rw1", "clk_200_ext", "lhc_clk", "tcds40_clk", "rt_x4_r0_clk",
      "rt_x12_r0_clk", "lf_x4_r0_clk", "lf_x12_r0_clk", "clk_100", "clk_325",
      "rt_r0_p", "rt_r0_n", "rt_r0_l", "rt_r0_i", "rt_r0_g", "rt_r0_e", "rt_r0_b",
      "lf_r0_y", "lf_r0_w", "lf_r0_u", "lf_r0_r", "lf_r0_af", "lf_r0_ad", "lf_r0_ab",
      "rt_r1_p", "rt_r1_n", "rt_r1_l", "rt_r1_i", "rt_r1_g", "rt_r1_e", "rt_r1_b",
      "lf_r1_y", "lf_r1_w", "lf_r1_u", "lf_r1_r", "lf_r1_af", "lf_r1_ad", "lf_r1_ab"};

  // these values are from Table 2 of the Rev3 Synthesizer testing document. This corresponds to step 1.1.1
  const uint32_t EXPECTED_FREQ_R0A_F1[] = {
      0, 0, 200000000, 40000000, 55000000, 280000000, 160000000,
      150000000, 300000000, 100000000, 325000000, 280000000, 160000000,
      200000000, 160000000, 280000000, 160000000, 160000000, 300000000,
      150000000, 300000000, 300000000, 150000000, 300000000, 60000000,
      116000000, 170000000, 272000000, 113000000, 143000000, 286000000,
      155000000, 163000000, 312000000, 176000000, 296000000, 168000000,
      132000000, 220000000};

  const uint32_t EXPECTED_FREQ_R0A_F2[] = {
      0, 0, 200000000, 40000000, 55000000, 140000000, 320000000,
      130000000, 260000000, 100000000, 325000000, 140000000, 320000000,
      200000000, 320000000, 140000000, 320000000, 320000000, 260000000,
      130000000, 260000000, 260000000, 130000000, 260000000, 40000000,
      226000000, 340000000, 136000000, 174000000, 232000000, 348000000,
      310000000, 134000000, 336000000, 326000000, 268000000, 156000000,
      148000000, 110000000};
  // these values are from Table 2 of the Rev3 Synthesizer testing document. This corresponds to step 1.1.3
  const uint32_t EXPECTED_FREQ_R0B_F1[] = {
      0, 0, 200000000, 40000000, 55000000, 270000000, 155000000,
      145000000, 290000000, 100000000, 325000000, 270000000, 155000000,
      200000000, 155000000, 270000000, 155000000, 155000000, 290000000,
      145000000, 290000000, 290000000, 145000000, 290000000, 60000000,
      116000000, 170000000, 272000000, 113000000, 143000000, 286000000,
      155000000, 163000000, 312000000, 176000000, 296000000, 168000000,
      132000000, 220000000};

  const uint32_t EXPECTED_FREQ_R0B_F2[] = {
      0, 0, 200000000, 40000000, 55000000, 135000000, 310000000,
      125000000, 250000000, 100000000, 325000000, 135000000, 310000000,
      200000000, 310000000, 135000000, 310000000, 310000000, 250000000,
      125000000, 250000000, 250000000, 125000000, 250000000, 40000000,
      226000000, 340000000, 136000000, 174000000, 232000000, 348000000,
      310000000, 134000000, 336000000, 326000000, 268000000, 156000000,
      148000000, 110000000};

  const float TOLERANCE = .01f; // 1% tolerance seems to be needed for 100MHz clock (reads 99.3MHz)

  SemaphoreHandle_t s5 = getSemaphore(5);
  if (i == 0) { // on first entry, grab the semaphore and set the mux
    if (acquireI2CSemaphoreTime(s5, 10) != pdTRUE) {
      int copied = snprintf(m, SCRATCH_SIZE, "Failed to acquire I2C semaphore\r\n");
      // do I have the semaphore already?
      if (xSemaphoreGetMutexHolder(s5) == xTaskGetCurrentTaskHandle()) {
        snprintf(m + copied, SCRATCH_SIZE - copied, "Note: I2C semaphore already held by this task\r\n");
      }
      return pdFALSE;
    }
    bool have_semaphore = xSemaphoreGetMutexHolder(s5) == xTaskGetCurrentTaskHandle();
    if (!have_semaphore) {
      snprintf(m, SCRATCH_SIZE, "Semaphore not held after acquire\r\n");
      return pdFALSE;
    }

    // set the mux to the desired FPGA -- page 4.04 of schematics. Channel 2 for F1, channel 0 for F2
    unsigned mux_val = 0x1 << 2; // default to F1, which is channel 2
    if (fpga == 1) {
      mux_val = 0x1 << 0; // F2 is channel 0
    }
    int r = apollo_i2c_ctl_w(5, 0x70, 1, mux_val);
    if (r != 0) {
      snprintf(m, SCRATCH_SIZE, "Failed to set mux (%d, %s)\r\n", r, SMBUS_get_error(r));
      xSemaphoreGive(s5);
      i = 0;
      return pdFALSE;
    }
  }
  uint32_t *EXPECTED_FREQ;
  // select which expected frequency table to use based on test. Also do any other setup for the
  // test in question, such as select the appropriate clock input using the I2C I/O expanders.
  if (test == 1) {
    if (fpga == 0) {
      EXPECTED_FREQ = (uint32_t *)EXPECTED_FREQ_R0A_F1;
    }
    else {
      EXPECTED_FREQ = (uint32_t *)EXPECTED_FREQ_R0A_F2;
    }
  }
  else if (test == 2) {
    if (fpga == 0) {
      EXPECTED_FREQ = (uint32_t *)EXPECTED_FREQ_R0B_F1;
    }
    else {
      EXPECTED_FREQ = (uint32_t *)EXPECTED_FREQ_R0B_F2;
    }
  }
  else {
    snprintf(m + copied, SCRATCH_SIZE - copied, "Invalid test and FPGA combo (%d,%d)\r\n",
             fpga, test);
    xSemaphoreGive(s5);
    i = 0;
    return pdFALSE;
  }

  // ---------------------------------
  // R0A/R0B selection
  // ---------------------------------
  // on first entry, set clock input
  if (i == 0) {
    bool useR0A = true;
    switch (test) {
      case 1:
        useR0A = true;
        break;
      case 2:
        useR0A = false;
        break;
    }

    // grab semaphore for I2C 2
    SemaphoreHandle_t s2 = getSemaphore(2);
    if (acquireI2CSemaphoreTime(s2, 10) != pdTRUE) {
      snprintf(m, SCRATCH_SIZE, "Failed to acquire I2C2 semaphore\r\n");
      xSemaphoreGive(s5);
      i = 0;
      return pdFALSE;
    }

    // set the mux to channel as appropriate for I/O expander for F1/F2 (page 4.03 of schematics)
    uint8_t channel = 0x1 << 6; // channel 6
    if (fpga == 1) {            // f2
      channel = 0x1 << 7;       // channel 7
    }
    int r = apollo_i2c_ctl_w(2, 0x70, 1, channel); // set to appropriate channel
    uint8_t i2c_addr = 0x20;                       // f1
    if (fpga == 1) {
      i2c_addr = 0x21; // f2
    }
    // read-modify-write the clock select register to select R0A or R0B. For F1 this is register P0 of U88,
    // for F2 this is register P0 of U83.
    // Bit 0-3 selects R0A (0x0) or R0B (0xF)
    // For the TCA9555, for register P0X, you read from address 0x0, write to address 0x2.
    // see schematics page 4.03 for I/O expander details.
#define TCA9555_REG_INPUT_P0  0x0
#define TCA9555_REG_OUTPUT_P0 0x2
    uint32_t data;
    r += apollo_i2c_ctl_reg_r(2, i2c_addr, 1, TCA9555_REG_INPUT_P0, 1, &data); // read current value
                                                                               //      copied += snprintf(m + copied, SCRATCH_SIZE - copied, "Pre  TCA9555 value: 0x%04x\r\n",
                                                                               //                data);
    data &= 0xF0;                                                              // clear bottom nybble
    if (useR0A) {
      data |= 0x0; // set nybble low to 0x0
    }
    else {
      data |= 0xF; // set nybble low to 0xF
    }
    r += apollo_i2c_ctl_reg_w(2, i2c_addr, 1, TCA9555_REG_OUTPUT_P0, 1, data); // set nybble to new value
    // read back for testing
    // r += apollo_i2c_ctl_reg_r(2, i2c_addr, 1, TCA9555_REG_INPUT_P0, 1, &data); // read current value

    r += apollo_i2c_ctl_w(2, 0x70, 1, 0x0); // clear the mux
    if (r) {
      copied += snprintf(m + copied, SCRATCH_SIZE - copied,
                         "Failed to set R0A/R0B (%d)\r\n", r);
      xSemaphoreGive(s5);
      xSemaphoreGive(s2);
      i = 0;
      return pdFALSE;
    }
    xSemaphoreGive(s2); // release I2C 2 semaphore
    // need to wait a second or so -- frequency counters in the FPGA count for a second
    vTaskDelay(pdMS_TO_TICKS(1200));
  } // first entry

  const int NUM_REGISTERS = 39;

  for (; i < NUM_REGISTERS; ++i) {
    uint32_t data;
    uint16_t reg_addr = i << 2;
    int r = apollo_i2c_ctl_reg_r(5, 0x2b, 1, reg_addr, 4, &data);
    if (r != 0) {
      snprintf(m + copied, SCRATCH_SIZE - copied, "Failed to read FPGA registers %d (%s)\r\n",
               i, SMBUS_get_error(r));
      xSemaphoreGive(s5);
      i = 0;
      return pdFALSE;
    }

    copied += snprintf(m + copied, SCRATCH_SIZE - copied,
                       "F%d r%02d: % 10d (0x%08x) %s ",
                       fpga + 1, i, data, data, names[i]);

    if (i < 2) { // first two are test r/w registers
      copied += snprintf(m + copied, SCRATCH_SIZE - copied, "N/A\r\n");
    }
    else {
      uint32_t actual_freq = data;
      float diff = fabs((1.0f * EXPECTED_FREQ[i] - actual_freq) / EXPECTED_FREQ[i]);
      if (diff < TOLERANCE) {
        copied += snprintf(m + copied, SCRATCH_SIZE - copied, "MATCH\r\n");
      }
      else {
        copied += snprintf(m + copied, SCRATCH_SIZE - copied, "NON-MATCH (exp %d)\r\n", EXPECTED_FREQ[i]);
      }
    }

    if ((SCRATCH_SIZE - copied) < 80) { // run out of buffer space -- print out line and resume
      ++i;
      return pdTRUE;
    }
  }
  // clear the mux
  int r = apollo_i2c_ctl_w(5, 0x70, 1, 0);
  if (r != 0) {
    copied += snprintf(m + copied, SCRATCH_SIZE - copied,
                       "Failed to clear mux %s\r\n", SMBUS_get_error(r));
  }
  // return the semaphore, reset the index
  xSemaphoreGive(s5);
  i = 0;
  return pdFALSE;
}

// read clock program names from clock chips
BaseType_t clk_prog_name(int argc, char **argv, char *m)
{
  // argument is which clock chip to read. Should be in range 0-4.
  char *endptr;
  int i = strtol(argv[1], &endptr, 10);
  if (endptr == argv[1] || *endptr != '\0' || i < 0 || i > 4) {
    snprintf(m, SCRATCH_SIZE, "Clock chip should be in range 0-4 (got %s)\r\n", argv[1]);
    return pdFALSE;
  }
  char from_chip[10];
  char from_eeprom[10];
  SemaphoreHandle_t s = getSemaphore(2);
  if (acquireI2CSemaphoreTime(s, 1) != pdTRUE) {
    snprintf(m, SCRATCH_SIZE, "Failed to acquire I2C semaphore\r\n");
    return pdFALSE;
  }
  getClockProgram(i, from_chip, from_eeprom);
  xSemaphoreGive(s);

  snprintf(m, SCRATCH_SIZE, "CLK%d (R%s): %s %s\r\n", i, clk_ids[i], from_chip, from_eeprom);
  return pdFALSE;
}

// reset clock synthesizers
BaseType_t clk_reset(int argc, char **argv, char *m)
{
  // argument is which clock chip to reset. Should be in range 0-4, not string.
  char *endptr;
  int i = strtol(argv[1], &endptr, 10);
  if (endptr == argv[1] || *endptr != '\0' || i < 0 || i > 4) { // make sure we don't get e.g. r0a
    snprintf(m, SCRATCH_SIZE, "Clock chip should be in range 0-4 (got %s)\r\n", argv[1]);
    return pdFALSE;
  }
  SemaphoreHandle_t s = getSemaphore(2);
  if (acquireI2CSemaphoreTime(s, 1) != pdTRUE) {
    snprintf(m, SCRATCH_SIZE, "Failed to acquire I2C semaphore\r\n");
    return pdFALSE;
  }
  int ret = resetClockSynth(i);
  if (ret != 0) {
    snprintf(m, SCRATCH_SIZE, "Failed to reset clock synthesizer R%s (%s)\r\n",
             clk_ids[i], SMBUS_get_error(ret));
  }
  else {
    snprintf(m, SCRATCH_SIZE, "Clock synthesizer R%s reset\r\n", clk_ids[i]);
  }
  xSemaphoreGive(s);
  return pdFALSE;
}

#if defined(REV2) || defined(REV3)
// this command takes one argument
BaseType_t clearclk_ctl(int argc, char **argv, char *m)
{
  int copied = 0;
  int status = -1; // shut up clang compiler warning

  // acquire the semaphore
  if (acquireI2CSemaphore(i2c2_sem) == pdFAIL) {
    snprintf(m + copied, SCRATCH_SIZE - copied, "%s: couldn't get semaphore in time\r\n", argv[0]);
    return pdFALSE;
  }
  status = clear_clk_stickybits();
  // check if we have the semaphore
  if (xSemaphoreGetMutexHolder(i2c2_sem) == xTaskGetCurrentTaskHandle()) {
    xSemaphoreGive(i2c2_sem);
  }

  if (status != 0)
    snprintf(m + copied, SCRATCH_SIZE - copied, "%s operation failed (%d)\r\n", argv[0], status);

  return pdFALSE;
}

// this command takes one argument (from triplet version but will take two argument to include an input from config versions for octlet eeprom)
BaseType_t init_load_clock_ctl(int argc, char **argv, char *m)
{
  int copied = 0;
  char *clk_id_names[5] = {"r0a", "r0b", "r1a", "r1b", "r1c"};
  BaseType_t i = strtol(argv[1], NULL, 10);
  if (i < 0 || i > 4) {
    snprintf(m + copied, SCRATCH_SIZE - copied,
             "Invalid clock chip %ld , the clock id options are r0a:0, r0b:1, r1a:2, "
             "r1b:3 and r1c:4 \r\n",
             i);
    return pdFALSE;
  }
  copied += snprintf(m + copied, SCRATCH_SIZE - copied, "%s is programming clock %s. \r\n", argv[0], clk_id_names[i]);
  int status = -1; // shut up clang compiler warning
  enum power_system_state power_state = getPowerControlState();
  if (power_state != POWER_ON) { // if the power state is not fully on
    snprintf(m + copied, SCRATCH_SIZE - copied, " 3V3 died. skip loadclock\r\n");
    return pdFALSE; // skip this iteration
  }

  // acquire the semaphore
  if (acquireI2CSemaphore(i2c2_sem) == pdFAIL) {
    snprintf(m + copied, SCRATCH_SIZE - copied, "%s: could not get semaphore in time\r\n", argv[0]);
    return pdFALSE;
  }
  status = init_load_clk(i); // status is 0 if all registers can be written to a clock chip. otherwise, it implies that some write registers fail in a certain list.
  // if we have a semaphore, give it
  if (xSemaphoreGetMutexHolder(i2c2_sem) == xTaskGetCurrentTaskHandle()) {
    xSemaphoreGive(i2c2_sem);
  }

  if (status == 0) {
    snprintf(m + copied, SCRATCH_SIZE - copied,
             "clock synthesizer with id %s successfully programmed. \r\n", clk_id_names[i]);
  }
  else {
    snprintf(m + copied, SCRATCH_SIZE - copied, "%s operation failed \r\n", argv[0]);
  }
  return pdFALSE;
}
#endif // REV2 or REV3
