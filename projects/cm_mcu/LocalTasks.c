/*
 * LocalTasks.c
 *
 *  Created on: Apr 23, 2020
 *      Author: wittich
 *
 *  Implementation files for various utilities for tasks
 *
 *  DUE TO LIMITATIONS OF MACOS this file is called LocalTasks.c not Tasks.c, as there is a FreeRTOS
 *  file called tasks.c and MacOS default file system can't tell these apart.
 */
#include <stdbool.h>
#include <stdint.h>
#include <string.h> // memset
#include <time.h> // struct tm

// ROM header must come before MAP header
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "driverlib/hibernate.h"

#include "Tasks.h"
#include "MonitorTask.h"
#include "InterruptHandlers.h"

#include "common/pinsel.h"
#include "common/smbus_units.h"
#include "I2CCommunication.h"
#include "common/log.h"
#include "common/printf.h"

#define FPGA_MON_NDEVICES_PER_FPGA  2
#define FPGA_MON_NFPGA              2
#define FPGA_MON_NDEVICES           8
#define FPGA_MON_NCOMMANDS          1
#define FPGA_MON_NVALUES_PER_DEVICE 1
#define FPGA_MON_NVALUES            (FPGA_MON_NCOMMANDS * FPGA_MON_NDEVICES * FPGA_MON_NVALUES_PER_DEVICE)

#define LOG_FACILITY LOG_SERVICE

// FPGA arguments for monitoring task
#ifdef REV1
struct dev_i2c_addr_t fpga_addrs[] = {
    {"VU7P", 0x70, 1, 0x36},    // VU7P FPGA SL0
    {"KU15P", 0x70, 0, 0x36},   // KU15P FPGA
    {"VU7PSL1", 0x70, 1, 0x34}, // VU7P FPGA SL1
};
#define F1F2_NDEVICES 3

struct dev_i2c_addr_t fpga_addrs_f1only[] = {
    {"KU15P", 0x70, 0, 0x36},
};
#define F1_NDEVICES 1

struct dev_i2c_addr_t fpga_addrs_f2only[] = {
    {"VU7P", 0x70, 1, 0x36},    // VU7P FPGA SL0
    {"VU7PSL1", 0x70, 1, 0x34}, // VU7P FPGA SL1
};
#define F2_NDEVICES 2

#elif defined(REV2)
struct dev_i2c_addr_t fpga_addrs[] = {
    {"F1_0", 0x70, 3, 0x36}, // F1 X0Y0
    {"F1_1", 0x70, 3, 0x34}, // F1 X0Y1
    {"F1_2", 0x70, 3, 0x47}, // F1 X1Y0
    {"F1_3", 0x70, 3, 0x45}, // F1 X1Y1
    {"F2_0", 0x70, 1, 0x36}, // F2 X0Y0
    {"F2_1", 0x70, 1, 0x34}, // F2 X0Y1
    {"F2_2", 0x70, 1, 0x47}, // F2 X1Y0
    {"F2_3", 0x70, 1, 0x45}, // F2 X1Y1
};
#define F1F2_NDEVICES 8

struct dev_i2c_addr_t fpga_addrs_f1only[] = {
    {"F1_0", 0x70, 3, 0x36}, // F1 X0Y0
    {"F1_1", 0x70, 3, 0x34}, // F1 X0Y1
    {"F1_2", 0x70, 3, 0x47}, // F1 X1Y0
    {"F1_3", 0x70, 3, 0x45}, // F1 X1Y1
};
#define F1_NDEVICES 4

struct dev_i2c_addr_t fpga_addrs_f2only[] = {
    {"F2_0", 0x70, 1, 0x36}, // F2 X0Y0
    {"F2_1", 0x70, 1, 0x34}, // F2 X0Y1
    {"F2_2", 0x70, 1, 0x47}, // F2 X1Y0
    {"F2_3", 0x70, 1, 0x45}, // F2 X1Y1
};
#define F2_NDEVICES 4

#endif

struct pm_command_t pm_command_fpga[] = {
    {0x8d, 2, "READ_TEMPERATURE_1", "C", PM_LINEAR11},
};

// only one of these might be valid
float pm_fpga[FPGA_MON_NVALUES] = {0};

struct MonitorTaskArgs_t fpga_args = {
    .name = "XIMON",
    .devices = fpga_addrs,
    .n_devices = F1F2_NDEVICES,
    .commands = pm_command_fpga,
    .n_commands = FPGA_MON_NCOMMANDS,
    .pm_values = pm_fpga,
    .n_values = FPGA_MON_NVALUES,
    .n_pages = 1,
#ifdef REV1
    .smbus = &g_sMaster6,
    .smbus_status = &eStatus6,
#elif defined(REV2)
    .smbus = &g_sMaster5,
    .smbus_status = &eStatus5,
#endif
    .xSem = NULL,
    .requirePower = true,
    .stack_size = 4096U,
};
#ifdef REV1
// Power supply arguments for Monitoring task
// Supply Address | Voltages | Priority
// ---------------+----------|-----------
//       0x40     | 3.3 & 1.8|     2
//       0x44     | KVCCINT  |     1
//       0x43     | KVCCINT  |     1
//       0x46     | VVCCINT  |     1
//       0x45     | VVCCINT  |     1
struct dev_i2c_addr_t pm_addrs_dcdc[] = {
    {"3V3/1V8", 0x70, 0, 0x40},  // Dual supply 1.8 / 3.3 V
    {"KVCCINT1", 0x70, 1, 0x44}, // first vccint, KU15P
    {"KVCCINT2", 0x70, 2, 0x43}, // second vccint, KU15P
    {"VVCCINT1", 0x70, 3, 0x46}, // first vccint, VU7P
    {"VVCCINT2", 0x70, 4, 0x45}, // second vccint, VU7P
};
#elif defined (REV2) // REV1
// Power supply arguments for Monitoring task
// Supply Address | Voltages  | Priority
// ---------------+-----------|-----------
//       0x40     | 3.3 & 1.8 |     2
//       0x44     | F1VCCINT  |     1
//       0x43     | F1VCCINT  |     1
//       0x46     | F2VCCINT  |     1
//       0x45     | F2VCCINT  |     1
struct dev_i2c_addr_t pm_addrs_dcdc[] = {
    {"3V3/1V8", 0x70, 0, 0x40},   // Dual supply 1.8 / 3.3 V
    {"F1VCCINT1", 0x70, 1, 0x44}, // first vccint, F1
    {"F1VCCINT2", 0x70, 2, 0x43}, // second vccint, F1
    {"F2VCCINT1", 0x70, 3, 0x44}, // first vccint, F2
    {"F2VCCINT2", 0x70, 4, 0x43}, // second vccint, F2
    {"F1AVTT/CC", 0x70, 5, 0x40}, // AVCC/AVTT for F1
    {"F2AVTT/CC", 0x70, 6, 0x40}, // AVCC/AVTT for F2
};
#else
#error "need to define either Rev1 or Rev2"
#endif // REV1

// this function is run once in the dcdc monitoring task
struct pm_command_t extra_cmds[] = {
    {0x0, 1, "PAGE", "", PM_STATUS},
    {0x1, 1, "OPERATION", "", PM_STATUS},
    {0x33, 2, "FREQUENCY_SWITCH", "Hz", PM_LINEAR11},
    {0xEA, 32, "SNAPSHOP", "", PM_STATUS},
    {0xF3, 1, "SNAPSHOP_CONTROL", "", PM_STATUS},
    {0x28, 2, "VOUT_DROOP", "", PM_LINEAR11},
    {0xD5, 1, "MULTIPHASE_RAMP_GAIN", "", PM_STATUS},
};

void snapdump(struct dev_i2c_addr_t *add, uint8_t page, uint8_t snapshot[32], bool reset)
{
  while (xSemaphoreTake(dcdc_args.xSem, (TickType_t)10) == pdFALSE)
    ;
  // page register
  int r = apollo_pmbus_rw(&g_sMaster1, &eStatus1, false, add, &extra_cmds[0], &page);
  if (r) {
    log_error(LOG_SERVICE, "page\r\n");
  }

  // actual command -- snapshot control copy NVRAM for reading
  uint8_t cmd = 0x1;
  r = apollo_pmbus_rw(&g_sMaster1, &eStatus1, false, add, &extra_cmds[4], &cmd);
  if (r) {
    log_error(LOG_SERVICE, "ctrl\r\n");
  }
  // actual command -- read snapshot
  tSMBusStatus r2 =
      SMBusMasterBlockRead(&g_sMaster1, add->dev_addr, extra_cmds[3].command, &snapshot[0]);
  if (r2 != SMBUS_OK) {
    log_error(LOG_SERVICE, "block %d\r\n", r2);
  }
  while ((r2 = SMBusStatusGet(&g_sMaster1)) == SMBUS_TRANSFER_IN_PROGRESS) {
    vTaskDelay(pdMS_TO_TICKS(10)); // wait
  }
  if (r2 != SMBUS_TRANSFER_COMPLETE) {
    log_error(LOG_SERVICE, "SMBUS %d\r\n", r2);
  }
  if (reset) {
    // reset SNAPSHOT. This will fail if the device is on.
    cmd = 0x3;
    r = apollo_pmbus_rw(&g_sMaster1, &eStatus1, false, add, &extra_cmds[4], &cmd);
    if (r) {
      log_error(LOG_SERVICE, "error reset\r\n");
    }
  }
  xSemaphoreGive(dcdc_args.xSem);
}

// Initialization function for the LGA80D. These settings
// need to be called when the supply output is OFF
// this is currently not ensured in this code.
void LGA80D_init(void)
{
  while (xSemaphoreTake(dcdc_args.xSem, (TickType_t)10) == pdFALSE)
    ;
  log_info(LOG_SERVICE, "LGA80D_init\r\n");
  // set up the switching frequency
  uint16_t freqlin11 = float_to_linear11(457.14f);
  uint16_t drooplin11 = float_to_linear11(0.0700f);
  // we do the same for all devices except the 0th one, which is the 1.8/3.3V device
  for (int dev = 1; dev < NSUPPLIES_PS; dev += 1) {
    for (uint8_t page = 0; page < 2; ++page) {
      // page register
      int r = apollo_pmbus_rw(&g_sMaster1, &eStatus1, false, pm_addrs_dcdc + dev, &extra_cmds[0],
                              &page);
      if (r) {
        log_debug(LOG_SERVICE, "dev = %d, page = %d, r= %d\r\n", dev, page, r);
        log_error(LOG_SERVICE, "LGA80D(0)\r\n");
      }
      // actual command -- frequency switch
      r = apollo_pmbus_rw(&g_sMaster1, &eStatus1, false, pm_addrs_dcdc + dev, &extra_cmds[2],
                          (uint8_t *)&freqlin11);
      if (r) {
        log_error(LOG_SERVICE, "LGA80D(1)\r\n");
      }
      // actual command -- vout_droop switch
      r = apollo_pmbus_rw(&g_sMaster1, &eStatus1, false, pm_addrs_dcdc + dev, &extra_cmds[5],
                          (uint8_t *)&drooplin11);
      if (r) {
        log_error(LOG_SERVICE, "LGA80D(2)\r\n");
      }
      // actual command -- multiphase_ramp_gain switch
      uint8_t val = 0x7U; // by suggestion of Artesian
      r = apollo_pmbus_rw(&g_sMaster1, &eStatus1, false, pm_addrs_dcdc + dev, &extra_cmds[6], &val);
      if (r) {
        log_error(LOG_SERVICE, "LGA80D(3)\r\n");
      }
    }
  }
  xSemaphoreGive(dcdc_args.xSem);

  return;
}

// if you change the length of this array, you also need to change
// NCOMMANDS_PS in MonitorTask.h
// TODO make this fix automatic
struct pm_command_t pm_command_dcdc[] = {
    {0x8d, 2, "READ_TEMPERATURE_1", "C", PM_LINEAR11},
    {0x8f, 2, "READ_TEMPERATURE_3", "C", PM_LINEAR11},
    {0x88, 2, "READ_VIN", "V", PM_LINEAR11},
    {0x8B, 2, "READ_VOUT", "V", PM_LINEAR16U},
    {0x8c, 2, "READ_IOUT", "A", PM_LINEAR11},
    {0x79, 2, "STATUS_WORD", "", PM_STATUS},
    {0x4F, 2, "OT_FAULT_LIMIT", "C", PM_LINEAR11},
    {0xE7, 2, "IOUT_AVG_OC_FAULT_LIMIT", "A", PM_LINEAR11},
    {0x95, 2, "READ_FREQUENCY", "Hz", PM_LINEAR11},
    {0x46, 2, "IOUT_OC_FAULT_LIMIT", "A", PM_LINEAR11},
    {0x44, 2, "VOUT_UV_FAULT_LIMIT", "V", PM_LINEAR16U},
    {0x37, 2, "INTERLEAVE", "", PM_STATUS},
    {0x80, 1, "STATUS_MFR_SPECIFIC", "", PM_STATUS},
    {0x28, 2, "VOUT_DROOP", "V/A", PM_LINEAR11},
    {0xD5, 1, "MULTIPHASE_RAMP_GAIN", "", PM_STATUS},
    {0x57, 2, "VIN_UV_WARN_LIMIT", "V", PM_LINEAR11},
    {0x58, 2, "VIN_UV_FAULT_LIMIT", "V", PM_LINEAR11},
    {0xD1, 2, "USER_CONFIG", "", PM_STATUS},
    {0x01, 2, "OPERATION", "", PM_STATUS},
    {0x02, 2, "ON_OFF_CONFIG", "", PM_STATUS},
};
float dcdc_values[NSUPPLIES_PS * NPAGES_PS * NCOMMANDS_PS];

struct MonitorTaskArgs_t dcdc_args = {
    .name = "PSMON",
    .devices = pm_addrs_dcdc,
    .n_devices = NSUPPLIES_PS,
    .commands = pm_command_dcdc,
    .n_commands = NCOMMANDS_PS,
    .pm_values = dcdc_values,
    .n_values = NSUPPLIES_PS * NPAGES_PS * NCOMMANDS_PS,
    .n_pages = NPAGES_PS,
    .smbus = &g_sMaster1,
    .smbus_status = &eStatus1,
    .xSem = NULL,
    .requirePower = false,
    .stack_size = 4096U,
};

static int fpga_f1 = -1;
static int fpga_f2 = -1;
int get_f1_index()
{
  return fpga_f1;
}
int get_f2_index()
{
  return fpga_f2;
}
void set_f1_index(int index)
{
  fpga_f1 = index;
  return;
}
void set_f2_index(int index)
{
  fpga_f2 = index;
  return;
}

void initFPGAMon()
{
  // check if we are to include both FPGAs or not
  bool f1_enable = isFPGAF1_PRESENT();
  bool f2_enable = isFPGAF2_PRESENT();
#ifndef REV1 // FIXME REMOVE THESE
  write_gpio_pin(JTAG_FROM_SM, 1);
  write_gpio_pin(FPGA_CFG_FROM_FLASH, 0);
  write_gpio_pin(F1_FPGA_PROGRAM, 0);
#endif // not REV1
#ifndef DEBUG // todo: just log this
  configASSERT(f1_enable || f2_enable);
#endif // DEBUG
  if (!f1_enable && f2_enable) {
    fpga_args.devices = fpga_addrs_f2only;
    fpga_args.n_devices = F2_NDEVICES;
    set_f2_index(0);
#ifndef REV1
    write_gpio_pin(_F1_JTAG_BYPASS, 0);
    write_gpio_pin(_F2_JTAG_BYPASS, 1);
#endif // REV1
  }
  else if (!f2_enable && f1_enable) {
    fpga_args.devices = fpga_addrs_f1only;
    fpga_args.n_devices = F1_NDEVICES;
    set_f1_index(0);
#ifndef REV1
    write_gpio_pin(_F1_JTAG_BYPASS, 1);
    write_gpio_pin(_F2_JTAG_BYPASS, 0);
#endif // REV1
  }
  else {
    set_f2_index(0);
    set_f1_index(1);
#ifndef REV1
    write_gpio_pin(_F1_JTAG_BYPASS, 1);
    write_gpio_pin(_F2_JTAG_BYPASS, 1);
#endif // REV1
  }
}

#ifdef REV2
// initialize the real-time clock, which lives in the Hibernate Module in the TM4C1294NCPDT
extern uint32_t g_ui32SysClock;

void InitRTC()
{
  // Enable the RTC module
  ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_HIBERNATE);
  // wait for it to be ready
  while (! ROM_SysCtlPeripheralReady(SYSCTL_PERIPH_HIBERNATE) )
  {

  }
  // Enable the clocking. AFAIK the argument is not used
  ROM_HibernateEnableExpClk(g_ui32SysClock);
  // Set to use external crystal with 12 pF drive
  ROM_HibernateClockConfig(HIBERNATE_OSC_LOWDRIVE );
  // enable the RTC
  ROM_HibernateRTCEnable();
  // set the RTC to calendar mode
  ROM_HibernateCounterMode(HIBERNATE_COUNTER_24HR);
  //  // set to a default value
  //  struct tm now = {
  //    .tm_sec = 0,
  //    .tm_min = 0,
  //    .tm_hour = 0,
  //    .tm_mday = 23,
  //    .tm_mon = 10, // month goes from 0-11
  //    .tm_year = 121, // year is since 1900
  //    .tm_wday = 0,
  //    .tm_yday = 0,
  //    .tm_isdst = 0,
  //  };
  // ROM_HibernateCalendarSet(&now);
}
#endif // REV2
#ifdef REV1
void init_registers_clk()
{
  // =====================================================
  // CMv1 Schematic 4.03 I2C CLOCK SOURCE CONTROL

  // 1a) U93 inputs vs. outputs (I2C address 0x20 on I2C channel #2)
  // All used signals are outputs [P03..P00], [P14..P10].
  // All unused signals should be inputs [P07..P04], [P17..P15].

  // # set I2C switch on channel 2 (U94, address 0x70) to port 6 
  apollo_i2c_ctl_w(2, 0x70, 1, 0x40);
  apollo_i2c_ctl_reg_w( 2, 0x20, 1, 0x06, 1, 0xf0); // 11110000 [P07..P00]
  apollo_i2c_ctl_reg_w( 2, 0x20, 1, 0x07, 1, 0xe0); // 11100000 [P17..P10]

  // 1b) U93 default output values (I2C address 0x20 on I2C channel #2)
  // The outputs on P00, P01, P02, and P03 should default to "0".
  // This causes the muxes on sheet 2.06 to use clocks from the SM (high
  // quality / high performance) rather than clocks from the synthesizer.

  // The outputs on P10 and P11 should default to "0". Selection of which 
  // input clock to use on the synthesizer will be defined in the 
  // configuration file for this chip. It will not be switchable under 
  // program control.

  // The output on P12 should default to "0". The synthesizer outputs will be 
  // enabled, even if we are not using it.

  // The outputs on P13 and P14 should default to "1". This negates the 
  // active-lo "RESET" input on the synthesizer and the legacy TTC logic.

  // # set I2C switch on channel 2 (U94, address 0x70) to port 6 
  apollo_i2c_ctl_w( 2, 0x70, 1, 0x40);
  apollo_i2c_ctl_reg_w( 2, 0x20, 1, 0x02, 1, 0xf0); // 11110000 [P07..P00]
  apollo_i2c_ctl_reg_w( 2, 0x20, 1, 0x03, 1, 0xf8); // 11111000 [P17..P10]

  // 2a) U92 inputs vs. outputs (I2C address 0x21 on I2C channel #2)
  // The signals on P00, P01, and P02 are inputs.
  // There other signals are unused and should be set as inputs.
  // There are no outputs.

  // # set I2C switch on channel 2 (U94, address 0x70) to port 7 
  apollo_i2c_ctl_w( 2, 0x70, 1, 0x80);
  apollo_i2c_ctl_reg_w( 2, 0x21, 1, 0x06, 1, 0xff); // 11111111 [P07..P00]
  apollo_i2c_ctl_reg_w( 2, 0x21, 1, 0x07, 1, 0xff); // 11111111 [P17..P10]

  // 2b) U92 default output values (I2C address 0x21 on I2C channel #2)
  // All signals are inputs so nothing needs to be done.
}
void init_registers_ff()
{

  // =====================================================
  // CMv1 Schematic 4.05 I2C KU15P OPTICS

  // 3a) U102 inputs vs. outputs (I2C address 0x20 on I2C channel #4)
  // All signals are inputs.

  // # set first I2C switch on channel 4 (U100, address 0x70) to port 7 
  apollo_i2c_ctl_w( 4, 0x70, 1, 0x80);
  apollo_i2c_ctl_reg_w( 4, 0x20, 1, 0x06, 1, 0xff); // 11111111 [P07..P00]
  apollo_i2c_ctl_reg_w( 4, 0x20, 1, 0x07, 1, 0xff); // 11111111 [P17..P10]

  // 3b) U102 default output values (I2C address 0x20 on I2C channel #4)
  // All signals are inputs so nothing needs to be done.

  // 4a) U1 inputs vs. outputs (I2C address 0x21 on I2C channel #4)
  // The "/K_FF_RIGHT_RESET" signal on P10 and "/K_FF_LEFT_RESET" signal on 
  // P11 are outputs.
  // All other signals are inputs

  // # set second I2C switch on channel 4 (U17, address 0x71) to port 6 
  apollo_i2c_ctl_w( 4, 0x71, 1, 0x40);
  apollo_i2c_ctl_reg_w( 4, 0x21, 1, 0x06, 1, 0xff); // 11111111 [P07..P00]
  apollo_i2c_ctl_reg_w( 4, 0x21, 1, 0x07, 1, 0xfc); // 11111100 [P17..P10]

  // 4b) U1 default output values (I2C address 0x21 on I2C channel #4)
  // The outputs on P10 and P11 should default to "1".
  // This negates the active-lo "RESET" inputs on the KU15P FireFlys

  // # set second I2C switch on channel 4 (U17, address 0x71) to port 6 
  apollo_i2c_ctl_w( 4, 0x71, 1, 0x40);
  apollo_i2c_ctl_reg_w( 4, 0x21, 1, 0x02, 1, 0x00); // 00000000 [P07..P00]
  apollo_i2c_ctl_reg_w( 4, 0x21, 1, 0x03, 1, 0x01); // 00000011 [P17..P10]

  // =====================================================
  // CMv1 Schematic 4.06 I2C VU7P OPTICS

  // 5a) U103 inputs vs. outputs (I2C address 0x20 on I2C channel #3)
  // All signals are inputs.

  // # set third I2C switch on channel 3 (U4, address 0x72) to port 0 
  apollo_i2c_ctl_w( 3, 0x72, 1, 0x01);
  apollo_i2c_ctl_reg_w( 3, 0x20, 1, 0x06, 1, 0xff); // 11111111 [P07..P00]
  apollo_i2c_ctl_reg_w( 3, 0x20, 1, 0x07, 1, 0xff); // 11111111 [P17..P10]

  // 5b) U103 default output values (I2C address 0x20 on I2C channel #3)
  // All signals are inputs so nothing needs to be done.

  // 6a) U5 inputs vs. outputs (I2C address 0x21 on I2C channel #3)
  // All signals are inputs.

  // # set third I2C switch on channel 3 (U4, address 0x72) to port 1 
  apollo_i2c_ctl_w( 3, 0x72, 1, 0x02);
  apollo_i2c_ctl_reg_w( 3, 0x21, 1, 0x06, 1, 0xff); // 11111111 [P07..P00]
  apollo_i2c_ctl_reg_w( 3, 0x21, 1, 0x07, 1, 0xff); // 11111111 [P17..P10]

  // 6b) U5 default output values (I2C address 0x21 on I2C channel #3)
  // All signals are inputs so nothing needs to be done.

  // 7a) U6 inputs vs. outputs (I2C address 0x22 on I2C channel #3)
  // The "/V_FF_RIGHT_RESET" signal on P10 and "/V_FF_LEFT_RESET" signal on 
  // P11 are outputs. The "SFP..." signals on P12 and P13 are also outputs.
  // All other signals are inputs

  // # set third I2C switch on channel 3 (U4, address 0x72) to port 2 
  apollo_i2c_ctl_w( 3, 0x72, 1, 0x04);
  apollo_i2c_ctl_reg_w( 3, 0x22, 1, 0x06, 1, 0xff); // 11111111 [P07..P00]
  apollo_i2c_ctl_reg_w( 3, 0x22, 1, 0x07, 1, 0xf0); // 11110000 [P17..P10]

  // 7b) U6 default output values (I2C address 0x22 on I2C channel #3)
  // The outputs on P10 and P11 should default to "1".
  // This negates the active-lo "RESET" inputs on the VU7P FireFlys.
  // The outputs on P12 should default to "0" to enable the optical output.
  // The output on P13 should default to "0" until determined otherwise.

  // # set third I2C switch on channel 3 (U4, address 0x72) to port 2 
  apollo_i2c_ctl_w( 4, 0x72, 1, 0x04);
  apollo_i2c_ctl_reg_w( 4, 0x21, 1, 0x02, 1, 0x00); // 00000000 [P07..P00]
  apollo_i2c_ctl_reg_w( 4, 0x21, 1, 0x03, 1, 0x03); // 00000011 [P17..P10]
}
#endif // REV1
#ifdef REV2
void init_registers_clk()
{
  // initialize the external I2C registers for the clocks and for the optical devices.

  // =====================================================
  // CMv2 Schematic 4.03 I2C CLOCK CONTROL

  // 1a) U88 inputs vs. outputs (I2C address 0x20 on I2C channel #2)
  // The "/INT..." signals on P04 and P05 are inputs.
  // The unused signals on P06, P11, P16, and P17 should be inputs.
  // The remaining 10 signals are outputs.

  // # set I2C switch on channel 2 (U84, address 0x70) to port 6
  apollo_i2c_ctl_w(2, 0x70, 1, 0x40);
  apollo_i2c_ctl_reg_w(2, 0x20, 1, 0x06, 1, 0x70); //  01110000 [P07..P00]
  apollo_i2c_ctl_reg_w(2, 0x20, 1, 0x07, 1, 0xc2); //  11000010 [P17..P10]

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
  apollo_i2c_ctl_w(2, 0x70, 1, 0x40);
  apollo_i2c_ctl_reg_w(2, 0x20, 1, 0x02, 1, 0x80); //  10000000 [P07..P00]
  apollo_i2c_ctl_reg_w(2, 0x20, 1, 0x03, 1, 0x01); //  00000001 [P17..P10]

  // 2a) U83 inputs vs. outputs (I2C address 0x21 on I2C channel #2)
  // The "/INT..." signals on P04, P05, and P06 are inputs.
  // There ane no unused signals.
  // The remaining 13 signals are outputs.

  // # set I2C switch on channel 2 (U84, address 0x70) to port 7
  apollo_i2c_ctl_w(2, 0x70, 1, 0x80);
  apollo_i2c_ctl_reg_w(2, 0x21, 1, 0x06, 1, 0x70); //  01110000 [P07..P00]
  apollo_i2c_ctl_reg_w(2, 0x21, 1, 0x07, 1, 0x00); //  00000000 [P17..P10]

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
  apollo_i2c_ctl_w(2, 0x70, 1, 0x80);
  apollo_i2c_ctl_reg_w(2, 0x21, 1, 0x02, 1, 0x80); //  10000000 [P07..P00]
  apollo_i2c_ctl_reg_w(2, 0x21, 1, 0x03, 1, 0x03); //  00000011 [P17..P10]
}
void init_registers_ff()
{
  // =====================================================
  // CMv2 Schematic 4.05 I2C FPGA#1 OPTICS

  // 3a) U15 inputs vs. outputs (I2C address 0x20 on I2C channel #4)
  // All signals are inputs.

  // # set first I2C switch on channel 4 (U14, address 0x70) to port 7
  apollo_i2c_ctl_w(4, 0x70, 1, 0x80);
  apollo_i2c_ctl_reg_w(4, 0x20, 1, 0x06, 1, 0xff); //  11111111 [P07..P00]
  apollo_i2c_ctl_reg_w(4, 0x20, 1, 0x07, 1, 0xff); //  11111111 [P17..P10]

  // 3b) U15 default output values (I2C address 0x20 on I2C channel #4)
  // All signals are inputs so nothing needs to be done.

  // 4a) U18 inputs vs. outputs (I2C address 0x21 on I2C channel #4)
  // The "/F1_FF_RESET" signal on P10 is an output
  // The "EN_...3V8" signals on P11, P12, and P13 are outputs.
  // All other signals are inputs

  // # set second I2C switch on channel 4 (U17, address 0x71) to port 6
  apollo_i2c_ctl_w(4, 0x71, 1, 0x40);
  apollo_i2c_ctl_reg_w(4, 0x21, 1, 0x06, 1, 0xff); //  11111111 [P07..P00]
  apollo_i2c_ctl_reg_w(4, 0x21, 1, 0x07, 1, 0xf0); //  11110000 [P17..P10]

  // 4b) U18 default output values (I2C address 0x21 on I2C channel #4)
  // The output on P10 should default to "1".
  // This negates the active-lo "RESET" input on the FPGA#1 FireFlys
  // The outputs on P11, P12, and P13 should default to "0"
  // This disables the 3.8 volt power supplies on the three FireFly
  // 12-lane transmitter sites for FPGA#1.

  // # set second I2C switch on channel 4 (U17, address 0x71) to port 6
  apollo_i2c_ctl_w(4, 0x71, 1, 0x40);
  apollo_i2c_ctl_reg_w(4, 0x21, 1, 0x02, 1, 0x00); //  00000000 [P07..P00]
  apollo_i2c_ctl_reg_w(4, 0x21, 1, 0x03, 1, 0x01); //  00000001 [P17..P10]

  // =====================================================
  // CMv2 Schematic 4.06 I2C FPGA#2 OPTICS

  // 5a) U10 inputs vs. outputs (I2C address 0x20 on I2C channel #3)
  // All signals are inputs.

  // # set first I2C switch on channel 3 (U9, address 0x70) to port 7
  apollo_i2c_ctl_w(3, 0x70, 1, 0x80);
  apollo_i2c_ctl_reg_w(3, 0x20, 1, 0x06, 1, 0xff); //  11111111 [P07..P00]
  apollo_i2c_ctl_reg_w(3, 0x20, 1, 0x07, 1, 0xff); //  11111111 [P17..P10]

  // 5b) U10 default output values (I2C address 0x20 on I2C channel #3)
  // All signals are inputs so nothing needs to be done.

  // 6a) U12 inputs vs. outputs (I2C address 0x21 on I2C channel #3)
  // The "/F2_FF_RESET" signal on P10 is an output
  // The "EN_...3V8" signals on P11, P12, and P13 are outputs.
  // All other signals are inputs

  // # set second I2C switch on channel 3 (U11, address 0x71) to port 6
  apollo_i2c_ctl_w(3, 0x71, 1, 0x40);
  apollo_i2c_ctl_reg_w(3, 0x21, 1, 0x06, 1, 0xff); //  11111111 [P07..P00]
  apollo_i2c_ctl_reg_w(3, 0x21, 1, 0x07, 1, 0xf0); //  11110000 [P17..P10]

  // 6b) U12 default output values (I2C address 0x21 on I2C channel #3)
  // The output on P10 should default to "1".
  // This negates the active-lo "RESET" input on the FPGA#2 FireFlys
  // The outputs on P11, P12, and P13 should default to "0"
  // This disables the 3.8 volt power supplies on the three FireFly
  // 12-lane transmitter sites for FPGA#2.

  // # set second I2C switch on channel 3 (U11, address 0x71) to port 6
  apollo_i2c_ctl_w(3, 0x71, 1, 0x40);
  apollo_i2c_ctl_reg_w(3, 0x21, 1, 0x02, 1, 0x00); //  00000000 [P07..P00]
  apollo_i2c_ctl_reg_w(3, 0x21, 1, 0x03, 1, 0x01); //  00000001 [P17..P10]
}
#endif // REV2

#ifdef REV2
int init_load_clk(int clk_n)
{

  if (getPowerControlState() != POWER_ON) {
	  log_debug(LOG_SERVICE, "LGA80D_init() needs more time to turn on power supplies \r\n");
	  vTaskDelay(pdMS_TO_TICKS(3000)); // wait 3s
  }
  clk_n = clk_n -1;
  int PreambleList_row[5] = {6,0,3,0,0};
  int RegisterList_row[5] = {378,0,587,0,0};
  int PostambleList_row[5] = {3,0,5,0,0};
  char *clk_ids[5] = {"r0a","r0b","r1a","r1b","r1c"};
  uint8_t i2c_mux_masks[5] = {0x01,0x02,0x04,0x08,0x10};
  uint8_t i2c_addrs[5] = {0x77,0x6b,0x6b,0x6b,0x6b};

  //apollo_i2c_ctl_w(2, 0x70, 1, 0xc0);
  apollo_i2c_ctl_w(2, 0x70, 1, i2c_mux_masks[clk_n]);


  log_debug(LOG_SERVICE, "Start programming clock %s \r\n", clk_ids[clk_n]);
  uint32_t reg0;
  uint32_t reg1;
  uint32_t data;
  uint32_t check_page;
  uint32_t check_data;


  int first_page = 32*(clk_n);

  log_debug(LOG_SERVICE, "Loading clock %s PreambleList from EEPROM \r\n", clk_ids[clk_n]);
  bool ChangePage = true;
  int HighByte = -1;
  int status_w = -10;
  int status_r = -10;

  for (int i = 0; i < PreambleList_row[clk_n]*3; ++i){
	  if ((i+1) % 3 == 0){ // this is when we retrieve two-byte register and data
		  uint16_t packed_reg0_address = (first_page << 8) + i-2;
		  apollo_i2c_ctl_reg_r(2, 0x50, 2, packed_reg0_address, 1, &reg0);

		  uint16_t packed_reg1_address = (first_page << 8) + i-1;
		  apollo_i2c_ctl_reg_r(2, 0x50, 2, packed_reg1_address, 1, &reg1);

		  uint16_t packed_data_address = (first_page << 8) + i;
		  apollo_i2c_ctl_reg_r(2, 0x50, 2, packed_data_address, 1, &data);

		  int NewHighByte = reg0;

		  if (NewHighByte != HighByte) {
			  ChangePage = true;
		  }
		  else {
			  ChangePage = false;
		  }
		  HighByte = NewHighByte;
		  uint16_t LowByte_reg_addr = reg1;

		  if (ChangePage) {
			  log_debug(LOG_SERVICE, "check page written to clk = 0x%08x\r\n", NewHighByte);
			  status_w = apollo_i2c_ctl_reg_w(2, i2c_addrs[clk_n], 1, 0x01, 1, NewHighByte);
		  }
		  status_r = apollo_i2c_ctl_reg_r(2, i2c_addrs[clk_n], 1, 0x01, 1, &check_page);
		  log_debug(LOG_SERVICE, "check page read from clk = 0x%08x\r\n", check_page);

		  if (status_w != 0 || status_r != 0){
			  log_debug(LOG_SERVICE, "error in w/r statuses for page");
			  log_error(LOG_SERVICE, "write status is %d & read status is %d \r\n",status_w,status_r);
			  return status_w;
		  }


		  log_debug(LOG_SERVICE, "check data written to clk = 0x%08x\r\n", data);
		  status_w = apollo_i2c_ctl_reg_w(2, i2c_addrs[clk_n], 1, LowByte_reg_addr, 1, data);

		  status_r = apollo_i2c_ctl_reg_r(2, i2c_addrs[clk_n], 1, LowByte_reg_addr, 1, &check_data);
		  log_debug(LOG_SERVICE, "check data read from clk = 0x%08x\r\n", check_data);
		  if (status_w != 0 || status_r != 0){
			  log_debug(LOG_SERVICE, "error in w/r statuses for data");
			  log_error(LOG_SERVICE, "write status is %d & read status is %d \r\n",status_w,status_r);
			  return status_w;
		  }



	  }
  }

  vTaskDelay(pdMS_TO_TICKS(3000)); //300 ms minimum

  int reg_page = first_page;
  int reg_pages[3];

  log_debug(LOG_SERVICE, "Loading clock %s RegisterList from EEPROM \r\n", clk_ids[clk_n]);
  ChangePage = true;
  HighByte = -1;
  status_w = -10;
  status_r = -10;
  for (int i = 0; i < RegisterList_row[clk_n]*3; ++i){
	  //vTaskDelay(pdMS_TO_TICKS(330));
	  if ((i+1) % 128 == 1){
		  reg_page += 1;
	  }
	  if ((i+1) % 3 == 1){
		  reg_pages[0] = reg_page;
	  }
	  if ((i+1) % 3 == 2){
		  reg_pages[1] = reg_page;
	  }
	  if ((i+1) % 3 == 0){ // this is when we retrieve two-byte register and data stored in three sequential lines from EEPROM
		  reg_pages[2] = reg_page;

		  uint16_t packed_reg0_address = (reg_pages[0] << 8) + i-2;
		  apollo_i2c_ctl_reg_r(2, 0x50, 2, packed_reg0_address, 1, &reg0);

		  uint16_t packed_reg1_address = (reg_pages[1] << 8) + i-1;
		  apollo_i2c_ctl_reg_r(2, 0x50, 2, packed_reg1_address, 1, &reg1);

		  uint16_t packed_data_address = (reg_pages[2] << 8) + i;
		  apollo_i2c_ctl_reg_r(2, 0x50, 2, packed_data_address, 1, &data);

		  int NewHighByte = reg0;


		  if (NewHighByte != HighByte) {
			  ChangePage = true;
		  }
		  else {
			  ChangePage = false;
		  }
		  HighByte = NewHighByte;
		  uint16_t LowByte_reg_addr = reg1;

		  if (ChangePage) {
			  log_debug(LOG_SERVICE, "check page written to clk = 0x%08x\r\n", NewHighByte);
			  status_w = apollo_i2c_ctl_reg_w(2, i2c_addrs[clk_n], 1, 0x01, 1, NewHighByte);
		  }
		  status_r = apollo_i2c_ctl_reg_r(2, i2c_addrs[clk_n], 1, 0x01, 1, &check_page);
		  log_debug(LOG_SERVICE, "check page read from clk = 0x%08x\r\n", check_page);

		  if (status_w != 0 || status_r != 0){
			  log_debug(LOG_SERVICE, "error in w/r statuses for page");
			  log_error(LOG_SERVICE, "write status is %d & read status is %d \r\n",status_w,status_r);
			  return status_w;
		  }


		  log_debug(LOG_SERVICE, "check data written to clk = 0x%08x\r\n", data);
		  status_w = apollo_i2c_ctl_reg_w(2, i2c_addrs[clk_n], 1, LowByte_reg_addr, 1, data);

		  status_r = apollo_i2c_ctl_reg_r(2, i2c_addrs[clk_n], 1, LowByte_reg_addr, 1, &check_data);
		  log_debug(LOG_SERVICE, "check data read from clk = 0x%08x\r\n", check_data);
		  if (status_w != 0 || status_r != 0){
			  log_debug(LOG_SERVICE, "error in w/r statuses for data");
			  log_error(LOG_SERVICE, "write status is %d & read status is %d \r\n",status_w,status_r);
			  return status_w;
		  }



	  }

  }

  vTaskDelay(pdMS_TO_TICKS(3000)); //300 ms minimum

  int last_page = 32*(clk_n+1)-1;
  log_debug(LOG_SERVICE, "Loading clock %s PostambleList from EEPROM \r\n", clk_ids[clk_n]);
  ChangePage = true;
  HighByte = -1;
  status_w = -10;
  status_r = -10;
  for (int i = 0; i < PostambleList_row[clk_n]*3; ++i){
	  if ((i+1) % 3 == 0){ // this is when we retrieve two-byte register and data stored in three sequential lines from EEPROM
		  uint16_t packed_reg0_address = (last_page << 8) + i-2;
		  apollo_i2c_ctl_reg_r(2, 0x50, 2, packed_reg0_address, 1, &reg0);

		  uint16_t packed_reg1_address = (last_page << 8) + i-1;
		  apollo_i2c_ctl_reg_r(2, 0x50, 2, packed_reg1_address, 1, &reg1);

		  uint16_t packed_data_address = (last_page << 8) + i;
		  apollo_i2c_ctl_reg_r(2, 0x50, 2, packed_data_address, 1, &data);

		  int NewHighByte = reg0;

		  if (NewHighByte != HighByte) {
			  ChangePage = true;
		  }
		  else {
			  ChangePage = false;
		  }
		  HighByte = NewHighByte;
		  uint16_t LowByte_reg_addr = reg1;
		  if (ChangePage) {
			  log_debug(LOG_SERVICE, "check page written to clk = 0x%08x\r\n", NewHighByte);
			  status_w = apollo_i2c_ctl_reg_w(2, i2c_addrs[clk_n], 1, 0x01, 1, NewHighByte);
		  }
		  status_r = apollo_i2c_ctl_reg_r(2, i2c_addrs[clk_n], 1, 0x01, 1, &check_page);
		  log_debug(LOG_SERVICE, "check page read from clk = 0x%08x\r\n", check_page);
		  // note that 0x001C register performs a soft reset rather than store data stored in three sequential lines from EEPROM

		  if (status_w != 0 || status_r != 0){
			  log_debug(LOG_SERVICE, "error in w/r statuses for page");
			  log_error(LOG_SERVICE, "write status is %d & read status is %d \r\n",status_w,status_r);
			  return status_w;
		  }


		  log_debug(LOG_SERVICE, "check data written to clk = 0x%08x\r\n", data);
		  status_w = apollo_i2c_ctl_reg_w(2, i2c_addrs[clk_n], 1, LowByte_reg_addr, 1, data);

		  status_r = apollo_i2c_ctl_reg_r(2, i2c_addrs[clk_n], 1, LowByte_reg_addr, 1, &check_data);
		  log_debug(LOG_SERVICE, "check data read from clk = 0x%08x\r\n", check_data);
		  if (status_w != 0 || status_r != 0){
			  log_debug(LOG_SERVICE, "error in w/r statuses for data");
			  log_error(LOG_SERVICE, "write status is %d & read status is %d \r\n",status_w,status_r);
			  return status_w;
		  }


	  }
  }
  return status_w;

}
#endif // REV2
