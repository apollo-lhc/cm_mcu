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
    {"F1AVCC/TT", 0x70, 5, 0x40}, // AVCC/AVTT for F1
    {"F2AVCC/TT", 0x70, 6, 0x40}, // AVCC/AVTT for F2
};
#else
#error "need to define either Rev1 or Rev2"
#endif // REV1
void Print(const char *);

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
  // set to a default value
  struct tm now = {
    .tm_sec = 0,
    .tm_min = 0,
    .tm_hour = 0,
    .tm_mday = 23,
    .tm_mon = 10, // month goes from 0-11
    .tm_year = 121, // year is since 1900
    .tm_wday = 0,
    .tm_yday = 0,
    .tm_isdst = 0,
  };
  ROM_HibernateCalendarSet(&now);
}
#endif // REV2
