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
#include "Tasks.h"
#include "MonitorTask.h"
#include "InterruptHandlers.h"

#include "common/pinsel.h"


// FPGA arguments for monitoring task
struct dev_i2c_addr_t fpga_addrs[] = {
    {"VU7P", 0x70, 1, 0x36},
    {"KU15P", 0x70, 0, 0x36},
};

struct dev_i2c_addr_t fpga_addrs_kuonly[] = {
    {"KU15P", 0x70, 0, 0x36},
};

struct dev_i2c_addr_t fpga_addrs_vuonly[] = {
    {"VU7P", 0x70, 1, 0x36},
};


struct pm_command_t pm_command_fpga[] = {
    { 0x8d, 2, "READ_TEMPERATURE_1", "C", PM_LINEAR11 },
};

// only one of these might be valid
float pm_fpga[2] = {0.0f,0.0f};

struct MonitorTaskArgs_t fpga_args = {
    .name = "XIMON",
    .devices = fpga_addrs,
    .n_devices = 2,
    .commands = pm_command_fpga,
    .n_commands = 1,
    .pm_values = pm_fpga,
    .n_values = 2,
    .n_pages = 1,
    .smbus = &g_sMaster6,
    .smbus_status = &eStatus6,
};

// Power supply arguments for Monitoring task
// Supply Address | Voltages | Priority
// ---------------+----------|-----------
//       0x40     | 3.3 & 1.8|     2
//       0x44     | KVCCINT  |     1
//       0x43     | KVCCINT  |     1
//       0x46     | VVCCINT  |     1
//       0x45     | VVCCINT  |     1
struct dev_i2c_addr_t pm_addrs_dcdc[] = {
    {"3V3/1V8", 0x70, 0, 0x40},
    {"KVCCINT1", 0x70, 1, 0x44},
    {"KVCCINT2", 0x70, 2, 0x43},
    {"VVCCINT1", 0x70, 3, 0x46},
    {"VVCCINT2", 0x70, 4, 0x45},
};


struct pm_command_t pm_command_dcdc[] = {
        { 0x8d, 2, "READ_TEMPERATURE_1", "C", PM_LINEAR11 },
        { 0x8f, 2, "READ_TEMPERATURE_3", "C", PM_LINEAR11 },
        { 0x88, 2, "READ_VIN", "V", PM_LINEAR11 },
        { 0x8B, 2, "READ_VOUT", "V", PM_LINEAR16U },
        { 0x8c, 2, "READ_IOUT", "A", PM_LINEAR11 },
        //{ 0x4F, 2, "OT_FAULT_LIMIT", "C", PM_LINEAR11},
        { 0x79, 2, "STATUS_WORD", "", PM_STATUS },
        //{ 0xE7, 2, "IOUT_AVG_OC_FAULT_LIMIT", "A", PM_LINEAR11 },
        //{ 0x95, 2, "READ_FREQUENCY", "Hz", PM_LINEAR11},
      };
float dcdc_values[NSUPPLIES_PS*NPAGES_PS*NCOMMANDS_PS];
struct MonitorTaskArgs_t dcdc_args = {
    .name = "PSMON",
    .devices = pm_addrs_dcdc,
    .n_devices = NSUPPLIES_PS,
    .commands = pm_command_dcdc,
    .n_commands = NCOMMANDS_PS,
    .pm_values = dcdc_values,
    .n_values = NSUPPLIES_PS*NPAGES_PS*NCOMMANDS_PS,
    .n_pages = NPAGES_PS,
    .smbus = &g_sMaster1,
    .smbus_status = &eStatus1,
};





static int fpga_ku = -1;
static int fpga_vu = -1;
int get_ku_index()
{
  return fpga_ku;
}
int get_vu_index()
{
  return fpga_vu;
}
void set_ku_index(int index)
{
  fpga_ku = index;
  return;
}
void set_vu_index(int index)
{
  fpga_vu = index;
  return;
}

void initFPGAMon()
{
  // check if we are to include both FPGAs or not
  bool ku_enable = (read_gpio_pin(TM4C_DIP_SW_1) == 1);
  bool vu_enable = (read_gpio_pin(TM4C_DIP_SW_2) == 1);
  configASSERT(ku_enable || vu_enable);
  if ( ! ku_enable && vu_enable ) {
    fpga_args.devices = fpga_addrs_vuonly;
    fpga_args.n_devices = 1;
    set_vu_index(0);
  }
  else if ( ! vu_enable && ku_enable ) {
    fpga_args.devices = fpga_addrs_kuonly;
    fpga_args.n_devices = 1;
    set_ku_index(0);
  }

}
