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

#include "Tasks.h"
#include "MonitorTask.h"
#include "InterruptHandlers.h"

#include "common/pinsel.h"
#include "common/smbus_units.h"
// local sprintf prototype
int snprintf( char *buf, unsigned int count, const char *format, ... );


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
    .initfcn = NULL,
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

int apollo_pmbus_rw(tSMBus *smbus, volatile tSMBusStatus *smbus_status,
                    bool read, struct dev_i2c_addr_t* add,
                    struct pm_command_t * cmd, uint8_t * value)
{
  // write to the I2C mux
  uint8_t data;
  // select the appropriate output for the mux
  data = 0x1U << add->mux_bit;
  tSMBusStatus r = SMBusMasterI2CWrite(smbus, add->mux_addr, &data, 1);
  if ( r != SMBUS_OK ) {
    return -1;
  }
  while ( SMBusStatusGet(smbus) == SMBUS_TRANSFER_IN_PROGRESS) {
    vTaskDelay( pdMS_TO_TICKS( 10 )); // wait
  }
  if ( *smbus_status != SMBUS_OK ) {
    return -2;
  }
  // read/write to the device itself
  if ( read ) {
    r = SMBusMasterByteWordRead(smbus, add->dev_addr, cmd->command,
        value, cmd->size);
  }
  else { // write
    r = SMBusMasterByteWordWrite(smbus, add->dev_addr, cmd->command,
        value, cmd->size);
  }
  if ( r  != SMBUS_OK ) {
    return -3;
  }
  while ( SMBusStatusGet(smbus) == SMBUS_TRANSFER_IN_PROGRESS) {
    vTaskDelay( pdMS_TO_TICKS( 10 )); // wait
  }
  // this is checking the return from the interrupt
  if (*smbus_status != SMBUS_OK ) {
    return -4;
  }
  // if we get here, a successful read/write command

  return 0;
}

void Print(const char*);

// this function is run once in the dcdc monitoring task
struct pm_command_t extra_cmds[] = {
    {0x0,  1, "PAGE", "", PM_STATUS},
    {0x1,  1, "OPERATION", "", PM_STATUS},
    {0x33, 2, "FREQUENCY_SWITCH", "Hz", PM_LINEAR11},
    {0xEA, 32, "SNAPSHOP", "", PM_STATUS},
    {0xF3, 1, "SNAPSHOP_CONTROL", "", PM_STATUS},
};

void snapdump(struct dev_i2c_addr_t *add, uint8_t page,
              uint8_t snapshot[32], bool reset)
{
  while(xSemaphoreTake(xMonSem, (TickType_t) 10) == pdFALSE)
    ;
  // page register
  int r = apollo_pmbus_rw(&g_sMaster1, &eStatus1,
      false, add, &extra_cmds[0], &page);
  if ( r ) {
    Print("error in snapdump (0)\r\n");
  }

  // actual command -- snapshot control copy NVRAM for reading
  uint8_t cmd = 0x1;
  r = apollo_pmbus_rw(&g_sMaster1, &eStatus1,
      false, add, &extra_cmds[4], &cmd);
  if ( r ) {
    Print("error in scdump 1\r\n");
  }
  // actual command -- read snapshot
  tSMBusStatus r2 = SMBusMasterBlockRead(&g_sMaster1, add->dev_addr,
      extra_cmds[3].command, &snapshot[0]);
  if ( r2 != SMBUS_OK ) {
    Print("error setting up block read (scdump 2)\r\n");
  }
  while ( (r2 = SMBusStatusGet(&g_sMaster1)) == SMBUS_TRANSFER_IN_PROGRESS) {
    vTaskDelay( pdMS_TO_TICKS( 10 )); // wait
  }
  if ( r2 != SMBUS_TRANSFER_COMPLETE ) {
    Print("error in scdump(3)\r\n");
  }
  if ( reset ) {
    // reset SNAPSHOT. This will fail if the device is on.
    cmd = 0x3;
    r = apollo_pmbus_rw(&g_sMaster1, &eStatus1,
        false, add,&extra_cmds[4],  &cmd);
    if ( r ) {
      Print("error in scdump 4\r\n");
    }
  }
  xSemaphoreGive(xMonSem);
}
 
void dcdc_initfcn(void)
{
  // set up the switching frequency
  //uint16_t freqlin11 = float_to_linear11(457.14);
  uint16_t freqlin11 = float_to_linear11(800.);
  for ( int dev = 1; dev < 4; dev += 2 ) {
    for (uint8_t page = 0; page < 2; ++ page ) {
      // page register
      int r = apollo_pmbus_rw(&g_sMaster1, &eStatus1,
          false, pm_addrs_dcdc+dev, &extra_cmds[0], &page);
      if ( r ) {
        Print("error in dcdc_initfcn (0)\r\n");
      }
      // actual command -- frequency switch
      r = apollo_pmbus_rw(&g_sMaster1, &eStatus1,
          false, pm_addrs_dcdc+dev,&extra_cmds[2],  (uint8_t*)&freqlin11);
      if ( r ) {
        Print("error in dcdc_initfcn (1)\r\n");
      }
    }
  }
  return;
}


// if you change the length of this array, you also need to change
// NCOMMANDS_PS in MonitorTask.h
// TODO make this fix automatic
struct pm_command_t pm_command_dcdc[] = {
        { 0x8d, 2, "READ_TEMPERATURE_1", "C", PM_LINEAR11 },
        { 0x8f, 2, "READ_TEMPERATURE_3", "C", PM_LINEAR11 },
        { 0x88, 2, "READ_VIN", "V", PM_LINEAR11 },
        { 0x8B, 2, "READ_VOUT", "V", PM_LINEAR16U },
        { 0x8c, 2, "READ_IOUT", "A", PM_LINEAR11 },
        { 0x79, 2, "STATUS_WORD", "", PM_STATUS },
        { 0x4F, 2, "OT_FAULT_LIMIT", "C", PM_LINEAR11},
        { 0xE7, 2, "IOUT_AVG_OC_FAULT_LIMIT", "A", PM_LINEAR11 },
        { 0x95, 2, "READ_FREQUENCY", "Hz", PM_LINEAR11},
        { 0x46, 2, "IOUT_OC_FAULT_LIMIT", "A", PM_LINEAR11},
        { 0x44, 2, "VOUT_UV_FAULT_LIMIT", "V", PM_LINEAR16U},
        { 0x37, 2, "INTERLEAVE", "", PM_STATUS},
        { 0x80, 1, "STATUS_MFR_SPECIFIC", "", PM_STATUS},
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
    //.initfcn = &dcdc_initfcn,
    .initfcn = NULL,
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
  else {
    set_vu_index(0);
    set_ku_index(1);
  }

}
