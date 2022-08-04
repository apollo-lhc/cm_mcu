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
#include <time.h>   // struct tm

// ROM header must come before MAP header
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "driverlib/hibernate.h"

#include "Tasks.h"
#include "MonitorTask.h"
#include "MonitorI2CTask.h"
#include "InterruptHandlers.h"

#include "common/pinsel.h"
#include "common/smbus_units.h"
#include "common/smbus_helper.h"
#include "I2CCommunication.h"
#include "common/log.h"
#include "common/printf.h"


struct dev_moni2c_addr_t ff_moni2c_addrs[NFIREFLIES] = {
    {"F1_1  12 Tx", FF_I2CMUX_1_ADDR, 0, 0x50}, //
    {"F1_1  12 Rx", FF_I2CMUX_1_ADDR, 1, 0x54}, //
    {"F1_2  12 Tx", FF_I2CMUX_1_ADDR, 3, 0x50}, //
    {"F1_2  12 Rx", FF_I2CMUX_1_ADDR, 4, 0x54}, //
    {"F1_3  12 Tx", FF_I2CMUX_2_ADDR, 3, 0x50}, //
    {"F1_3  12 Rx", FF_I2CMUX_2_ADDR, 4, 0x54}, //
    {"F1_4 4 XCVR", FF_I2CMUX_1_ADDR, 2, 0x50}, //
    {"F1_5 4 XCVR", FF_I2CMUX_2_ADDR, 0, 0x50}, //
    {"F1_6 4 XCVR", FF_I2CMUX_2_ADDR, 1, 0x50}, //
    {"F1_7 4 XCVR", FF_I2CMUX_2_ADDR, 2, 0x50}, //
    {"F2_1  12 Tx", FF_I2CMUX_1_ADDR, 0, 0x50}, //
    {"F2_1  12 Rx", FF_I2CMUX_1_ADDR, 1, 0x54}, //
    {"F2_2  12 Tx", FF_I2CMUX_1_ADDR, 3, 0x50}, //
    {"F2_2  12 Rx", FF_I2CMUX_1_ADDR, 4, 0x54}, //
    {"F2_3  12 Tx", FF_I2CMUX_2_ADDR, 3, 0x50}, //
    {"F2_3  12 Rx", FF_I2CMUX_2_ADDR, 4, 0x54}, //
    {"F2_4 4 XCVR", FF_I2CMUX_1_ADDR, 2, 0x50}, //
    {"F2_5 4 XCVR", FF_I2CMUX_2_ADDR, 0, 0x50}, //
    {"F2_6 4 XCVR", FF_I2CMUX_2_ADDR, 1, 0x50}, //
    {"F2_7 4 XCVR", FF_I2CMUX_2_ADDR, 2, 0x50}, //

};


// FFDAQ arguments for monitoring i2c task of 4-channel firefly ports connected to FPGA1

#ifdef REV2
struct dev_moni2c_addr_t ffldaq_f1_moni2c_addrs[NFIREFLIES_DAQ_F1] = {
    { "F1_4 4 XCVR", FF_I2CMUX_1_ADDR, 2, 0x50}, //
    { "F1_5 4 XCVR", FF_I2CMUX_2_ADDR, 0, 0x50}, //
    { "F1_6 4 XCVR", FF_I2CMUX_2_ADDR, 1, 0x50}, // FF 8
    { "F1_7 4 XCVR", FF_I2CMUX_2_ADDR, 2, 0x50}, //
};
struct sm_command_t sm_command_ffldaq_f1[] = {
    { 1, 0x00, 0x02, 1, "FF_STATUS_REG", 0, 7, "", SM_STATUS },
    { 1, 0x00, 0x16, 1, "FF_TEMPERATURE", 0, 7, "C", SM_STATUS },
    { 1, 0x00, 0x03, 1, "FF_LOS_ALARM", 0, 7, "", SM_STATUS},   // 3 reg address, single-byte output
    { 1, 0x00, 0x05, 1, "FF_CDR_LOL_ALARM", 0, 7, "", SM_STATUS},    // 5 reg address, single-byte output

};
uint16_t ffldaq_f1_values[NSUPPLIES_FFLDAQ_F1 * NCOMMANDS_FFLDAQ_F1];
int8_t ffldaq_f1_vendor_part[16];

struct MonitorI2CTaskArgs_t ffldaq_f1_args = {
    .name = "FFDAQ",
    .devices = ffldaq_f1_moni2c_addrs,
    .i2c_dev = I2C_DEVICE_F1,
    .n_devices = NSUPPLIES_FFLDAQ_F1,
    .commands = sm_command_ffldaq_f1,
    .n_commands = NCOMMANDS_FFLDAQ_F1,
    .n_values = NSUPPLIES_FFLDAQ_F1 * NPAGES_FFLDAQ_F1 * NCOMMANDS_FFLDAQ_F1,
    .n_pages = NPAGES_FFLDAQ_F1,
    .sm_values = ffldaq_f1_values,
    .smbus = &g_sMaster4,
    .smbus_status = &eStatus4,
    .xSem = NULL,
    .requirePower = true,
    .stack_size = 4096U,
    .sm_vendor_part = ffldaq_f1_vendor_part,};

// FFIT arguments for monitoring i2c task of 12-channel firefly ports connected to FPGA1

struct sm_command_t sm_command_fflit_f1[] = {
    { 1, 0x00, 0x02, 1, "FF_STATUS_REG", 0, 7, "", SM_STATUS },
    { 1, 0x00, 0x16, 1, "FF_TEMPERATURE", 0, 7, "C", SM_STATUS },
    { 2, 0x00, 0x07, 2, "FF_LOS_ALARM", 0, 7, "", SM_STATUS}, // 7 reg address, lower-byte output
    { 2, 0x00, 0x14, 2, "FF_CDR_LOL_ALARM", 0, 7, "", SM_STATUS}, // 14 reg address, lower-byte output

};

struct dev_moni2c_addr_t fflit_f1_moni2c_addrs[NFIREFLIES_IT_F1] = { { "F1_1  12 Tx",
    FF_I2CMUX_1_ADDR, 0, 0x50}, //
    { "F1_1  12 Rx", FF_I2CMUX_1_ADDR, 1, 0x54}, //
    { "F1_2  12 Tx", FF_I2CMUX_1_ADDR, 3, 0x50}, //
    { "F1_2  12 Rx", FF_I2CMUX_1_ADDR, 4, 0x54}, //
    { "F1_3  12 Tx", FF_I2CMUX_2_ADDR, 3, 0x50}, //
    { "F1_3  12 Rx", FF_I2CMUX_2_ADDR, 4, 0x54}, //
};

uint16_t fflit_f1_values[NSUPPLIES_FFLIT_F1 * NCOMMANDS_FFLIT_F1];
int8_t fflit_f1_vendor_part[16];

struct MonitorI2CTaskArgs_t fflit_f1_args = {
    .name = "FFIT",
    .devices = fflit_f1_moni2c_addrs,
    .i2c_dev = I2C_DEVICE_F1,
    .n_devices = NSUPPLIES_FFLIT_F1,
    .commands = sm_command_fflit_f1,
    .n_commands = NCOMMANDS_FFLIT_F1,
    .n_values = NSUPPLIES_FFLIT_F1 * NPAGES_FFLIT_F1 * NCOMMANDS_FFLIT_F1,
    .n_pages = NPAGES_FFLIT_F1,
    .sm_values = fflit_f1_values,
    .smbus = &g_sMaster4,
    .smbus_status = &eStatus4,
    .xSem = NULL,
    .requirePower = true,
    .stack_size = 4096U,
    .sm_vendor_part = fflit_f1_vendor_part,};


// FFDAQV arguments for monitoring i2c task of 4-channel firefly ports connected to FPGA2

struct dev_moni2c_addr_t ffldaq_f2_moni2c_addrs[NFIREFLIES_DAQ_F2] = {
    {"F2_4 4 XCVR", FF_I2CMUX_1_ADDR, 2, 0x50}, //
    {"F2_5 4 XCVR", FF_I2CMUX_2_ADDR, 0, 0x50}, //
    {"F2_6 4 XCVR", FF_I2CMUX_2_ADDR, 1, 0x50}, //
    {"F2_7 4 XCVR", FF_I2CMUX_2_ADDR, 2, 0x50}, //
};

struct sm_command_t sm_command_ffldaq_f2[] = {
    { 1, 0x00, 0x02, 1, "FF_STATUS_REG", 0, 7, "", SM_STATUS },
    { 1, 0x00, 0x16, 1, "FF_TEMPERATURE", 0, 7, "C", SM_STATUS },
    { 1, 0x00, 0x03, 1, "FF_LOS_ALARM", 0, 7, "", SM_STATUS},  // 3 reg address, single-byte output
    { 1, 0x00, 0x05, 1, "FF_CDR_LOL_ALARM", 0, 7, "", SM_STATUS}, // 5 reg address, single-byte output

};
uint16_t ffldaq_f2_values[NSUPPLIES_FFLDAQ_F2 * NCOMMANDS_FFLDAQ_F2];
int8_t ffldaq_f2_vendor_part[16];

struct MonitorI2CTaskArgs_t ffldaq_f2_args = {
    .name = "FFDAQV",
    .devices = ffldaq_f2_moni2c_addrs,
    .i2c_dev = I2C_DEVICE_F2,
    .n_devices = NSUPPLIES_FFLDAQ_F2,
    .commands = sm_command_ffldaq_f2,
    .n_commands = NCOMMANDS_FFLDAQ_F2,
    .n_values = NSUPPLIES_FFLDAQ_F2 * NPAGES_FFLDAQ_F2 * NCOMMANDS_FFLDAQ_F2,
    .n_pages = NPAGES_FFLDAQ_F2,
    .sm_values = ffldaq_f2_values,
    .smbus = &g_sMaster3,
    .smbus_status = &eStatus3,
    .xSem = NULL,
    .requirePower = true,
    .stack_size = 4096U,
    .sm_vendor_part = ffldaq_f2_vendor_part,};


// FFITV arguments for monitoring i2c task of 12-channel firefly ports connected to FPGA2

struct sm_command_t sm_command_fflit_f2[] = {
    { 1, 0x00, 0x02, 1, "FF_STATUS_REG", 0, 7, "", SM_STATUS },
    { 1, 0x00, 0x16, 1, "FF_TEMPERATURE", 0, 7, "C", SM_STATUS },
    { 2, 0x00, 0x07, 2, "FF_LOS_ALARM", 0, 7, "", SM_STATUS}, // 7 reg address, high-byte output
    { 2, 0x00, 0x14, 2, "FF_CDR_LOL_ALARM", 0, 7, "", SM_STATUS}, // 14 reg address, high-byte output
};

struct dev_moni2c_addr_t fflit_f2_moni2c_addrs[NFIREFLIES_IT_F2] = {
    {"F2_1  12 Tx", FF_I2CMUX_1_ADDR, 0, 0x50}, //
    {"F2_1  12 Rx", FF_I2CMUX_1_ADDR, 1, 0x54}, //
    {"F2_2  12 Tx", FF_I2CMUX_1_ADDR, 3, 0x50}, //
    {"F2_2  12 Rx", FF_I2CMUX_1_ADDR, 4, 0x54}, //
    {"F2_3  12 Tx", FF_I2CMUX_2_ADDR, 3, 0x50}, //
    {"F2_3  12 Rx", FF_I2CMUX_2_ADDR, 4, 0x54}, //
};

uint16_t fflit_f2_values[NSUPPLIES_FFLIT_F2 * NCOMMANDS_FFLIT_F2];
int8_t fflit_f2_vendor_part[16];

struct MonitorI2CTaskArgs_t fflit_f2_args = {
    .name = "FFITV",
    .devices = fflit_f2_moni2c_addrs,
    .i2c_dev = I2C_DEVICE_F2,
    .n_devices = NSUPPLIES_FFLIT_F2,
    .commands = sm_command_fflit_f2,
    .n_commands = NCOMMANDS_FFLIT_F2,
    .n_values = NSUPPLIES_FFLIT_F2 * NPAGES_FFLIT_F2 * NCOMMANDS_FFLIT_F2,
    .n_pages = NPAGES_FFLIT_F2,
    .sm_values = fflit_f2_values,
    .smbus = &g_sMaster3,
    .smbus_status = &eStatus3,
    .xSem = NULL,
    .requirePower = true,
    .stack_size = 4096U,
    .sm_vendor_part = fflit_f2_vendor_part,};


// Clock arguments for monitoring task

struct dev_moni2c_addr_t clk_moni2c_addrs[] = {
    //{"r0b", 0x70, 1, 0x6b}, // CLK R0B : Si5395-REVA **need .csv file from Charlie**
    {"r1a", 0x70, 2, 0x6b}, // CLK R1A : Si5395-REVA
    {"r1b", 0x70, 3, 0x6b}, // CLK R1B : Si5395-REVA
    //{"r1c", 0x70, 4, 0x6b}, // CLK R1C : Si5395-REVA **need .csv file from Charles**
};


struct sm_command_t sm_command_clk[] = {
    //device information on page 0 : table 16.2 and 16.4
    {1, 0x00,0x02, 1, "PN_BASE", 0, 7, "", SM_STATUS}, //page 0x00
    {1, 0x00,0x03, 1, "PN_BASE", 0, 7, "", SM_STATUS}, //page 0x00
    {1, 0x00,0x05, 1, "DEVICE_REV", 0, 7, "", SM_STATUS}, //page 0x00
    {1, 0x00,0x0B, 1, "I2C_ADDR", 0, 6, "", SM_STATUS}, //page 0x00
    //sticky flags on page 0 : table 16.12 and 16.13
    /*
    {1, 0x00,0x11, 1, "SYSINCAL_FLG", 0, 0, "", SM_STATUS}, //page 0x00
    {1, 0x00,0x11, 1, "LOSXAXB_FLG", 1, 1, "", SM_STATUS}, //page 0x00
    {1, 0x00,0x11, 1, "XAXB_ERR_FLG", 3, 3, "", SM_STATUS}, //page 0x00
    {1, 0x00,0x11, 1, "SMBUS_TIMEOUT_FLG", 5, 5, "", SM_STATUS}, //page 0x00
    {1, 0x00,0x12, 1, "LOS_FLG", 0, 3, "", SM_STATUS}, //page 0x00
    {1, 0x00,0x12, 1, "OOF_FLG", 4, 7, "", SM_STATUS}, //page 0x00
    */
    //internal statuses on page 0 : table 16.8 and 16.9
    //{1, 0x00,0x0C, 1, "SYSINCAL", 0, 0, "", SM_STATUS}, //page 0x00
    {1, 0x00,0x0C, 1, "LOSXAXB", 1, 1, "", SM_STATUS}, //page 0x00
    //{1, 0x00,0x0C, 1, "XAXB_ERR", 3, 3, "", SM_STATUS}, //page 0x00
    //{1, 0x00,0x0C, 1, "SMBUS_TIMEOUT", 5, 5, "", SM_STATUS}, //page 0x00
    {1, 0x00,0x0D, 1, "LOS", 0, 3, "", SM_STATUS}, //page 0x00
    {1, 0x00,0x0D, 1, "OOF", 4, 7, "", SM_STATUS}, //page 0x00
    {1, 0x00,0x0E, 1, "LOL", 1, 1, "", SM_STATUS}, //page 0x00
    //automatic input selection on page 4 and 5 : table 5.3
    /*
    {1, 0x04,0x87, 1, "ZDM_EN", 0, 0, "", SM_STATUS}, //page 0x04
    {1, 0x04,0x87, 1, "ZDM_AUTOSW_EN", 4, 4, "", SM_STATUS}, //page 0x04
    {1, 0x05,0x36, 1, "CLK_SWITCH_MODE", 0, 1, "", SM_STATUS}, //page 0x05
    {1, 0x05,0x36, 1, "HSW_EN", 2, 2, "", SM_STATUS}, //page 0x05
    {1, 0x05,0x38, 1, "IN0_PRIORITY", 0, 2, "", SM_STATUS}, //page 0x05
    {1, 0x05,0x38, 1, "IN1_PRIORITY", 4, 6, "", SM_STATUS}, //page 0x05
    {1, 0x05,0x39, 1, "IN2_PRIORITY", 0, 2, "", SM_STATUS}, //page 0x05
    {1, 0x05,0x39, 1, "IN3_PRIORITY", 4, 6, "", SM_STATUS}, //page 0x05
    {1, 0x05,0x37, 1, "IN_LOS_MSK", 0, 3, "", SM_STATUS}, //page 0x05
    {1, 0x05,0x37, 1, "IN_OOF_MSK", 4, 7, "", SM_STATUS}, //page 0x05
    */

};

// only one of these might be valid
uint16_t clk_values[NSUPPLIES_CLK * NPAGES_CLK * NCOMMANDS_CLK];
int8_t clk_vendor_part[16];

struct MonitorI2CTaskArgs_t clock_args = {
    .name = "CLKSI",
    .devices = clk_moni2c_addrs,
    .i2c_dev = I2C_DEVICE_CLK,
    .n_devices = NSUPPLIES_CLK,
    .commands = sm_command_clk,
    .n_commands = NCOMMANDS_CLK,
    .n_values = NSUPPLIES_CLK * NPAGES_CLK * NCOMMANDS_CLK,
    .n_pages = NPAGES_CLK,
    .sm_values = clk_values,
    .smbus = &g_sMaster2,
    .smbus_status = &eStatus2,
    .xSem = NULL,
    .requirePower = true,
    .stack_size = 4096U,
    .sm_vendor_part = clk_vendor_part,};


struct dev_moni2c_addr_t clkr0a_moni2c_addrs[] = {
    {"r0a", 0x70, 0, 0x77}, // CLK R0A : Si5341-REVD
};

struct sm_command_t sm_command_clkr0a[] = {
    //device information on page 0 : table 14.4 and 14.6
    {1, 0x00,0x02, 1, "PN_BASE", 0, 7, "", SM_STATUS}, //page 0x00
    {1, 0x00,0x03, 1, "PN_BASE", 0, 7, "", SM_STATUS}, //page 0x00
    {1, 0x00,0x05, 1, "DEVICE_REV", 0, 7, "", SM_STATUS}, //page 0x00
    {1, 0x00,0x0B, 1, "I2C_ADDR", 0, 6, "", SM_STATUS}, //page 0x00 for testing
    //sticky flags on page 0 : table 4.5
    /*
    {1, 0x00,0x11, 1, "SYSINCAL_FLG", 0, 0, "", SM_STATUS}, //page 0x00
    {1, 0x00,0x11, 1, "LOSXAXB_FLG", 1, 1, "", SM_STATUS}, //page 0x00
    {1, 0x00,0x11, 1, "LOSREF_FLG", 2, 2, "", SM_STATUS}, //page 0x00
    {1, 0x00,0x11, 1, "LOS_FLG", 3, 3, "", SM_STATUS}, //page 0x00
    {1, 0x00,0x11, 1, "SMBUS_TIMEOUT_FLG", 5, 5, "", SM_STATUS}, //page 0x00
    {1, 0x00,0x12, 1, "LOSIN_FLG", 0, 3, "", SM_STATUS}, //page 0x00
    */
    //internal statuses on page 0 : table 4.5
    {1, 0x00,0x0C, 1, "SYSINCAL", 0, 0, "", SM_STATUS}, //page 0x00
    {1, 0x00,0x0C, 1, "LOSXAXB", 1, 1, "", SM_STATUS}, //page 0x00
    {1, 0x00,0x0C, 1, "LOSREF", 2, 2, "", SM_STATUS}, //page 0x00
    {1, 0x00,0x0C, 1, "LOS", 3, 3, "", SM_STATUS}, //page 0x00
    {1, 0x00,0x0C, 1, "SMBUS_TIMEOUT", 5, 5, "", SM_STATUS}, //page 0x00
    {1, 0x00,0x0D, 1, "LOSIN", 0, 3, "", SM_STATUS}, //page 0x00
    // enable zero delay mode on page 9 : table 14.82
    //{1, 0x09,0x1C, 1, "ZDM_EN", 0, 7, "", SM_STATUS}, //page 0x09

};

uint16_t clkr0a_values[NSUPPLIES_CLKR0A * NPAGES_CLKR0A * NCOMMANDS_CLKR0A];
int8_t clkr0a_vendor_part[16];

struct MonitorI2CTaskArgs_t clockr0a_args = {
    .name = "CLKR0A",
    .devices = clkr0a_moni2c_addrs,
    .i2c_dev = I2C_DEVICE_CLK,
    .n_devices = NSUPPLIES_CLKR0A,
    .commands = sm_command_clkr0a,
    .n_commands = NCOMMANDS_CLKR0A,
    .n_values = NSUPPLIES_CLKR0A * NPAGES_CLKR0A * NCOMMANDS_CLKR0A,
    .n_pages = NPAGES_CLKR0A,
    .sm_values = clkr0a_values,
    .smbus = &g_sMaster2,
    .smbus_status = &eStatus2,
    .xSem = NULL,
    .requirePower = true,
    .stack_size = 4096U,
    .sm_vendor_part = clkr0a_vendor_part,};

int8_t getFFtemp(const uint8_t i)
{
  int i1 = 1;
  int8_t val;
  configASSERT(i < NFIREFLIES);
  if (0 <= i && i < NFIREFLIES_IT_F1){
    int index = i * (fflit_f1_args.n_commands * fflit_f1_args.n_pages) + i1;
    val = fflit_f1_args.sm_values[index];
  }

  else if (NFIREFLIES_IT_F1 <= i && i < NFIREFLIES_IT_F1 + NFIREFLIES_DAQ_F1 ) {
    int index = (i-NFIREFLIES_IT_F1) * (ffldaq_f1_args.n_commands * ffldaq_f1_args.n_pages) + i1;
    val = ffldaq_f1_args.sm_values[index];
  }
  /*
  else if (NFIREFLIES_F1 <= whichff && whichff < NFIREFLIES_F1 + NFIREFLIES_IT_F2) {
    int index = (whichff-NFIREFLIES_F1) * (fflit_f2_args.n_commands * fflit_f2_args.n_pages) + i1;
    uint8_t val = fflit_f2_args.sm_values[index];
    }
  else {
    int index = (whichff-NFIREFLIES_F1-NFIREFLIES_IT_F2) * (ffldaq_f2_args.n_commands * ffldaq_f2_args.n_pages) + i1;
    uint8_t val = ffldaq_f2_args.sm_values[index];
    }
   */
  return val;
}

#endif //REV 2

#ifdef DEBUG_FIF
int8_t *getFFserialnum(const uint8_t i)
{
  configASSERT(i < NFIREFLIES);
  return ff_status[i].serial_num;
}

#endif

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
struct dev_i2c_addr_t fpga_addrs[] = { { "F1_0", 0x70, 3, 0x36 }, // F1 X0Y0
		{ "F1_1", 0x70, 3, 0x34 }, // F1 X0Y1
		{ "F1_2", 0x70, 3, 0x47 }, // F1 X1Y0
		{ "F1_3", 0x70, 3, 0x45 }, // F1 X1Y1
		{ "F2_0", 0x70, 1, 0x36 }, // F2 X0Y0
		{ "F2_1", 0x70, 1, 0x34 }, // F2 X0Y1
		{ "F2_2", 0x70, 1, 0x47 }, // F2 X1Y0
		{ "F2_3", 0x70, 1, 0x45 }, // F2 X1Y1
		};
#define F1F2_NDEVICES 8

struct dev_i2c_addr_t fpga_addrs_f1only[] = {
    {"F1_0", 0x70, 3, 0x36}, // F1 X0Y0
    {"F1_1", 0x70, 3, 0x34}, // F1 X0Y1
    {"F1_2", 0x70, 3, 0x47}, // F1 X1Y0
    {"F1_3", 0x70, 3, 0x45}, // F1 X1Y1
};
#define F1_NDEVICES   4

struct dev_i2c_addr_t fpga_addrs_f2only[] = {
    {"F2_0", 0x70, 1, 0x36}, // F2 X0Y0
    {"F2_1", 0x70, 1, 0x34}, // F2 X0Y1
    {"F2_2", 0x70, 1, 0x47}, // F2 X1Y0
    {"F2_3", 0x70, 1, 0x45}, // F2 X1Y1
};
#define F2_NDEVICES   4

#endif

struct pm_command_t pm_command_fpga[] = { { 0x8d, 2, "READ_TEMPERATURE_1", "C",
		PM_LINEAR11 }, };

// only one of these might be valid
float pm_fpga[FPGA_MON_NVALUES] = { 0 };

struct MonitorTaskArgs_t fpga_args = { .name = "XIMON", .devices = fpga_addrs,
		.n_devices = F1F2_NDEVICES, .commands = pm_command_fpga, .n_commands =
		FPGA_MON_NCOMMANDS, .pm_values = pm_fpga, .n_values =
		FPGA_MON_NVALUES, .n_pages = 1,
#ifdef REV1
    .smbus = &g_sMaster6,
    .smbus_status = &eStatus6,
#elif defined(REV2)
		.smbus = &g_sMaster5, .smbus_status = &eStatus5,
#endif
		.xSem = NULL, .requirePower = true, .stack_size = 4096U, };
#ifdef REV1
// Power supply arguments for Monitoring task
// Supply Address | Voltages | Priority
// ---------------+----------|-----------
//       0x40     | 3.3 & 1.8|     2
//       0x44     | KVCCINT  |     1
//       0x43     | KVCCINT  |     1
//       0x46     | VVCCINT  |     1
//       0x45     | VVCCINT  |     1
struct dev_i2c_addr_t pm_addrs_dcdc[N_PM_ADDRS_DCDC] = {
    {"3V3/1V8", 0x70, 0, 0x40},  // Dual supply 1.8 / 3.3 V
    {"KVCCINT1", 0x70, 1, 0x44}, // first vccint, KU15P
    {"KVCCINT2", 0x70, 2, 0x43}, // second vccint, KU15P
    {"VVCCINT1", 0x70, 3, 0x46}, // first vccint, VU7P
    {"VVCCINT2", 0x70, 4, 0x45}, // second vccint, VU7P
};
#elif defined(REV2) // REV1
// Power supply arguments for Monitoring task
// Supply Address | Voltages  | Priority
// ---------------+-----------|-----------
//       0x40     | 3.3 & 1.8 |     2
//       0x44     | F1VCCINT  |     1
//       0x43     | F1VCCINT  |     1
//       0x46     | F2VCCINT  |     1
//       0x45     | F2VCCINT  |     1

struct dev_i2c_addr_t pm_addrs_dcdc[N_PM_ADDRS_DCDC] = {
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
struct pm_command_t extra_cmds[N_EXTRA_CMDS] = {
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
void LGA80D_init(void) {
	while (xSemaphoreTake(dcdc_args.xSem, (TickType_t) 10) == pdFALSE)
		;
	log_info(LOG_SERVICE, "LGA80D_init\r\n");
	// set up the switching frequency
	uint16_t freqlin11 = float_to_linear11(457.14f);
	uint16_t drooplin11 = float_to_linear11(0.0700f);
	// we do the same for all devices except the 0th one, which is the 1.8/3.3V device
	for (int dev = 1; dev < NSUPPLIES_PS; dev += 1) {
		for (uint8_t page = 0; page < 2; ++page) {
			// page register
			int r = apollo_pmbus_rw(&g_sMaster1, &eStatus1, false,
					pm_addrs_dcdc + dev, &extra_cmds[0], &page);
			if (r) {
				log_debug(LOG_SERVICE, "dev = %d, page = %d, r= %d\r\n", dev,
						page, r);
				log_error(LOG_SERVICE, "LGA80D(0)\r\n");
			}
			// actual command -- frequency switch
			r = apollo_pmbus_rw(&g_sMaster1, &eStatus1, false,
					pm_addrs_dcdc + dev, &extra_cmds[2], (uint8_t*) &freqlin11);
			if (r) {
				log_error(LOG_SERVICE, "LGA80D(1)\r\n");
			}
			// actual command -- vout_droop switch
			r = apollo_pmbus_rw(&g_sMaster1, &eStatus1, false,
					pm_addrs_dcdc + dev, &extra_cmds[5],
					(uint8_t*) &drooplin11);
			if (r) {
				log_error(LOG_SERVICE, "LGA80D(2)\r\n");
			}
			// actual command -- multiphase_ramp_gain switch
			uint8_t val = 0x7U; // by suggestion of Artesian
			r = apollo_pmbus_rw(&g_sMaster1, &eStatus1, false,
					pm_addrs_dcdc + dev, &extra_cmds[6], &val);
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

struct MonitorTaskArgs_t dcdc_args = { .name = "PSMON",
		.devices = pm_addrs_dcdc, .n_devices = NSUPPLIES_PS, .commands =
				pm_command_dcdc, .n_commands = NCOMMANDS_PS, .pm_values =
				dcdc_values,
		.n_values = NSUPPLIES_PS * NPAGES_PS * NCOMMANDS_PS, .n_pages =
		NPAGES_PS, .smbus = &g_sMaster1, .smbus_status = &eStatus1,
		.xSem = NULL, .requirePower = false, .stack_size = 4096U, };

static int fpga_f1 = -1;
static int fpga_f2 = -1;
int get_f1_index() {
	return fpga_f1;
}
int get_f2_index() {
	return fpga_f2;
}
void set_f1_index(int index) {
	fpga_f1 = index;
	return;
}
void set_f2_index(int index) {
	fpga_f2 = index;
	return;
}

void initFPGAMon() {
	// check if we are to include both FPGAs or not
	bool f1_enable = isFPGAF1_PRESENT();
	bool f2_enable = isFPGAF2_PRESENT();
#ifndef REV1 // FIXME REMOVE THESE
  write_gpio_pin(JTAG_FROM_SM, 1);
  write_gpio_pin(FPGA_CFG_FROM_FLASH, 0);
  write_gpio_pin(F1_FPGA_PROGRAM, 0);
#endif        // not REV1
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
	} else if (!f2_enable && f1_enable) {
		fpga_args.devices = fpga_addrs_f1only;
		fpga_args.n_devices = F1_NDEVICES;
		set_f1_index(0);
#ifndef REV1
		write_gpio_pin(_F1_JTAG_BYPASS, 1);
		write_gpio_pin(_F2_JTAG_BYPASS, 0);
#endif // REV1
	} else {
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
  while (!ROM_SysCtlPeripheralReady(SYSCTL_PERIPH_HIBERNATE)) {
  }
  // Enable the clocking. AFAIK the argument is not used
  ROM_HibernateEnableExpClk(g_ui32SysClock);
  // Set to use external crystal with 12 pF drive
  ROM_HibernateClockConfig(HIBERNATE_OSC_LOWDRIVE);
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

  // grab the semaphore to ensure unique access to I2C controller
  if (clock_args.xSem != NULL) {
    while (xSemaphoreTake(clock_args.xSem, (TickType_t) 10) == pdFALSE)
      ;
  }

  // # set I2C switch on channel 2 (U94, address 0x70) to port 6
  apollo_i2c_ctl_w(2, 0x70, 1, 0x40);
  apollo_i2c_ctl_reg_w(2, 0x20, 1, 0x06, 1, 0xf0); // 11110000 [P07..P00]
  apollo_i2c_ctl_reg_w(2, 0x20, 1, 0x07, 1, 0xe0); // 11100000 [P17..P10]

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
  apollo_i2c_ctl_w(2, 0x70, 1, 0x40);
  apollo_i2c_ctl_reg_w(2, 0x20, 1, 0x02, 1, 0xf0); // 11110000 [P07..P00]
  apollo_i2c_ctl_reg_w(2, 0x20, 1, 0x03, 1, 0xf8); // 11111000 [P17..P10]

  // 2a) U92 inputs vs. outputs (I2C address 0x21 on I2C channel #2)
  // The signals on P00, P01, and P02 are inputs.
  // There other signals are unused and should be set as inputs.
  // There are no outputs.

  // # set I2C switch on channel 2 (U94, address 0x70) to port 7
  apollo_i2c_ctl_w(2, 0x70, 1, 0x80);
  apollo_i2c_ctl_reg_w(2, 0x21, 1, 0x06, 1, 0xff); // 11111111 [P07..P00]
  apollo_i2c_ctl_reg_w(2, 0x21, 1, 0x07, 1, 0xff); // 11111111 [P17..P10]

  // 2b) U92 default output values (I2C address 0x21 on I2C channel #2)
  // All signals are inputs so nothing needs to be done.

  xSemaphoreGive(clock_args.xSem);
}
void init_registers_ff()
{

  // =====================================================
  // CMv1 Schematic 4.05 I2C KU15P OPTICS

  // 3a) U102 inputs vs. outputs (I2C address 0x20 on I2C channel #4)
  // All signals are inputs.

  // # set first I2C switch on channel 4 (U100, address 0x70) to port 7
  apollo_i2c_ctl_w(4, 0x70, 1, 0x80);
  apollo_i2c_ctl_reg_w(4, 0x20, 1, 0x06, 1, 0xff); // 11111111 [P07..P00]
  apollo_i2c_ctl_reg_w(4, 0x20, 1, 0x07, 1, 0xff); // 11111111 [P17..P10]

  // 3b) U102 default output values (I2C address 0x20 on I2C channel #4)
  // All signals are inputs so nothing needs to be done.

  // 4a) U1 inputs vs. outputs (I2C address 0x21 on I2C channel #4)
  // The "/K_FF_RIGHT_RESET" signal on P10 and "/K_FF_LEFT_RESET" signal on
  // P11 are outputs.
  // All other signals are inputs

  // # set second I2C switch on channel 4 (U17, address 0x71) to port 6
  apollo_i2c_ctl_w(4, 0x71, 1, 0x40);
  apollo_i2c_ctl_reg_w(4, 0x21, 1, 0x06, 1, 0xff); // 11111111 [P07..P00]
  apollo_i2c_ctl_reg_w(4, 0x21, 1, 0x07, 1, 0xfc); // 11111100 [P17..P10]

  // 4b) U1 default output values (I2C address 0x21 on I2C channel #4)
  // The outputs on P10 and P11 should default to "1".
  // This negates the active-lo "RESET" inputs on the KU15P FireFlys

  // # set second I2C switch on channel 4 (U17, address 0x71) to port 6
  apollo_i2c_ctl_w(4, 0x71, 1, 0x40);
  apollo_i2c_ctl_reg_w(4, 0x21, 1, 0x02, 1, 0x00); // 00000000 [P07..P00]
  apollo_i2c_ctl_reg_w(4, 0x21, 1, 0x03, 1, 0x01); // 00000011 [P17..P10]

  // =====================================================
  // CMv1 Schematic 4.06 I2C VU7P OPTICS

  // 5a) U103 inputs vs. outputs (I2C address 0x20 on I2C channel #3)
  // All signals are inputs.

  // # set third I2C switch on channel 3 (U4, address 0x72) to port 0
  apollo_i2c_ctl_w(3, 0x72, 1, 0x01);
  apollo_i2c_ctl_reg_w(3, 0x20, 1, 0x06, 1, 0xff); // 11111111 [P07..P00]
  apollo_i2c_ctl_reg_w(3, 0x20, 1, 0x07, 1, 0xff); // 11111111 [P17..P10]

  // 5b) U103 default output values (I2C address 0x20 on I2C channel #3)
  // All signals are inputs so nothing needs to be done.

  // 6a) U5 inputs vs. outputs (I2C address 0x21 on I2C channel #3)
  // All signals are inputs.

  // # set third I2C switch on channel 3 (U4, address 0x72) to port 1
  apollo_i2c_ctl_w(3, 0x72, 1, 0x02);
  apollo_i2c_ctl_reg_w(3, 0x21, 1, 0x06, 1, 0xff); // 11111111 [P07..P00]
  apollo_i2c_ctl_reg_w(3, 0x21, 1, 0x07, 1, 0xff); // 11111111 [P17..P10]

  // 6b) U5 default output values (I2C address 0x21 on I2C channel #3)
  // All signals are inputs so nothing needs to be done.

  // 7a) U6 inputs vs. outputs (I2C address 0x22 on I2C channel #3)
  // The "/V_FF_RIGHT_RESET" signal on P10 and "/V_FF_LEFT_RESET" signal on
  // P11 are outputs. The "SFP..." signals on P12 and P13 are also outputs.
  // All other signals are inputs

  // # set third I2C switch on channel 3 (U4, address 0x72) to port 2
  apollo_i2c_ctl_w(3, 0x72, 1, 0x04);
  apollo_i2c_ctl_reg_w(3, 0x22, 1, 0x06, 1, 0xff); // 11111111 [P07..P00]
  apollo_i2c_ctl_reg_w(3, 0x22, 1, 0x07, 1, 0xf0); // 11110000 [P17..P10]

  // 7b) U6 default output values (I2C address 0x22 on I2C channel #3)
  // The outputs on P10 and P11 should default to "1".
  // This negates the active-lo "RESET" inputs on the VU7P FireFlys.
  // The outputs on P12 should default to "0" to enable the optical output.
  // The output on P13 should default to "0" until determined otherwise.

  // # set third I2C switch on channel 3 (U4, address 0x72) to port 2
  apollo_i2c_ctl_w(4, 0x72, 1, 0x04);
  apollo_i2c_ctl_reg_w(4, 0x21, 1, 0x02, 1, 0x00); // 00000000 [P07..P00]
  apollo_i2c_ctl_reg_w(4, 0x21, 1, 0x03, 1, 0x03); // 00000011 [P17..P10]
}
#endif // REV1
#ifdef REV2
void init_registers_clk() {
	// initialize the external I2C registers for the clocks and for the optical devices.

	// =====================================================
	// CMv2 Schematic 4.03 I2C CLOCK CONTROL

	// 1a) U88 inputs vs. outputs (I2C address 0x20 on I2C channel #2)
	// The "/INT..." signals on P04 and P05 are inputs.
	// The unused signals on P06, P11, P16, and P17 should be inputs.
	// The remaining 10 signals are outputs.

  // grab the semaphore to ensure unique access to I2C controller
  if (clock_args.xSem != NULL) {
    while (xSemaphoreTake(clock_args.xSem, (TickType_t) 10) == pdFALSE)
      ;
  }

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

	xSemaphoreGive(clock_args.xSem);
}
void init_registers_ff() {

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

#define EEPROM_MAX_PER_PAGE 126

static int load_clk_registers(int reg_count, uint16_t reg_page, uint16_t i2c_addrs)
{
  int8_t HighByte = -1; // keep track when reg0 is changed
  int status_w = -1;

  for (int i = 0; i < reg_count * 3; ++i) {

    if ((i + 1) % EEPROM_MAX_PER_PAGE == 1 && HighByte != -1) {
      reg_page += 1;
    }

    if ((i + 1) % 3 == 0) { // this is when we retrieve two-byte address and data stored in three sequential lines from eeprom
      uint32_t triplet;     // two-byte address and data, both read from EEPROM
      uint16_t packed_reg0_address = (reg_page << 8) + (i - 2) % EEPROM_MAX_PER_PAGE;
      int status_r = apollo_i2c_ctl_reg_r(CLOCK_I2C_DEV, CLOCK_I2C_EEPROM_ADDR, 2,
                                          packed_reg0_address, 3, &triplet); //read triplet from eeprom
      if (status_r != 0) {
        log_error(LOG_SERVICE, "read failed: %s\r\n", SMBUS_get_error(status_r));
        xSemaphoreGive(clock_args.xSem);
        return status_r;
      }
      // organize the three bytes
      uint8_t data = (triplet >> 16) & 0xFFU; //high byte of two-byte address (a page of clock config to keep track when writing data to a clock chip)
      uint8_t reg1 = (triplet >> 8) & 0xFFU;  // low byte of two-byte address
      uint8_t reg0 = triplet & 0xFFU;         // data for each address

      if (reg0 != HighByte) { // new page
        log_debug(LOG_SERVICE, "Change page to %x\r\n", reg0);
        status_w = apollo_i2c_ctl_reg_w(CLOCK_I2C_DEV, i2c_addrs, 1, 0x01, 1, reg0); // write a page change to a clock chip
        if (status_w != 0) {
          log_error(LOG_SERVICE, "write failed: %s\r\n", SMBUS_get_error(status_w));
          xSemaphoreGive(clock_args.xSem);
          return status_w; // fail writing and exit
        }
        HighByte = reg0; //update the current high byte or page
      }

      status_w = apollo_i2c_ctl_reg_w(CLOCK_I2C_DEV, i2c_addrs, 1, reg1, 1, data); //write data to a clock chip
      if (status_w != 0) {
        log_error(LOG_SERVICE, "write status is %d \r\n", status_w);
        xSemaphoreGive(clock_args.xSem);
        return status_w; // fail writing and exit
      }
    }
  }
  //xSemaphoreGive(clock_args.xSem);
  return status_w;
}

int init_load_clk(int clk_n)
{

  while (getPowerControlState() != POWER_ON) {
    vTaskDelay(pdMS_TO_TICKS(10)); //delay 10 ms
  }

  char *clk_ids[5] = {"r0a", "r0b", "r1a", "r1b", "r1c"};
  uint8_t i2c_addrs = CLOCK_CHIP_COMMON_I2C_ADDR; // i2c address of a clock chip
  if (clk_n == 0)
    i2c_addrs = CLOCK_CHIP_R0A_I2C_ADDR;

  // grab the semaphore to ensure unique access to I2C controller
  if (clock_args.xSem != NULL) {
    while (xSemaphoreTake(clock_args.xSem, (TickType_t) 10) == pdFALSE)
      ;
  }

  apollo_i2c_ctl_w(CLOCK_I2C_DEV, CLOCK_I2C_MUX_ADDR, 1, 1 << clk_n);
  uint16_t init_preamble_page = 32 * (clk_n);
  uint16_t init_register_page = 32 * (clk_n) + 1;
  uint16_t init_postamble_page = 32 * (clk_n + 1) - 1;

  uint32_t PreambleList_row; //the size of preamble list in a clock config file store at the end of the last eeprom page of a clock
  int status_r = apollo_i2c_ctl_reg_r(CLOCK_I2C_DEV, CLOCK_I2C_EEPROM_ADDR, 2, (init_postamble_page << 8) + 0x007C, 1, &PreambleList_row);

  if (status_r != 0) {
    log_error(LOG_SERVICE, "PreL read error: %s\r\n", SMBUS_get_error(status_r));
    xSemaphoreGive(clock_args.xSem);
    return status_r; // fail reading and exit
  }

  uint32_t RegisterList_row; //the size of register list in a clock config file store at the end of the last eeprom page of a clock
  status_r = apollo_i2c_ctl_reg_r(CLOCK_I2C_DEV, CLOCK_I2C_EEPROM_ADDR, 2, (init_postamble_page << 8) + 0x007D, 2, &RegisterList_row);
  if (status_r != 0) {
    log_error(LOG_SERVICE, "RL read error: %s\r\n", SMBUS_get_error(status_r));
    xSemaphoreGive(clock_args.xSem);
    return status_r; // fail reading and exit
  }

  uint32_t PostambleList_row; //the size of postamble list in a clock config file store at the end of the last eeprom page of a clock
  status_r = apollo_i2c_ctl_reg_r(CLOCK_I2C_DEV, CLOCK_I2C_EEPROM_ADDR, 2, (init_postamble_page << 8) + 0x007F, 1, &PostambleList_row);
  if (status_r != 0) {
    log_error(LOG_SERVICE, "PosL read error: %s\r\n", SMBUS_get_error(status_r));
    xSemaphoreGive(clock_args.xSem);
    return status_r; // fail reading and exit
  }

  log_debug(LOG_SERVICE, "Start programming clock %s\r\n", clk_ids[clk_n]);
  log_debug(LOG_SERVICE, "Loading clock %s PreambleList from EEPROM\r\n", clk_ids[clk_n]);
  int status_w = load_clk_registers(PreambleList_row, init_preamble_page, i2c_addrs);
  if (status_w != 0) {
    log_error(LOG_SERVICE, "PreL write error %d\r\n", status_w);
    xSemaphoreGive(clock_args.xSem);
    return status_w;
  }
  vTaskDelay(pdMS_TO_TICKS(330)); //300 ms minimum delay
  log_debug(LOG_SERVICE, "Loading clock %s RegisterList from EEPROM\r\n", clk_ids[clk_n]);
  status_w = load_clk_registers(RegisterList_row, init_register_page, i2c_addrs);
  if (status_w != 0) {
    log_error(LOG_SERVICE, "RegL write error %d\r\n", status_w);
    xSemaphoreGive(clock_args.xSem);
    return status_w;
  }
  vTaskDelay(pdMS_TO_TICKS(330)); //300 ms minimum delay
  log_debug(LOG_SERVICE, "Loading clock %s PostambleList from EEPROM\r\n", clk_ids[clk_n]);
  status_w = load_clk_registers(PostambleList_row, init_postamble_page, i2c_addrs);
  if (status_w != 0) {
    log_error(LOG_SERVICE, "PosL write error %d\r\n", status_w);
    xSemaphoreGive(clock_args.xSem);
    return status_w;
  }
  xSemaphoreGive(clock_args.xSem);
  return status_w;
}
#endif // REV2
