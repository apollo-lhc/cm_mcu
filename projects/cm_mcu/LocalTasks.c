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
#include <sys/_types.h>
#include <time.h> // struct tm

// ROM header must come before MAP header
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "driverlib/hibernate.h"

#include "Tasks.h"
#include "MonitorTask.h"
#include "MonitorI2CTask.h"
#include "InterruptHandlers.h"
#include "Semaphore.h"

#include "common/pinsel.h"
#include "common/smbus_units.h"
#include "common/smbus_helper.h"
#include "I2CCommunication.h"
#include "common/log.h"
#include "common/printf.h"
#include "inc/hw_memmap.h"

convert_8_t tmp1;

// local prototype
void Print(const char *str);

uint32_t ff_PRESENT_mask = 0; // global variable from getting combined ff signals
uint32_t ff_USER_mask = 0;    // global variable of ff signals from user input
#ifdef REV2
uint32_t f1_ff12xmit_4v0_sel = 0; // global variable for FPGA1 12-ch xmit ff's power-supply physical selection
uint32_t f2_ff12xmit_4v0_sel = 0; // global variable for FPGA2 12-ch xmit ff's power-supply physical selection

struct ff_bit_mask_t ff_bitmask_args[] = {
    {0U, 0U}, // {3, 6} bits correspond to ffl12_f1 devices
    {0U, 0U}, // {0, 4} and bits correspond to ffldaq_f1 devices
    {0U, 0U}, // {3, 6} bits correspond to ffl12_f2 devices
    {0U, 0U}, // {0, 4} bits correspond to ffldaq_f2 devices
};

#endif
// outputs from *_PRESENT pins for constructing ff_PRESENT_mask
#ifdef REV1
//      4.05 I2C KU15P OPTICS
uint32_t present_FFLDAQ_F1, present_FFL12_F1,
    //      4.06 I2C VU7P OPTICS (the I/O expanders at 0x20 and 0x21 have mixed 4-ch (FFLDAQ) and 12-ch (FFL12) pins)
    present_0X20_F2, present_0X21_F2, present_FFLDAQ_0X20_F2, present_FFL12_0X20_F2,
    present_FFLDAQ_0X21_F2, present_FFL12_0X21_F2 = 0;
#elif defined(REV2)
//      4.05 I2C FPGA31 OPTICS
uint32_t present_FFLDAQ_F1, present_FFL12_F1,
    //      4.06 I2C FPGA2 OPTICS
    present_FFLDAQ_F2, present_FFL12_F2 = 0;
#endif // REV2

#ifdef REV1
// -------------------------------------------------
//
// REV 1
//
// -------------------------------------------------

struct dev_moni2c_addr_t ff_moni2c_addrs[NFIREFLIES] = {
    {"K01  12 Tx GTH", FF_I2CMUX_1_ADDR, 0, 0x50}, //
    {"K01  12 Rx GTH", FF_I2CMUX_1_ADDR, 1, 0x54}, //
    {"K02  12 Tx GTH", FF_I2CMUX_1_ADDR, 2, 0x50}, //
    {"K02  12 Rx GTH", FF_I2CMUX_1_ADDR, 3, 0x54}, //
    {"K03  12 Tx GTH", FF_I2CMUX_1_ADDR, 4, 0x50}, //
    {"K03  12 Rx GTH", FF_I2CMUX_1_ADDR, 5, 0x54}, //
    {"K04 4 XCVR GTY", FF_I2CMUX_2_ADDR, 0, 0x50}, //
    {"K05 4 XCVR GTY", FF_I2CMUX_2_ADDR, 1, 0x50}, //
    {"K06 4 XCVR GTY", FF_I2CMUX_2_ADDR, 2, 0x50}, //
    {"K07  12 Tx GTY", FF_I2CMUX_2_ADDR, 3, 0x50}, //
    {"K07  12 Rx GTY", FF_I2CMUX_2_ADDR, 4, 0x54}, //
    {"V01 4 XCVR GTY", FF_I2CMUX_1_ADDR, 0, 0x50}, //
    {"V02 4 XCVR GTY", FF_I2CMUX_1_ADDR, 1, 0x50}, //
    {"V03 4 XCVR GTY", FF_I2CMUX_1_ADDR, 2, 0x50}, //
    {"V04 4 XCVR GTY", FF_I2CMUX_1_ADDR, 3, 0x50}, //
    {"V05 4 XCVR GTY", FF_I2CMUX_1_ADDR, 4, 0x50}, //
    {"V06 4 XCVR GTY", FF_I2CMUX_1_ADDR, 5, 0x50}, //
    {"V07 4 XCVR GTY", FF_I2CMUX_2_ADDR, 0, 0x50}, //
    {"V08 4 XCVR GTY", FF_I2CMUX_2_ADDR, 1, 0x50}, //
    {"V09 4 XCVR GTY", FF_I2CMUX_2_ADDR, 2, 0x50}, //
    {"V10 4 XCVR GTY", FF_I2CMUX_2_ADDR, 3, 0x50}, //
    {"V11  12 Tx GTY", FF_I2CMUX_1_ADDR, 6, 0x50}, //
    {"V11  12 Rx GTY", FF_I2CMUX_1_ADDR, 7, 0x54}, //
    {"V12  12 Tx GTY", FF_I2CMUX_2_ADDR, 4, 0x50}, //
    {"V12  12 Rx GTY", FF_I2CMUX_2_ADDR, 5, 0x54}, //
};

struct arg_moni2c_ff_t ff_moni2c_arg[NFIREFLY_ARG] = {
    {"FFL12", &ffl12_f1_args, 0, 0, 6},     //
    {"FFLDAQ", &ffldaq_f1_args, 6, 0, 3},   //
    {"FFL12", &ffl12_f1_args, 9, 6, 2},     //
    {"FFLDAQ", &ffldaq_f2_args, 11, 0, 10}, //
    {"FFL12", &ffl12_f2_args, 21, 0, 4},    //
};

#elif defined(REV2)
// -------------------------------------------------
//
// REV 2
//
// -------------------------------------------------
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

struct arg_moni2c_ff_t ff_moni2c_arg[NFIREFLY_ARG] = {
    {"FFL12", &ffl12_f1_args, 0, 0, 6},    //
    {"FFLDAQ", &ffldaq_f1_args, 6, 0, 4},  //
    {"FFL12", &ffl12_f2_args, 10, 0, 6},   //
    {"FFLDAQ", &ffldaq_f2_args, 16, 0, 4}, //
};
#else
#error "Define either Rev1 or Rev2"
#endif

// FFDAQ arguments for monitoring i2c task of 4-channel firefly ports connected to FPGA1
#ifdef REV1
struct dev_moni2c_addr_t ffldaq_f1_moni2c_addrs[NFIREFLIES_DAQ_F1] = {
    {"K04 4 XCVR GTY", FF_I2CMUX_2_ADDR, 0, 0x50}, //
    {"K05 4 XCVR GTY", FF_I2CMUX_2_ADDR, 1, 0x50}, //
    {"K06 4 XCVR GTY", FF_I2CMUX_2_ADDR, 2, 0x50}, //
};
#elif defined(REV2)
struct dev_moni2c_addr_t ffldaq_f1_moni2c_addrs[NFIREFLIES_DAQ_F1] = {
    {"F1_4 4 XCVR", FF_I2CMUX_1_ADDR, 2, 0x50}, //
    {"F1_5 4 XCVR", FF_I2CMUX_2_ADDR, 0, 0x50}, //
    {"F1_6 4 XCVR", FF_I2CMUX_2_ADDR, 1, 0x50}, //
    {"F1_7 4 XCVR", FF_I2CMUX_2_ADDR, 2, 0x50}, //
};
#else
#error "Define either Rev1 or Rev2"
#endif

struct sm_command_t sm_command_ffldaq_f1[] = {
    {1, 0x00, 0x02, 2, "FF_STATUS_REG", 0xff, "", PM_STATUS},
    {1, 0x00, 0x16, 2, "FF_TEMPERATURE", 0xff, "C", PM_STATUS},
    {1, 0x00, 0x03, 1, "FF_LOS_ALARM", 0xff, "", PM_STATUS},
    {1, 0x00, 0x05, 1, "FF_CDR_LOL_ALARM", 0xff, "", PM_STATUS},
    {2, 0x00, 0x22, 2, "FF_CH01_OPT_POW", 0xff, "mw", PM_STATUS}, // read 4 Rx-ch registers with increasing addresses
    {2, 0x00, 0x24, 2, "FF_CH02_OPT_POW", 0xff, "mw", PM_STATUS},
    {2, 0x00, 0x26, 2, "FF_CH03_OPT_POW", 0xff, "mw", PM_STATUS},
    {2, 0x00, 0x28, 2, "FF_CH04_OPT_POW", 0xff, "mw", PM_STATUS},
};

uint16_t ffldaq_f1_values[NSUPPLIES_FFLDAQ_F1 * NCOMMANDS_FFLDAQ_F1];

struct MonitorI2CTaskArgs_t ffldaq_f1_args = {
    .name = "FFDAQ",
    .devices = ffldaq_f1_moni2c_addrs,
    .i2c_dev = I2C_DEVICE_F1,
    .n_devices = NSUPPLIES_FFLDAQ_F1,
    .commands = sm_command_ffldaq_f1,
    .n_commands = NCOMMANDS_FFLDAQ_F1,
    .n_values = NSUPPLIES_FFLDAQ_F1 * NPAGES_FFLDAQ_F1 * NCOMMANDS_FFLDAQ_F1,
    .n_pages = NPAGES_FFLDAQ_F1,
    .selpage_reg = FF_SELPAGE_REG,
    .sm_values = ffldaq_f1_values,
    .smbus = &g_sMaster4,
    .smbus_status = &eStatus4,
    .xSem = NULL,
    .stack_size = 4096U,
};

// FF12 arguments for monitoring i2c task of 12-channel firefly ports connected to FPGA1

// register maps for IT-DTC Fireflies 12-ch part -- future will be CERN-B but currently is 14Gbps ECUO
struct sm_command_t sm_command_fflit_f1[] = {
    {1, 0x00, 0x02, 2, "FF_STATUS_REG", 0xff, "", PM_STATUS},
    {1, 0x00, 0x16, 2, "FF_TEMPERATURE", 0xff, "C", PM_STATUS},
    {2, 0x00, 0x07, 1, "FF_LOS_ALARM", 0xffff, "", PM_STATUS},
    {2, 0x00, 0x14, 1, "FF_CDR_LOL_ALARM", 0xffff, "", PM_STATUS},
    // there are no registers to read optical power for 14Gbps ECUO.
    // registers below are a placeholder with a reading equal to zero
    // the reason we need them because n_commands is fixed
    {1, 0x00, 0x00, 1, "FF_CH01_OPT_POW", 0xff, "mw", PM_STATUS},
    {1, 0x00, 0x00, 1, "FF_CH02_OPT_POW", 0xff, "mw", PM_STATUS},
    {1, 0x00, 0x00, 1, "FF_CH03_OPT_POW", 0xff, "mw", PM_STATUS},
    {1, 0x00, 0x00, 1, "FF_CH04_OPT_POW", 0xff, "mw", PM_STATUS},
    {1, 0x00, 0x00, 1, "FF_CH05_OPT_POW", 0xff, "mw", PM_STATUS},
    {1, 0x00, 0x00, 1, "FF_CH06_OPT_POW", 0xff, "mw", PM_STATUS},
    {1, 0x00, 0x00, 1, "FF_CH07_OPT_POW", 0xff, "mw", PM_STATUS},
    {1, 0x00, 0x00, 1, "FF_CH08_OPT_POW", 0xff, "mw", PM_STATUS},
    {1, 0x00, 0x00, 1, "FF_CH09_OPT_POW", 0xff, "mw", PM_STATUS},
    {1, 0x00, 0x00, 1, "FF_CH10_OPT_POW", 0xff, "mw", PM_STATUS},
    {1, 0x00, 0x00, 1, "FF_CH11_OPT_POW", 0xff, "mw", PM_STATUS},
    {1, 0x00, 0x00, 1, "FF_CH12_OPT_POW", 0xff, "mw", PM_STATUS},

};
// register maps for OT-DTC Fireflies 12-ch part -- 25Gbps ECUO (no connected devices to test as of 08.04.22)
// **commands below have not been tested yet**
struct sm_command_t sm_command_fflot_f1[] = {
    {1, 0x00, 0x02, 2, "FF_STATUS_REG", 0xff, "", PM_STATUS},
    {1, 0x00, 0x16, 2, "FF_TEMPERATURE", 0xff, "C", PM_STATUS},
    {2, 0x00, 0x07, 1, "FF_LOS_ALARM", 0xffff, "", PM_STATUS},
    {2, 0x00, 0x14, 1, "FF_CDR_LOL_ALARM", 0xffff, "", PM_STATUS},
    {2, 0x01, 0xe4, 2, "FF_CH01_OPT_POW", 0xff, "mw", PM_STATUS}, // read 12 Rx-ch registers  with decreasing addresses
    {2, 0x01, 0xe2, 2, "FF_CH02_OPT_POW", 0xff, "mw", PM_STATUS},
    {2, 0x01, 0xe0, 2, "FF_CH03_OPT_POW", 0xff, "mw", PM_STATUS},
    {2, 0x01, 0xde, 2, "FF_CH04_OPT_POW", 0xff, "mw", PM_STATUS},
    {2, 0x01, 0xdc, 2, "FF_CH05_OPT_POW", 0xff, "mw", PM_STATUS},
    {2, 0x01, 0xda, 2, "FF_CH06_OPT_POW", 0xff, "mw", PM_STATUS},
    {2, 0x01, 0xd8, 2, "FF_CH07_OPT_POW", 0xff, "mw", PM_STATUS},
    {2, 0x01, 0xd6, 2, "FF_CH08_OPT_POW", 0xff, "mw", PM_STATUS},
    {2, 0x01, 0xd4, 2, "FF_CH09_OPT_POW", 0xff, "mw", PM_STATUS},
    {2, 0x01, 0xd2, 2, "FF_CH10_OPT_POW", 0xff, "mw", PM_STATUS},
    {2, 0x01, 0xd0, 2, "FF_CH11_OPT_POW", 0xff, "mw", PM_STATUS},
    {2, 0x01, 0xce, 2, "FF_CH12_OPT_POW", 0xff, "mw", PM_STATUS},

};

#ifdef REV1
struct dev_moni2c_addr_t ffl12_f1_moni2c_addrs[NFIREFLIES_IT_F1] = {
    {"K01  12 Tx GTH", FF_I2CMUX_1_ADDR, 0, 0x50}, //
    {"K01  12 Rx GTH", FF_I2CMUX_1_ADDR, 1, 0x54}, //
    {"K02  12 Tx GTH", FF_I2CMUX_1_ADDR, 2, 0x50}, //
    {"K02  12 Rx GTH", FF_I2CMUX_1_ADDR, 3, 0x54}, //
    {"K03  12 Tx GTH", FF_I2CMUX_1_ADDR, 4, 0x50}, //
    {"K03  12 Rx GTH", FF_I2CMUX_1_ADDR, 5, 0x54}, //
    {"K07  12 Tx GTY", FF_I2CMUX_2_ADDR, 3, 0x50}, //
    {"K07  12 Rx GTY", FF_I2CMUX_2_ADDR, 4, 0x54}, //
};
#elif defined(REV2)
struct dev_moni2c_addr_t ffl12_f1_moni2c_addrs[NFIREFLIES_IT_F1] = {
    {"F1_1  12 Tx",
     FF_I2CMUX_1_ADDR, 0, 0x50},                //
    {"F1_1  12 Rx", FF_I2CMUX_1_ADDR, 1, 0x54}, //
    {"F1_2  12 Tx", FF_I2CMUX_1_ADDR, 3, 0x50}, //
    {"F1_2  12 Rx", FF_I2CMUX_1_ADDR, 4, 0x54}, //
    {"F1_3  12 Tx", FF_I2CMUX_2_ADDR, 3, 0x50}, //
    {"F1_3  12 Rx", FF_I2CMUX_2_ADDR, 4, 0x54}, //
};
#else
#error "Define either Rev1 or Rev2"
#endif

uint16_t ffl12_f1_values[NSUPPLIES_FFL12_F1 * NCOMMANDS_FFL12_F1];

struct MonitorI2CTaskArgs_t ffl12_f1_args = {
    .name = "FF12",
    .devices = ffl12_f1_moni2c_addrs,
    .i2c_dev = I2C_DEVICE_F1,
    .n_devices = NSUPPLIES_FFL12_F1,
    .commands = sm_command_fflot_f1, // 25Gbps by default but if the 14Gbsp 12-ch part is found, the set of commands is changed in INIT task
    .n_commands = NCOMMANDS_FFL12_F1,
    .n_values = NSUPPLIES_FFL12_F1 * NPAGES_FFL12_F1 * NCOMMANDS_FFL12_F1,
    .n_pages = NPAGES_FFL12_F1,
    .selpage_reg = FF_SELPAGE_REG,
    .sm_values = ffl12_f1_values,
    .smbus = &g_sMaster4,
    .smbus_status = &eStatus4,
    .xSem = NULL,
    .stack_size = 4096U,
};

// FFDAQV arguments for monitoring i2c task of 4-channel firefly ports connected to FPGA2
#ifdef REV1
struct dev_moni2c_addr_t ffldaq_f2_moni2c_addrs[NFIREFLIES_DAQ_F2] = {
    {"V01 4 XCVR GTY", FF_I2CMUX_1_ADDR, 0, 0x50}, //
    {"V02 4 XCVR GTY", FF_I2CMUX_1_ADDR, 1, 0x50}, //
    {"V03 4 XCVR GTY", FF_I2CMUX_1_ADDR, 2, 0x50}, //
    {"V04 4 XCVR GTY", FF_I2CMUX_1_ADDR, 3, 0x50}, //
    {"V05 4 XCVR GTY", FF_I2CMUX_1_ADDR, 4, 0x50}, //
    {"V06 4 XCVR GTY", FF_I2CMUX_1_ADDR, 5, 0x50}, //
    {"V07 4 XCVR GTY", FF_I2CMUX_2_ADDR, 0, 0x50}, //
    {"V08 4 XCVR GTY", FF_I2CMUX_2_ADDR, 1, 0x50}, //
    {"V09 4 XCVR GTY", FF_I2CMUX_2_ADDR, 2, 0x50}, //
    {"V10 4 XCVR GTY", FF_I2CMUX_2_ADDR, 3, 0x50}, //
};
#elif defined(REV2)
struct dev_moni2c_addr_t ffldaq_f2_moni2c_addrs[NFIREFLIES_DAQ_F2] = {
    {"F2_4 4 XCVR", FF_I2CMUX_1_ADDR, 2, 0x50}, //
    {"F2_5 4 XCVR", FF_I2CMUX_2_ADDR, 0, 0x50}, //
    {"F2_6 4 XCVR", FF_I2CMUX_2_ADDR, 1, 0x50}, //
    {"F2_7 4 XCVR", FF_I2CMUX_2_ADDR, 2, 0x50}, //
};
#else
#error "Define either Rev1 or Rev2"
#endif

struct sm_command_t sm_command_ffldaq_f2[] = {
    {1, 0x00, 0x02, 2, "FF_STATUS_REG", 0xff, "", PM_STATUS},
    {1, 0x00, 0x16, 2, "FF_TEMPERATURE", 0xff, "C", PM_STATUS},
    {1, 0x00, 0x03, 1, "FF_LOS_ALARM", 0xff, "", PM_STATUS},
    {1, 0x00, 0x05, 1, "FF_CDR_LOL_ALARM", 0xff, "", PM_STATUS},
    {2, 0x00, 0x22, 2, "FF_CH01_OPT_POW", 0xff, "mw", PM_STATUS}, // read 4 Rx-ch registers with increasing addresses
    {2, 0x00, 0x24, 2, "FF_CH02_OPT_POW", 0xff, "mw", PM_STATUS},
    {2, 0x00, 0x26, 2, "FF_CH03_OPT_POW", 0xff, "mw", PM_STATUS},
    {2, 0x00, 0x28, 2, "FF_CH04_OPT_POW", 0xff, "mw", PM_STATUS},
};
uint16_t ffldaq_f2_values[NSUPPLIES_FFLDAQ_F2 * NCOMMANDS_FFLDAQ_F2];

struct MonitorI2CTaskArgs_t ffldaq_f2_args = {
    .name = "FFDAV",
    .devices = ffldaq_f2_moni2c_addrs,
    .i2c_dev = I2C_DEVICE_F2,
    .n_devices = NSUPPLIES_FFLDAQ_F2,
    .commands = sm_command_ffldaq_f2,
    .n_commands = NCOMMANDS_FFLDAQ_F2,
    .n_values = NSUPPLIES_FFLDAQ_F2 * NPAGES_FFLDAQ_F2 * NCOMMANDS_FFLDAQ_F2,
    .n_pages = NPAGES_FFLDAQ_F2,
    .selpage_reg = FF_SELPAGE_REG,
    .sm_values = ffldaq_f2_values,
    .smbus = &g_sMaster3,
    .smbus_status = &eStatus3,
    .xSem = NULL,
    .stack_size = 4096U,
};

// FF12V arguments for monitoring i2c task of 12-channel firefly ports connected to FPGA2

// register maps for IT-DTC Fireflies 12-ch part -- future will be CERN-B but currently is 14Gbps ECUO
struct sm_command_t sm_command_fflit_f2[] = {
    {1, 0x00, 0x02, 2, "FF_STATUS_REG", 0xff, "", PM_STATUS},
    {1, 0x00, 0x16, 2, "FF_TEMPERATURE", 0xff, "C", PM_STATUS},
    {2, 0x00, 0x07, 1, "FF_LOS_ALARM", 0xffff, "", PM_STATUS},
    {2, 0x00, 0x14, 1, "FF_CDR_LOL_ALARM", 0xffff, "", PM_STATUS},
    // there are no registers to read optical power for 14Gbps ECUO.
    // registers below are a placeholder with a reading equal to zero
    // the reason we need them because n_commands is fixed
    {1, 0x00, 0x00, 1, "FF_CH01_OPT_POW", 0xff, "mw", PM_STATUS},
    {1, 0x00, 0x00, 1, "FF_CH02_OPT_POW", 0xff, "mw", PM_STATUS},
    {1, 0x00, 0x00, 1, "FF_CH03_OPT_POW", 0xff, "mw", PM_STATUS},
    {1, 0x00, 0x00, 1, "FF_CH04_OPT_POW", 0xff, "mw", PM_STATUS},
    {1, 0x00, 0x00, 1, "FF_CH05_OPT_POW", 0xff, "mw", PM_STATUS},
    {1, 0x00, 0x00, 1, "FF_CH06_OPT_POW", 0xff, "mw", PM_STATUS},
    {1, 0x00, 0x00, 1, "FF_CH07_OPT_POW", 0xff, "mw", PM_STATUS},
    {1, 0x00, 0x00, 1, "FF_CH08_OPT_POW", 0xff, "mw", PM_STATUS},
    {1, 0x00, 0x00, 1, "FF_CH09_OPT_POW", 0xff, "mw", PM_STATUS},
    {1, 0x00, 0x00, 1, "FF_CH10_OPT_POW", 0xff, "mw", PM_STATUS},
    {1, 0x00, 0x00, 1, "FF_CH11_OPT_POW", 0xff, "mw", PM_STATUS},
    {1, 0x00, 0x00, 1, "FF_CH12_OPT_POW", 0xff, "mw", PM_STATUS},
};
// register maps for OT-DTC Fireflies 12-ch part -- 25Gbps ECUO (no connected devices to test as of 08.04.22)
// **commands below have not been tested yet**
struct sm_command_t sm_command_fflot_f2[] = {
    {1, 0x00, 0x02, 2, "FF_STATUS_REG", 0xff, "", PM_STATUS},
    {1, 0x00, 0x16, 2, "FF_TEMPERATURE", 0xff, "C", PM_STATUS},
    {2, 0x00, 0x07, 1, "FF_LOS_ALARM", 0xffff, "", PM_STATUS},
    {2, 0x00, 0x14, 1, "FF_CDR_LOL_ALARM", 0xffff, "", PM_STATUS},
    {2, 0x01, 0xe4, 2, "FF_CH01_OPT_POW", 0xff, "mw", PM_STATUS}, // read 12 Rx-ch registers  with decreasing addresses
    {2, 0x01, 0xe2, 2, "FF_CH02_OPT_POW", 0xff, "mw", PM_STATUS},
    {2, 0x01, 0xe0, 2, "FF_CH03_OPT_POW", 0xff, "mw", PM_STATUS},
    {2, 0x01, 0xde, 2, "FF_CH04_OPT_POW", 0xff, "mw", PM_STATUS},
    {2, 0x01, 0xdc, 2, "FF_CH05_OPT_POW", 0xff, "mw", PM_STATUS},
    {2, 0x01, 0xda, 2, "FF_CH06_OPT_POW", 0xff, "mw", PM_STATUS},
    {2, 0x01, 0xd8, 2, "FF_CH07_OPT_POW", 0xff, "mw", PM_STATUS},
    {2, 0x01, 0xd6, 2, "FF_CH08_OPT_POW", 0xff, "mw", PM_STATUS},
    {2, 0x01, 0xd4, 2, "FF_CH09_OPT_POW", 0xff, "mw", PM_STATUS},
    {2, 0x01, 0xd2, 2, "FF_CH10_OPT_POW", 0xff, "mw", PM_STATUS},
    {2, 0x01, 0xd0, 2, "FF_CH11_OPT_POW", 0xff, "mw", PM_STATUS},
    {2, 0x01, 0xce, 2, "FF_CH12_OPT_POW", 0xff, "mw", PM_STATUS},
};

#ifdef REV1
struct dev_moni2c_addr_t ffl12_f2_moni2c_addrs[NFIREFLIES_IT_F2] = {
    {"V11  12 Tx GTY", FF_I2CMUX_1_ADDR, 6, 0x50}, //
    {"V11  12 Rx GTY", FF_I2CMUX_1_ADDR, 7, 0x54}, //
    {"V12  12 Tx GTY", FF_I2CMUX_2_ADDR, 4, 0x50}, //
    {"V12  12 Rx GTY", FF_I2CMUX_2_ADDR, 5, 0x54}, //
};
#elif defined(REV2)
struct dev_moni2c_addr_t ffl12_f2_moni2c_addrs[NFIREFLIES_IT_F2] = {
    {"F2_1  12 Tx", FF_I2CMUX_1_ADDR, 0, 0x50}, //
    {"F2_1  12 Rx", FF_I2CMUX_1_ADDR, 1, 0x54}, //
    {"F2_2  12 Tx", FF_I2CMUX_1_ADDR, 3, 0x50}, //
    {"F2_2  12 Rx", FF_I2CMUX_1_ADDR, 4, 0x54}, //
    {"F2_3  12 Tx", FF_I2CMUX_2_ADDR, 3, 0x50}, //
    {"F2_3  12 Rx", FF_I2CMUX_2_ADDR, 4, 0x54}, //
};
#else
#error "Define either Rev1 or Rev2"
#endif

uint16_t ffl12_f2_values[NSUPPLIES_FFL12_F2 * NCOMMANDS_FFL12_F2];

struct MonitorI2CTaskArgs_t ffl12_f2_args = {
    .name = "FF12V",
    .devices = ffl12_f2_moni2c_addrs,
    .i2c_dev = I2C_DEVICE_F2,
    .n_devices = NSUPPLIES_FFL12_F2,
    .commands = sm_command_fflot_f2, // 25Gbps by default but if the 14Gbsp 12-ch part is found, the set of commands is changed in INIT task
    .n_commands = NCOMMANDS_FFL12_F2,
    .n_values = NSUPPLIES_FFL12_F2 * NPAGES_FFL12_F2 * NCOMMANDS_FFL12_F2,
    .n_pages = NPAGES_FFL12_F2,
    .selpage_reg = FF_SELPAGE_REG,
    .sm_values = ffl12_f2_values,
    .smbus = &g_sMaster3,
    .smbus_status = &eStatus3,
    .xSem = NULL,
    .stack_size = 4096U,
};

#ifdef REV2
// Clock arguments for monitoring task

struct clk_program_t clkprog_args[] = {
    {"", ""}, //
    {"", ""}, //
    {"", ""}, //
    {"", ""}, //
    {"", ""}, //
};

struct dev_moni2c_addr_t clk_moni2c_addrs[CLOCK_NUM_SI5395] = {
    {"r0b", 0x70, 1, 0x6b, 0x264E}, // CLK R0B : Si5395-REVA #regs = 587 (read at 0x1F7D in EEPROM) if change, addr 0x264E will have to change
    {"r1a", 0x70, 2, 0x6b, 0x464E}, // CLK R1A : Si5395-REVA #regs = 587 (read at 0x5F7D in EEPROM) if change, addr 0x464E will have to change
    {"r1b", 0x70, 3, 0x6b, 0x664E}, // CLK R1B : Si5395-REVA #regs = 584 (read at 0x7F7D in EEPROM) if change, addr 0x664E will have to change
    {"r1c", 0x70, 4, 0x6b, 0x864E}, // CLK R1C : Si5395-REVA #regs = 587 (read at 0x9F7D in EEPROM) if change, addr 0x864E will have to change
};

struct sm_command_t sm_command_clk[] = {
    // device information on page 0 : table 16.2 and 16.4
    {1, 0x00, 0x02, 2, "PN_BASE", 0xffff, "", PM_STATUS},    // page 0x00
    {1, 0x00, 0x05, 1, "DEVICE_REV", 0xff, "", PM_STATUS}, // page 0x00
    {1, 0x00, 0x0B, 1, "I2C_ADDR", 0x7f, "", PM_STATUS},   // page 0x00
    // internal statuses on page 0 : table 16.8 and 16.9
    {1, 0x00, 0x0C, 1, "LOSXAXB", 0x02, "", PM_STATUS},   // page 0x00
    {1, 0x00, 0x0D, 1, "LOSOOF_IN", 0xff, "", PM_STATUS}, // page 0x00
    {1, 0x00, 0x0E, 1, "LOL", 0x02, "", PM_STATUS},       // page 0x00
    // internal error flags : table 16.12
    {1, 0x00, 0x11, 1, "STICKY_FLG", 0x27, "", PM_STATUS}, // page 0x00
};

uint16_t clk_values[NSUPPLIES_CLK * NPAGES_CLK * NCOMMANDS_CLK];

struct MonitorI2CTaskArgs_t clock_args = {
    .name = "CLKSI",
    .devices = clk_moni2c_addrs,
    .i2c_dev = I2C_DEVICE_CLK,
    .n_devices = NSUPPLIES_CLK,
    .commands = sm_command_clk,
    .n_commands = NCOMMANDS_CLK,
    .n_values = NSUPPLIES_CLK * NPAGES_CLK * NCOMMANDS_CLK,
    .n_pages = NPAGES_CLK,
    .selpage_reg = CLK_SELPAGE_REG,
    .sm_values = clk_values,
    .smbus = &g_sMaster2,
    .smbus_status = &eStatus2,
    .xSem = NULL,
    .stack_size = 4096U,
};

struct dev_moni2c_addr_t clkr0a_moni2c_addrs[CLOCK_NUM_SI5341] = {
    {"r0a", 0x70, 0, 0x77, 0x45D}, // CLK R0A : Si5341-REVD with #regs = 378 (read at 0x1F7D in EEPROM) if change, addr 0x45D will have to change
};

struct sm_command_t sm_command_clkr0a[] = {
    // device information on page 0 : table 14.4 and 14.6
    {1, 0x00, 0x02, 2, "PN_BASE", 0xffff, "", PM_STATUS},    // page 0x00
    {1, 0x00, 0x05, 1, "DEVICE_REV", 0xff, "", PM_STATUS}, // page 0x00
    {1, 0x00, 0x0B, 1, "I2C_ADDR", 0xff, "", PM_STATUS},   // page 0x00
    // internal statuses on page 0 : table 14.5
    {1, 0x00, 0x0C, 1, "STATUS", 0x35, "", PM_STATUS}, // page 0x00
    {1, 0x00, 0x0D, 1, "LOS", 0x15, "", PM_STATUS},    // page 0x00
    // sticky bits of status bits : table 14.12
    {1, 0x00, 0x12, 1, "LOSIN_FLG", 0xf, "", PM_STATUS}, // page 0x00
    // sticky bits of status bits : table 14.12
    {1, 0x00, 0x11, 1, "STICKY_FLG", 0x2f, "", PM_STATUS}, // page 0x00
};

uint16_t clkr0a_values[NSUPPLIES_CLKR0A * NPAGES_CLKR0A * NCOMMANDS_CLKR0A];

struct MonitorI2CTaskArgs_t clockr0a_args = {
    .name = "CLKR0A",
    .devices = clkr0a_moni2c_addrs,
    .i2c_dev = I2C_DEVICE_CLK,
    .n_devices = NSUPPLIES_CLKR0A,
    .commands = sm_command_clkr0a,
    .n_commands = NCOMMANDS_CLKR0A,
    .n_values = NSUPPLIES_CLKR0A * NPAGES_CLKR0A * NCOMMANDS_CLKR0A,
    .n_pages = NPAGES_CLKR0A,
    .selpage_reg = CLK_SELPAGE_REG,
    .sm_values = clkr0a_values,
    .smbus = &g_sMaster2,
    .smbus_status = &eStatus2,
    .xSem = NULL,
    .stack_size = 4096U,
};
#endif // REV2

void setFFmask(uint32_t ff_combined_present)
{

  log_info(LOG_SERVICE, "Setting a bit mask of enabled Fireflys to 1 \r\n");

  // int32_t data = (~ff_combined_present) & 0xFFFFFU; // the bit value for an FF mask is an inverted bit value of the PRESENT signals
#ifdef REV1
  uint32_t data = (~ff_combined_present) & 0x1FFFFFFU;
#elif defined(REV2)
  uint32_t data = (~ff_combined_present) & 0xFFFFFU;
#endif // REV1
  ff_USER_mask = read_eeprom_single(EEPROM_ID_FF_ADDR);
  ff_PRESENT_mask = data;
  uint64_t block = EEPROMBlockFromAddr(ADDR_FF);

  uint64_t unlock = EPRMMessage((uint64_t)EPRM_UNLOCK_BLOCK, block, PASS);
  xQueueSendToBack(xEPRMQueue_in, &unlock, portMAX_DELAY);

  uint64_t message = EPRMMessage((uint64_t)EPRM_WRITE_SINGLE, ADDR_FF, data);
  xQueueSendToBack(xEPRMQueue_in, &message, portMAX_DELAY);

  uint64_t lock = EPRMMessage((uint64_t)EPRM_LOCK_BLOCK, block << 32, 0);
  xQueueSendToBack(xEPRMQueue_in, &lock, portMAX_DELAY);

  return;
}

void readFFpresent(void)
{
  // grab the semaphore to ensure unique access to I2C controller
  // otherwise, block its operations indefinitely until it's available
  acquireI2CSemaphoreBlock(i2c4_sem);

#ifdef REV1
  // to port 7
  apollo_i2c_ctl_w(4, 0x70, 1, 0x80);
  apollo_i2c_ctl_reg_r(4, 0x20, 1, 0x01, 1, &present_FFL12_F1);
  // to port 6
  apollo_i2c_ctl_w(4, 0x71, 1, 0x40);
  apollo_i2c_ctl_reg_r(4, 0x21, 1, 0x00, 1, &present_FFLDAQ_F1);
#elif defined(REV2)
  // to port 7
  apollo_i2c_ctl_w(4, 0x70, 1, 0x80);
  apollo_i2c_ctl_reg_r(4, 0x20, 1, 0x01, 1, &present_FFL12_F1);
  // to port 6
  apollo_i2c_ctl_w(4, 0x71, 1, 0x40);
  apollo_i2c_ctl_reg_r(4, 0x21, 1, 0x00, 1, &present_FFLDAQ_F1);
  apollo_i2c_ctl_reg_r(4, 0x21, 1, 0x01, 1, &f1_ff12xmit_4v0_sel); // reading FPGA1 12-ch xmit FF's power-supply physical selection (i.e either 3.3v or 4.0v)
#endif

  // if we have a semaphore, give it
  if (xSemaphoreGetMutexHolder(i2c4_sem) == xTaskGetCurrentTaskHandle()) {
    xSemaphoreGive(i2c4_sem);
  }

  // grab the semaphore to ensure unique access to I2C controller
  // otherwise, block its operations indefinitely until it's available
  acquireI2CSemaphoreBlock(i2c3_sem);

#ifdef REV1
  // to port 0
  apollo_i2c_ctl_w(3, 0x72, 1, 0x01);
  apollo_i2c_ctl_reg_r(3, 0x20, 1, 0x01, 1, &present_0X20_F2);
  // to port 1
  apollo_i2c_ctl_w(3, 0x72, 1, 0x02);
  apollo_i2c_ctl_reg_r(3, 0x21, 1, 0x01, 1, &present_0X21_F2);
#elif defined(REV2)
  // to port 7
  apollo_i2c_ctl_w(3, 0x70, 1, 0x80);
  apollo_i2c_ctl_reg_r(3, 0x20, 1, 0x01, 1, &present_FFL12_F2);
  // to port 6
  apollo_i2c_ctl_w(3, 0x71, 1, 0x40);
  apollo_i2c_ctl_reg_r(3, 0x21, 1, 0x00, 1, &present_FFLDAQ_F2);
  apollo_i2c_ctl_reg_r(3, 0x21, 1, 0x01, 1, &f2_ff12xmit_4v0_sel); // reading FPGA2 12-ch xmit FF's power-supply physical selection (i.e either 3.3v or 4.0v)

#endif
  // if we have a semaphore, give it
  if (xSemaphoreGetMutexHolder(i2c3_sem) == xTaskGetCurrentTaskHandle()) {
    xSemaphoreGive(i2c3_sem);
  }

#ifdef REV1
  uint32_t present_FFL12_BOTTOM_F1 = present_FFL12_F1 & 0x3FU;    // bottom 6 bits
  uint32_t present_FFL12_TOP_F1 = (present_FFL12_F1 >> 6) & 0x3U; // top 2 bits
  present_FFLDAQ_F1 = (present_FFLDAQ_F1 >> 5) & 0x7U;            // bits 5-7
  present_FFL12_0X20_F2 = (present_0X20_F2 >> 6) & 0x3U;          // bit 6-7
  present_FFLDAQ_0X20_F2 = present_0X20_F2 & 0x3FU;               // bottom 6 bits
  present_FFL12_0X21_F2 = (present_0X21_F2 >> 4) & 0x3U;          // bit 4-5
  present_FFLDAQ_0X21_F2 = (present_0X21_F2 >> 2) & 0xFU;         // bit 4 bits

  uint32_t ff_combined_present = ((present_FFL12_0X21_F2) << 23) |  // 2 bits
                                 ((present_FFL12_0X20_F2) << 21) |  // 2 bits
                                 ((present_FFLDAQ_0X21_F2) << 17) | // 4 bits
                                 ((present_FFLDAQ_0X20_F2) << 11) | // 6 bits
                                 ((present_FFL12_TOP_F1) << 9) |    // 2 bits
                                 (present_FFLDAQ_F1) << 6 |         // 3 bits
                                 ((present_FFL12_BOTTOM_F1));       // 6 bits

#elif defined(REV2)
  present_FFL12_F1 = present_FFL12_F1 & 0x3FU;                     // bottom 6 bits
  present_FFL12_F2 = present_FFL12_F2 & 0x3FU;                     // bottom 6 bits
  present_FFLDAQ_F1 = (present_FFLDAQ_F1 >> 4) & 0xFU;             // bits 4-7
  present_FFLDAQ_F2 = (present_FFLDAQ_F2 >> 4) & 0xFU;             // bits 4-7

  uint32_t ff_combined_present = ((present_FFLDAQ_F2) << 16) | // 4 bits
                                 ((present_FFL12_F2) << 10) |  // 6 bits
                                 (present_FFLDAQ_F1) << 6 |    // 4 bits
                                 ((present_FFL12_F1));         // 6 bits

  ff_bitmask_args[1].present_bit_mask = (~present_FFLDAQ_F1) & 0xFU; // 4 bits
  ff_bitmask_args[0].present_bit_mask = (~present_FFL12_F1) & 0x3FU; // 6 bits
  ff_bitmask_args[3].present_bit_mask = (~present_FFLDAQ_F2) & 0xFU; // 4 bits
  ff_bitmask_args[2].present_bit_mask = (~present_FFL12_F2) & 0x3FU; // 6 bits

  f1_ff12xmit_4v0_sel = (f1_ff12xmit_4v0_sel >> 4) & 0x7; // bits 4-6
  f2_ff12xmit_4v0_sel = (f2_ff12xmit_4v0_sel >> 4) & 0x7; // bits 4-6
#endif

  setFFmask(ff_combined_present);
}

bool isEnabledFF(int ff)
{
  // firefly config stored in on-board EEPROM via user input
  // and firefly config via PRESENT signals at the first boot
  // must be true for a firefly to be enabled.
  if (!((1 << ff) & ff_PRESENT_mask) || !((1 << ff) & ff_USER_mask)) {
    return false;
  }
  else {
    return true;
  }
}

unsigned isFFStale(void)
{
  TickType_t now = pdTICKS_TO_S(xTaskGetTickCount());
  TickType_t last[4];
  last[0] = pdTICKS_TO_S(ffl12_f1_args.updateTick);
  last[1] = pdTICKS_TO_S(ffldaq_f1_args.updateTick);
  last[2] = pdTICKS_TO_S(ffl12_f2_args.updateTick);
  last[3] = pdTICKS_TO_S(ffldaq_f2_args.updateTick);

  unsigned mask = 0U;
  for (int ff_t = 0; ff_t < 4; ++ff_t) {
    if (checkStale(last[ff_t], now)) {
      mask |= (1U << ff_t);
    }
  }

  return mask; // bits set for stale tasks. no bits set == not stale.
}

// this will return the tick of the _lowest_ set bit.
TickType_t getFFupdateTick(int mask)
{
  log_debug(LOG_SERVICE, "mask = %x\r\n", mask);
  if (__builtin_popcount(mask) == 0) {
    log_warn(LOG_SERVICE, "empty mask\r\n");
  }
  if (mask & 0x1U) {
    return ffl12_f1_args.updateTick;
  }
  else if (mask & 0x02U) {
    return ffldaq_f1_args.updateTick;
  }
  else if (mask & 0x04U) {
    return ffl12_f2_args.updateTick;
  }
  else {
    return ffldaq_f2_args.updateTick;
  }
}

uint16_t getFFtemp(const uint8_t i)
{
  int i1 = 1;
  int8_t val;
  configASSERT(i < NFIREFLIES);
  if (i < NFIREFLIES_IT_F1) {
    int index = i * (ffl12_f1_args.n_commands * ffl12_f1_args.n_pages) + i1;
    val = ffl12_f1_args.sm_values[index];
  }

  else if (NFIREFLIES_IT_F1 <= i && i < NFIREFLIES_IT_F1 + NFIREFLIES_DAQ_F1) {
    int index = (i - NFIREFLIES_IT_F1) * (ffldaq_f1_args.n_commands * ffldaq_f1_args.n_pages) + i1;
    val = ffldaq_f1_args.sm_values[index];
  }

  else if (NFIREFLIES_F1 <= i && i < NFIREFLIES_F1 + NFIREFLIES_IT_F2) {
    int index = (i - NFIREFLIES_F1) * (ffl12_f2_args.n_commands * ffl12_f2_args.n_pages) + i1;
    val = ffl12_f2_args.sm_values[index];
  }
  else {
    int index = (i - NFIREFLIES_F1 - NFIREFLIES_IT_F2) * (ffldaq_f2_args.n_commands * ffldaq_f2_args.n_pages) + i1;
    val = ffldaq_f2_args.sm_values[index];
  }

  return val;
}

#ifdef REV2
uint16_t getFFavgoptpow(const uint8_t i)
{

  uint16_t avg_val = 0;
  uint16_t sum_val = 0;
  configASSERT(i < NFIREFLIES);

  for (int n = 0; n < 4; ++n) {
    if (ff_moni2c_arg[n].int_idx <= i && i < ff_moni2c_arg[n].int_idx + ff_moni2c_arg[n].num_dev) {
      for (int i1 = 4; i1 < ff_moni2c_arg[n].arg->n_commands; ++i1) {
        int dev = i - ff_moni2c_arg[n].int_idx + ff_moni2c_arg[n].dev_int_idx;
        int index = dev * (ff_moni2c_arg[n].arg->n_commands * ff_moni2c_arg[n].arg->n_pages) + i1;
        sum_val += ff_moni2c_arg[n].arg->sm_values[index];
      }
      avg_val = sum_val / (ff_moni2c_arg[n].arg->n_commands - 4);
    }
  }

  return avg_val;
}

uint16_t getFFpresentbit(const uint8_t i)
{
  if (i > 3) {
    log_warn(LOG_SERVICE, "caught %d > total fireflies %d\r\n", i, NFIREFLIES);
    return 56;
  }
  uint16_t val = ff_bitmask_args[i].present_bit_mask;

  return val;
}

void getFFpart()
{
  // Write device vendor part for identifying FF device
  uint8_t nstring = VENDOR_STOP_BIT_FF12 - VENDOR_START_BIT_FF12 + 1;
  char vendor_string[nstring];
  uint8_t data;

  SemaphoreHandle_t semaphores[2] = {i2c4_sem, i2c3_sem};
  const int ff_ndev_offset[2] = {0, NFIREFLIES_IT_F1 + NFIREFLIES_DAQ_F1};
  const uint32_t ndevices[2] = {NSUPPLIES_FFL12_F1 / 2, NSUPPLIES_FFL12_F2 / 2};
  const uint32_t dev_present_mask[2] = {present_FFL12_F1, present_FFL12_F2};
  const uint32_t dev_xmit_4v0_sel[2] = {f1_ff12xmit_4v0_sel, f2_ff12xmit_4v0_sel};

  struct MonitorI2CTaskArgs_t args_st[2] = {ffl12_f1_args, ffl12_f2_args};

  for (int f = 0; f < 2; ++f) {

    // grab the semaphore to ensure unique access to I2C controller
    // otherwise, block its operations indefinitely until it's available
    acquireI2CSemaphoreBlock(semaphores[f]);
    uint32_t tmp_ffpart_bit_mask = 0U;
    bool detect_ff = false;
    for (uint8_t n = 0; n < ndevices[f]; n++) {
      uint8_t vendor_data_rxch[4];
      int8_t vendor_part_rxch[17];

      data = 0x1U << args_st[f].devices[(2 * n) + 1].mux_bit;
      log_debug(LOG_SERVICE, "Mux set to 0x%02x\r\n", data);
      int rmux = apollo_i2c_ctl_w(args_st[f].i2c_dev, args_st[f].devices[(2 * n) + 1].mux_addr, 1, data);
      if (rmux != 0) {
        log_warn(LOG_SERVICE, "Mux write error %s\r\n", SMBUS_get_error(rmux));
      }
      for (uint8_t i = VENDOR_START_BIT_FF12; i < VENDOR_STOP_BIT_FF12; i++) {
        uint32_t vendor_char_rxch;
        int res = apollo_i2c_ctl_reg_r(args_st[f].i2c_dev, args_st[f].devices[(2 * n) + 1].dev_addr, 1, (uint16_t)i, 1, &vendor_char_rxch);
        if (res != 0) {
          log_warn(LOG_SERVICE, "GetFFpart read Error %s, break\r\n", SMBUS_get_error(res));
          vendor_part_rxch[i - VENDOR_START_BIT_FF12] = 0;
          break;
        }
        for (int j = 0; j < 4; ++j) {
          vendor_data_rxch[j] = (vendor_char_rxch >> (3 - j) * 8) & 0xFF;
        }

        tmp1.us = vendor_data_rxch[3]; // change from uint_8 to int8_t, preserving bit pattern
        vendor_part_rxch[i - VENDOR_START_BIT_FF12] = tmp1.s;
        vendor_part_rxch[i - VENDOR_START_BIT_FF12 + 1] = '\0'; // null-terminated
      }

      char *vendor_string_rxch = (char *)vendor_part_rxch;

      if ((dev_present_mask[f] & (1 << (2 * n))) == 0) { // check that there is a FF installed in this ch
        if (!detect_ff) {
          detect_ff = true;
          if (strstr(vendor_string_rxch, "14") == NULL && strstr(vendor_string_rxch, "CRRNB") == NULL) { // the first 25Gbs 12-ch detected on FPGA1(2)
            tmp_ffpart_bit_mask = tmp_ffpart_bit_mask | (0x1U << n);                                     // bit 1 for a 25Gbs ch and assign to a Bit-mask of Firefly 12-ch part
          }
          else {
            if (f == 0)
              ffl12_f1_args.commands = sm_command_fflit_f1; // if the 14Gbsp 12-ch part is found, change the set of commands to sm_command_fflit_f1
            else
              ffl12_f2_args.commands = sm_command_fflit_f2; // if the 14Gbsp 12-ch part is found, change the set of commands to sm_command_fflit_f2
          }
          log_info(LOG_SERVICE, "Getting Firefly 12-ch part (FPGA%d): %s \r\n:", f + 1, vendor_string_rxch);
          strncpy(vendor_string, vendor_string_rxch, nstring);
        }
        else {
          if (strstr(vendor_string_rxch, "14") == NULL && strstr(vendor_string_rxch, "CRRNB") == NULL) {
            tmp_ffpart_bit_mask = tmp_ffpart_bit_mask | (0x1U << n); // bit 1 for a 25Gbs ch and assign to a Bit-mask of Firefly 12-ch part
          }
          else {
            if (strncmp(vendor_string_rxch, vendor_string, nstring) != 0) {
              log_info(LOG_SERVICE, "Different Firefly 12-ch part(FPGA%d) on %s \r\n:", f + 1, ff_moni2c_addrs[(2 * n) + 1 + ff_ndev_offset[f]].name);
              log_info(LOG_SERVICE, "with %s \r\n:", vendor_string_rxch);
            }
          }
        }
      }
      else {
        log_info(LOG_SERVICE, "No Firefly 12-ch part(FPGA%d) on %s \r\n:", f + 1, ff_moni2c_addrs[(2 * n) + 1 + ff_ndev_offset[f]].name);
      }
      memset(vendor_data_rxch, 0, sizeof(vendor_data_rxch));
      memset(vendor_part_rxch, 0, sizeof(vendor_part_rxch));
      rmux = apollo_i2c_ctl_w(args_st[f].i2c_dev, args_st[f].devices[(2 * n) + 1].mux_addr, 1, 0);
      if (rmux != 0) {
        log_warn(LOG_SERVICE, "Mux write error %s\r\n", SMBUS_get_error(rmux));
      }
      log_debug(LOG_SERVICE, "%s: reset mux\r\n", args_st[f].devices[(2 * n) + 1].name);
    }

    log_debug(LOG_SERVICE, "Bit-mask of Firefly 12-ch part (FPGA%d): 0x%02x \r\n:", f + 1, tmp_ffpart_bit_mask);
    log_debug(LOG_SERVICE, "Bit-mask of xmit_3v8_sel(FPGA%d): 0x%02x \r\n:", f + 1, dev_xmit_4v0_sel[f]);
    // Warning if 25Gbs found but is connected to 3.3V or Non-25Gbs found but is connected to 3.8V
    if ((dev_xmit_4v0_sel[f] ^ tmp_ffpart_bit_mask) != 0U) {
      log_warn(LOG_SERVICE, "FPGA%d 12-ch FFs have unmatched xmit_3v8_sel(0x%02x) and 12-ch ff-mask(0x%02x) \r\n", f + 1, dev_xmit_4v0_sel[f], tmp_ffpart_bit_mask);
    }

    if (f == 0)
      ff_bitmask_args[0].ffpart_bit_mask = tmp_ffpart_bit_mask;
    else
      ff_bitmask_args[2].ffpart_bit_mask = tmp_ffpart_bit_mask;

    // if we have a semaphore, give it
    if (xSemaphoreGetMutexHolder(semaphores[f]) == xTaskGetCurrentTaskHandle()) {
      xSemaphoreGive(semaphores[f]);
    }
  }
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
#define F1_NDEVICES   4

struct dev_i2c_addr_t fpga_addrs_f2only[] = {
    {"F2_0", 0x70, 1, 0x36}, // F2 X0Y0
    {"F2_1", 0x70, 1, 0x34}, // F2 X0Y1
    {"F2_2", 0x70, 1, 0x47}, // F2 X1Y0
    {"F2_3", 0x70, 1, 0x45}, // F2 X1Y1
};
#define F2_NDEVICES   4

#endif

struct pm_command_t pm_command_fpga[] = {
    {0x8d, 2, "READ_TEMPERATURE_1", "C",
     PM_LINEAR11},
};

// only one of these might be valid
float pm_fpga[FPGA_MON_NVALUES] = {0};

struct MonitorTaskArgs_t fpga_args = {
    .name = "XIMON",
    .devices = fpga_addrs,
    .n_devices = F1F2_NDEVICES,
    .commands = pm_command_fpga,
    .n_commands =
        FPGA_MON_NCOMMANDS,
    .pm_values = pm_fpga,
    .n_values =
        FPGA_MON_NVALUES,
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
  // grab the semaphore to ensure unique access to I2C controller
  if (acquireI2CSemaphore(i2c1_sem) == pdFAIL) {
    log_warn(LOG_SERVICE, "could not get semaphore in time\r\n");
    return;
  }
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

  // if we have a semaphore, give it
  if (xSemaphoreGetMutexHolder(i2c1_sem) == xTaskGetCurrentTaskHandle()) {
    xSemaphoreGive(i2c1_sem);
  }
}

// Initialization function for the LGA80D. These settings
// need to be called when the supply output is OFF
// this is currently not ensured in this code.
void LGA80D_init(void)
{

  // grab the semaphore to ensure unique access to I2C controller
  // otherwise, block its operations indefinitely until it's available
  acquireI2CSemaphoreBlock(i2c1_sem);

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
                          pm_addrs_dcdc + dev, &extra_cmds[2], (uint8_t *)&freqlin11);
      if (r) {
        log_error(LOG_SERVICE, "LGA80D(1)\r\n");
      }
      // actual command -- vout_droop switch
      r = apollo_pmbus_rw(&g_sMaster1, &eStatus1, false,
                          pm_addrs_dcdc + dev, &extra_cmds[5],
                          (uint8_t *)&drooplin11);
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

  // if we have a semaphore, give it
  if (xSemaphoreGetMutexHolder(i2c1_sem) == xTaskGetCurrentTaskHandle()) {
    xSemaphoreGive(i2c1_sem);
  }
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
int get_f1_index(void)
{
  return fpga_f1;
}
int get_f2_index(void)
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

void initFPGAMon(void)
{
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
#ifdef REV1
    // previous FPGA has 1 SLR
    set_f1_index(1);
#else  // REV2
    // previous FPGA has 4 SLR
    set_f1_index(4); // this should not be a magic number
#endif // not REV1
#ifndef REV1
    write_gpio_pin(_F1_JTAG_BYPASS, 1);
    write_gpio_pin(_F2_JTAG_BYPASS, 1);
#endif // REV1
  }
}

#ifdef REV2
// initialize the real-time clock, which lives in the Hibernate Module in the TM4C1294NCPDT
extern uint32_t g_ui32SysClock;

void InitRTC(void)
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
}
#endif // REV2
#ifdef REV1
int init_registers_clk(void)
{
  // =====================================================
  // CMv1 Schematic 4.03 I2C CLOCK SOURCE CONTROL

  // 1a) U93 inputs vs. outputs (I2C address 0x20 on I2C channel #2)
  // All used signals are outputs [P03..P00], [P14..P10].
  // All unused signals should be inputs [P07..P04], [P17..P15].

  // grab the semaphore to ensure unique access to I2C controller
  // otherwise, block its operations indefinitely until it's available
  acquireI2CSemaphoreBlock(i2c1_sem);

  // # set I2C switch on channel 2 (U94, address 0x70) to port 6
  int status = apollo_i2c_ctl_w(2, 0x70, 1, 0x40);
  status += apollo_i2c_ctl_reg_w(2, 0x20, 1, 0x06, 1, 0xf0); // 11110000 [P07..P00]
  status += apollo_i2c_ctl_reg_w(2, 0x20, 1, 0x07, 1, 0xe0); // 11100000 [P17..P10]

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
  status += apollo_i2c_ctl_w(2, 0x70, 1, 0x40);
  status += apollo_i2c_ctl_reg_w(2, 0x20, 1, 0x02, 1, 0xf0); // 11110000 [P07..P00]
  status += apollo_i2c_ctl_reg_w(2, 0x20, 1, 0x03, 1, 0xf8); // 11111000 [P17..P10]

  // 2a) U92 inputs vs. outputs (I2C address 0x21 on I2C channel #2)
  // The signals on P00, P01, and P02 are inputs.
  // There other signals are unused and should be set as inputs.
  // There are no outputs.

  // # set I2C switch on channel 2 (U94, address 0x70) to port 7
  status += apollo_i2c_ctl_w(2, 0x70, 1, 0x80);
  status += apollo_i2c_ctl_reg_w(2, 0x21, 1, 0x06, 1, 0xff); // 11111111 [P07..P00]
  status += apollo_i2c_ctl_reg_w(2, 0x21, 1, 0x07, 1, 0xff); // 11111111 [P17..P10]

  // 2b) U92 default output values (I2C address 0x21 on I2C channel #2)
  // All signals are inputs so nothing needs to be done.

  // if we have a semaphore, give it
  if (xSemaphoreGetMutexHolder(i2c2_sem) == xTaskGetCurrentTaskHandle()) {
    xSemaphoreGive(i2c2_sem);
  }
  return status;
}
void init_registers_ff(void)
{

  // =====================================================
  // CMv1 Schematic 4.05 I2C KU15P OPTICS

  // 3a) U102 inputs vs. outputs (I2C address 0x20 on I2C channel #4)
  // All signals are inputs.

  // grab the semaphore to ensure unique access to I2C controller
  // otherwise, block its operations indefinitely until it's available
  acquireI2CSemaphoreBlock(i2c4_sem);

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

  // 7b) U6 default output values (I2C address 0x22 on I2C channel #3)
  // The outputs on P10 and P11 should default to "1".
  // This negates the active-lo "RESET" inputs on the VU7P FireFlys.
  // The outputs on P12 should default to "0" to enable the optical output.
  // The output on P13 should default to "0" until determined otherwise.
  // # set third I2C switch on channel 3 (U4, address 0x72) to port 2
  apollo_i2c_ctl_w(4, 0x72, 1, 0x04);
  apollo_i2c_ctl_reg_w(4, 0x21, 1, 0x02, 1, 0x00); // 00000000 [P07..P00]
  apollo_i2c_ctl_reg_w(4, 0x21, 1, 0x03, 1, 0x03); // 00000011 [P17..P10]

  // if we have a semaphore, give it
  if (xSemaphoreGetMutexHolder(i2c4_sem) == xTaskGetCurrentTaskHandle()) {
    xSemaphoreGive(i2c4_sem);
  }

  // grab the semaphore to ensure unique access to I2C controller
  // otherwise, block its operations indefinitely until it's available
  acquireI2CSemaphoreBlock(i2c3_sem);

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

  // if we have a semaphore, give it
  if (xSemaphoreGetMutexHolder(i2c3_sem) == xTaskGetCurrentTaskHandle()) {
    xSemaphoreGive(i2c3_sem);
  }
}
#endif // REV1
#ifdef REV2
int init_registers_clk(void)
{
  // initialize the external I2C registers for the clocks and for the optical devices.

  // =====================================================
  // CMv2 Schematic 4.03 I2C CLOCK CONTROL

  // 1a) U88 inputs vs. outputs (I2C address 0x20 on I2C channel #2)
  // The "/INT..." signals on P04 and P05 are inputs.
  // The unused signals on P06, P11, P16, and P17 should be inputs.
  // The remaining 10 signals are outputs.

  // grab the semaphore to ensure unique access to I2C controller
  // otherwise, block its operations indefinitely until it's available
  acquireI2CSemaphoreBlock(i2c2_sem);

  // # set I2C switch on channel 2 (U84, address 0x70) to port 6
  int status = apollo_i2c_ctl_w(2, 0x70, 1, 0x40);
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

  // if we have a semaphore, give it
  if (xSemaphoreGetMutexHolder(i2c2_sem) == xTaskGetCurrentTaskHandle()) {
    xSemaphoreGive(i2c2_sem);
  }
  return status;
}
void init_registers_ff(void)
{

  // =====================================================
  // CMv2 Schematic 4.05 I2C FPGA#1 OPTICS

  // 3a) U15 inputs vs. outputs (I2C address 0x20 on I2C channel #4)
  // All signals are inputs.

  // grab the semaphore to ensure unique access to I2C controller
  // otherwise, block its operations indefinitely until it's available
  acquireI2CSemaphoreBlock(i2c4_sem);

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

  // if we have a semaphore, give it
  if (xSemaphoreGetMutexHolder(i2c4_sem) == xTaskGetCurrentTaskHandle()) {
    xSemaphoreGive(i2c4_sem);
  }

  // grab the semaphore to ensure unique access to I2C controller
  // otherwise, block its operations indefinitely until it's available
  acquireI2CSemaphoreBlock(i2c3_sem);

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

  // if we have a semaphore, give it
  if (xSemaphoreGetMutexHolder(i2c3_sem) == xTaskGetCurrentTaskHandle()) {
    xSemaphoreGive(i2c3_sem);
  }
}
#endif // REV2

#ifdef REV2

#define EEPROM_MAX_PER_PAGE 126

// You must claim the semaphore at a higher level than this
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
                                          packed_reg0_address, 3, &triplet); // read triplet from eeprom
      if (status_r != 0) {
        log_error(LOG_SERVICE, "read failed: %s\r\n", SMBUS_get_error(status_r));
        return status_r;
      }
      // organize the three bytes
      uint8_t data = (triplet >> 16) & 0xFFU; // high byte of two-byte address (a page of clock config to keep track when writing data to a clock chip)
      uint8_t reg1 = (triplet >> 8) & 0xFFU;  // low byte of two-byte address
      uint8_t reg0 = triplet & 0xFFU;         // data for each address

      if (reg0 != HighByte) { // new page
        log_debug(LOG_SERVICE, "Change page to %x\r\n", reg0);
        status_w = apollo_i2c_ctl_reg_w(CLOCK_I2C_DEV, i2c_addrs, 1, 0x01, 1, reg0); // write a page change to a clock chip
        if (status_w != 0) {
          log_error(LOG_SERVICE, "write failed: %s\r\n", SMBUS_get_error(status_w));
          return status_w; // fail writing and exit
        }
        HighByte = reg0; // update the current high byte or page
      }

      status_w = apollo_i2c_ctl_reg_w(CLOCK_I2C_DEV, i2c_addrs, 1, reg1, 1, data); // write data to a clock chip
      if (status_w != 0) {
        log_error(LOG_SERVICE, "write status is %d \r\n", status_w);
        return status_w; // fail writing and exit
      }
    }
  }
  return status_w;
}

int init_load_clk(int clk_n)
{
  // this function requires semaphore give/take at a larger scope to handle its task.
  while (getPowerControlState() != POWER_ON) {
    vTaskDelay(pdMS_TO_TICKS(10)); // delay 10 ms
  }

  char *clk_ids[5] = {"r0a", "r0b", "r1a", "r1b", "r1c"};
  uint8_t i2c_addrs = CLOCK_CHIP_COMMON_I2C_ADDR; // i2c address of a clock chip
  if (clk_n == 0)
    i2c_addrs = CLOCK_CHIP_R0A_I2C_ADDR;

  int status_r = apollo_i2c_ctl_w(CLOCK_I2C_DEV, CLOCK_I2C_MUX_ADDR, 1, 1 << clk_n);
  if (status_r != 0) {
    log_error(LOG_SERVICE, "Mux error: %s\r\n", SMBUS_get_error(status_r));
    return status_r; // fail reading and exit
  }
  uint16_t init_preamble_page = 32 * (clk_n);
  uint16_t init_register_page = 32 * (clk_n) + 1;
  uint16_t init_postamble_page = 32 * (clk_n + 1) - 1;

  uint32_t PreambleList_row; // the size of preamble list in a clock config file store at the end of the last eeprom page of a clock
  status_r = apollo_i2c_ctl_reg_r(CLOCK_I2C_DEV, CLOCK_I2C_EEPROM_ADDR, 2, (init_postamble_page << 8) + 0x007C, 1, &PreambleList_row);
  if (status_r != 0) {
    log_error(LOG_SERVICE, "PreL read error: %s\r\n", SMBUS_get_error(status_r));
    return status_r; // fail reading and exit
  }

  if (PreambleList_row == 0xff) {
    log_warn(LOG_SERVICE, "Quit.. garbage EEPROM of %s PreL\r\n", clk_ids[clk_n]);
    return 1; // fail reading and exit
  }

  uint32_t RegisterList_row; // the size of register list in a clock config file store at the end of the last eeprom page of a clock
  status_r = apollo_i2c_ctl_reg_r(CLOCK_I2C_DEV, CLOCK_I2C_EEPROM_ADDR, 2, (init_postamble_page << 8) + 0x007D, 2, &RegisterList_row);
  if (status_r != 0) {
    log_error(LOG_SERVICE, "RL read error: %s\r\n", SMBUS_get_error(status_r));
    return status_r; // fail reading and exit
  }

  if (RegisterList_row == 0xffff) {
    log_warn(LOG_SERVICE, "Quit.. garbage EEPROM of %s RegL\r\n", clk_ids[clk_n]);
    return 1; // fail reading and exit
  }

  uint32_t PostambleList_row; // the size of postamble list in a clock config file store at the end of the last eeprom page of a clock
  status_r = apollo_i2c_ctl_reg_r(CLOCK_I2C_DEV, CLOCK_I2C_EEPROM_ADDR, 2, (init_postamble_page << 8) + 0x007F, 1, &PostambleList_row);
  if (status_r != 0) {
    log_error(LOG_SERVICE, "PosL read error: %s\r\n", SMBUS_get_error(status_r));
    return status_r; // fail reading and exit
  }

  if (PostambleList_row == 0xff) {
    log_warn(LOG_SERVICE, "Quit.. garbage EEPROM of %s PostL\r\n", clk_ids[clk_n]);
    return 1; // fail reading and exit
  }

  log_debug(LOG_SERVICE, "Start programming clock %s\r\n", clk_ids[clk_n]);
  log_debug(LOG_SERVICE, "Loading clock %s PreambleList from EEPROM\r\n", clk_ids[clk_n]);
  int status_w = load_clk_registers(PreambleList_row, init_preamble_page, i2c_addrs);
  if (status_w != 0) {
    log_error(LOG_SERVICE, "PreL write error %d\r\n", status_w);
    return status_w;
  }
  vTaskDelay(pdMS_TO_TICKS(330)); // 300 ms minimum delay
  log_debug(LOG_SERVICE, "Loading clock %s RegisterList from EEPROM\r\n", clk_ids[clk_n]);
  status_w = load_clk_registers(RegisterList_row, init_register_page, i2c_addrs);
  if (status_w != 0) {
    log_error(LOG_SERVICE, "RegL write error %d\r\n", status_w);
    return status_w;
  }
  vTaskDelay(pdMS_TO_TICKS(330)); // 300 ms minimum delay
  log_debug(LOG_SERVICE, "Loading clock %s PostambleList from EEPROM\r\n", clk_ids[clk_n]);
  status_w = load_clk_registers(PostambleList_row, init_postamble_page, i2c_addrs);
  if (status_w != 0) {
    log_error(LOG_SERVICE, "PosL write error %d\r\n", status_w);
    return status_w;
  }

  return status_w;
}
#endif // REV2

#ifdef REV2
// Enable or disable the 3.8V power supplies for the SamTec Fireflies
// In Rev2 we write to the I/O expander(s) (one on each I2C bus for each
// FPGA) to set these bits. In Rev2 we don't need to do a read/modify/write
// cycle because the other relevant bits are either inputs and the write does not
// affect them, or active high resets (bit0). See schematic pages 4.05 and 4.06.
// For each FPGA (F1 and F2),
// FF_1-FF3 are selectable. bit mask is 0x0e

int enable_3v8(UBaseType_t ffmask[2], bool turnOff)
{
  // i2cw 4 0x71 1 0x40
  // i2cwr 4 0x21 1 0x03 1 0x0f
  SemaphoreHandle_t semaphores[2] = {i2c4_sem, i2c3_sem};
  uint32_t i2c_device[2] = {4, 3};
  static const UBaseType_t mask = 0xeU;    // which bits in the i/o expander
  static const uint8_t muxbit = 0x1U << 6; // which output of the mux
  static const uint8_t muxaddr = 0x71;     // address of mux on i2c bus
  static const uint8_t ioexp_addr = 0x21;  // address of i/o expander on i2c bus
  static const uint8_t ioexp_reg_addr = 3; // register address in i/o expander
  // loop over 2 i2c modules
  for (int i = 0; i < 2; ++i) {
    if (ffmask[i] == 0) { // this device is not selected
      continue;
    }
    // grab the relevant semaphore
    // grab the semaphore to ensure unique access to I2C controller
    if (acquireI2CSemaphore(semaphores[i]) == pdFAIL) {
      log_warn(LOG_SERVICE, "could not get semaphore in time\r\n");
      return 5;
    }
    // mux setting
    int result = apollo_i2c_ctl_w(i2c_device[i], muxaddr, 1, muxbit);
    if (result) {
      log_warn(LOG_SERVICE, "mux err %d\r\n", result);
    }
    else {
      UBaseType_t val = ffmask[i];
      if (turnOff) {
        val = ~val; // invert bits when turning off
      }
      val &= mask; // mask out extra bits extraneously set
      val |= 0x01; // make sure active low reset bit stays deasserted (i.e., LSB is high)
      result = apollo_i2c_ctl_reg_w(i2c_device[i], ioexp_addr, 1, ioexp_reg_addr, 1, val);
      if (result) {
        log_warn(LOG_SERVICE, "expand wr %d\r\n", result);
      }
    }

    // if we have a semaphore, give it
    if (xSemaphoreGetMutexHolder(semaphores[i]) == xTaskGetCurrentTaskHandle()) {
      xSemaphoreGive(semaphores[i]);
    }
  }
  return 0;
}
#endif // REV2
