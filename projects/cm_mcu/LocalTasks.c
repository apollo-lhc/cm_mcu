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

#include "FireflyUtils.h"
#include "MonitorTaskI2C.h"
#include "driverlib/hibernate.h"

#include "Tasks.h"
#include "MonitorTask.h"
#include "InterruptHandlers.h"
#include "Semaphore.h"

#include "common/pinsel.h"
#include "common/smbus_units.h"
#include "common/smbus_helper.h"
#include "I2CCommunication.h"
#include "common/log.h"

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
struct dev_moni2c_addr_t ff_moni2c_addrs_f1[NFIREFLIES_F1] = {
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
};
struct dev_moni2c_addr_t ff_moni2c_addrs_f2[NFIREFLIES_F2] = {
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
struct dev_moni2c_addr_t ff_moni2c_addrs_f1[NFIREFLIES_F1] = {
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
};
struct dev_moni2c_addr_t ff_moni2c_addrs_f2[NFIREFLIES_F2] = {
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
#elif defined(REV3)
// -------------------------------------------------
//
// REV 3
//
// -------------------------------------------------
struct dev_moni2c_addr_t ff_moni2c_addrs[NFIREFLIES] = {
    {"F1_1  12 Tx", FF_I2CMUX_1_ADDR, 0, 0x50}, //
    {"F1_1  12 Rx", FF_I2CMUX_1_ADDR, 1, 0x54}, //
    {"F1_2  12 Tx", FF_I2CMUX_1_ADDR, 3, 0x50}, //
    {"F1_2  12 Rx", FF_I2CMUX_1_ADDR, 4, 0x54}, //
    {"F1_3  12 Tx", FF_I2CMUX_2_ADDR, 0, 0x50}, //
    {"F1_3  12 Rx", FF_I2CMUX_2_ADDR, 1, 0x54}, //
    {"F1_4  12 Tx", FF_I2CMUX_2_ADDR, 3, 0x50}, //
    {"F1_4  12 Rx", FF_I2CMUX_2_ADDR, 4, 0x54}, //
    {"F1_5 4 XCVR", FF_I2CMUX_1_ADDR, 2, 0x50}, //
    {"F1_6 4 XCVR", FF_I2CMUX_2_ADDR, 2, 0x50}, //
    {"F2_1  12 Tx", FF_I2CMUX_1_ADDR, 0, 0x50}, //
    {"F2_1  12 Rx", FF_I2CMUX_1_ADDR, 1, 0x54}, //
    {"F2_2  12 Tx", FF_I2CMUX_1_ADDR, 3, 0x50}, //
    {"F2_2  12 Rx", FF_I2CMUX_1_ADDR, 4, 0x54}, //
    {"F2_3  12 Tx", FF_I2CMUX_2_ADDR, 0, 0x50}, //
    {"F2_3  12 Rx", FF_I2CMUX_2_ADDR, 1, 0x54}, //
    {"F2_4  12 Tx", FF_I2CMUX_2_ADDR, 3, 0x50}, //
    {"F2_4  12 Rx", FF_I2CMUX_2_ADDR, 4, 0x54}, //
    {"F2_5 4 XCVR", FF_I2CMUX_1_ADDR, 2, 0x50}, //
    {"F2_6 4 XCVR", FF_I2CMUX_2_ADDR, 2, 0x50}, //

};
struct dev_moni2c_addr_t ff_moni2c_addrs_f1[NFIREFLIES_F1] = {
    {"F1_1  12 Tx", FF_I2CMUX_1_ADDR, 0, 0x50}, //
    {"F1_1  12 Rx", FF_I2CMUX_1_ADDR, 1, 0x54}, //
    {"F1_2  12 Tx", FF_I2CMUX_1_ADDR, 3, 0x50}, //
    {"F1_2  12 Rx", FF_I2CMUX_1_ADDR, 4, 0x54}, //
    {"F1_3  12 Tx", FF_I2CMUX_2_ADDR, 0, 0x50}, //
    {"F1_3  12 Rx", FF_I2CMUX_2_ADDR, 1, 0x54}, //
    {"F1_4  12 Tx", FF_I2CMUX_2_ADDR, 3, 0x50}, //
    {"F1_4  12 Rx", FF_I2CMUX_2_ADDR, 4, 0x54}, //
    {"F1_5 4 XCVR", FF_I2CMUX_1_ADDR, 2, 0x50}, //
    {"F1_6 4 XCVR", FF_I2CMUX_2_ADDR, 2, 0x50}, //
};
struct dev_moni2c_addr_t ff_moni2c_addrs_f2[NFIREFLIES_F2] = {
    {"F2_1  12 Tx", FF_I2CMUX_1_ADDR, 0, 0x50}, //
    {"F2_1  12 Rx", FF_I2CMUX_1_ADDR, 1, 0x54}, //
    {"F2_2  12 Tx", FF_I2CMUX_1_ADDR, 3, 0x50}, //
    {"F2_2  12 Rx", FF_I2CMUX_1_ADDR, 4, 0x54}, //
    {"F2_3  12 Tx", FF_I2CMUX_2_ADDR, 0, 0x50}, //
    {"F2_3  12 Rx", FF_I2CMUX_2_ADDR, 1, 0x54}, //
    {"F2_4  12 Tx", FF_I2CMUX_2_ADDR, 3, 0x50}, //
    {"F2_4  12 Rx", FF_I2CMUX_2_ADDR, 4, 0x54}, //
    {"F2_5 4 XCVR", FF_I2CMUX_1_ADDR, 2, 0x50}, //
    {"F2_6 4 XCVR", FF_I2CMUX_2_ADDR, 2, 0x50}, //

};

#else
#error "Define Revision!"
#endif

// FFDAQ arguments for monitoring i2c task of 4-channel firefly ports connected to FPGA1
#ifdef REV1
struct dev_moni2c_addr_t ffl4_f1_moni2c_addrs[NFIREFLIES_DAQ_F1] = {
    {"K04 4 XCVR GTY", FF_I2CMUX_2_ADDR, 0, 0x50}, //
    {"K05 4 XCVR GTY", FF_I2CMUX_2_ADDR, 1, 0x50}, //
    {"K06 4 XCVR GTY", FF_I2CMUX_2_ADDR, 2, 0x50}, //
};
#endif

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
#endif
// FFDAQV arguments for monitoring i2c task of 4-channel firefly ports connected to FPGA2
#ifdef REV1
struct dev_moni2c_addr_t ffl4_f2_moni2c_addrs[NFIREFLIES_DAQ_F2] = {
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
// #else
// #error "Define either Rev1 or Rev2"
#endif

#ifdef REV1
struct dev_moni2c_addr_t ffl12_f2_moni2c_addrs[NFIREFLIES_IT_F2] = {
    {"V11  12 Tx GTY", FF_I2CMUX_1_ADDR, 6, 0x50}, //
    {"V11  12 Rx GTY", FF_I2CMUX_1_ADDR, 7, 0x54}, //
    {"V12  12 Tx GTY", FF_I2CMUX_2_ADDR, 4, 0x50}, //
    {"V12  12 Rx GTY", FF_I2CMUX_2_ADDR, 5, 0x54}, //
};
#endif

#if defined(REV2) || defined(REV3)
// Clock arguments for monitoring task

struct clk_program_t clkprog_args[] = {
    {"", ""}, //
    {"", ""}, //
    {"", ""}, //
    {"", ""}, //
    {"", ""}, //
};

#endif

#ifdef REV2

struct dev_moni2c_addr_t clk_moni2c_addrs[NDEVICES_CLK] = {
    {"r0a", 0x70, 0, 0x77, 0x45D},  // CLK R0A : Si5341-REVD with #regs = 378 (read at 0x1F7D in EEPROM) if change, addr 0x45D will have to change
    {"r0b", 0x70, 1, 0x6b, 0x264E}, // CLK R0B : Si5395-REVA #regs = 587 (read at 0x1F7D in EEPROM) if change, addr 0x264E will have to change
    {"r1a", 0x70, 2, 0x6b, 0x464E}, // CLK R1A : Si5395-REVA #regs = 587 (read at 0x5F7D in EEPROM) if change, addr 0x464E will have to change
    {"r1b", 0x70, 3, 0x6b, 0x664E}, // CLK R1B : Si5395-REVA #regs = 584 (read at 0x7F7D in EEPROM) if change, addr 0x664E will have to change
    {"r1c", 0x70, 4, 0x6b, 0x864E}, // CLK R1C : Si5395-REVA #regs = 587 (read at 0x9F7D in EEPROM) if change, addr 0x864E will have to change
};
#elif defined(REV3)
struct dev_moni2c_addr_t clk_moni2c_addrs[NDEVICES_CLK] = {
    {"r0a", 0x70, 0, 0x6b, 0x45D},  // CLK R0A : Si5395-REVA with #regs = 378 (read at 0x1F7D in EEPROM) if change, addr 0x45D will have to change
    {"r0b", 0x70, 1, 0x6b, 0x264E}, // CLK R0B : Si5395-REVA #regs = 587 (read at 0x1F7D in EEPROM) if change, addr 0x264E will have to change
    {"r1a", 0x70, 2, 0x6b, 0x464E}, // CLK R1A : Si5395-REVA #regs = 587 (read at 0x5F7D in EEPROM) if change, addr 0x464E will have to change
    {"r1b", 0x70, 3, 0x6b, 0x664E}, // CLK R1B : Si5395-REVA #regs = 584 (read at 0x7F7D in EEPROM) if change, addr 0x664E will have to change
    {"r1c", 0x70, 4, 0x6b, 0x864E}, // CLK R1C : Si5395-REVA #regs = 587 (read at 0x9F7D in EEPROM) if change, addr 0x864E will have to change
};

#endif // REV2

#define FPGA_MON_NDEVICES_PER_FPGA  2
#define FPGA_MON_NFPGA              2
#define FPGA_MON_NDEVICES           8
#define FPGA_MON_NCOMMANDS          1
#define FPGA_MON_NVALUES_PER_DEVICE 1
#define FPGA_MON_NVALUES            (FPGA_MON_NCOMMANDS * FPGA_MON_NDEVICES * FPGA_MON_NVALUES_PER_DEVICE)

#define LOG_FACILITY LOG_SERVICE

// FPGA arguments for monitoring task
// Note: the I2C address for SL0 is 0x32 and the PMBUS address for SL0 is 0x36.
// See UG580 table 3-20 and nearby (v1.10.1)
// Other SLRs are only accessible if explicitly programmed.
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

#elif defined(REV2) || defined(REV3)
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
#elif defined(REV2) || defined(REV3)
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
#elif defined(REV2) || defined(REV3)
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
#error "Define Revision!"
#endif // REV

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
    log_error(LOG_SERVICE, "page w fail, dev 0x%x (%s)\r\n", add->dev_addr, add->name);
    return;
  }

  // actual command -- snapshot control copy NVRAM for reading
  uint8_t cmd = 0x1;
  r = apollo_pmbus_rw(&g_sMaster1, &eStatus1, false, add, &extra_cmds[4], &cmd);
  if (r) {
    log_error(LOG_SERVICE, "ctrl w fail, dev 0x%x (%s)\r\n", add->dev_addr, add->name);
    return;
  }
  // actual command -- read snapshot
  tSMBusStatus r2 =
      SMBusMasterBlockRead(&g_sMaster1, add->dev_addr, extra_cmds[3].command, &snapshot[0]);
  if (r2 != SMBUS_OK) {
    log_error(LOG_SERVICE, "block %d\r\n", r2);
    return;
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
      log_error(LOG_SERVICE, "error reset %s\r\n", add->name);
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

#if defined(REV2) || defined(REV3)
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
#if defined(REV2) || defined(REV3)
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

  status += apollo_i2c_ctl_w(2, 0x70, 1, 0x0); // reset the mux

  // if we have a semaphore, give it
  if (xSemaphoreGetMutexHolder(i2c2_sem) == xTaskGetCurrentTaskHandle()) {
    xSemaphoreGive(i2c2_sem);
  }
  return status;
}
#endif // REV2 || REV3

#ifdef REV2
void init_registers_ff(void)
{
  log_info(LOG_SERVICE, "%s\r\n", __func__);
  int result;
  // =====================================================
  // CMv2 Schematic 4.05 I2C FPGA#1 OPTICS

  // 3a) U15 inputs vs. outputs (I2C address 0x20 on I2C channel #4)
  // All signals are inputs.

  // grab the semaphore to ensure unique access to I2C controller
  // otherwise, block its operations indefinitely until it's available
  acquireI2CSemaphoreBlock(i2c4_sem);

  // # set first I2C switch on channel 4 (U14, address 0x70) to port 7
  result = apollo_i2c_ctl_w(4, 0x70, 1, 0x80);
  result += apollo_i2c_ctl_reg_w(4, 0x20, 1, 0x06, 1, 0xff); //  11111111 [P07..P00]
  result += apollo_i2c_ctl_reg_w(4, 0x20, 1, 0x07, 1, 0xff); //  11111111 [P17..P10]

  // clear first I2C switch on channel 4
  result += apollo_i2c_ctl_w(4, 0x70, 1, 0x0);

  // 3b) U15 default output values (I2C address 0x20 on I2C channel #4)
  // All signals are inputs so nothing needs to be done.

  // 4a) U18 inputs vs. outputs (I2C address 0x21 on I2C channel #4)
  // The "/F1_FF_RESET" signal on P10 is an output
  // The "EN_...3V8" signals on P11, P12, and P13 are outputs.
  // All other signals are inputs

  // # set second I2C switch on channel 4 (U17, address 0x71) to port 6
  result += apollo_i2c_ctl_w(4, 0x71, 1, 0x40);
  result += apollo_i2c_ctl_reg_w(4, 0x21, 1, 0x06, 1, 0xff); //  11111111 [P07..P00]
  result += apollo_i2c_ctl_reg_w(4, 0x21, 1, 0x07, 1, 0xf0); //  11110000 [P17..P10]

  // 4b) U18 default output values (I2C address 0x21 on I2C channel #4)
  // The output on P10 should default to "1".
  // This negates the active-lo "RESET" input on the FPGA#1 FireFlys
  // The outputs on P11, P12, and P13 should default to "0"
  // This disables the 3.8 volt power supplies on the three FireFly
  // 12-lane transmitter sites for FPGA#1.

  // # set second I2C switch on channel 4 (U17, address 0x71) to port 6
  result += apollo_i2c_ctl_w(4, 0x71, 1, 0x40);
  result += apollo_i2c_ctl_reg_w(4, 0x21, 1, 0x02, 1, 0x00); //  00000000 [P07..P00]
  result += apollo_i2c_ctl_reg_w(4, 0x21, 1, 0x03, 1, 0x01); //  00000001 [P17..P10]

  // clear 2nd I2C switch on channel 4
  result += apollo_i2c_ctl_w(4, 0x70, 1, 0x0);
  if (result) {
    log_error(LOG_SERVICE, "\tFailed to initialize FPGA#1 optics\r\n");
  }

  // if we have a semaphore, give it
  if (xSemaphoreGetMutexHolder(i2c4_sem) == xTaskGetCurrentTaskHandle()) {
    xSemaphoreGive(i2c4_sem);
  }

  // grab the semaphore to ensure unique access to I2C controller
  // otherwise, block its operations indefinitely until it's available
  acquireI2CSemaphoreBlock(i2c3_sem);
  result = 0;
  // =====================================================
  // CMv2 Schematic 4.06 I2C FPGA#2 OPTICS

  // 5a) U10 inputs vs. outputs (I2C address 0x20 on I2C channel #3)
  // All signals are inputs.

  // # set first I2C switch on channel 3 (U9, address 0x70) to port 7
  result += apollo_i2c_ctl_w(3, 0x70, 1, 0x80);
  result += apollo_i2c_ctl_reg_w(3, 0x20, 1, 0x06, 1, 0xff); //  11111111 [P07..P00]
  result += apollo_i2c_ctl_reg_w(3, 0x20, 1, 0x07, 1, 0xff); //  11111111 [P17..P10]

  // clear first I2C switch on channel 3
  result += apollo_i2c_ctl_w(3, 0x70, 1, 0x0);

  // 5b) U10 default output values (I2C address 0x20 on I2C channel #3)
  // All signals are inputs so nothing needs to be done.

  // 6a) U12 inputs vs. outputs (I2C address 0x21 on I2C channel #3)
  // The "/F2_FF_RESET" signal on P10 is an output
  // The "EN_...3V8" signals on P11, P12, and P13 are outputs.
  // All other signals are inputs

  // # set second I2C switch on channel 3 (U11, address 0x71) to port 6
  result += apollo_i2c_ctl_w(3, 0x71, 1, 0x40);
  result += apollo_i2c_ctl_reg_w(3, 0x21, 1, 0x06, 1, 0xff); //  11111111 [P07..P00]
  result += apollo_i2c_ctl_reg_w(3, 0x21, 1, 0x07, 1, 0xf0); //  11110000 [P17..P10]

  // 6b) U12 default output values (I2C address 0x21 on I2C channel #3)
  // The output on P10 should default to "1".
  // This negates the active-lo "RESET" input on the FPGA#2 FireFlys
  // The outputs on P11, P12, and P13 should default to "0"
  // This disables the 3.8 volt power supplies on the three FireFly
  // 12-lane transmitter sites for FPGA#2.

  // # set second I2C switch on channel 3 (U11, address 0x71) to port 6
  result += apollo_i2c_ctl_w(3, 0x71, 1, 0x40);
  result += apollo_i2c_ctl_reg_w(3, 0x21, 1, 0x02, 1, 0x00); //  00000000 [P07..P00]
  result += apollo_i2c_ctl_reg_w(3, 0x21, 1, 0x03, 1, 0x01); //  00000001 [P17..P10]

  // clear 2nd I2C switch on channel 3
  result += apollo_i2c_ctl_w(3, 0x71, 1, 0x0);

  if (result) {
    log_error(LOG_SERVICE, "\tFailed to initialize FPGA#2 optics\r\n");
  }

  // if we have a semaphore, give it
  if (xSemaphoreGetMutexHolder(i2c3_sem) == xTaskGetCurrentTaskHandle()) {
    xSemaphoreGive(i2c3_sem);
  }
}
#elif defined(REV3)
void init_registers_ff(void)
{
  log_info(LOG_SERVICE, "%s\r\n", __func__);
  int result;
  // =====================================================
  // CMv3 Schematic 4.05 I2C FPGA#1 OPTICS

  // 3a) U15 inputs vs. outputs (I2C address 0x20 on I2C channel #4)
  // All signals are inputs.

  // grab the semaphore to ensure unique access to I2C controller
  // otherwise, block its operations indefinitely until it's available
  acquireI2CSemaphoreBlock(i2c4_sem);

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
  result += apollo_i2c_ctl_reg_w(4, 0x21, 1, 0x03, 1, 0x00); //  00000001 [P17..P10]

  // clear 2nd I2C switch on channel 4
  result += apollo_i2c_ctl_w(4, 0x70, 1, 0x0);
  if (result) {
    log_error(LOG_SERVICE, "\tFailed to initialize FPGA#1 optics\r\n");
  }

  // if we have a semaphore, give it
  if (xSemaphoreGetMutexHolder(i2c4_sem) == xTaskGetCurrentTaskHandle()) {
    xSemaphoreGive(i2c4_sem);
  }

  // grab the semaphore to ensure unique access to I2C controller
  // otherwise, block its operations indefinitely until it's available
  acquireI2CSemaphoreBlock(i2c3_sem);
  result = 0;
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
  result += apollo_i2c_ctl_reg_w(3, 0x21, 1, 0x06, 1, 0x7f); //  11111111 [P07..P00]
  result += apollo_i2c_ctl_reg_w(3, 0x21, 1, 0x07, 1, 0xf0); //  11110000 [P17..P10]

  // 6b) U12 default output values (I2C address 0x21 on I2C channel #3)
  // The output on P07 should default to "1".
  // This negates the active-lo "RESET" input on the FPGA#2 FireFlys
  // The outputs on P10, P11, P12, and P13 should default to "0"
  // This disables the 3.8 volt power supplies on the three FireFly
  // 12-lane transmitter sites for FPGA#2.

  // # set second I2C switch on channel 3 (U11, address 0x71) to port 6
  result += apollo_i2c_ctl_w(3, 0x71, 1, 0x40);
  result += apollo_i2c_ctl_reg_w(3, 0x21, 1, 0x02, 1, 0x80); //  00000000 [P07..P00]
  result += apollo_i2c_ctl_reg_w(3, 0x21, 1, 0x03, 1, 0x00); //  00000001 [P17..P10]

  // clear 2nd I2C switch on channel 3
  result += apollo_i2c_ctl_w(3, 0x71, 1, 0x0);

  if (result) {
    log_error(LOG_SERVICE, "\tFailed to initialize FPGA#2 optics\r\n");
  }

  // if we have a semaphore, give it
  if (xSemaphoreGetMutexHolder(i2c3_sem) == xTaskGetCurrentTaskHandle()) {
    xSemaphoreGive(i2c3_sem);
  }
}

#endif // REV2

#if defined(REV2) || defined(REV3)

#define EEPROM_MAX_PER_PAGE 126

// You must claim the semaphore at a higher level than this
static int load_clk_registers(uint32_t reg_count, uint16_t reg_page, uint16_t i2c_addrs)
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
#ifdef REV2                                       // only Rev2 has a different i2c address for R0A
  if (clk_n == 0) {
    i2c_addrs = CLOCK_CHIP_R0A_I2C_ADDR;
  }
#endif // REV2
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

// Load all clocks from EEPROM. Should be run on every transition to the POWER_ON
// state in the power control state machine.
// This function will load all clocks from EEPROM, clear sticky bits, and print out
// the clock program names. Use semaphores to ensure unique access to the I2C controller.
int load_all_clocks(void)
{
  log_info(LOG_SERVICE, "Start CLK config\r\n");
  // grab the semaphore to ensure unique access to I2C controller
  // otherwise, block its operations indefinitely until it's available
  acquireI2CSemaphoreBlock(i2c2_sem);
  int status = 0;
  for (int i = 0; i < CLOCK_NUM_CLOCKS; ++i) {
    status += init_load_clk(i); // load each clock config from EEPROM
    // get and print out the file name
    vTaskDelay(pdMS_TO_TICKS(500));
    getClockProgram(i, clkprog_args[i].progname_clkdesgid, clkprog_args[i].progname_eeprom);
    log_info(LOG_SERVICE, "CLK%d: %s\r\n", i, clkprog_args[i].progname_clkdesgid);
  }
  status += clear_clk_stickybits();
  if (status != 0) {
    log_info(LOG_SERVICE, "Clear clock sticky bits failed\r\n");
  }
  log_info(LOG_SERVICE, "Clocks configured\r\n");
  // check if we have the semaphore
  if (xSemaphoreGetMutexHolder(i2c2_sem) == xTaskGetCurrentTaskHandle()) {
    xSemaphoreGive(i2c2_sem);
  }

  return status;
}

#endif // REV2

#if defined(REV2) || defined(REV3)
// Enable or disable the 3.8V power supplies for the SamTec Fireflies
// In Rev2 we write to the I/O expander(s) (one on each I2C bus for each
// FPGA) to set these bits. In Rev2 we don't need to do a read/modify/write
// cycle because the other relevant bits are either inputs and the write does not
// affect them, or active high resets (bit0). See schematic pages 4.05 and 4.06.
// Rev3 moves the reset bit elsewhere.
#define EN3V8_MUXBIT         (0x1U << 6)
#define EN3V8_MUXADDR        0x71 // mux addresses on I2C Bus
#define EN3V8_IOEXP_ADDR     0x21 // device address on I2C Bus
#define EN3V8_IOEXP_REG_ADDR 0x3  // register address in TCA9555 I/O expander
#ifdef REV2
#define EN3V8_IOEXP_REG_MASK 0xEU // which bits in the i/o expander register
#elif defined(REV3)
#define EN3V8_IOEXP_REG_MASK 0xFU // which bits in the i/o expander register
#endif

int enable_3v8(UBaseType_t ffmask[2], bool turnOff)
{
  // i2cw 4 0x71 1 0x40
  // i2cwr 4 0x21 1 0x03 1 0x0f
  SemaphoreHandle_t semaphores[2] = {i2c4_sem, i2c3_sem};
  uint32_t i2c_device[2] = {4, 3};
  int result = 0;
  // dump input ffmask
  log_debug(LOG_SERVICE, "ffmask[0] 0x%x, ffmask[1] 0x%x\r\n", ffmask[0], ffmask[1]);
  // loop over 2 i2c modules
  for (int i = 0; i < 2; ++i) {
    if (ffmask[i] == 0) { // this device is not selected
      continue;
    }
    // grab the relevant semaphore
    // grab the semaphore to ensure unique access to I2C controller
    if (acquireI2CSemaphore(semaphores[i]) == pdFAIL) {
      log_warn(LOG_SERVICE, "could not get semaphore in time\r\n");
      return SEM_ACCESS_ERROR;
    }
    // mux setting
    result += apollo_i2c_ctl_w(i2c_device[i], EN3V8_MUXADDR, 1, EN3V8_MUXBIT);
    if (result) {
      log_warn(LOG_SERVICE, "mux err %d\r\n", result);
    }
    else {
      UBaseType_t val = ffmask[i];
      if (turnOff) {
        val = ~val; // invert bits when turning off
      }
#ifdef REV2
      val = (val << 1) & EN3V8_IOEXP_REG_MASK; // set bits 1-3, and mask out extra bits extraneously set
      val |= 0x01;                             // make sure active low reset bit stays deasserted (i.e., LSB is high)
#elif defined(REV3)
      val &= EN3V8_IOEXP_REG_MASK; // bottom 4 bits only, no reset here anymore
#endif // REV2
      result += apollo_i2c_ctl_reg_w(i2c_device[i], EN3V8_IOEXP_ADDR, 1, EN3V8_IOEXP_REG_ADDR, 1, val);
      if (result) {
        log_warn(LOG_SERVICE, "expand wr %d\r\n", result);
      }
    }

    // clear the mux
    result += apollo_i2c_ctl_w(i2c_device[i], EN3V8_MUXADDR, 1, 0);

    // if we have a semaphore, give it
    if (xSemaphoreGetMutexHolder(semaphores[i]) == xTaskGetCurrentTaskHandle()) {
      xSemaphoreGive(semaphores[i]);
    }
  }
  return result;
}
#endif // REV2

// return board information stored in on-board EEPROM
void get_board_info(uint32_t *rev, uint32_t *id)
{
  // read the board info from the EEPROM
  uint32_t sn = read_eeprom_single(EEPROM_ID_SN_ADDR);
  *id = sn >> 16;
  *rev = sn & 0xff;
}
