#ifndef MONUTILS_H
#define MONUTILS_H

#include "MonitorTaskI2C_new.h"

int FireflyType(int device);
int ClockType(int device);

// firefly monitoring
extern struct MonitorI2CTaskArgs_new_t ff_f1_args;
extern struct MonitorI2CTaskArgs_new_t ff_f2_args;
// clock monitoring
extern struct MonitorI2CTaskArgs_new_t clk_args;

extern struct dev_moni2c_addr_t clk_moni2c_addrs[NDEVICES_CLK];

#endif // MONUTILS_H
