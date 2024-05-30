#ifndef MONUTILS_H
#define MONUTILS_H

#include "MonitorTaskI2C.h"

int FireflyType(int device);
int ClockType(int device);

// firefly monitoring
extern struct MonitorTaskI2CArgs_t ff_f1_args;
extern struct MonitorTaskI2CArgs_t ff_f2_args;
// clock monitoring
extern struct MonitorTaskI2CArgs_t clk_args;

extern struct dev_moni2c_addr_t clk_moni2c_addrs[NDEVICES_CLK];

#endif // MONUTILS_H
