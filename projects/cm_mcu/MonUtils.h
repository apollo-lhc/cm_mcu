#ifndef MONUTILS_H
#define MONUTILS_H

#include "MonitorTaskI2C_new.h"

int FireflyType(int device);

// firefly monitoring
extern struct MonitorI2CTaskArgs_new_t ff_f1_args;
extern struct MonitorI2CTaskArgs_new_t ff_f2_args;
// clock monitoring
extern struct MonitorI2CTaskArgs_new_t clk_args;
#endif // MONUTILS_H
