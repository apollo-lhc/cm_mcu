#include "MonitorTaskI2C_new.h"
#include "MonI2C_addresses.h"
#include "FireflyUtils.h"

int FireflyType(int device)
{
  switch (device) {
    case 0:
    case 1:
    case 2:
    case 3: {
      // fixme: this should be a check on CERN-B or ECUO-[RT]12-25
      // with a function call
      return DEVICE_25G12;
    }
    case 4:
    case 5:
    case 6:
    case 7:
      return DEVICE_25G4;
    default:
      return DEVICE_NONE;
  }
}

#ifdef REV2
// For rev2 clocks there is one 5341 and 4 5395s
int ClockType(int device)
{
  switch (device) {
    case 0:
      return DEVICE_SI5341;
    case 1:
    case 2:
    case 3:
    case 4:
      return DEVICE_SI5395;
    default:
      return DEVICE_NONE;
  }
}

struct MonitorI2CTaskArgs_new_t ff_f1_args = {
    .name = "FF_F1",
    .devices = ffl12_f2_moni2c_addrs,
    .i2c_dev = I2C_DEVICE_F1,
    .n_devices = NDEVICES_FFL4_F1, // FIXME: NDEVICES_FF_F1
    .commands = sm_command_test_FF,
    .n_commands = NCOMMANDS_FF,
    .selpage_reg = FF_SELPAGE_REG,
    .xSem = NULL,
    .stack_size = 4096U,
    .typeCallback = FireflyType,
    .presentCallback = isEnabledFF,
};

struct MonitorI2CTaskArgs_new_t ff_f2_args = {
    .name = "FF_F2",
    .devices = ffl12_f2_moni2c_addrs,
    .i2c_dev = I2C_DEVICE_F2,
    .n_devices = NDEVICES_FFL12_F2, // FIXME: NDEVICES_FF_F2
    .commands = sm_command_test_FF,
    .n_commands = NCOMMANDS_FF,
    .selpage_reg = FF_SELPAGE_REG,
    .xSem = NULL,
    .stack_size = 4096U,
    .typeCallback = FireflyType,
    .presentCallback = isEnabledFF,
};

#endif // REV2
