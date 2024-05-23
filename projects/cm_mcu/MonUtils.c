#include "MonitorI2CTask.h"
#include "MonitorTaskI2C_new.h"
#include "MonI2C_addresses.h"
#include "FireflyUtils.h"
#include "MonUtils.h"

// for 12 channel parts, there is one Tx and Rx device.
// for 4 channel parts, there is one XCVR part
int FireflyType(int device)
{
  device = device %NFIREFLIES_F1; // F1 and F2 devices are the same.
  switch (device) {
    case 0:
    case 1:
    case 2:
    case 3:
    case 4:
    case 5: {
      // FIXME: this should be a check on CERN-B or ECUO-[RT]12-25
      // with a function call
      return DEVICE_CERNB;
    }
    case 6:
    case 7:
    case 8:
    case 9:
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

bool isEnabledFF_F2(int device)
{
  // firefly devices on F2 are devices 10-19
  return isEnabledFF(device+NFIREFLIES_F1);
}

struct MonitorI2CTaskArgs_new_t ff_f1_args = {
    .name = "FF_F1",
    .devices = ff_moni2c_addrs_f1,
    .i2c_dev = I2C_DEVICE_F1,
    .n_devices = NFIREFLIES_F1,
    .commands = sm_command_test_FF_F1,
    .n_commands = NCOMMANDS_FF_F1,
    .selpage_reg = FF_SELPAGE_REG,
    .xSem = NULL,
    .stack_size = 4096U,
    .typeCallback = FireflyType,
    .presentCallback = isEnabledFF,
};

struct MonitorI2CTaskArgs_new_t ff_f2_args = {
    .name = "FF_F2",
    .devices = ff_moni2c_addrs_f2,
    .i2c_dev = I2C_DEVICE_F2,
    .n_devices = NFIREFLIES_F2,
    .commands = sm_command_test_FF_F2,
    .n_commands = NCOMMANDS_FF_F2,
    .selpage_reg = FF_SELPAGE_REG,
    .xSem = NULL,
    .stack_size = 4096U,
    .typeCallback = FireflyType,
    .presentCallback = isEnabledFF_F2,
};

#endif // REV2
