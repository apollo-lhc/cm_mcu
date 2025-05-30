#include "MonitorTaskI2C.h"
#include "MonI2C_addresses.h"
#include "FireflyUtils.h"
#include "Tasks.h"
#include "common/log.h"
#include "MonUtils.h"

#ifdef REV1
int FireflyType(int device)
{
  switch (device) {
    case 0:
    case 1:
    case 2:
    case 3:
    case 4:
    case 5:
    case 9:
    case 10:
    case 21:
    case 22:
    case 23:
    case 24:
      // in Rev1 there is no 3.8V so we cheat and call all of these CERN-B
      return DEVICE_CERNB;
    case 6:
    case 7:
    case 8:
    case 11:
    case 12:
    case 13:
    case 14:
    case 15:
    case 16:
    case 17:
    case 18:
    case 19:
    case 20:
      return DEVICE_25G4;
    default:
      return DEVICE_NONE;
  }
}

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
#elif defined(REV2)
// for 12 channel parts, there is one Tx and Rx device.
// for 4 channel parts, there is one XCVR part
int FireflyType(int device)
{
  bool isF1 = device < NFIREFLIES_F1;
  device = device % NFIREFLIES_F1; // F1 and F2 devices are the same.
  switch (device) {
    case 0:
    case 1:
    case 2:
    case 3:
    case 4:
    case 5: {
      // this mask is set in pairs
      uint8_t mask = getFFpartbit(0);
      int thistype;
      if (!isF1) {
        mask = getFFpartbit(2);
      }
      if (mask & (0x1U << (device))) {
        thistype = DEVICE_25G12;
      }
      else {
        thistype = DEVICE_CERNB;
      }
      log_debug(LOG_MONI2C, "%s: %s device %d is type %d (25G mask %x)\r\n", __func__, isF1 ? "F1" : "F2",
                device, thistype, mask);
      return thistype;
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
#elif defined(REV3)
// for 12 channel parts, there is one Tx and Rx device.
// for 4 channel parts, there is one XCVR part
int FireflyType(int device)
{
  bool isF1 = device < NFIREFLIES_F1;
  device = device % NFIREFLIES_F1; // F1 and F2 devices are the same.
  switch (device) {
    case 0:
    case 1:
    case 2:
    case 3:
    case 4:
    case 5:
    case 6:
    case 7: {
      // //this mask is set in pairs
      uint8_t mask = getFFpartbit(0);
      int thistype;
      if (!isF1) {
        // mask = ff_bitmask_args[2].ffpart_bit_mask;
        mask = getFFpartbit(2);
      }
      if (mask & (0x1U << (device))) {
        thistype = DEVICE_25G12;
      }
      else {
        thistype = DEVICE_CERNB;
      }
      log_debug(LOG_MONI2C, "%s: %s device %d is type %d (25G mask %x)\r\n", __func__, isF1 ? "F1" : "F2",
                device, thistype, mask);
      return thistype;
    }
    case 8:
    case 9:
      return DEVICE_25G4;
    default:
      return DEVICE_NONE;
  }
}

// For rev3 all clocks are SI5395
int ClockType(int device)
{
  switch (device) {
    case 0:
    case 1:
    case 2:
    case 3:
    case 4:
      return DEVICE_SI5395;
    default:
      return DEVICE_NONE;
  }
}

#endif // REVISION

bool isEnabledFF_F2(int device)
{
  // firefly devices on F2 are devices 10-19
  return isEnabledFF(device + NFIREFLIES_F1);
}

struct MonitorTaskI2CArgs_t ff_f1_args = {
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

struct MonitorTaskI2CArgs_t ff_f2_args = {
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

#if defined(REV2) || defined(REV3)
struct MonitorTaskI2CArgs_t clk_args = {
    .name = "CLK",
    .devices = clk_moni2c_addrs,
    .i2c_dev = I2C_DEVICE_CLK,
    .n_devices = NDEVICES_CLK,
    .commands = sm_command_test_CLK,
    .n_commands = NCOMMANDS_CLK,
    .selpage_reg = CLK_SELPAGE_REG,
    .xSem = NULL,
    .stack_size = 4096U,
    .typeCallback = ClockType,
    .presentCallback = NULL,
};

#endif // REV2
