#include "MonitorTaskI2C_new.h"

int FirefflyType(int device)
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
#endif // REV2
