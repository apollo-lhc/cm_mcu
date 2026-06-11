#ifndef SMBUS_HELPER_H
#define SMBUS_HELPER_H
#include "smbus.h"
const char *SMBUS_get_error(tSMBusStatus error);

// Check if the SMBus error is a NACK (Not Acknowledge) error
inline bool SMBUS_is_NACK(tSMBusStatus error)
{
  return (error == SMBUS_DATA_ACK_ERROR) || (error == SMBUS_ADDR_ACK_ERROR);
}

#endif // SMBUS_HELPER_H
