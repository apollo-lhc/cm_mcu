#include "smbus_helper.h"

const char *SMBUS_get_error(tSMBusStatus error)
{
  switch (error) {
    case SMBUS_OK:
      return "OK";
      break;
    case SMBUS_TIMEOUT:
      return "TIMEOUT";
      break;
    case SMBUS_PERIPHERAL_BUSY:
      return "PERIPHERAL_BUSY";
      break;
    case SMBUS_BUS_BUSY:
      return "BUS_BUSY";
      break;
    case SMBUS_ARB_LOST:
      return "ARB_LOST";
      break;
    case SMBUS_ADDR_ACK_ERROR:
      return "ADDR_ACK_ERROR";
      break;
    case SMBUS_DATA_ACK_ERROR:
      return "DATA_ACK_ERROR";
      break;
    case SMBUS_PEC_ERROR:
      return "PEC_ERROR";
      break;
    case SMBUS_DATA_SIZE_ERROR:
      return "DATA_SIZE_ERROR";
      break;
    case SMBUS_MASTER_ERROR:
      return "MASTER_ERROR";
      break;
    case SMBUS_SLAVE_ERROR:
      return "SLAVE_ERROR";
      break;
    case SMBUS_SLAVE_QCMD_0:
      return "SLAVE_QCMD_0";
      break;
    case SMBUS_SLAVE_FIRST_BYTE:
      return "SLAVE_FIRST_BYTE";
      break;
    case SMBUS_SLAVE_ADDR_PRIMARY:
      return "SLAVE_ADDR_PRIMARY";
      break;
    case SMBUS_SLAVE_ADDR_SECONDARY:
      return "SLAVE_ADDR_SECONDARY";
      break;
    case SMBUS_TRANSFER_IN_PROGRESS:
      return "TRANSFER_IN_PROGRESS";
      break;
    case SMBUS_TRANSFER_COMPLETE:
      return "TRANSFER_COMPLETE";
      break;
    case SMBUS_SLAVE_NOT_READY:
      return "SLAVE_NOT_READY";
      break;
    case SMBUS_FIFO_ERROR:
      return "FIFO_ERROR";
      break;
    default:
      return "UNKNOWN";
      break;
  }
}
