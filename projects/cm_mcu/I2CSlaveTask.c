/*
 * I2CSlaveTask.c
 *
 *  Created on: Nov 6, 2019
 *      Author: pw94
 */
#include "inc/hw_memmap.h"
#include "InterruptHandlers.h"
#include "Tasks.h"
#include "common/smbus.h"

struct I2CSlaveTaskArgs_t {
  tSMBus *s;
};

void I2CSlaveTask(void *parameters)
{
  struct I2CSlaveTaskArgs_t * args = parameters;

  // set up the control data structure for the I2C slave.
  tSMBus * slave = args->s;
  SMBusSlaveInit(slave, I2C0_BASE);
  SMBusSlaveAddressSet(slave, 0, 0x40); // set my i2c address
  SMBusSlaveI2CEnable(slave); // allow raw I2C commands
  SMBusSlaveIntEnable(slave); // enable my interrupts

  // startup code
  SMBusSlaveTransferInit(slave); // this is called per transfer
  // start
  //const TickType_t xMaxBlockTime = pdMS_TO_TICKS( 20000 );
  uint8_t address_requested = 0;
  uint8_t rxbuffer[2]; const uint8_t rxbuffer_sz = 2;
  uint8_t txbuffer[8]; const uint8_t txbuffer_sz = 8;
  // loop forever
  for (;;) {
    // block on notification from the I2C Slave interrupt handler
    // Wait to be notified that the transmission is complete.
    //unsigned long ulNotificationValue =
    ulTaskNotifyTake( pdTRUE, 0 );

    // we received an interrupt.
    tSMBusStatus retval = SMBusSlaveIntProcess(slave);
    switch (retval) {
    case SMBUS_SLAVE_FIRST_BYTE:
      address_requested = slave->pui8RxBuffer[0];
      break;
    case SMBUS_DATA_SIZE_ERROR:
      break;
    case SMBUS_SLAVE_ERROR:
      break;
    case SMBUS_TRANSFER_COMPLETE:
      break;
    case SMBUS_SLAVE_QCMD_0:
    case SMBUS_SLAVE_QCMD_1:
      break;

    case SMBUS_SLAVE_NOT_READY:
      // set up the transmission
      // ...
      if ( address_requested == 0) {
        txbuffer[0] = 0x55;
        txbuffer[1] = 0xaa;
      }
      else if ( address_requested == 1 ) {
        txbuffer[0] = 0xa5;
        txbuffer[1] = 0xee;
      }
      SMBusSlaveTxBufferSet(slave,txbuffer,txbuffer_sz);
        // Send the data
      SMBusSlaveDataSend(slave);
      break;
    case SMBUS_OK: // successful finish of processing
      // reset
      SMBusSlaveRxBufferSet(slave,rxbuffer,rxbuffer_sz);
      SMBusSlaveTransferInit(slave);
      break;
    default:
      break;
    }

  }
}
