/*
 * I2CSlaveTask.c
 *
 *  Created on: Nov 6, 2019
 *      Author: pw94
 */
// includes for types
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>

#include "inc/hw_memmap.h"
#include "inc/hw_ints.h" // to be removed
#include "driverlib/rom.h" // to be removed
#include "driverlib/i2c.h" // to be removed
#include "InterruptHandlers.h"
#include "Tasks.h"
//#include "common/smbus.h"
#include "I2CSlaveTask.h"


void I2CSlaveTask(void *parameters)
{
	TaskNotifyI2CSlave = xTaskGetCurrentTaskHandle();
//  struct I2CSlaveTaskArgs_t * args = parameters;

  // set up the control data structure for the I2C slave.
  //tSMBus * slave = args->smbus;

//  uint8_t address_requested = 0;
//  uint8_t rxbuffer[2] = {0}; //const uint8_t rxbuffer_sz = 2;
//  uint8_t txbuffer[8] =  {0}; //const uint8_t txbuffer_sz = 8;
  uint8_t buffer;
  // loop forever
  ROM_I2CSlaveEnable(I2C0_BASE);

  for (;;) {

    // block on notification from the I2C Slave interrupt handler
    // Wait to be notified that the transmission is complete.
    //unsigned long ulNotificationValue =
    //ulTaskNotifyTake( pdTRUE, portMAX_DELAY );
    uint32_t interruptStatus;
    xTaskNotifyWait(0UL,0xFFFFFFFFUL, &interruptStatus, portMAX_DELAY);
    TaskNotifyI2CSlave = xTaskGetCurrentTaskHandle();

    // read the I2C slave status register I2CSCSR
    uint32_t status = I2CSlaveStatus(I2C0_BASE);
    // we received an interrupt.
    // only possible values of the interrupt are the values set in the slave
    // interrupt enable
    // I2C_SLAVE_INT_DATA, I2C_SLAVE_INT_STOP
    // (maybe I2C_SLAVE_INT_START if we add it )
    if ( interruptStatus & I2C_SLAVE_INT_STOP) {
      ;
    }
    else if  ( interruptStatus & I2C_SLAVE_INT_DATA )  {
      if ( status & I2C_SLAVE_ACT_TREQ ) {
        I2CSlaveDataPut(I2C0_BASE, buffer);
      }
      else if ( status & I2C_SLAVE_ACT_RREQ ) {
        buffer = I2CSlaveDataGet(I2C0_BASE);
      }
    }

  }
}
