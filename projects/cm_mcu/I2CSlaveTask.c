/*
 * I2CSlaveTask.c
 *
 *  Created on: Nov 6, 2019
 *      Author: pw94 (Peter Wittich)
 */
// includes for types
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>

#include "inc/hw_memmap.h"
#include "inc/hw_ints.h" // to be removed
#include "inc/hw_i2c.h"
#include "inc/hw_types.h"
#include "driverlib/rom.h" // to be removed
#include "driverlib/i2c.h" // to be removed
#include "InterruptHandlers.h"
#include "Tasks.h"
#include "common/uart.h"
//#include "common/smbus.h"
#include "I2CSlaveTask.h"

#define CLI_BASE UART4_BASE

void Print(const char* str);



void I2CSlaveTask(void *parameters)
{
	TaskNotifyI2CSlave = xTaskGetCurrentTaskHandle();
	// struct I2CSlaveTaskArgs_t * args = parameters;

  // set up the control data structure for the I2C slave.
  //tSMBus * slave = args->smbus;

  uint16_t buffer[8] = {0xAAU};
  // loop forever
  ROM_I2CSlaveEnable(I2C0_BASE);

  enum I2C_STATE {I2C_READY, I2C_FIRSTBYTE, I2C_RECEIVE_B0, I2C_RECEIVE_B1,
    I2C_TRANSMIT_B0, I2C_TRANSMIT_B1 };

  enum I2C_STATE theState = I2C_READY;
  BaseType_t addr = 0;
  uint16_t val = 0;

  for (;;) {
    // block on notification from the I2C Slave interrupt handler
    // Wait to be notified that the transmission is complete.
    TickType_t xTicksToWait = portMAX_DELAY;
    if ( theState != I2C_READY ) xTicksToWait = pdMS_TO_TICKS(100);
    uint32_t interruptStatus;
    unsigned long ulNotificationValue = xTaskNotifyWait(0UL,0xFFFFFFFFUL,
        &interruptStatus, xTicksToWait);
    if ( ulNotificationValue == pdFALSE ) {
      theState = I2C_READY;
      addr = 0U;
      val = 0U;
      continue;
    }
    TaskNotifyI2CSlave = xTaskGetCurrentTaskHandle();
    uint32_t raw = HWREG(I2C0_BASE + I2C_O_SRIS);
    char tmp[64];
    snprintf(tmp, 64, "Raw INT is 0x%08lx\n", raw);
    Print(tmp);
    // read the I2C slave status register I2CSCSR
    uint32_t status = ROM_I2CSlaveStatus(I2C0_BASE);
    // we received an interrupt.
    // only possible values of the interrupt are the values set in the slave
    // interrupt enable
    // I2C_SLAVE_INT_DATA  **
    // I2C_SLAVE_INT_STOP
    // I2C_SLAVE_INT_START
    if ( interruptStatus & I2C_SLAVE_INT_STOP) {
      ;
    }
    else if  ( interruptStatus & I2C_SLAVE_INT_DATA )  {
      uint8_t b;
      if ( status & I2C_SLAVE_ACT_TREQ ) { // transmission request
        switch (theState) {
        case I2C_READY: // non-register transmit request
          b = buffer[0] & 0xFFU;
          ROM_I2CSlaveDataPut(I2C0_BASE, b);
          break;
        case I2C_FIRSTBYTE: // we received a byte and are now asked to transmit
          // register read
          b = buffer[addr] & 0xFFU;
          ROM_I2CSlaveDataPut(I2C0_BASE, b);
          theState = I2C_TRANSMIT_B0;
          break;
        case I2C_TRANSMIT_B0: // transmit final byte and return to READY
          b = (buffer[addr]>>8) & 0xFFU;
          ROM_I2CSlaveDataPut(I2C0_BASE, b);
          theState = I2C_READY;
          break;
        default:
          // error ? return to I2C ready state
          theState = I2C_READY;
          break;
        }
      }
      else if ( status & I2C_SLAVE_ACT_RREQ ) { // receive request
        if ( status & I2C_SLAVE_ACT_RREQ_FBR ) { // first byte
           addr = I2CSlaveDataGet(I2C0_BASE);
           theState = I2C_FIRSTBYTE;
        }
        else { // not first byte
          switch ( theState) {
          case I2C_FIRSTBYTE:
            val = I2CSlaveDataGet(I2C0_BASE);
            theState = I2C_RECEIVE_B0;
            break;
          case I2C_RECEIVE_B0:
            val |= (I2CSlaveDataGet(I2C0_BASE) << 8);
            buffer[addr] = val;
            theState = I2C_READY;
            break;
          default:
            // error ? return to I2C ready state
            theState = I2C_READY;
            break;
          }
        }
      } // receive request
    }

  }
}
