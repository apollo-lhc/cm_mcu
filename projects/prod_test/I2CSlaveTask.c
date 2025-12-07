/*
 * I2CSlaveTask.c
 *
 *  Created on: Nov 6, 2019
 *      Author: pw94 (Peter Wittich)
 */
// includes for types
#include <stdint.h>
#include <stdbool.h>

#include "inc/hw_memmap.h"
#include "driverlib/rom.h" // to be removed
#include "driverlib/i2c.h" // to be removed
#include "InterruptHandlers.h"
#include "common/log.h"


#if defined(REV1) || defined(REV2) || defined(REV3) // why is this here
#define SLAVE_I2C_BASE I2C0_BASE
#endif

static uint8_t testreg = 0x0U;



static const char * const msg = "This is a long test message from the CM-MCU I2C slave.\r\n";
const int msg_len = 66;

static uint8_t getSlaveData(uint8_t address)
{
  uint8_t value = 0x00U;
  if ( address == 0x0U) { // test register
    value = testreg;
  }
  else if ( address > 0 && address < msg_len ) {
    value = msg[address - 1];
  }
  else {
    log_warn(LOG_I2C, "Invalid register read at address 0x%02x\r\n", address);
  }
  return value;
}

int nextCommand = 0;
int page = 0;
// only the test register is r/w; everything else is
// silently ignored.
static void setSlaveData(uint8_t addr, uint8_t val)
{
  if ( addr == 0x0U) { // test register
    testreg = val;
  }
}

void I2CSlaveTask(void *parameters)
{
  TaskNotifyI2CSlave = xTaskGetCurrentTaskHandle();


  ROM_I2CSlaveEnable(SLAVE_I2C_BASE);

  enum I2C_STATE {
    I2C_READY,
    I2C_FIRSTBYTE,
    I2C_RECEIVE_B0,
    I2C_RECEIVE_B1,
    I2C_TRANSMIT_B0,
    I2C_TRANSMIT_B1
  };

  enum I2C_STATE theState = I2C_READY;
  uint8_t addr = 0;
  uint16_t val = 0;

  // loop forever
  for (;;) {
    log_trace(LOG_I2C, "state is %d\r\n", (int)theState);
    // block on notification from the I2C Slave interrupt handler
    // Wait to be notified that the transmission is complete.
    // we wait indefinitely between transactions, but time out after some time
    // during transactions.
    TickType_t xTicksToWait = portMAX_DELAY;
    if (theState != I2C_READY)
      xTicksToWait = pdMS_TO_TICKS(100);
    uint32_t interruptStatus;
    unsigned long ulNotificationValue =
        xTaskNotifyWait(0UL, 0xFFFFFFFFUL, &interruptStatus, xTicksToWait);
    if (ulNotificationValue == pdFALSE) {
      theState = I2C_READY;
      addr = 0U;
      val = 0U;
      continue;
    }
    TaskNotifyI2CSlave = xTaskGetCurrentTaskHandle();

    log_trace(LOG_I2C, "INT is 0x%08x\r\n", interruptStatus);
    // read the I2C slave status register I2CSCSR
    uint32_t status = ROM_I2CSlaveStatus(SLAVE_I2C_BASE);
    log_trace(LOG_I2C, "Slave status is 0x%08x\r\n", status);

    // we received an interrupt.
    // only possible values of the interrupt are the values set in the slave
    // interrupt enable
    // I2C_SLAVE_INT_DATA  **
    // I2C_SLAVE_INT_STOP
    // I2C_SLAVE_INT_START
    if (interruptStatus & I2C_SLAVE_INT_STOP) {
      log_trace(LOG_I2C, "Stop interrupt received");
    }
    else if (interruptStatus & I2C_SLAVE_INT_DATA) {
      if (status == I2C_SLAVE_ACT_TREQ) { // transmission request
        switch (theState) {
          case I2C_READY: // non-register transmit request
            // we ignore these, but to keep the TM4C happy we put something
            // on the bus
            ROM_I2CSlaveDataPut(SLAVE_I2C_BASE, 0xFF);
            theState = I2C_READY;
            break;
          case I2C_FIRSTBYTE: // we received a byte and are now asked to transmit
          {
            // register read
            uint8_t b = getSlaveData(addr);
            ROM_I2CSlaveDataPut(SLAVE_I2C_BASE, b);
            log_trace(LOG_I2C, "byte 1 sent %x\r\n", b);
            theState = I2C_READY;
            break;
          }
          default:
            // error ? return to I2C ready state
            theState = I2C_READY;
            break;
        }
      }
      // below here, receive requests
      else if (status == I2C_SLAVE_ACT_RREQ_FBR) { // first byte
        addr = ROM_I2CSlaveDataGet(SLAVE_I2C_BASE);
        log_debug(LOG_I2C, "Address %d received\r\n", addr);
        theState = I2C_FIRSTBYTE;
      }
      else if (status == I2C_SLAVE_ACT_RREQ) { // not first byte
        switch (theState) {
          case I2C_READY: // we ignore this, but need to keep TM4C happy
            val = ROM_I2CSlaveDataGet(SLAVE_I2C_BASE);
            // stay in READY state
            theState = I2C_READY;
            break;
          case I2C_FIRSTBYTE:
            val = ROM_I2CSlaveDataGet(SLAVE_I2C_BASE);
            setSlaveData(addr, val);
            theState = I2C_READY;
            log_trace(LOG_I2C, "wrote byte 1\r\n");
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
