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
#include "inc/hw_ints.h" // to be removed
#include "inc/hw_i2c.h"
#include "inc/hw_types.h"
#include "driverlib/rom.h" // to be removed
#include "driverlib/i2c.h" // to be removed
#include "InterruptHandlers.h"
#include "Tasks.h"
#include "MonitorTask.h"
#include "common/LocalUart.h"
#include "common/log.h"
#include "I2CSlaveTask.h"

// Rev 2:
// All that needs to be done is rename local_fpga_{v,k}u to
// f{1,2} AFAIK.

// This slave task is designed currently only for access to
// registers, with a single byte address and a single byte data.
// This is to respond to the IPMC needs.
// If it were to be adapted to a broader use case the task should be
// modified as follows
// 1. pass in getters and setters for the data via the task arguments as callbacks
// 2. rewrite the state machine to allow multi-byte transfers,
//    presumably also set via the task argument.
// 3. Replace all instances of I2C0_BASE with the appropriate parameter

#if defined(REV1) || defined(REV2)
#define SLAVE_I2C_BASE I2C0_BASE
#endif

// IPMC register map documented here
// https://github.com/apollo-lhc/cm_mcu/wiki/MCU-slave-documentation
// These two functions below should be turned into a call-back function,
// passed in via the arguments to the task, if I ever need to have more than
// one slave.
static uint8_t testreg = 0x0U;

static int local_fpga_f2, local_fpga_f1;

static uint8_t getSlaveData(uint8_t address)
{
  uint8_t value = 0x00U;
  switch (address) {
    case 0x0U: // reserved
      value = testreg;
      break;
    case 0x10U:                       // MCU temperature
      value = (uint8_t)getADCvalue(20) + 0.5f; // always valid
      break;
    case 0x12U: // FPGA F2 temp
      value = (uint8_t)(local_fpga_f2 >= 0 ? fpga_args.pm_values[local_fpga_f2] : 0U);
      if (value == 0)
        value = 0xFFU; // invalid value
      break;
    case 0x14U: // FPGA F1 temp
      value = (uint8_t)(local_fpga_f1 >= 0 ? fpga_args.pm_values[local_fpga_f1] : 0U);
      if (value == 0)
        value = 0xFFU; // invalid value
      break;
    case 0x16U: // hottest FF temp
    {
      int8_t imax_temp = -55; // turn off value
      for (int i = 0; i < NFIREFLIES; ++i) {
        int8_t v = getFFtemp(i);
        if (v > imax_temp)
          imax_temp = v;
      }
      if (imax_temp < 0)
        value = 0xFFU; // invalid
      else
        value = (uint8_t)imax_temp;
    } break;
    case 0x18U: // hottest regulator temp
    {
      float max_temp = -99.0f;
      for (int ps = 0; ps < dcdc_args.n_devices; ++ps) {
        for (int page = 0; page < dcdc_args.n_pages; ++page) {
          float thistemp = dcdc_args.pm_values[ps * (dcdc_args.n_commands * dcdc_args.n_pages) +
                                               page * dcdc_args.n_commands + 0];
          if (thistemp > max_temp)
            max_temp = thistemp;
        }
      }
      if (max_temp < 0)
        value = 0xFFU; // invalid
      else
        value = (uint8_t)(max_temp + 0.5f);
    } break;
    default:
      value = 0xFFU;
      break;
  }
  return value;
}

// only the test register is r/w; everything else is
// silently ignored.
static void setSlaveData(uint8_t addr, uint8_t val)
{
  // ignore other addresses
  if (addr == 0)
    testreg = val;
}

void I2CSlaveTask(void *parameters)
{
  TaskNotifyI2CSlave = xTaskGetCurrentTaskHandle();

  local_fpga_f1 = get_f1_index();
  local_fpga_f2 = get_f2_index();

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

    static UBaseType_t vv = 4096;
    CHECK_TASK_STACK_USAGE(vv);
  }
}
