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
#include "MonitorTask.h"
#include "common/uart.h"
#include "I2CSlaveTask.h"

// This slave task is designed currently only for access to
// registers, with a single byte address and a single byte data.
// This is to respond to the IPMC needs.
// If it were to be adapted to a broader use case the task should be
// modified as follows
// 1. pass in getters and setters for the data via the task arguments as callbacks
// 2. rewrite the state machine to allow multi-byte transfers,
//    presumably also set via the task argument.
// 3. Replace all instances of I2C0_BASE with the appropriate parameter




//#define DEBUG_I2CSLAVE
#ifdef DEBUG_I2CSLAVE
void Print(const char* str);
#endif



// IPMC register map documented here
// https://github.com/apollo-lhc/cm_mcu/wiki/MCU-slave-documentation
// These two functions below should be turned into a call-back function,
// passed in via the arguments to the task, if I ever need to have more than
// one slave.
static
uint8_t testreg = 0x0U;

static int fpga_vu, fpga_ku;

static uint8_t getSlaveData(uint8_t address)
{
  uint8_t value = 0x00U;
  switch (address) {
    case 0x0U: // reserved
      value = testreg;
      break;
    case 0x10U: // MCU temperature
      value = getADCvalue(20)+0.5; // always valid
      break;
    case 0x12U: // FPGA VU temp
      value = (uint8_t) fpga_vu>=0?fpga_args.pm_values[fpga_vu]:0U;
      if ( value == 0 ) value = 0xFFU; // invalid value
      break;
    case 0x14U: // FPGA KU temp
      value = (uint8_t) fpga_ku>=0?fpga_args.pm_values[fpga_ku]:0U;
      if ( value == 0 ) value = 0xFFU; // invalid value 
      break;
    case 0x16U: // hottest FF temp
    {
      int8_t imax_temp = -55; // turn off value
      for ( int i = 0; i < NFIREFLIES; ++i ) {
        int8_t v = getFFvalue(i);
        if ( v > imax_temp )
          imax_temp = v;
      }
      if ( imax_temp < 0 )
        value = 0xFFU; // invalid 
      else
        value = (uint8_t) imax_temp;
    }
    break;
    case 0x18U: // hottest regulator temp
    {
      float max_temp = -99.0;
      for (int ps = 0; ps < dcdc_args.n_devices; ++ps ) {
        for ( int page = 0; page < dcdc_args.n_pages; ++page ) {
          float thistemp = dcdc_args.pm_values[ps*(dcdc_args.n_commands*dcdc_args.n_pages)
                                               +page*dcdc_args.n_commands+0];
          if ( thistemp > max_temp )
            max_temp = thistemp;
        }
      }
      if ( max_temp < 0  )
        value = 0xFFU; // invalid
      else
        value = (uint8_t) max_temp+0.5;
    }
    break;
    default:
      value = 0xFFU;
      break;
  }
  return value;
}

// only the test register is r/w; everything else is
// silently ignored.
static
void setSlaveData(uint8_t addr, uint8_t val)
{
  // ignore other addresses
  if ( addr == 0 )
    testreg = val;
}


void I2CSlaveTask(void *parameters)
{
  TaskNotifyI2CSlave = xTaskGetCurrentTaskHandle();
  // struct I2CSlaveTaskArgs_t * args = parameters;

  fpga_ku = get_ku_index();
  fpga_vu = get_vu_index();


  ROM_I2CSlaveEnable(I2C0_BASE);

  enum I2C_STATE {I2C_READY, I2C_FIRSTBYTE, I2C_RECEIVE_B0, I2C_RECEIVE_B1,
    I2C_TRANSMIT_B0, I2C_TRANSMIT_B1 };

  enum I2C_STATE theState = I2C_READY;
  uint8_t addr = 0;
  uint16_t val = 0;
#ifdef DEBUG_I2CSLAVE
  char tmp[64];
#endif

  // loop forever
  for (;;) {
#ifdef DEBUG_I2CSLAVE
    snprintf(tmp, 64, "state is %d\r\n", (int)theState);
    Print(tmp);
#endif

    // block on notification from the I2C Slave interrupt handler
    // Wait to be notified that the transmission is complete.
    // we wait indefinitely between transactions, but time out after some time
    // during transactions.
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

#ifdef DEBUG_I2CSLAVE
    snprintf(tmp, 64, "\tINT is 0x%08x\r\n", interruptStatus);
    Print(tmp);
#endif
    // read the I2C slave status register I2CSCSR
    uint32_t status = ROM_I2CSlaveStatus(I2C0_BASE);
#ifdef DEBUG_I2CSLAVE
    snprintf(tmp, 64, "Slave status is 0x%08x\r\n", status);
    Print(tmp);
#endif

    // we received an interrupt.
    // only possible values of the interrupt are the values set in the slave
    // interrupt enable
    // I2C_SLAVE_INT_DATA  **
    // I2C_SLAVE_INT_STOP
    // I2C_SLAVE_INT_START
    if ( interruptStatus & I2C_SLAVE_INT_STOP) {
#ifdef DEBUG_I2CSLAVE
      Print("Stop interrupt received");
#endif
    }
    else if  ( interruptStatus & I2C_SLAVE_INT_DATA )  {
      if ( status == I2C_SLAVE_ACT_TREQ ) { // transmission request
        switch (theState) {
        case I2C_READY: // non-register transmit request
          // we ignore these, but to keep the TM4C happy we put something
          // on the bus
          ROM_I2CSlaveDataPut(I2C0_BASE, 0xFF);
          theState = I2C_READY;
          break;
        case I2C_FIRSTBYTE: // we received a byte and are now asked to transmit
        {
          // register read
          uint8_t b = getSlaveData(addr);
#ifdef DEBUG_I2CSLAVE
          snprintf(tmp, 64, "byte 1 sent %x\r\n",b); Print(tmp);
#endif
          ROM_I2CSlaveDataPut(I2C0_BASE, b);
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
      else if ( status == I2C_SLAVE_ACT_RREQ_FBR ) { // first byte
        addr = ROM_I2CSlaveDataGet(I2C0_BASE);
#ifdef DEBUG_I2CSLAVE
        snprintf(tmp, 64, "Address %d received\r\n",addr);
        Print(tmp);
#endif
        theState = I2C_FIRSTBYTE;
      }
      else if ( status == I2C_SLAVE_ACT_RREQ ) {// not first byte
        switch ( theState) {
        case I2C_READY: // we ignore this, but need to keep TM4C happy
          val  = ROM_I2CSlaveDataGet(I2C0_BASE);
          // stay in READY state
          theState = I2C_READY;
        break;
        case I2C_FIRSTBYTE:
          val = ROM_I2CSlaveDataGet(I2C0_BASE);
          setSlaveData(addr, val);
          theState = I2C_READY;
#ifdef DEBUG_I2CSLAVE
          Print("wrote byte 1\r\n");
#endif
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

