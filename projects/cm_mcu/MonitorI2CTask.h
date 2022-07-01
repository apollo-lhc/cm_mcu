/*
 * MonitorI2CTask.h
 *
 *  Created on: June 30, 2022
 *      Author: pkotamnives
 */

#ifndef PROJECTS_CM_MCU_MONITORI2CTASK_H_
#define PROJECTS_CM_MCU_MONITORI2CTASK_H_

#include "common/smbus.h"
#include "semphr.h"
#include "MonitorTask.h"

extern float sm_values[];

extern SemaphoreHandle_t xMonSem;

// pilfered and adapted from http://billauer.co.il/blog/2018/01/c-pmbus-xilinx-fpga-kc705/
//enum sm_type { SM_VOLTAGE, SM_NONVOLTAGE, SM_STATUS, SM_LINEAR11, SM_LINEAR16U, SM_LINEAR16S } ;

struct sm_command_t {
  unsigned char command; // I2c register address
  int size;              // number of bytes to read
  char *name;            // text describing command
  char *units;           // units for pretty printing
  enum pm_type type;     // how to decode command (L11 or bitfield or ...)
};


struct MonitorI2CTaskArgs_t {
  const char *name;                    // name to be assigned to the task
  struct dev_i2c_addr_t *devices;      // list of devices to query
  int n_devices;                       // number of devices
  struct sm_command_t *commands;       // list of commands
  const uint8_t n_commands;                // number of commands
  float *sm_values;                    // place to store results
  const int n_values;                  // number of results
  const uint8_t n_pages;                   // number of pages to loop over
  tSMBus *smbus;                       // pointer to I2C controller
  volatile tSMBusStatus *smbus_status; // pointer to I2C status
  volatile TickType_t updateTick;      // last update time, in ticks
  SemaphoreHandle_t xSem;              // semaphore for controlling access to device
  bool requirePower;                   // true if device requires power
  UBaseType_t stack_size;              // stack size of task
};

#define NSUPPLIES_FFLXCVR (1)
#define NCOMMANDS_FFLXCVR 3  // number of entries in fflxcvr_ array
#define NPAGES_FFLXCVR    1   // number of pages on the 4-channel firefly ports with 25 Gbps

//extern struct MonitorI2CTaskArgs_t ffl12c14_args;
//extern struct MonitorI2CTaskArgs_t ffl12c25_args;
extern struct MonitorI2CTaskArgs_t fflxcvr_args;

#endif /* PROJECTS_CM_MCU_MONITORI2CTASK_H_ */
