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

extern float sm_values[];

extern SemaphoreHandle_t xMonSem;

// pilfered and adapted from http://billauer.co.il/blog/2018/01/c-pmbus-xilinx-fpga-kc705/
//enum sm_type { SM_VOLTAGE, SM_NONVOLTAGE, SM_STATUS, SM_LINEAR11, SM_LINEAR16U, SM_LINEAR16S } ;
enum sm_type { SM_STATUS,
               SM_LINEAR11,
               SM_LINEAR16U,
               SM_LINEAR16S };

struct sm_command_t {
  int reg_size;              // number of bytes of register/command
  unsigned char command; // I2c register address
  int size;              // number of bytes to read
  float sm_value;      // output from read
  char *name;            // text describing command
  char *units;           // units for pretty printing
  enum sm_type type;     // how to decode command (L11 or bitfield or ...)
};

// how to find an I2C device, with a mux infront of it.
struct dev_moni2c_addr_t {
  char *name;
  uint8_t mux_addr; // I2C address of the Mux
  uint8_t mux_bit;  // port of the mux; write value 0x1U<<mux_bit to the mux register
  uint8_t dev_addr; // I2C address of device.
  char *instance; // MONI2C instance
};


struct MonitorI2CTaskArgs_t {
  const char *name;                    // name to be assigned to the task
  struct dev_moni2c_addr_t *devices;      // list of devices to query
  int n_devices;                       // number of devices
  struct sm_command_t *commands;       // list of commands
  const uint8_t n_commands;                // number of commands
  const int n_values;                  // number of results
  const uint8_t n_pages;                   // number of pages to loop over
  tSMBus *smbus;                       // pointer to I2C controller
  volatile tSMBusStatus *smbus_status; // pointer to I2C status
  volatile TickType_t updateTick;      // last update time, in ticks
  SemaphoreHandle_t xSem;              // semaphore for controlling access to device
  bool requirePower;                   // true if device requires power
  UBaseType_t stack_size;              // stack size of task
};

#define NSUPPLIES_FFLDAQ (4)
#define NCOMMANDS_FFLDAQ 3  // number of entries in ffldaq_ array
#define NPAGES_FFLDAQ    1   // number of pages on the 4-channel firefly ports with 25 Gbps

//extern struct MonitorI2CTaskArgs_t ffl12c14_args;
//extern struct MonitorI2CTaskArgs_t ffl12c25_args;
extern struct MonitorI2CTaskArgs_t ffldaq_args;

#endif /* PROJECTS_CM_MCU_MONITORI2CTASK_H_ */
