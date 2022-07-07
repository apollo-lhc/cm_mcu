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
#include "Tasks.h"

// REV1
#ifndef REV2
#define NFIREFLIES_F1 11
#define NFIREFLIES_F2 14
#else // REV2
// REV 2
#define NFIREFLIES_F1 10
#define NFIREFLIES_F2 10
#endif // REV 2
#define NFIREFLIES (NFIREFLIES_F1 + NFIREFLIES_F2)

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
  char *name;            // text describing command
  char *units;           // units for pretty printing
  enum sm_type type;     // how to decode command (L11 or bitfield or ...)
};

// how to find an I2C device, with a mux infront of it.

struct MonitorI2CTaskArgs_t {
  const char *name;                    // name to be assigned to the task
  struct dev_moni2c_addr_t *devices;      // list of devices to query
  int n_devices;                       // number of devices
  struct sm_command_t *commands;       // list of commands
  const uint8_t n_commands;                // number of commands
  const int n_values;                  // number of results
  const uint8_t n_pages;                   // number of pages to loop over
  uint8_t *sm_values;
  tSMBus *smbus;                       // pointer to I2C controller
  volatile tSMBusStatus *smbus_status; // pointer to I2C status
  volatile TickType_t updateTick;      // last update time, in ticks
  SemaphoreHandle_t xSem;              // semaphore for controlling access to device
  bool requirePower;                   // true if device requires power
  UBaseType_t stack_size;              // stack size of task
};

#define NSUPPLIES_FFLDAQ (20)
#define NCOMMANDS_FFLDAQ 3  // number of entries in ffldaq_ array
#define NPAGES_FFLDAQ    1   // number of pages on the 4-channel firefly ports with 25 Gbps

#define NSUPPLIES_FFLIT (20)
#define NCOMMANDS_FFLIT 3  // number of entries in fflit_ array
#define NPAGES_FFLIT    1   // number of pages on the 12-channel firefly ports with 14 Gbps

extern struct dev_moni2c_addr_t ff_moni2c_addrs[NFIREFLIES];
extern struct MonitorI2CTaskArgs_t fflit_args;
//extern struct MonitorI2CTaskArgs_t fflot_args;
extern struct MonitorI2CTaskArgs_t ffldaq_args;

#endif /* PROJECTS_CM_MCU_MONITORI2CTASK_H_ */
