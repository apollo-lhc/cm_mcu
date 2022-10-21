/*
 * MonitorI2CTask.h
 *
 *  Created on: June 30, 2022
 *      Author: pkotamnives
 */

#ifndef PROJECTS_CM_MCU_MONITORI2CTASK_H_
#define PROJECTS_CM_MCU_MONITORI2CTASK_H_

#include "common/smbus.h"
#include "FreeRTOS.h"
#include "semphr.h"
#include "Tasks.h"

struct sm_command_t {
  int reg_size;          // number of bytes of register/command
  unsigned char page;    // I2C page address
  unsigned char command; // I2c register address
  int size;              // number of bytes to read
  char *name;            // text describing command
  uint16_t bit_mask;     // begin bit mask
  char *units;           // units for pretty printing
  enum pm_type type;     // how to decode command (L11 or bitfield or ...)
};

// how to find an I2C device, with a mux infront of it.

struct MonitorI2CTaskArgs_t {
  const char *name;                  // name to be assigned to the task
  struct dev_moni2c_addr_t *devices; // list of devices to query
  int i2c_dev;                       // i2c controller #
  int n_devices;                     // number of devices
  struct sm_command_t *commands;     // list of commands
  const uint8_t n_commands;          // number of commands
  const int n_values;                // number of results
  const uint8_t n_pages;             // number of pages to loop over
  uint16_t *sm_values;
  tSMBus *smbus;                       // pointer to I2C controller
  volatile tSMBusStatus *smbus_status; // pointer to I2C status
  TickType_t updateTick;               // last update time, in ticks
  SemaphoreHandle_t xSem;              // semaphore for controlling access to device
  UBaseType_t stack_size;              // stack size of task
};

#ifndef REV2
#define NSUPPLIES_FFLDAQ_F1 (3)
#else // REV2
#define NSUPPLIES_FFLDAQ_F1 (4)
#endif                        // REV 2
#define NCOMMANDS_FFLDAQ_F1 4 // number of commands
#define NPAGES_FFLDAQ_F1    1 // number of pages on the 4-channel firefly ports

#ifndef REV2
#define NSUPPLIES_FFL12_F1 (8)
#else // REV1
#define NSUPPLIES_FFL12_F1 (6)
#endif                       // REV 2
#define NCOMMANDS_FFL12_F1 4 // number of commands
#define NPAGES_FFL12_F1    1 // number of pages on the 12-channel firefly ports

#ifndef REV2
#define NSUPPLIES_FFLDAQ_F2 (10)
#else // REV1
#define NSUPPLIES_FFLDAQ_F2 (4)
#endif                        // REV 2
#define NCOMMANDS_FFLDAQ_F2 4 // number of commands
#define NPAGES_FFLDAQ_F2    1 // number of pages on the 4-channel firefly ports

#ifndef REV2
#define NSUPPLIES_FFL12_F2 (4)
#else // REV1
#define NSUPPLIES_FFL12_F2 (6)
#endif                       // REV 2
#define NCOMMANDS_FFL12_F2 4 // number of commands
#define NPAGES_FFL12_F2    1 // number of pages on the 12-channel firefly ports

extern struct dev_moni2c_addr_t ffl12_f1_moni2c_addrs[NFIREFLIES_IT_F1];
extern struct dev_moni2c_addr_t ffldaq_f1_moni2c_addrs[NFIREFLIES_DAQ_F1];
extern struct dev_moni2c_addr_t ffl12_f2_moni2c_addrs[NFIREFLIES_IT_F2];
extern struct dev_moni2c_addr_t ffldaq_f2_moni2c_addrs[NFIREFLIES_DAQ_F2];
extern struct MonitorI2CTaskArgs_t ffl12_f1_args;
extern struct MonitorI2CTaskArgs_t ffldaq_f1_args;
extern struct MonitorI2CTaskArgs_t ffl12_f2_args;
extern struct MonitorI2CTaskArgs_t ffldaq_f2_args;

#define NSUPPLIES_CLK    (4)
#define NCOMMANDS_CLK    7 // number of commands
#define NPAGES_CLK       1 //
#define NSUPPLIES_CLKR0A (1)
#define NCOMMANDS_CLKR0A 6 // number of commands
#define NPAGES_CLKR0A    1 //

extern struct MonitorI2CTaskArgs_t clock_args;
extern struct MonitorI2CTaskArgs_t clockr0a_args;

#endif /* PROJECTS_CM_MCU_MONITORI2CTASK_H_ */
