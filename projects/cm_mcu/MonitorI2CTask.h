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
  int i2c_dev;                         // i2c controller #
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
  int8_t *sm_vendor_part;
};

#define NSUPPLIES_FFLDAQ_F1 (4)
#define NCOMMANDS_FFLDAQ_F1 7  // number of commands
#define NPAGES_FFLDAQ_F1    1   // number of pages on the 4-channel firefly ports

#define NSUPPLIES_FFLIT_F1 (6)
#define NCOMMANDS_FFLIT_F1 7  // number of commands
#define NPAGES_FFLIT_F1    1   // number of pages on the 12-channel firefly ports

#define NSUPPLIES_FFLDAQ_F2 (4)
#define NCOMMANDS_FFLDAQ_F2 7  // number of commands
#define NPAGES_FFLDAQ_F2    1   // number of pages on the 4-channel firefly ports

#define NSUPPLIES_FFLIT_F2 (6)
#define NCOMMANDS_FFLIT_F2 7  // number of commands
#define NPAGES_FFLIT_F2    1   // number of pages on the 12-channel firefly ports

extern struct dev_moni2c_addr_t fflit_f1_moni2c_addrs[NFIREFLIES_IT_F1];
extern struct dev_moni2c_addr_t ffldaq_f1_moni2c_addrs[NFIREFLIES_DAQ_F1];
extern struct dev_moni2c_addr_t fflit_f2_moni2c_addrs[NFIREFLIES_IT_F2];
extern struct dev_moni2c_addr_t ffldaq_f2_moni2c_addrs[NFIREFLIES_DAQ_F2];
extern struct MonitorI2CTaskArgs_t fflit_f1_args;
extern struct MonitorI2CTaskArgs_t ffldaq_f1_args;
extern struct MonitorI2CTaskArgs_t fflit_f2_args;
extern struct MonitorI2CTaskArgs_t ffldaq_f2_args;

#define CLK_MON_NDEVICES           2
#define CLK_MON_NCOMMANDS          9
#define CLK_MON_NVALUES_PER_DEVICE 9
#define CLK_MON_NVALUES            (CLK_MON_NDEVICES * CLK_MON_NVALUES_PER_DEVICE)

#endif /* PROJECTS_CM_MCU_MONITORI2CTASK_H_ */
