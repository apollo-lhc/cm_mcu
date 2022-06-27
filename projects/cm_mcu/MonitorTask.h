/*
 * MonitorTask.h
 *
 *  Created on: May 28, 2019
 *      Author: wittich
 */

#ifndef PROJECTS_CM_MCU_MONITORTASK_H_
#define PROJECTS_CM_MCU_MONITORTASK_H_

#include "common/smbus.h"
#include "FreeRTOS.h"
#include "semphr.h"

extern SemaphoreHandle_t xMonSem;

// pilfered and adapted from http://billauer.co.il/blog/2018/01/c-pmbus-xilinx-fpga-kc705/
enum pm_type { PM_VOLTAGE,
               PM_NONVOLTAGE,
               PM_STATUS,
               PM_LINEAR11,
               PM_LINEAR16U,
               PM_LINEAR16S };

struct pm_command_t {
  unsigned char command; // I2c register address
  int size;              // number of bytes to read
  char *name;            // text describing command
  char *units;           // units for pretty printing
  enum pm_type type;     // how to decode command (L11 or bitfield or ...)
};

// how to find an I2C device, with a mux infront of it.
struct dev_i2c_addr_t {
  char *name;
  uint8_t mux_addr; // I2C address of the Mux
  uint8_t mux_bit;  // port of the mux; write value 0x1U<<mux_bit to the mux register
  uint8_t dev_addr; // I2C address of device.
};

struct MonitorTaskArgs_t {
  const char *name;                    // name to be assigned to the task
  struct dev_i2c_addr_t *devices;      // list of devices to query
  int n_devices;                       // number of devices
  struct pm_command_t *commands;       // list of commands
  const uint8_t n_commands;            // number of commands
  float *pm_values;                    // place to store results
  const int n_values;                  // number of results
  const uint8_t n_pages;               // number of pages to loop over
  tSMBus *smbus;                       // pointer to I2C controller
  volatile tSMBusStatus *smbus_status; // pointer to I2C status
  volatile TickType_t updateTick;      // last update time, in ticks
  SemaphoreHandle_t xSem;              // semaphore for controlling access to device
  const bool requirePower;             // true if device requires power
  UBaseType_t stack_size;              // stack size of task
};
// DC-DC converter
#ifdef REV1
#define NSUPPLIES_PS (5) // 5 devices, 2 pages each
#elif defined(REV2)
#define NSUPPLIES_PS (7) // 7 devices, 2 pages each
#endif
#define NCOMMANDS_PS 20 // number of entries in dcdc_ array
#define NPAGES_PS    2 // number of pages on the power supplies.

extern struct MonitorTaskArgs_t dcdc_args;
extern struct MonitorTaskArgs_t fpga_args;

#endif /* PROJECTS_CM_MCU_MONITORTASK_H_ */
