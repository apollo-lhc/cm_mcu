/*
 * MonitorTask.h
 *
 *  Created on: May 28, 2019
 *      Author: wittich
 */

#ifndef PROJECTS_CM_MCU_MONITORTASK_H_
#define PROJECTS_CM_MCU_MONITORTASK_H_

#include "common/smbus.h"


extern float pm_values[];
#define ABS(x) ((x)<0?(-(x)):(x))

// pilfered and adapted from http://billauer.co.il/blog/2018/01/c-pmbus-xilinx-fpga-kc705/
enum { PM_VOLTAGE, PM_NONVOLTAGE, PM_STATUS, PM_LINEAR11, PM_LINEAR16U, PM_LINEAR16S } pm_types ;

struct pm_command_t {
  unsigned char command;
  int size;
  char *name;
  char *units;
  int type;
};

// how to find an I2C device, with a mux infront of it.
struct dev_i2c_addr_t {
  char *name;
  uint8_t mux_addr; // I2C address of the Mux
  uint8_t mux_bit;  // port of the mux; write value 0x1U<<mux_bit to the mux register
  uint8_t dev_addr; // I2C address of device.
};

struct MonitorTaskArgs_t {
  const char *name;
  struct dev_i2c_addr_t * devices;
  int n_devices;
  struct pm_command_t * commands;
  const int n_commands;
  float *pm_values;
  const int n_values;
  const int n_pages;
  tSMBus *smbus;
  volatile tSMBusStatus *smbus_status;
  TickType_t updateTick;
};
// DC-DC converter
#define NSUPPLIES_PS (5) // 5 devices, 2 pages each
#define NCOMMANDS_PS 6 // number of entries in above array
#define NPAGES_PS    2 // number of pages on the power supplies.

extern struct MonitorTaskArgs_t dcdc_args;
extern struct MonitorTaskArgs_t fpga_args;



#endif /* PROJECTS_CM_MCU_MONITORTASK_H_ */
