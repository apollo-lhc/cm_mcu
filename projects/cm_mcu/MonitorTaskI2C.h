/*
 * MonitorI2CTask.h
 *
 */

#ifndef PROJECTS_CM_MCU_MONITORTASKI2C_H_
#define PROJECTS_CM_MCU_MONITORTASKI2C_H_

#include "FreeRTOS.h" // IWYU pragma: keep
#include "semphr.h"
#include "Tasks.h"

struct i2c_reg_command_t {
  int reg_size;             // number of bytes of register/command
  unsigned char page;       // I2C page address
  unsigned char command[4]; // I2c register address
  int size;                 // number of bytes to read
  char *name;               // text describing command
  uint16_t bit_mask;        // begin bit mask
  char *units;              // units for pretty printing
  enum pm_type type;        // how to decode command (L11 or bitfield or ...)
  uint16_t (*devicelist)(void);
  void (*storeData)(uint16_t data, int which); // store data in location which
  uint16_t (*retrieveData)(int which);         // retrieve data from location which
};

// how to find an I2C device, with a mux infront of it.

typedef bool (*MonTaskFcnPointer)(int device);
typedef int (*MonTaskI2CTypeFcnPointer)(int); // what kind of device we have (i.e., for FF, CERN-B, B04, Y12-14, Y12-25, etc.)

struct MonitorTaskI2CArgs_t {
  const char *name;                      // name to be assigned to the task
  struct dev_moni2c_addr_t *devices;     // list of devices to query
  int i2c_dev;                           // i2c controller #
  int n_devices;                         // number of devices
  struct i2c_reg_command_t *commands;    // list of commands
  const uint8_t n_commands;              // number of commands
  const uint16_t selpage_reg;            // register for selecting page
  TickType_t updateTick;                 // last update time, in ticks
  SemaphoreHandle_t xSem;                // semaphore for controlling access to device
  UBaseType_t stack_size;                // stack size of task
  MonTaskFcnPointer presentCallback;     // callback for present check
  MonTaskI2CTypeFcnPointer typeCallback; // callback for type check
};

#define FF_SELPAGE_REG  0x7f
#define CLK_SELPAGE_REG 0x1

#define NDEVICES_CLK 5

// for autogenerated code
#define DEVICE_CERNB  0x01
#define DEVICE_14G    0x02
#define DEVICE_25G4   0x04
#define DEVICE_25G12  0x08
#define DEVICE_SI5341 0x01
#define DEVICE_SI5395 0x02

#define DEVICE_NONE 0x80

extern struct dev_moni2c_addr_t clk_moni2c_addrs[NDEVICES_CLK];

#endif /* PROJECTS_CM_MCU_MONITORTASKI2C_H_ */