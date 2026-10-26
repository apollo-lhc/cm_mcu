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
#include "Tasks.h"

extern SemaphoreHandle_t xMonSem;

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
  const char *name;                     // name to be assigned to the task
  const struct dev_i2c_addr_t *devices; // list of devices to query
  int n_devices;                        // number of devices
  const struct pm_command_t *commands;  // list of commands
  const uint8_t n_commands;             // number of commands
  float *pm_values;                     // place to store results
  const int n_values;                   // number of results
  const uint8_t n_pages;                // number of pages to loop over
  tSMBus *smbus;                        // pointer to I2C controller
  volatile tSMBusStatus *smbus_status;  // pointer to I2C status
  volatile TickType_t updateTick;       // last update time, in ticks
  bool ignoreNACK;                      // true if NACKs should be ignored
  SemaphoreHandle_t xSem;               // semaphore for controlling access to device
  const bool requirePower;              // true if device requires power
  UBaseType_t stack_size;               // stack size of task
};
// DC-DC converter
#ifdef REV1
#define NSUPPLIES_PS (5)             // 5 devices, 2 pages each
#elif defined(REV2) || defined(REV3) // Rev 2 or 3
#define NSUPPLIES_PS (7)             // 7 devices, 2 pages each
#endif
#define NCOMMANDS_PS 6 // number of entries in dcdc_ array
#define NPAGES_PS    2 // number of pages on the power supplies.

// The LGA80D specifies 0..175 C for its temperature thresholds. Treat a
// READ_TEMPERATURE_1 value outside that range as invalid rather than allowing
// a malformed but ACKed Linear11 word to power down the board.
#define LGA80D_TEMP_MIN_C (0.0f)
#define LGA80D_TEMP_MAX_C (175.0f)

// LGA80D READ_TEMPERATURE_1 and _3 reads outside that range, for the ZynqMon diagnostics
// block, one record per sensor: saturating counts of 0xFFFF and of other bad words, and that
// sensor's latest bad read. PSMON writes them in a critical section and getLGA80DBadTempCopy()
// copies them in one, so a reader never mixes two events.
struct lga80d_badt_t {
  uint16_t n_ffff;  // raw word 0xFFFF (-0.50 C)
  uint16_t n_other; // any other out-of-range word
  uint16_t raw;     // latest bad read: raw word
  uint16_t row;     // latest bad read: ps * NPAGES_PS + page (the psmon row order); 0xFF = none
};
void getLGA80DBadTempCopy(struct lga80d_badt_t out[2]); // [0] TEMP_1, [1] TEMP_3

extern struct MonitorTaskArgs_t dcdc_args;
extern struct MonitorTaskArgs_t fpga_args;

#endif /* PROJECTS_CM_MCU_MONITORTASK_H_ */
