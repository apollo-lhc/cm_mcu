/*
 * CommandLineTask.c
 *
 *  Created on: Apr 7, 2019
 *      Author: wittich, rzou
 */

#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_types.h"
#include "inc/hw_memmap.h"
#include "inc/hw_nvic.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "driverlib/systick.h"
#include "driverlib/sysctl.h"
#include "driverlib/eeprom.h"

// local includes
#include "common/i2c_reg.h"
#include "common/uart.h"
#include "common/power_ctl.h"
#include "common/pinsel.h"
#include "common/smbus.h"
#include "common/utils.h"
#include "common/microrl.h"

// FreeRTOS includes
#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "stream_buffer.h"
#include "queue.h"

// strlen, strtol, and strncpy
#include <string.h>
#include <stdlib.h>

// TivaWare includes
#include "driverlib/uart.h"

#include "MonitorTask.h"
#include "CommandLineTask.h"
#include "I2CCommunication.h"
#include "Tasks.h"
#include "AlarmUtilities.h"

#include "clocksynth.h"

#ifdef DEBUG_CON
// prototype of mutex'd print
#define DPRINT(x) Print(x)
#else // DEBUG_CON
#define DPRINT(x)
#endif // DEBUG_CON

void Print(const char *str);

// local sprintf prototype
int snprintf(char *buf, unsigned int count, const char *format, ...);

#define MAX_INPUT_LENGTH  50
#define MAX_OUTPUT_LENGTH 512

extern tSMBus g_sMaster1;
extern tSMBusStatus eStatus1;
extern tSMBus g_sMaster2;
extern tSMBusStatus eStatus2;
extern tSMBus g_sMaster3;
extern tSMBusStatus eStatus3;
extern tSMBus g_sMaster4;
extern tSMBusStatus eStatus4;
extern tSMBus g_sMaster6;
extern tSMBusStatus eStatus6;
static tSMBus *p_sMaster = &g_sMaster4;
static tSMBusStatus *p_eStatus = &eStatus4;

#define SCRATCH_SIZE 512
static char m[SCRATCH_SIZE];

// Ugly hack for now -- I don't understand how to reconcile these
// two parts of the FreeRTOS-Plus code w/o casts-o-plenty
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wformat" // because of our mini-sprintf

static BaseType_t i2c_ctl_set_dev(int argc, char **argv)
{
  int s = SCRATCH_SIZE;
  BaseType_t i = strtol(argv[1], NULL, 10); // device number

  int status = apollo_i2c_ctl_set_dev(i);
  if (status == 0) {
    snprintf(m, s, "Setting i2c device to %d\r\n", i);
  }
  else if (status == -1) {
    snprintf(m, s, "Invalid i2c device %d (%s), only 1,2,3, 4 and 6 supported\r\n", i, argv[1]);
  }
  else if (status == -2) {
    snprintf(m, s, "%s: huh? line %d\r\n", argv[0], __LINE__);
  }
  else {
    snprintf(m, s, "%s: invalid return value. line %d\r\n", argv[0], __LINE__);
  }
  return pdFALSE;
}

#define I2C_CTL_MAX_BYTES 4

static BaseType_t i2c_ctl_r(int argc, char **argv)
{
  int s = SCRATCH_SIZE;
  BaseType_t address, nbytes;
  address = strtol(argv[1], NULL, 16);
  nbytes = strtol(argv[2], NULL, 10);
  uint8_t data[I2C_CTL_MAX_BYTES];

  int status = apollo_i2c_ctl_r(address, nbytes, data);
  if (status == 0) {
    snprintf(m, s, "%s: add: 0x%02x: value 0x%02x %02x %02x %02x\r\n", argv[0], address, data[3],
             data[2], data[1], data[0]);
  }
  else if (status == -1) {
    snprintf(m, s, "%s: operation failed (1)\r\n", argv[0]);
  }
  else if (status == -2) {
    snprintf(m, s, "%s: operation failed (2, value=%d)\r\n", argv[0], *p_eStatus);
  }
  else {
    snprintf(m, s, "%s: invalid return value. line %d\r\n", argv[0], __LINE__);
  }
  return pdFALSE;
}
static BaseType_t i2c_ctl_reg_r(int argc, char **argv)
{
  int s = SCRATCH_SIZE;
  BaseType_t address, reg_address, nbytes;
  address = strtol(argv[1], NULL, 16);
  reg_address = strtol(argv[2], NULL, 16);
  nbytes = strtol(argv[3], NULL, 10);
  uint8_t data[I2C_CTL_MAX_BYTES];
  uint8_t txdata = reg_address;

  int status = apollo_i2c_ctl_reg_r(address, txdata, nbytes, data);
  if (status == 0) {
    snprintf(m, s, "i2cr: add: 0x%02x, reg 0x%02x: value 0x%02x %02x %02x %02x\r\n", address,
             reg_address, data[3], data[2], data[1], data[0]);
  }
  else if (status == -1) {
    snprintf(m, s, "%s: operation failed (1)\r\n", argv[0]);
  }
  else if (status == -2) {
    snprintf(m, s, "%s: operation failed (2, value=%d)\r\n", argv[0], *p_eStatus);
  }
  else {
    snprintf(m, s, "%s: invalid return value. line %d\r\n", argv[0], __LINE__);
  }
  return pdFALSE;
}

static BaseType_t i2c_ctl_reg_w(int argc, char **argv)
{
  int s = SCRATCH_SIZE;
  // first byte is the register, others are the data
  BaseType_t address, reg_address, nbytes, packed_data;
  address = strtol(argv[1], NULL, 16);     // address
  reg_address = strtol(argv[2], NULL, 16); // register
  nbytes = strtol(argv[3], NULL, 16);      // number of bytes
  packed_data = strtol(argv[4], NULL, 16); // data

  int status = apollo_i2c_ctl_reg_w(address, reg_address, nbytes, packed_data);
  if (status == 0) {
    snprintf(m, s, "%s: Wrote to address 0x%x, register 0x%x, value 0x%08x (%d bytes)\r\n", argv[0],
             address, reg_address, packed_data, nbytes - 1);
  }
  else if (status == -1) {
    snprintf(m, s, "%s: operation failed (1)\r\n", argv[0]);
  }
  else if (status == -2) {
    snprintf(m, s, "%s: operation failed (2)\r\n", argv[0]);
  }
  else {
    snprintf(m, s, "%s: invalid return value. line %d\r\n", argv[0], __LINE__);
  }

  return pdFALSE;
}

static BaseType_t i2c_ctl_w(int argc, char **argv)
{
  int s = SCRATCH_SIZE;
  BaseType_t address, nbytes, value;
  address = strtol(argv[1], NULL, 16);
  nbytes = strtol(argv[2], NULL, 16);
  value = strtol(argv[3], NULL, 16);

  int status = apollo_i2c_ctl_w(address, nbytes, value);
  if (status == 0) {
    snprintf(m, s, "i2cwr: Wrote to address 0x%x, value 0x%08x (%d bytes)\r\n", address, value,
             nbytes);
  }
  else if (status == -1) {
    snprintf(m, s, "%s: write failed (1)\r\n", argv[0]);
  }
  else if (status == -2) {
    snprintf(m, s, "%s: write failed (2)\r\n", argv[0]);
  }
  else {
    snprintf(m, s, "%s: invalid return value. line %d\r\n", argv[0], __LINE__);
  }
  return pdFALSE;
}

extern struct gpio_pin_t oks[];

// send power control commands
static BaseType_t power_ctl(int argc, char **argv)
{
  int s = SCRATCH_SIZE;

  uint32_t message;
  if (strncmp(argv[1], "on", 2) == 0) {
    message = PS_ON; // turn on power supply
  }
  else if (strncmp(argv[1], "off", 3) == 0) {
    message = PS_OFF; // turn off power supply
  }
  else if (strncmp(argv[1], "clearfail", 9) == 0) {
    message = PS_ANYFAIL_ALARM_CLEAR;
  }
  else if (strncmp(argv[1], "status", 5) == 0) { // report status to UART
    int copied = 0;
    bool ku_enable = (read_gpio_pin(TM4C_DIP_SW_1) == 1);
    bool vu_enable = (read_gpio_pin(TM4C_DIP_SW_2) == 1);
    static int i = 0;
    if (i == 0) {
      copied += snprintf(m + copied, SCRATCH_SIZE - copied,
                         "%s:\r\nVU_ENABLE:\t%d\r\n"
                         "KU_ENABLE:\t%d\r\n",
                         argv[0], vu_enable, ku_enable);
      copied += snprintf(m + copied, SCRATCH_SIZE - copied, "State machine state: %s\r\n",
                         getPowerControlStateName(getPowerControlState()));
    }
    for (; i < N_PS_OKS; ++i) {
      enum ps_state j = getPSStatus(i);
      char *c;
      switch (j) {
        case PWR_UNKNOWN:
          c = "PWR_UNKNOWN";
          break;
        case PWR_ON:
          c = "PWR_ON";
          break;
        case PWR_OFF:
          c = "PWR_OFF";
          break;
        case PWR_DISABLED:
          c = "PWR_DISABLED";
          break;
        case PWR_FAILED:
          c = "PWR_FAILED";
          break;
        default:
          c = "UNKNOWN";
          break;
      }

      copied +=
          snprintf(m + copied, SCRATCH_SIZE - copied, "%15s: %s\r\n", pin_names[oks[i].name], c);
      if ((SCRATCH_SIZE - copied) < 20 && (i < N_PS_OKS)) {
        ++i;
        return pdTRUE;
      }
    }
    i = 0;
    return pdFALSE;
  }
  else {
    snprintf(m, s, "power_ctl: invalid argument %s received\r\n", argv[1]);
    return pdFALSE;
  }
  // Send a message to the power supply task, if needed
  xQueueSendToBack(xPwrQueue, &message, pdMS_TO_TICKS(10));
  m[0] = '\0'; // no output from this command

  return pdFALSE;
}

// takes 1-2 arguments
static BaseType_t alarm_ctl(int argc, char **argv)
{
  int s = SCRATCH_SIZE;
  if (argc < 2) {
    snprintf(m, s, "%s: need one or more arguments\r\n", argv[0]);
    return pdFALSE;
  }

  uint32_t message;
  if (strncmp(argv[1], "clear", 4) == 0) {
    message = ALM_CLEAR_ALL; // clear all alarms
    xQueueSendToBack(tempAlarmTask.xAlmQueue, &message, pdMS_TO_TICKS(10));
    m[0] = '\0'; // no output from this command

    return pdFALSE;
  }
  else if (strncmp(argv[1], "status", 5) == 0) { // report status to UART
    int copied = 0;
    copied += snprintf(m + copied, SCRATCH_SIZE - copied, "%s: ALARM status\r\n", argv[0]);
    int32_t stat = getAlarmStatus();
    copied += snprintf(m + copied, SCRATCH_SIZE - copied, "Raw: 0x%08x\r\n", stat);

    float ff_val = getAlarmTemperature(FF);
    int tens, frac;
    float_to_ints(ff_val, &tens, &frac);
    copied +=
        snprintf(m + copied, SCRATCH_SIZE - copied, "TEMP FFLY: %s \t Threshold: %02d.%02d\r\n",
                 (stat & ALM_STAT_FIREFLY_OVERTEMP) ? "ALARM" : "GOOD", tens, frac);

    float fpga_val = getAlarmTemperature(FPGA);
    float_to_ints(fpga_val, &tens, &frac);
    copied +=
        snprintf(m + copied, SCRATCH_SIZE - copied, "TEMP FPGA: %s \t Threshold: %02d.%02d\r\n",
                 (stat & ALM_STAT_FPGA_OVERTEMP) ? "ALARM" : "GOOD", tens, frac);

    float dcdc_val = getAlarmTemperature(DCDC);
    float_to_ints(dcdc_val, &tens, &frac);
    copied +=
        snprintf(m + copied, SCRATCH_SIZE - copied, "TEMP DCDC: %s \t Threshold: %02d.%02d\r\n",
                 (stat & ALM_STAT_DCDC_OVERTEMP) ? "ALARM" : "GOOD", tens, frac);

    float tm4c_val = getAlarmTemperature(TM4C);
    float_to_ints(tm4c_val, &tens, &frac);
    copied +=
        snprintf(m + copied, SCRATCH_SIZE - copied, "TEMP TM4C: %s \t Threshold: %02d.%02d\r\n",
                 (stat & ALM_STAT_TM4C_OVERTEMP) ? "ALARM" : "GOOD", tens, frac);
    configASSERT(copied < SCRATCH_SIZE);

    return pdFALSE;
  }
  else if (strcmp(argv[1], "settemp") == 0) {
    if (argc != 4) {
      snprintf(m, s, "Invalid command\r\n");
      return pdFALSE;
    }
    float newtemp = (float)strtol(argv[3], NULL, 10);
    char *device = argv[2];
    if (!strcmp(device, "ff")) {
      setAlarmTemperature(FF, newtemp);
      snprintf(m, s, "%s: set Firefly alarm temperature to %s\r\n", argv[0], argv[3]);
      return pdFALSE;
    }
    if (!strcmp(device, "fpga")) {
      setAlarmTemperature(FPGA, newtemp);
      snprintf(m, s, "%s: set FPGA alarm temperature to %s\r\n", argv[0], argv[3]);
      return pdFALSE;
    }
    if (!strcmp(device, "dcdc")) {
      setAlarmTemperature(DCDC, newtemp);
      snprintf(m, s, "%s: set DCDC alarm temperature to %s\r\n", argv[0], argv[3]);
      return pdFALSE;
    }
    if (!strcmp(device, "tm4c")) {
      setAlarmTemperature(TM4C, newtemp);
      snprintf(m, s, "%s: set TM4C alarm temperature to %s\r\n", argv[0], argv[3]);
      return pdFALSE;
    }
    else {
      snprintf(m, s, "%s is not a valid device.\r\n", argv[2]);
      return pdFALSE;
    }
  }
  else {
    snprintf(m, s, "%s: invalid argument %s received\r\n", argv[0], argv[1]);
    return pdFALSE;
  }
  return pdFALSE;
}

static BaseType_t i2c_scan(int argc, char **argv)
{
  // takes no arguments
  int copied = 0;
  copied += snprintf(m, SCRATCH_SIZE - copied, "i2c bus scan\r\n");
  copied += snprintf(m + copied, SCRATCH_SIZE - copied,
                     "     0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f\r\n00:         ");
  for (uint8_t i = 0x3; i < 0x78; ++i) {
    uint8_t data;
    if (i % 16 == 0)
      copied += snprintf(m + copied, SCRATCH_SIZE - copied, "\r\n%2x:", i);
    tSMBusStatus r = SMBusMasterI2CRead(p_sMaster, i, &data, 1);
    if (r != SMBUS_OK) {
      Print("i2c_scan: Probe failed 1\r\n");
    }
    while (SMBusStatusGet(p_sMaster) == SMBUS_TRANSFER_IN_PROGRESS) {
      vTaskDelay(pdMS_TO_TICKS(10)); // wait
    }
    if (*p_eStatus == SMBUS_OK)
      copied += snprintf(m + copied, SCRATCH_SIZE - copied, " %2x", i);
    else
      copied += snprintf(m + copied, SCRATCH_SIZE - copied, " --");
    configASSERT(copied < SCRATCH_SIZE);
  }
  copied += snprintf(m + copied, SCRATCH_SIZE - copied, "\r\n");
  configASSERT(copied < SCRATCH_SIZE);

  return pdFALSE;
}

// send LED commands
static BaseType_t led_ctl(int argc, char **argv)
{

  BaseType_t i1 = strtol(argv[1], NULL, 10);
  configASSERT(i1 != 99);

  uint32_t message = HUH; // default: message not understood
  if (i1 == 0) {          // ToDo: make messages less clunky. break out color.
    message = RED_LED_OFF;
  }
  else if (i1 == 1) {
    message = RED_LED_ON;
  }
  else if (i1 == 2) {
    message = RED_LED_TOGGLE;
  }
  else if (i1 == 3) {
    message = RED_LED_TOGGLE3;
  }
  else if (i1 == 4) {
    message = RED_LED_TOGGLE4;
  }
  // Send a message to the LED task
  xQueueSendToBack(xLedQueue, &message, pdMS_TO_TICKS(10));
  m[0] = '\0'; // no output from this command

  return pdFALSE;
}

// dump monitor information
static BaseType_t psmon_ctl(int argc, char **argv)
{
  int s = SCRATCH_SIZE;
  BaseType_t i1 = strtol(argv[1], NULL, 10);

  if (i1 < 0 || i1 >= dcdc_args.n_commands) {
    snprintf(m, s, "%s: Invalid argument, must be between 0 and %d\r\n", argv[0],
             dcdc_args.n_commands - 1);
    return pdFALSE;
  }
  // update times, in seconds
  TickType_t now = pdTICKS_TO_MS(xTaskGetTickCount()) / 1000;
  TickType_t last = pdTICKS_TO_MS(dcdc_args.updateTick) / 1000;
  int copied = 0;
  if ((now - last) > 60) {
    int mins = (now - last) / 60;
    copied += snprintf(m + copied, SCRATCH_SIZE - copied,
                       "%s: stale data, last update %d minutes ago\r\n", argv[0], mins);
  }
  copied += snprintf(m + copied, SCRATCH_SIZE - copied, "%s\r\n", dcdc_args.commands[i1].name);
  for (int ps = 0; ps < dcdc_args.n_devices; ++ps) {
    copied +=
        snprintf(m + copied, SCRATCH_SIZE - copied, "SUPPLY %s\r\n", dcdc_args.devices[ps].name);
    for (int page = 0; page < dcdc_args.n_pages; ++page) {
      float val = dcdc_args.pm_values[ps * (dcdc_args.n_commands * dcdc_args.n_pages) +
                                      page * dcdc_args.n_commands + i1];
      int tens, frac;
      float_to_ints(val, &tens, &frac);
      copied += snprintf(m + copied, SCRATCH_SIZE - copied, "VALUE %02d.%02d\t", tens, frac);
    }
    copied += snprintf(m + copied, SCRATCH_SIZE - copied, "\r\n");
  }

  return pdFALSE;
}

// this command takes no arguments
static BaseType_t adc_ctl(int argc, char **argv)
{
  int copied = 0;

  static int whichadc = 0;
  if (whichadc == 0) {
    copied += snprintf(m + copied, SCRATCH_SIZE - copied, "ADC outputs\r\n");
  }
  for (; whichadc < 21; ++whichadc) {
    float val = getADCvalue(whichadc);
    int tens, frac;
    float_to_ints(val, &tens, &frac);
    copied += snprintf(m + copied, SCRATCH_SIZE - copied, "%14s: %02d.%02d\r\n",
                       getADCname(whichadc), tens, frac);
    if ((SCRATCH_SIZE - copied) < 20 && (whichadc < 20)) {
      ++whichadc;
      return pdTRUE;
    }
  }
  whichadc = 0;
  return pdFALSE;
}

// this command takes no arguments and never returns.
static BaseType_t bl_ctl(int argc, char **argv)
{
  Print("Jumping to boot loader.\r\n");
  ROM_SysCtlDelay(100000);
  // this code is copied from the JumpToBootLoader()
  // stack from the boot_demo1 application in the
  // ek-tm4c129exl part of tiva ware.
  //
  // We must make sure we turn off SysTick and its interrupt before entering
  // the boot loader!
  //
  ROM_SysTickIntDisable();
  ROM_SysTickDisable();

  //
  // Disable all processor interrupts.  Instead of disabling them
  // one at a time, a direct write to NVIC is done to disable all
  // peripheral interrupts.
  //
  HWREG(NVIC_DIS0) = 0xffffffff;
  HWREG(NVIC_DIS1) = 0xffffffff;
  HWREG(NVIC_DIS2) = 0xffffffff;
  HWREG(NVIC_DIS3) = 0xffffffff;

  //
  // Return control to the boot loader.  This is a call to the SVC
  // handler in the boot loader.
  //
  (*((void (*)(void))(*(uint32_t *)0x2c)))();

  // the above points to a memory location in flash.
  // shut up compiler warning. This will never get called
  return pdFALSE;
}

// this command takes one argument
static BaseType_t clock_ctl(int argc, char **argv)
{
  int copied = 0;
  int status = -1; // shut up clang compiler warning
  BaseType_t i = strtol(argv[1], NULL, 10);
  if (!((i == 1) || (i == 2))) {
    copied +=
        snprintf(m + copied, SCRATCH_SIZE - copied,
                 "Invalid mode %d for clock, only 1 (reset) and 2 (program) supported\r\n", i);
    return pdFALSE;
  }
  copied += snprintf(m + copied, SCRATCH_SIZE - copied, "%s mode set to %d. \r\n", argv[0], i);
  if (i == 1) {
    status = initialize_clock();
    if (status == 0)
      copied += snprintf(m + copied, SCRATCH_SIZE - copied,
                         "clock synthesizer successfully initialized. \r\n");
  }
  else if (i == 2) {
    status = load_clock();
    if (status == 0)
      copied += snprintf(m + copied, SCRATCH_SIZE - copied,
                         "clock synthesizer successfully programmed. \r\n");
  }
  if (status == -1)
    copied += snprintf(m + copied, SCRATCH_SIZE - copied, "%s operation failed (1). \r\n", argv[0]);
  else if (status == -2)
    copied += snprintf(m + copied, SCRATCH_SIZE - copied, "%s operation failed (2). \r\n", argv[0]);
  else if (status != 0)
    copied += snprintf(m + copied, SCRATCH_SIZE - copied, "%s invalid return value. \r\n", argv[0]);
  return pdFALSE;
}

// this command takes no arguments
static BaseType_t ver_ctl(int argc, char **argv)
{
  int copied = 0;
  copied += snprintf(m + copied, SCRATCH_SIZE - copied, "Version %s built at %s.\r\n", gitVersion(),
                     buildTime());
  configASSERT(copied < SCRATCH_SIZE);

  return pdFALSE;
}

// this command takes up to two arguments
static BaseType_t ff_ctl(int argc, char **argv)
{
  // argument handling
  int copied = 0;
  static int whichff = 0;

  if (whichff == 0) {
    // check for stale data
    TickType_t now = pdTICKS_TO_MS(xTaskGetTickCount()) / 1000;
    TickType_t last = pdTICKS_TO_MS(getFFupdateTick()) / 1000;
    if ((now - last) > 60) {
      int mins = (now - last) / 60;
      copied += snprintf(m + copied, SCRATCH_SIZE - copied,
                         "%s: stale data, last update %d minutes ago\r\n", argv[0], mins);
    }
  }
  // parse command based on how many arguments it has
  if (argc == 1) { // default command: temps
    uint32_t ff_config = read_eeprom_single(EEPROM_ID_FF_ADDR);
    if (whichff == 0) {
      copied += snprintf(m + copied, SCRATCH_SIZE - copied, "FF temperatures\r\n");
    }
    for (; whichff < NFIREFLIES; ++whichff) {
      int8_t val = getFFtemp(whichff);
      const char *name = getFFname(whichff);
      if ((1 << whichff) & ff_config) // val > 0 )
        copied += snprintf(m + copied, SCRATCH_SIZE - copied, "%17s: %2d", name, val);
      else // dummy value
        copied += snprintf(m + copied, SCRATCH_SIZE - copied, "%17s: %2s", name, "--");
      bool isTx = (strstr(name, "Tx") != NULL);
      if (isTx)
        copied += snprintf(m + copied, SCRATCH_SIZE - copied, "\t");
      else
        copied += snprintf(m + copied, SCRATCH_SIZE - copied, "\r\n");
      if ((SCRATCH_SIZE - copied) < 20) {
        ++whichff;
        return pdTRUE;
      }
    }
    if (whichff % 2 == 1) {
      m[copied++] = '\r';
      m[copied++] = '\n';
      m[copied] = '\0';
    }
    whichff = 0;
  } // argc == 1
  else {
    int whichFF = 0;
    // handle the channel number first
    if (strncmp(argv[argc - 1], "all", 3) == 0) {
      whichFF = NFIREFLIES;
    }
    else { // commands with arguments. The last argument is always which FF module.
      whichFF = strtol(argv[argc - 1], NULL, 10);
      if (whichFF >= NFIREFLIES || (whichFF == 0 && strncmp(argv[argc - 1], "0", 1) != 0)) {
        snprintf(m + copied, SCRATCH_SIZE - copied, "%s: choose ff number less than %d\r\n",
                 argv[0], NFIREFLIES);
        return pdFALSE;
      }
    }
    // now process various commands.
    if (argc == 4) { // command + three arguments
      bool receiveAnswer = false;
      uint8_t code;
      uint32_t data = (whichFF & FF_MESSAGE_CODE_REG_FF_MASK) << FF_MESSAGE_CODE_REG_FF_OFFSET;
      ;
      ;
      if (strncmp(argv[1], "cdr", 3) == 0) {
        code = FFLY_DISABLE_CDR; // default: disable
        if (strncmp(argv[2], "on", 2) == 0) {
          code = FFLY_ENABLE_CDR;
        }
      }
      else if (strncmp(argv[1], "xmit", 4) == 0) {
        code = FFLY_DISABLE_TRANSMITTER;
        if (strncmp(argv[2], "on", 2) == 0) {
          code = FFLY_ENABLE_TRANSMITTER;
        }
      }
      else if (strncmp(argv[1], "rcvr", 4) == 0) {
        code = FFLY_DISABLE;
        if (strncmp(argv[2], "on", 2) == 0) {
          code = FFLY_ENABLE;
        }
      }
      // Add here
      else if (strncmp(argv[1], "regr", 4) == 0) {
        code = FFLY_READ_REGISTER;
        receiveAnswer = true;
        if (whichFF == NFIREFLIES) {
          copied += snprintf(m + copied, SCRATCH_SIZE - copied, "%s: cannot read all registers\r\n",
                             argv[0]);
          return pdFALSE;
        }
        // register number
        uint8_t regnum = strtol(argv[2], NULL, 16);
        copied +=
            snprintf(m + copied, SCRATCH_SIZE - copied, "%s: reading FF %s, register 0x%x\r\n",
                     argv[0], getFFname(whichFF), regnum);
        // pack channel and register into the data
        data |= ((regnum & FF_MESSAGE_CODE_REG_REG_MASK) << FF_MESSAGE_CODE_REG_REG_OFFSET);
      }
      else {
        snprintf(m + copied, SCRATCH_SIZE - copied, "%s: command %s not recognized\r\n", argv[0],
                 argv[1]);
        return pdFALSE;
      }
      uint32_t message =
          ((code & FF_MESSAGE_CODE_MASK) << FF_MESSAGE_CODE_OFFSET) | (data & FF_MESSAGE_DATA_MASK);
      xQueueSendToBack(xFFlyQueueIn, &message, pdMS_TO_TICKS(10));
      snprintf(m + copied, SCRATCH_SIZE - copied, "%s: command %s %s sent.\r\n", argv[0], argv[1],
               argv[2]);
      if (receiveAnswer) {
        BaseType_t f = xQueueReceive(xFFlyQueueOut, &message, pdMS_TO_TICKS(500));
        if (f == pdTRUE)
          snprintf(m + copied, SCRATCH_SIZE - copied, "%s: Command returned 0x%x.\r\n", argv[0],
                   message & 0xFFU);
        else
          snprintf(m + copied, SCRATCH_SIZE - copied, "%s: Command failed.\r\n", argv[0]);
      }
    }                     // argc == 4
    else if (argc == 5) { // command + five arguments
      // register write. model:
      // ff regw reg# val (0-23|all)
      // register read/write commands
      if (strncmp(argv[1], "regw", 4) == 0) {
        uint8_t code = FFLY_WRITE_REGISTER;
        // the two additional arguments
        // register number
        // value to be written
        uint8_t regnum = strtol(argv[2], NULL, 16);
        uint16_t value = strtol(argv[3], NULL, 16);
        uint8_t channel = whichFF;
        if (channel == NFIREFLIES) {
          channel = 0; // silently fall back to first channel
        }
        // pack channel, register and value into the data
        uint32_t data =
            ((regnum & FF_MESSAGE_CODE_REG_REG_MASK) << FF_MESSAGE_CODE_REG_REG_OFFSET) |
            ((value & FF_MESSAGE_CODE_REG_DAT_MASK) << FF_MESSAGE_CODE_REG_DAT_OFFSET) |
            ((channel & FF_MESSAGE_CODE_REG_FF_MASK) << FF_MESSAGE_CODE_REG_FF_OFFSET);
        uint32_t message = ((code & FF_MESSAGE_CODE_MASK) << FF_MESSAGE_CODE_OFFSET) |
                           ((data & FF_MESSAGE_DATA_MASK) << FF_MESSAGE_DATA_OFFSET);
        xQueueSendToBack(xFFlyQueueIn, &message, pdMS_TO_TICKS(10));
        snprintf(m + copied, SCRATCH_SIZE - copied,
                 "%s: write val 0x%x to register 0x%x, FF %d.\r\n", argv[0], value, regnum,
                 channel);

      } // end regw
      else {
        snprintf(m + copied, SCRATCH_SIZE - copied, "%s: command %s not understood\r\n", argv[0],
                 argv[1]);
        return pdFALSE;
      }
    } // argc == 5
    else {
      snprintf(m + copied, SCRATCH_SIZE - copied, "%s: command %s not understood\r\n", argv[0],
               argv[1]);
      return pdFALSE;
    }
  }
  return pdFALSE;
}

// this command takes up to one argument
static BaseType_t fpga_ctl(int argc, char **argv)
{
  if (argc == 2) {
    if (strncmp(argv[1], "done", 4) == 0) { // print out value of done pins
      int ku_done_ = read_gpio_pin(_K_FPGA_DONE);
      int vu_done_ = read_gpio_pin(_V_FPGA_DONE);
      snprintf(m, SCRATCH_SIZE, "KU_DONE* = %d\r\nVU_DONE* = %d\r\n", ku_done_, vu_done_);
      return pdFALSE;
    }
    else {
      snprintf(m, SCRATCH_SIZE, "%s: invalid command %s\r\n", argv[0], argv[1]);
      return pdFALSE;
    }
  }
  else if (argc != 1) {
    // error, invalid
    snprintf(m, SCRATCH_SIZE, "%s: invalid argument count %d\r\n", argv[0], argc);
    return pdFALSE;
  }
  else {
    int copied = 0;
    static int whichfpga = 0;
    int howmany = fpga_args.n_devices * fpga_args.n_pages;
    if (whichfpga == 0) {
      TickType_t now = pdTICKS_TO_MS(xTaskGetTickCount()) / 1000;
      TickType_t last = pdTICKS_TO_MS(getFFupdateTick()) / 1000;
      if ((now - last) > 60) {
        int mins = (now - last) / 60;
        copied += snprintf(m + copied, SCRATCH_SIZE - copied,
                           "%s: stale data, last update %d minutes ago\r\n", argv[0], mins);
      }

      copied += snprintf(m + copied, SCRATCH_SIZE - copied, "FPGA monitors\r\n");
      copied += snprintf(m + copied, SCRATCH_SIZE - copied, "%s\r\n", fpga_args.commands[0].name);
    }

    for (; whichfpga < howmany; ++whichfpga) {
      float val = fpga_args.pm_values[whichfpga];
      int tens, frac;
      float_to_ints(val, &tens, &frac);

      copied += snprintf(m + copied, SCRATCH_SIZE - copied, "%5s: %02d.%02d",
                         fpga_args.devices[whichfpga].name, tens, frac);
      if (whichfpga % 2 == 1)
        copied += snprintf(m + copied, SCRATCH_SIZE - copied, "\r\n");
      else
        copied += snprintf(m + copied, SCRATCH_SIZE - copied, "\t");
      if ((SCRATCH_SIZE - copied) < 20) {
        ++whichfpga;
        return pdTRUE;
      }
    }
    if (whichfpga % 2 == 1) {
      m[copied++] = '\r';
      m[copied++] = '\n';
      m[copied] = '\0';
    }
    whichfpga = 0;
    return pdFALSE;
  }
}
#include "common/smbus_units.h"
void snapdump(struct dev_i2c_addr_t *add, uint8_t page, uint8_t snapshot[32], bool reset);

typedef struct __attribute__((packed)) {
  linear11_val_t v_in;
  uint16_t v_out;
  linear11_val_t i_out;
  linear11_val_t i_out_max;
  linear11_val_t duty_cycle;
  linear11_val_t temperature;
  linear11_val_t unused1;
  linear11_val_t freq;
  uint8_t v_out_status;
  uint8_t i_out_status;
  uint8_t input_status;
  uint8_t temperature_status;
  uint8_t cml_status;
  uint8_t mfr_status;
  uint8_t flash_status;
  uint8_t unused[9];
} snapshot_t;

extern struct dev_i2c_addr_t pm_addrs_dcdc[];

static BaseType_t snapshot(int argc, char **argv)
{
  _Static_assert(sizeof(snapshot_t) == 32, "sizeof snapshot_t");
  int copied = 0;
  int page = strtol(argv[1], NULL, 10); // which LGA08D
  int which = page / 10;
  page = page % 10;
  if (page < 0 || page > 1) {
    copied += snprintf(m + copied, SCRATCH_SIZE - copied, "%s: page %d must be between 0-1\r\n",
                       argv[0], page);
    return pdFALSE;
  }
  if (which < 0 || which > 4) {
    copied += snprintf(m + copied, SCRATCH_SIZE - copied, "%s: device %d must be between 0-4\r\n",
                       argv[0], which);
    return pdFALSE;
  }
  copied += snprintf(m + copied, SCRATCH_SIZE - copied, "%s: page %d of device %s\r\n", argv[0],
                     page, pm_addrs_dcdc[which].name);

  bool reset = false;
  int ireset = strtol(argv[2], NULL, 10);
  if (ireset == 1)
    reset = true;

  uint8_t sn[32];
  snapdump(&pm_addrs_dcdc[which], page, sn, reset);
  snapshot_t *p0 = (snapshot_t *)&sn[0];
  int tens, fraction;
  float_to_ints(linear11_to_float(p0->v_in), &tens, &fraction);
  copied += snprintf(m + copied, SCRATCH_SIZE - copied, "VIN  = %d.%02d\r\n", tens, fraction);
  float_to_ints(linear16u_to_float(p0->v_out), &tens, &fraction);
  copied += snprintf(m + copied, SCRATCH_SIZE - copied, "VOUT = %d.%02d\r\n", tens, fraction);
  float_to_ints(linear11_to_float(p0->i_out), &tens, &fraction);
  copied += snprintf(m + copied, SCRATCH_SIZE - copied, "IOUT = %d.%02d\r\n", tens, fraction);
  float_to_ints(linear11_to_float(p0->i_out_max), &tens, &fraction);
  copied += snprintf(m + copied, SCRATCH_SIZE - copied, "IOUT MAX = %d.%02d\r\n", tens, fraction);
  float_to_ints(linear11_to_float(p0->duty_cycle), &tens, &fraction);
  copied += snprintf(m + copied, SCRATCH_SIZE - copied, "duty cycle = %d.%02d\r\n", tens, fraction);
  float_to_ints(linear11_to_float(p0->temperature), &tens, &fraction);
  copied += snprintf(m + copied, SCRATCH_SIZE - copied, "TEMP = %d.%02d\r\n", tens, fraction);
  float_to_ints(linear11_to_float(p0->freq), &tens, &fraction);
  copied +=
      snprintf(m + copied, SCRATCH_SIZE - copied, "switching freq = %d.%02d\r\n", tens, fraction);
  copied +=
      snprintf(m + copied, SCRATCH_SIZE - copied, "VOUT  STATUS: 0x%02x\r\n", p0->v_out_status);
  copied +=
      snprintf(m + copied, SCRATCH_SIZE - copied, "IOUT  STATUS: 0x%02x\r\n", p0->i_out_status);
  copied +=
      snprintf(m + copied, SCRATCH_SIZE - copied, "INPUT STATUS: 0x%02x\r\n", p0->input_status);
  copied += snprintf(m + copied, SCRATCH_SIZE - copied, "TEMP  STATUS: 0x%02x\r\n",
                     p0->temperature_status);
  copied += snprintf(m + copied, SCRATCH_SIZE - copied, "CML   STATUS: 0x%02x\r\n", p0->cml_status);
  copied += snprintf(m + copied, SCRATCH_SIZE - copied, "MFR   STATUS: 0x%02x\r\n", p0->mfr_status);
  copied +=
      snprintf(m + copied, SCRATCH_SIZE - copied, "flash STATUS: 0x%02x\r\n", p0->flash_status);
  return pdFALSE;
}

// this command takes no arguments since there is only one command
// right now.
static BaseType_t sensor_summary(int argc, char **argv)
{
  int copied = 0;
  // collect all sensor information
  // highest temperature for each
  // Firefly
  // FPGA
  // DCDC
  // TM4C
  float tm4c_temp = getADCvalue(ADC_INFO_TEMP_ENTRY);
  int tens, frac;
  float_to_ints(tm4c_temp, &tens, &frac);
  copied += snprintf(m + copied, SCRATCH_SIZE - copied, "MCU %02d.%02d\r\n", tens, frac);
  // Fireflies. These are reported as ints but we are asked
  // to report a float.
  int8_t imax_temp = -99.0;
  for (int i = 0; i < NFIREFLIES; ++i) {
    int8_t v = getFFtemp(i);
    if (v > imax_temp)
      imax_temp = v;
  }
  copied += snprintf(m + copied, SCRATCH_SIZE - copied, "FIREFLY %02d.0\r\n", imax_temp);
  // FPGAs.
  float max_fpga;
  if (fpga_args.n_devices == 2)
    max_fpga = MAX(fpga_args.pm_values[0], fpga_args.pm_values[1]);
  else
    max_fpga = fpga_args.pm_values[0];
  float_to_ints(max_fpga, &tens, &frac);
  copied += snprintf(m + copied, SCRATCH_SIZE - copied, "FPGA %02d.%02d\r\n", tens, frac);

  // DCDC. The first command is READ_TEMPERATURE_1.
  // I am assuming it stays that way!!!!!!!!
  float max_temp = -99.0;
  for (int ps = 0; ps < dcdc_args.n_devices; ++ps) {
    for (int page = 0; page < dcdc_args.n_pages; ++page) {
      float thistemp = dcdc_args.pm_values[ps * (dcdc_args.n_commands * dcdc_args.n_pages) +
                                           page * dcdc_args.n_commands + 0];
      if (thistemp > max_temp)
        max_temp = thistemp;
    }
  }
  float_to_ints(max_temp, &tens, &frac);
  copied += snprintf(m + copied, SCRATCH_SIZE - copied, "REG %02d.%02d\r\n", tens, frac);

  return pdFALSE;
}

// This command takes no arguments
static BaseType_t ff_status(int argc, char **argv)
{
  int copied = 0;

  static int whichff = 0;
  if (whichff == 0) {
    copied += snprintf(m + copied, SCRATCH_SIZE - copied, "FIREFLY STATUS:\r\n");
  }
  for (; whichff < 25; ++whichff) {
    int8_t status = getFFstatus(whichff);
    const char *name = getFFname(whichff);
    copied += snprintf(m + copied, SCRATCH_SIZE - copied, "%s %02d", name, status);

    bool isTx = (strstr(name, "Tx") != NULL);
    if (isTx)
      copied += snprintf(m + copied, SCRATCH_SIZE - copied, "\t");
    else
      copied += snprintf(m + copied, SCRATCH_SIZE - copied, "\r\n");

    if ((SCRATCH_SIZE - copied) < 20 && (whichff < 25)) {
      ++whichff;
      return pdTRUE;
    }
  }
  whichff = 0;
  return pdFALSE;
}

// This command takes no arguments
static BaseType_t restart_mcu(int argc, char **argv)
{
  int copied = 0;
  copied += snprintf(m + copied, SCRATCH_SIZE - copied, "Restarting MCU\r\n");
  MAP_SysCtlReset(); // This function does not return
  return pdFALSE;
}

// This command takes 1 argument, either k or v
static BaseType_t fpga_reset(int argc, char **argv)
{
  int copied = 0;
  const TickType_t delay = 1 / portTICK_PERIOD_MS; // 1 ms delay

  if (strcmp(argv[1], "v") == 0) {
    write_gpio_pin(V_FPGA_PROGRAM, 0x1);
    vTaskDelay(delay);
    write_gpio_pin(V_FPGA_PROGRAM, 0x0);
    copied += snprintf(m + copied, SCRATCH_SIZE - copied, "VU7P has been reset\r\n");
  }
  if (strcmp(argv[1], "k") == 0) {
    write_gpio_pin(K_FPGA_PROGRAM, 0x1);
    vTaskDelay(delay);
    write_gpio_pin(K_FPGA_PROGRAM, 0x0);
    copied += snprintf(m + copied, SCRATCH_SIZE - copied, "KU15P has been reset\r\n");
  }
  return pdFALSE;
}

// This command takes 1 arg, the address
static BaseType_t eeprom_read(int argc, char **argv)
{
  int copied = 0;

  uint32_t addr;
  addr = strtol(argv[1], NULL, 16);
  uint32_t block = EEPROMBlockFromAddr(addr);

  uint64_t data = read_eeprom_multi(addr);
  uint32_t data2 = (uint32_t)(data >> 32);
  uint32_t data1 = (uint32_t)data;
  copied += snprintf(m + copied, SCRATCH_SIZE - copied,
                     "Data read from EEPROM block %d: %08x %08x\r\n", block, data1, data2);

  return pdFALSE;
}

// This command takes 2 args, the address and 4 bytes of data to be written
static BaseType_t eeprom_write(int argc, char **argv)
{
  int copied = 0;

  uint32_t data, addr;
  data = strtoul(argv[2], NULL, 16);
  addr = strtoul(argv[1], NULL, 16);
  uint32_t block = EEPROMBlockFromAddr(addr);
  if ((block == 1) || ((EBUF_MINBLK <= block) && (block <= EBUF_MAXBLK))) {
    copied += snprintf(m + copied, SCRATCH_SIZE - copied, "Please choose available block\r\n");
    return pdFALSE;
  }
  write_eeprom(data, addr);
  copied += snprintf(m + copied, SCRATCH_SIZE - copied, "Data written to EEPROM block %d: %08x\r\n",
                     block, data);

  return pdFALSE;
}

// Takes 0 arguments
static BaseType_t eeprom_info(int argc, char **argv)
{
  int copied = 0;

  copied +=
      snprintf(m + copied, SCRATCH_SIZE - copied, "EEPROM has 96 blocks of 64 bytes each.\r\n");
  copied += snprintf(m + copied, SCRATCH_SIZE - copied, "Block 0 \t 0x0000-0x0040 \t Free.\r\n");
  copied += snprintf(m + copied, SCRATCH_SIZE - copied,
                     "Block 1 \t 0x0040-0x007c \t Apollo ID Information. Password: 0x12345678\r\n");
  copied += snprintf(m + copied, SCRATCH_SIZE - copied,
                     "Blocks %u-%u \t 0x%04x-0x%04x \t Error buffer.\r\n", EBUF_MINBLK, EBUF_MAXBLK,
                     EEPROMAddrFromBlock(EBUF_MINBLK), EEPROMAddrFromBlock(EBUF_MAXBLK + 1) - 4);

  return pdFALSE;
}

// Takes 3 arguments
static BaseType_t set_board_id(int argc, char **argv)
{
  int copied = 0;

  uint64_t pass, addr, data;
  pass = strtoul(argv[1], NULL, 16);
  addr = strtoul(argv[2], NULL, 16);
  data = strtoul(argv[3], NULL, 16);
  uint64_t block = EEPROMBlockFromAddr(addr);
  if (block != 1) {
    copied += snprintf(m + copied, SCRATCH_SIZE - copied, "Please input address in Block 1\r\n");
    return pdFALSE;
  }

  uint64_t unlock = EPRMMessage((uint64_t)EPRM_UNLOCK_BLOCK, block, pass);
  xQueueSendToBack(xEPRMQueue_in, &unlock, portMAX_DELAY);

  uint64_t message = EPRMMessage((uint64_t)EPRM_WRITE_SINGLE, addr, data);
  xQueueSendToBack(xEPRMQueue_in, &message, portMAX_DELAY);

  uint64_t lock = EPRMMessage((uint64_t)EPRM_LOCK_BLOCK, block << 32, 0);
  xQueueSendToBack(xEPRMQueue_in, &lock, portMAX_DELAY);

  if (pass != 0x12345678) {
    copied += snprintf(m + copied, SCRATCH_SIZE - copied,
                       "Wrong password. Type eeprom_info to get password.");
  } // data not printing correctly?

  return pdFALSE;
}

// one-time use, has one function and takes 0 arguments
static BaseType_t set_board_id_password(int argc, char **argv)
{
  int copied = 0;

  uint64_t message = EPRMMessage((uint64_t)EPRM_PASS_SET, 0, 0x12345678);
  xQueueSendToBack(xEPRMQueue_in, &message, portMAX_DELAY);

  copied += snprintf(m + copied, SCRATCH_SIZE - copied, "Block locked\r\n");

  return pdFALSE;
}

static BaseType_t board_id_info(int argc, char **argv)
{
  int copied = 0;
  ;

  uint32_t sn = read_eeprom_single(EEPROM_ID_SN_ADDR);
  uint32_t ff = read_eeprom_single(EEPROM_ID_FF_ADDR);

  uint32_t num = (uint32_t)sn >> 16;
  uint32_t rev = ((uint32_t)sn) & 0xff;

  copied += snprintf(m + copied, SCRATCH_SIZE - copied, "ID:%08x\r\n", (uint32_t)sn);

  copied += snprintf(m + copied, SCRATCH_SIZE - copied, "Board number: %x\r\n", num);
  copied += snprintf(m + copied, SCRATCH_SIZE - copied, "Revision: %x\r\n", rev);
  copied += snprintf(m + copied, SCRATCH_SIZE - copied, "Firefly config: %x\r\n", ff);

  return pdFALSE;
}

// This command takes 1 arg, the data to be written to the buffer
static BaseType_t errbuff_in(int argc, char **argv)
{
  int copied = 0;

  uint32_t data;
  data = strtoul(argv[1], NULL, 16);
  errbuffer_put(data, 0);
  copied +=
      snprintf(m + copied, SCRATCH_SIZE - copied, "Data written to EEPROM buffer: %x\r\n", data);

  return pdFALSE;
}
#define EBUFFOUT_MAX_ENTRIES 64
static BaseType_t errbuff_out(int argc, char **argv)
{
  int copied = 0;

  uint32_t num = strtoul(argv[1], NULL, 10);
  if (num > EBUFFOUT_MAX_ENTRIES) {
    copied += snprintf(m + copied, SCRATCH_SIZE - copied, "Please enter a number 64 or less \r\n");
    return pdFALSE;
  }
  uint32_t arr[EBUFFOUT_MAX_ENTRIES];
  uint32_t(*arrptr)[EBUFFOUT_MAX_ENTRIES] = &arr;
  errbuffer_get(num, arrptr);

  static int i = 0;
  if (i == 0) {
    copied += snprintf(m + copied, SCRATCH_SIZE - copied, "Entries in EEPROM buffer:\r\n");
  }
  for (; i < num; ++i) {
    uint32_t word = (*arrptr)[i];
    uint16_t errcode = EBUF_ERRCODE(word);
    // if this is a continuation and it's not the first entry we see
    if (errcode == EBUF_CONTINUATION && i != 0) {
      uint16_t errdata = EBUF_DATA(word);
      copied += snprintf(m + copied, SCRATCH_SIZE - copied, " 0x%02x", errdata);
    }
    else {
      copied += errbuffer_get_messagestr(word, m + copied, SCRATCH_SIZE - copied);
    }
    if ((SCRATCH_SIZE - copied) < 30 && (i < num)) { // catch when buffer is almost full
      ++i;
      return pdTRUE;
    }
  }
  copied += snprintf(m + copied, SCRATCH_SIZE - copied, "\r\n");
  i = 0;
  return pdFALSE;
}

// Takes no arguments
static BaseType_t errbuff_info(int argc, char **argv)
{
  int copied = 0;
  uint32_t cap, minaddr, maxaddr, head;
  uint16_t last, counter, n_continue;

  cap = errbuffer_capacity();
  minaddr = errbuffer_minaddr();
  maxaddr = errbuffer_maxaddr();
  head = errbuffer_head();
  last = errbuffer_last();
  counter = errbuffer_counter();
  n_continue = errbuffer_continue();

  copied += snprintf(m + copied, SCRATCH_SIZE - copied, "Capacity:        %d words\r\n", cap);
  copied += snprintf(m + copied, SCRATCH_SIZE - copied, "Min address:     0x%08x\r\n", minaddr);
  copied += snprintf(m + copied, SCRATCH_SIZE - copied, "Max address:     0x%08x\r\n", maxaddr);
  copied += snprintf(m + copied, SCRATCH_SIZE - copied, "Head address:    0x%08x\r\n", head);
  copied += snprintf(m + copied, SCRATCH_SIZE - copied, "Last entry:      0x%0x\r\n", last);
  copied += snprintf(m + copied, SCRATCH_SIZE - copied, "Message counter: %d\r\n", counter);
  copied += snprintf(m + copied, SCRATCH_SIZE - copied, "Continue codes:  %d\r\n", n_continue);

  return pdFALSE;
}

// Takes no arguments
static BaseType_t errbuff_reset(int argc, char **argv)
{
  errbuffer_reset();
  return pdFALSE;
}

// takes no arguments
static BaseType_t stack_ctl(int argc, char **argv)
{
  int copied = 0;
  int i = SystemStackWaterHighWaterMark();
  copied += snprintf(m + copied, SCRATCH_SIZE - copied, "stack: %d of %d untouched\r\n", i,
                     SYSTEM_STACK_SIZE);
  return pdFALSE;
}

static void TaskGetRunTimeStats(char *pcWriteBuffer, size_t bufferLength)
{
  TaskStatus_t *pxTaskStatusArray;
  volatile UBaseType_t uxArraySize, x;
  uint32_t ulTotalRunTime;

  // Make sure the write buffer does not contain a string.
  *pcWriteBuffer = 0x00;

  // Take a snapshot of the number of tasks in case it changes while this
  // function is executing.
  uxArraySize = uxTaskGetNumberOfTasks();

  // Allocate a TaskStatus_t structure for each task.  An array could be
  // allocated statically at compile time.
  pxTaskStatusArray = pvPortMalloc(uxArraySize * sizeof(TaskStatus_t));

  if (pxTaskStatusArray != NULL) {
    // Generate raw status information about each task.
    uxArraySize = uxTaskGetSystemState(pxTaskStatusArray, uxArraySize, &ulTotalRunTime);

    // For percentage calculations.
    ulTotalRunTime /= 100UL;

    // Avoid divide by zero errors.
    if (ulTotalRunTime > 0) {
      // For each populated position in the pxTaskStatusArray array,
      // format the raw data as human readable ASCII data
      for (x = 0; x < uxArraySize; x++) {
        // What percentage of the total run time has the task used?
        // This will always be rounded down to the nearest integer.
        // ulTotalRunTimeDiv100 has already been divided by 100.
        uint32_t ulStatsAsPercentage = pxTaskStatusArray[x].ulRunTimeCounter / ulTotalRunTime;

        if (ulStatsAsPercentage > 0UL) {
          snprintf(pcWriteBuffer, bufferLength, "%s\t%12u\t%2u%%\r\n",
                   pxTaskStatusArray[x].pcTaskName, pxTaskStatusArray[x].ulRunTimeCounter,
                   ulStatsAsPercentage);
        }
        else {
          // If the percentage is zero here then the task has
          // consumed less than 1% of the total run time.
          snprintf(pcWriteBuffer, bufferLength, "%s\t%12u\t<1%%\r\n",
                   pxTaskStatusArray[x].pcTaskName, pxTaskStatusArray[x].ulRunTimeCounter);
        }
        size_t added = strlen((char *)pcWriteBuffer);
        pcWriteBuffer += added;
        bufferLength -= added;
      }
    }

    // The array is no longer needed, free the memory it consumes.
    vPortFree(pxTaskStatusArray);
  }
}

static BaseType_t uptime(int argc, char **argv)
{
  int s = SCRATCH_SIZE;
  TickType_t now = xTaskGetTickCount() / (configTICK_RATE_HZ * 60); // time in minutes
  snprintf(m, s, "%s: MCU uptime %d minutes\r\n", argv[0], now);
  return pdFALSE;
}

#pragma GCC diagnostic pop
// WARNING: this command easily leads to stack overflows. It does not correctly
// ensure that there are no overwrites to pcCommandString.
static BaseType_t TaskStatsCommand(int argc, char **argv)
{
  int s = SCRATCH_SIZE;
  const char *const pcHeader = "            Time     %\r\n"
                               "********************************\r\n";
  BaseType_t xSpacePadding;
  int copied = 0;
  char *mm = m;
  /* Generate a table of task stats. */
  strncpy(mm, "Task", s);
  mm += strlen(m);
  copied += strlen(m);

  /* Minus three for the null terminator and half the number of characters in
  "Task" so the column lines up with the centre of the heading. */
  configASSERT(configMAX_TASK_NAME_LEN > 3);
  for (xSpacePadding = strlen("Task"); xSpacePadding < (configMAX_TASK_NAME_LEN - 3);
       xSpacePadding++) {
    /* Add a space to align columns after the task's name. */
    *mm = ' ';
    mm++;
    ++copied;

    /* Ensure always terminated. */
    *mm = 0x00;
  }
  strncpy(mm, pcHeader, SCRATCH_SIZE - copied);
  copied += strlen(pcHeader);
  TaskGetRunTimeStats(mm + strlen(pcHeader), SCRATCH_SIZE - copied);

  /* There is no more data to return after this single string, so return
  pdFALSE. */
  return pdFALSE;
}

static BaseType_t help_command_fcn(int argc, char **);

static BaseType_t suart_ctl(int argc, char **argv)
{
  int s = SCRATCH_SIZE, copied = 0;
  bool understood = true;
  uint32_t message = 0;
  if (argc == 2) {
    if (strncmp(argv[1], "on", 2) == 0) {
      message = ZYNQMON_ENABLE_TRANSMIT;
      copied += snprintf(m + copied, SCRATCH_SIZE - copied, "%s: transmit on\r\n", argv[0]);
    }
    else if (strncmp(argv[1], "off", 3) == 0) {
      message = ZYNQMON_DISABLE_TRANSMIT;
      copied += snprintf(m + copied, SCRATCH_SIZE - copied, "%s: transmit off\r\n", argv[0]);
    }
#ifdef ZYNQMON_TEST_MODE
    else if (strncmp(argv[1], "debug1", 6) == 0) {
      message = ZYNQMON_TEST_SINGLE;
      copied +=
          snprintf(m + copied, SCRATCH_SIZE - copied, "%s: debug mode 1 (single)\r\n", argv[0]);
    }
    else if (strncmp(argv[1], "debugraw", 8) == 0) {
      message = ZYNQMON_TEST_RAW;
      copied += snprintf(m + copied, SCRATCH_SIZE - copied, "%s: debug raw (single)\r\n", argv[0]);
    }
    else if (strncmp(argv[1], "debug2", 6) == 0) {
      message = ZYNQMON_TEST_INCREMENT;
      copied +=
          snprintf(m + copied, SCRATCH_SIZE - copied, "%s: debug mode 2 (increment)\r\n", argv[0]);
    }
    else if (strncmp(argv[1], "normal", 5) == 0) {
      message = ZYNQMON_TEST_OFF;
      copied += snprintf(m + copied, SCRATCH_SIZE - copied, "%s: regular mode\r\n", argv[0]);
    }
    else if (strncmp(argv[1], "sendone", 7) == 0) {
      message = ZYNQMON_TEST_SEND_ONE;
      copied += snprintf(m + copied, SCRATCH_SIZE - copied, "%s: send one\r\n", argv[0]);
    }
    else if (strncmp(argv[1], "status", 5) == 0) {
      uint8_t mode = getZYNQMONTestMode();
      uint8_t sensor = getZYNQMONTestSensor();
      uint16_t data = getZYNQMONTestData();
      copied += snprintf(m + copied, SCRATCH_SIZE - copied,
                         "%s: test mode = %s, sensor = 0x%x, data = 0x%x\r\n", argv[0],
                         mode == 0 ? "single" : "increment", sensor, data);
    }
#endif // ZYNQMON_TEST_MODE
    else {
      understood = false;
    }
  }
#ifdef ZYNQMON_TEST_MODE
  else if (argc == 4) {
    if (strncmp(argv[1], "settest", 7) == 0) {
      uint8_t sensor = strtol(argv[2], NULL, 16);
      uint16_t data = strtol(argv[3], NULL, 16);
      setZYNQMONTestData(sensor, data);
      copied += snprintf(m + copied, SCRATCH_SIZE - copied,
                         "%s: set test sensor, data to 0x%x, 0x%x\r\n", argv[0], sensor, data);
    }
    else {
      understood = false;
    }
  }
#endif // ZYNQMON_TEST_MODE
  else {
    understood = false;
  }

  if (!understood) {
    snprintf(m, s, "%s: message %s not understood\r\n", argv[0], argv[1]);
    return pdFALSE;
  }

  if (message) {
    copied +=
        snprintf(m + copied, SCRATCH_SIZE - copied, "%s: Sending message %s\r\n", argv[0], argv[1]);
    // Send a message to the SUART task
    xQueueSendToBack(xZynqMonQueue, &message, pdMS_TO_TICKS(10));
  }
  return pdFALSE;
}

static const char *const pcWelcomeMessage =
    "CLI based on microrl.\r\nType \"help\" to view a list of registered commands.\r\n";

struct command_t {
  const char *commandstr;
  BaseType_t (*interpreter)(int argc, char **);
  const char *helpstr;
  const int num_args;
};

#define NUM_COMMANDS (sizeof(commands) / sizeof(commands[0]))
static struct command_t commands[] = {
    {"adc", adc_ctl, "adc\r\n Displays a table showing the state of ADC inputs.\r\n", 0},
    {"alm", alarm_ctl, "alm (clear|status|settemp #)\r\n Get or clear status of alarm task.\r\n",
     -1},
    {"bootloader", bl_ctl, "bootloader\r\n Call the boot loader\r\n", 0},
    {"clock", clock_ctl,
     "clock\r\n Reset (1) or program the clock synthesizer to 156.25 MHz (2).\r\n", 1},
    {"eeprom_info", eeprom_info, "eeprom_info\r\n Prints information about the EEPROM.\r\n", 0},
    {"eeprom_read", eeprom_read,
     "eeprom_read <address>\r\n Reads 4 bytes from EEPROM. Address should be a multiple of 4.\r\n",
     1},
    {"eeprom_write", eeprom_write,
     "eeprom_write <address> <data>\r\n Writes <data> to <address> in EEPROM. <address> should be "
     "a "
     "multiple of 4.\r\n",
     2},
    {"errorlog_entry", errbuff_in,
     "errorlog_entry <data>\r\n Manual entry of 2-byte code into the eeprom error logger.\r\n", 1},
    {"errorlog", errbuff_out,
     "errorlog <n>\r\n Prints last n entries in the eeprom error logger.\r\n", 1},
    {"errorlog_info", errbuff_info,
     "errorlog_info\r\n Prints information about the eeprom error logger.\r\n", 0},
    {"errorlog_reset", errbuff_reset,
     "errorlog_reset <data>\r\n Resets the eeprom error logger.\r\n", 0},
    {"fpga_reset", fpga_reset, "fpga_reset (k|v)\r\n Reset Kintex (k) or Virtex (V) FPGA\r\n", 1},
    {"ff", ff_ctl,
     "ff <none> |(xmit|cdr on/off (0-23|all))| regw reg# val (0-23|all) | regr reg# (0-23)\r\n"
     " Firefly monitoring command\r\n",
     -1},
    {"fpga", fpga_ctl, "fpga (<none>|done)\r\n Displays a table showing the state of FPGAs.\r\n",
     -1},
    {"id", board_id_info, "id\r\n Prints board ID information.\r\n", 0},
    {"i2c_base", i2c_ctl_set_dev,
     "i2c_base <device>\r\n Set I2C controller number. Value between 0-9.\r\n", 1},
    {"i2cr", i2c_ctl_r,
     "i2cr <address> <number of bytes>\r\n Read I2C controller. Addr in hex.\r\n", 2},
    {"i2crr", i2c_ctl_reg_r,
     "i2crr <address> <reg> <number of bytes>\r\n Read I2C controller. Addr in hex\r\n", 3},
    {"i2cw", i2c_ctl_w, "i2cw <address> <number of bytes> <value>\r\n Write I2C controller.\r\n",
     3},
    {"i2cwr", i2c_ctl_reg_w,
     "i2cwr <address> <reg> <number of bytes>\r\n Write I2C controller.\r\n", 4},
    {
        "i2c_scan",
        i2c_scan,
        "i2c_scan\r\n Scan current I2C bus.\r\n",
        0,
    },
    {"help", help_command_fcn, "help\r\n This help command\r\n", -1},
    {"pwr", power_ctl,
     "pwr (on|off|status|clearfail)\r\n Turn on or off all power, get status or clear "
     "failures.\r\n",
     1},
    {"led", led_ctl, "led (0-4)\r\n Manipulate red LED.\r\n", 1},
    {"psmon", psmon_ctl, "psmon <#>\r\n Displays a table showing the state of power supplies.\r\n",
     1},
    {"snapshot", snapshot,
     "snapshot # (0|1)\r\n Dump snapshot register. #: which of 5 LGA80D (10*dev+page). 0|1 decide "
     "if to reset snapshot.\r\n",
     2},
    {"restart_mcu", restart_mcu, "restart_mcu\r\n Restart the microcontroller\r\n", 0},
    {
        "set_id",
        set_board_id,
        "set_id <password> <address> <data>\r\n Allows the user to set the board id "
        "information.\r\n",
        3,
    },
    {
        "set_id_password",
        set_board_id_password,
        "set_id_password\r\n One-time use: sets password for ID block.\r\n",
        0,
    },
    {
        "simple_sensor",
        sensor_summary,
        "simple_sensor\r\n Displays a table showing the state of temps.\r\n",
        0,
    },
    {
        "ff_status",
        ff_status,
        "ff_status\r\n Displays a table showing the status of the fireflies.\r\n",
        0,
    },
    {
        "suart",
        suart_ctl,
#ifdef ZYNQMON_TEST_MODE
        "suart (on|off|status|debug1|debug2|debugraw|normal|sendone|(settest <sensor> <val>))\r\n"
#else
        "suart (on|off)\r\n"
#endif // ZYNQMON_TEST_MODE
        " Control soft uart.\r\n",
        -1,
    },
    {
        "stack_usage",
        stack_ctl,
        "stack_usage\r\n Print out system stack high water mark.\r\n",
        0,
    },
    {"task-stats", TaskStatsCommand,
     "task-stats\r\n Displays a table showing the state of each FreeRTOS task\r\n", 0},
    {"uptime", uptime, "uptime\r\n Display uptime in minutes\r\n", 0},
    {"version", ver_ctl, "version\r\n Display information about MCU firmware version\r\n", 0},
};

static void U4Print(const char *str)
{
  UARTPrint(UART4_BASE, str);
}
static void U1Print(const char *str)
{
  UARTPrint(UART1_BASE, str);
}

struct microrl_user_data_t {
  uint32_t uart_base;
};

static BaseType_t help_command_fcn(int argc, char **argv)
{
  int copied = 0;
  static int i = 0;
  if (argc == 1) {
    for (; i < NUM_COMMANDS; ++i) {
      if ((SCRATCH_SIZE - copied) < strlen(commands[i].helpstr)) {
        return pdTRUE;
      }
      copied += snprintf(m + copied, SCRATCH_SIZE - copied, "%s", commands[i].helpstr);
    }
    i = 0;
    return pdFALSE;
  }
  else { // help on a specific command.
    // help for any command that matches the entered command
    for (int j = 0; j < NUM_COMMANDS; ++j) {
      if (strncmp(commands[j].commandstr, argv[1], strlen(argv[1])) == 0) {
        copied += snprintf(m + copied, SCRATCH_SIZE - copied, "%s", commands[j].helpstr);
        // return pdFALSE;
      }
    }
  }
  if (copied == 0) {
    copied += snprintf(m + copied, SCRATCH_SIZE - copied,
                       "%s: No command starting with %s found\r\n", argv[0], argv[1]);
  }
  return pdFALSE;
}

static int execute(void *p, int argc, char **argv)
{
  struct microrl_user_data_t *userdata = p;

  UARTPrint(userdata->uart_base, "\r\n"); // the microrl does not terminate the active command

  // find the command in the list
  // argc here includes the actual command itself, so the
  // number of supplied arguments is argc-1
  for (int i = 0; i < NUM_COMMANDS; ++i) {
    if (strncmp(commands[i].commandstr, argv[0], 256) == 0) {
      if ((argc == commands[i].num_args + 1) || commands[i].num_args < 0) {
        int retval = commands[i].interpreter(argc, argv);
        if (m[0] != '\0')
          UARTPrint(userdata->uart_base, m);
        while (retval == pdTRUE) {
          retval = commands[i].interpreter(argc, argv);
          if (m[0] != '\0')
            UARTPrint(userdata->uart_base, m);
        }
        m[0] = '\0';
        return 0;
      }
      else {
        snprintf(m, SCRATCH_SIZE,
                 "Wrong number of arguments for command %s: %d expected, got %d\r\n", argv[0],
                 commands[i].num_args, argc - 1);
        UARTPrint(userdata->uart_base, m);
        return 0;
      }
    }
  }
  UARTPrint(userdata->uart_base, "Command unknown: ");
  UARTPrint(userdata->uart_base, argv[0]);
  UARTPrint(userdata->uart_base, "\r\n");

  return 0;
}

// The actual task
void vCommandLineTask(void *pvParameters)
{
  uint8_t cRxedChar;

  configASSERT(pvParameters != 0);

  CommandLineTaskArgs_t *args = pvParameters;
  StreamBufferHandle_t uartStreamBuffer = args->UartStreamBuffer;
  uint32_t uart_base = args->uart_base;

  UARTPrint(uart_base, pcWelcomeMessage);
  struct microrl_user_data_t rl_userdata = {
      .uart_base = uart_base,
  };

  struct microrl_config rl_config = {
      .print = U4Print, // default to front panel
      // set callback for execute
      .execute = execute,
      .prompt_str = "% ",
      .prompt_length = 2,
      .userdata = &rl_userdata,
  };
  if (uart_base == UART1_BASE) {
    rl_config.print = U1Print; // switch to Zynq
  }
  microrl_t rl;
  microrl_init(&rl, &rl_config);
  microrl_set_execute_callback(&rl, execute);
  microrl_insert_char(&rl, ' '); // this seems to be necessary?

  for (;;) {
    /* This implementation reads a single character at a time.  Wait in the
       Blocked state until a character is received. */
    xStreamBufferReceive(uartStreamBuffer, &cRxedChar, 1, portMAX_DELAY);
    microrl_insert_char(&rl, cRxedChar);
  }
}
