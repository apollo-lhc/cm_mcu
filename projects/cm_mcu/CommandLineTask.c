/*
 * CommandLineTask.c
 *
 *  Created on: Apr 7, 2019
 *      Author: wittich
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

// FreeRTOS includes
#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "stream_buffer.h"
#include "queue.h"

#include "FreeRTOS_CLI.h"

// strlen, strtol, and strncpy
#include <string.h>
#include <stdlib.h>

// TivaWare includes
#include "driverlib/uart.h"

#include "MonitorTask.h"

#include "CommandLineTask.h"
#include "Tasks.h"

#ifdef DEBUG_CON
// prototype of mutex'd print
# define DPRINT(x) Print(x)
#else // DEBUG_CON
# define DPRINT(x)
#endif // DEBUG_CON

void Print(const char* str);

// local sprintf prototype
int snprintf( char *buf, unsigned int count, const char *format, ... );


#define MAX_INPUT_LENGTH    50
#define MAX_OUTPUT_LENGTH   512


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
static tSMBusStatus * p_eStatus = &eStatus4;

// use these mutexes to ensure that the
// state of the I2C muxes is not changed
//extern SemaphoreHandle_t xI2C1Mutex;
//extern SemaphoreHandle_t xI2C2Mutex;
//extern SemaphoreHandle_t xI2C3Mutex;
//extern SemaphoreHandle_t xI2C4Mutex;
//extern SemaphoreHandle_t xI2C6Mutex;
//
//static SemaphoreHandle_t xCurrentI2CMutex;

// Ugly hack for now -- I don't understand how to reconcile these
// two parts of the FreeRTOS-Plus code w/o casts-o-plenty
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpointer-sign"
#pragma GCC diagnostic ignored "-Wdiscarded-qualifiers"
#pragma GCC diagnostic ignored "-Wformat=" // because of our mini-sprintf

static BaseType_t i2c_ctl_set_dev(char *m, size_t s, const char *mm)
{
  int8_t *p1;
  BaseType_t p1l;
  p1 = FreeRTOS_CLIGetParameter(mm, 1, &p1l); // device number
  p1[p1l] = 0x00; // terminate strings
  BaseType_t i = strtol(p1, NULL, 10);
  if ( ! ((i == 1)||(i==2)||(i==3)||(i==4)||(i==6))) {
    snprintf(m, s, "Invalid i2c device %d (%s), only 1,2,3, 4 and 6 supported\r\n", i, p1);
    return pdFALSE;
  }
  switch (i) {
    case 1:
      p_sMaster = &g_sMaster1;
      p_eStatus = &eStatus1;
      break;
    case 2:
      p_sMaster = &g_sMaster2;
      p_eStatus = &eStatus2;
      break;
    case 3:
      p_sMaster = &g_sMaster3;
      p_eStatus = &eStatus3;
      break;
    case 4:
      p_sMaster = &g_sMaster4;
      p_eStatus = &eStatus4;
      break;
    case 6:
      p_sMaster = &g_sMaster6;
      p_eStatus = &eStatus6;
      break;
    default:
      snprintf(m, s, "%s: huh? line %d\r\n", __func__, __LINE__);
      return pdFALSE;
      break;
  }
  snprintf(m, s,"Setting i2c device to %d \r\n", i);
  return pdFALSE;
}

static BaseType_t i2c_ctl_r(char *m, size_t s, const char *mm)
{

  int8_t *p1, *p2;
  BaseType_t p1l, p2l;
  p1 = FreeRTOS_CLIGetParameter(mm, 1, &p1l); // address
  p2 = FreeRTOS_CLIGetParameter(mm, 2, &p2l); // number of bytes
  p1[p1l] = 0x00; // terminate strings
  p2[p2l] = 0x00; // terminate strings

  BaseType_t address, nbytes;
  address = strtol(p1, NULL, 16);
  nbytes = strtol(p2, NULL, 10);
  const int MAX_BYTES=4;
  uint8_t data[MAX_BYTES];
  memset(data,0,MAX_BYTES*sizeof(data[0]));
  if ( nbytes > MAX_BYTES )
    nbytes = MAX_BYTES;

  snprintf(m, s, "i2c_ctl_r: Read %d bytes from I2C address 0x%x\r\n", nbytes, address);
  DPRINT(m);

  tSMBusStatus r = SMBusMasterI2CRead(p_sMaster, address, data, nbytes);
  if (r != SMBUS_OK) {
    snprintf(m,s, "%s: operation failed (1)\r\n", __func__);
    return pdFALSE;
  }
  while (SMBusStatusGet(p_sMaster) == SMBUS_TRANSFER_IN_PROGRESS) {
    vTaskDelay(pdMS_TO_TICKS(10));
  }
  if ( *p_eStatus != SMBUS_OK) {
    snprintf(m,s, "%s: operation failed (2, value=%d)\r\n", __func__, *p_eStatus);
    return pdFALSE;
  }

  snprintf(m, s, "%s: add: 0x%02x: value 0x%02x %02x %02x %02x\r\n", __func__,
           address, data[3], data[2], data[1], data[0]);
  return pdFALSE;
}
static BaseType_t i2c_ctl_reg_r(char *m, size_t s, const char *mm)
{

  int8_t *p1, *p2, *p3;
  BaseType_t p1l, p2l, p3l;
  p1 = FreeRTOS_CLIGetParameter(mm, 1, &p1l); // address
  p2 = FreeRTOS_CLIGetParameter(mm, 2, &p2l); // register
  p3 = FreeRTOS_CLIGetParameter(mm, 3, &p3l); // number of bytes
  p1[p1l] = 0x00; // terminate strings
  p2[p2l] = 0x00; // terminate strings
  p3[p3l] = 0x00; // terminate strings

  BaseType_t address, reg_address, nbytes;
  address = strtol(p1, NULL, 16);
  reg_address = strtol(p2, NULL, 16);
  nbytes = strtol(p3, NULL, 10);
  const int MAX_BYTES=4;
  uint8_t data[MAX_BYTES];
  uint8_t txdata = reg_address;
  memset(data,0,MAX_BYTES*sizeof(data[0]));
  if ( nbytes > MAX_BYTES )
    nbytes = MAX_BYTES;
  snprintf(m, s, "i2c_ctl_reg_r: Read %d bytes from I2C address 0x%x, reg 0x%x\r\n",
           nbytes, address, reg_address);
  Print(m);

  tSMBusStatus r = SMBusMasterI2CWriteRead(p_sMaster,address,&txdata,1,data,nbytes);
  if (r != SMBUS_OK) {
    snprintf(m,s, "%s: operation failed (1)\r\n", __func__);
    return pdFALSE;
  }
  while (SMBusStatusGet(p_sMaster) == SMBUS_TRANSFER_IN_PROGRESS) {
    vTaskDelay(pdMS_TO_TICKS(10));
  }
  if ( *p_eStatus != SMBUS_OK) {
    snprintf(m,s, "%s: operation failed (2, value=%d)\r\n", __func__, *p_eStatus);
    return pdFALSE;
  }


  snprintf(m, s, "i2cr: add: 0x%02x, reg 0x%02x: value 0x%02x %02x %02x %02x\r\n",
           address, reg_address, data[3], data[2], data[1], data[0]);
  return pdFALSE;
}

static BaseType_t i2c_ctl_reg_w(char *m, size_t s, const char *mm)
{

  int8_t *p1, *p2, *p3, *p4;
  BaseType_t p1l, p2l, p3l, p4l;
  p1 = FreeRTOS_CLIGetParameter(mm, 1, &p1l); //
  p2 = FreeRTOS_CLIGetParameter(mm, 2, &p2l); //
  p3 = FreeRTOS_CLIGetParameter(mm, 3, &p3l); //
  p4 = FreeRTOS_CLIGetParameter(mm, 4, &p4l); //
  p1[p1l] = 0x00; // terminate strings
  p2[p2l] = 0x00; // terminate strings
  p3[p3l] = 0x00; // terminate strings
  p4[p4l] = 0x00; // terminate strings
  // first byte is the register, others are the data
  BaseType_t address, reg_address, nbytes, packed_data;
  address = strtol(p1, NULL, 16); // address
  reg_address = strtol(p2, NULL, 16); // register
  nbytes = strtol(p3, NULL, 16); // number of bytes
  packed_data = strtol(p4, NULL, 16); // data
  const int MAX_BYTES=4;
  uint8_t data[MAX_BYTES+1];
  data[0] = reg_address;
  // pack the bytes into the data array, offset by
  // one due to the address
  for (int i = 1; i < MAX_BYTES+1; ++i ) {
    data[i] = (packed_data >> (i-1)*8) & 0xFFUL;
  }
  nbytes++; // to account for the register address
  if ( nbytes > MAX_BYTES )
    nbytes = MAX_BYTES;
  snprintf(m, s, "%s: write 0x%08x to address 0x%02x, register 0x%02x (%d bytes including reg addr byte)\r\n", __func__,
           packed_data, address, reg_address, nbytes);
  DPRINT(m);
  tSMBusStatus r = SMBusMasterI2CWrite(p_sMaster, address, data, nbytes);
  if (r != SMBUS_OK) {
    snprintf(m,s, "%s: operation failed (1)\r\n", __func__);
    return pdFALSE;
  }
  while (SMBusStatusGet(p_sMaster) == SMBUS_TRANSFER_IN_PROGRESS) {
    vTaskDelay(pdMS_TO_TICKS(10));
  }
  if ( *p_eStatus != SMBUS_OK) {
    snprintf(m,s, "%s: operation failed (2)\r\n", __func__);
    return pdFALSE;
  }

  snprintf(m, s, "%s: Wrote to address 0x%x, register 0x%x, value 0x%08x (%d bytes)\r\n", __func__,
           address, reg_address, packed_data, nbytes-1);
  return pdFALSE;
}


static BaseType_t i2c_ctl_w(char *m, size_t s, const char *mm)
{

  int8_t *p1, *p3, *p4;
  BaseType_t p1l, p3l, p4l;
  p1 = FreeRTOS_CLIGetParameter(mm, 1, &p1l); // address
  p3 = FreeRTOS_CLIGetParameter(mm, 2, &p3l); // value(s)
  p4 = FreeRTOS_CLIGetParameter(mm, 3, &p4l); // byte to write
  p1[p1l] = 0x00; // terminate strings
  p3[p3l] = 0x00; // terminate strings
  p4[p4l] = 0x00; // terminate strings

  BaseType_t address, nbytes, value;
  address = strtol(p1, NULL, 16);
  nbytes = strtol(p3, NULL, 16);
  value = strtol(p4, NULL, 16);
  const int MAX_BYTES=4;
  uint8_t data[MAX_BYTES];
  for (int i = 0; i < MAX_BYTES; ++i ) {
    data[i] = (value >> i*8) & 0xFFUL;
  }
  if ( nbytes > MAX_BYTES )
    nbytes = MAX_BYTES;
  snprintf(m, s, "%s: write 0x%x to address 0x%x  (%d bytes)\r\n", __func__,
           value, address, nbytes);
  DPRINT(m);

  tSMBusStatus r = SMBusMasterI2CWrite(p_sMaster, address, data, nbytes);
  if (r != SMBUS_OK) {
    snprintf(m,s, "%s: write failed (1)\r\n", __func__);
    return pdFALSE;
  }
  while (SMBusStatusGet(p_sMaster) == SMBUS_TRANSFER_IN_PROGRESS) {
    vTaskDelay(pdMS_TO_TICKS(10));
  }
  if ( *p_eStatus != SMBUS_OK) {
    snprintf(m,s, "%s: write failed (2)\r\n", __func__);
    return pdFALSE;
  }

  snprintf(m, s, "i2cwr: Wrote to address 0x%x, value 0x%08x (%d bytes)\r\n",
           address, value, nbytes);
  return pdFALSE;
}


extern struct gpio_pin_t oks[];

// send power control commands
static BaseType_t power_ctl(char *m, size_t s, const char *mm)
{
  int8_t *p1;
  BaseType_t p1l;
  p1 = FreeRTOS_CLIGetParameter(mm, 1, &p1l);
  p1[p1l] = 0x00; // terminate strings

  uint32_t message;
  if ( strcmp(p1, "on") == 0 ) {
    message = PS_ON; // turn on power supply
  }
  else if ( strcmp(p1, "off")  == 0 ) {
    message = PS_OFF; // turn off power supply
  }
  else if ( strcmp(p1, "status") == 0 ) { // report status to UART
    int copied = 0;
//    copied += snprintf(m+copied, s-copied, "power_ctl:\r\nLowest ena: %d\r\n",
//        getLowestEnabledPSPriority());
    bool ku_enable = (read_gpio_pin(TM4C_DIP_SW_1) == 1);
    bool vu_enable = (read_gpio_pin(TM4C_DIP_SW_2) == 1);
    copied += snprintf(m+copied, s-copied, "pwr_ctl:\r\nVU_ENABLE:\t%d\r\n"
        "KU_ENABLE:\t%d\r\n", vu_enable, ku_enable);
    for ( int i = 0; i < N_PS_OKS; ++i ) {
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
              default:
                c = "UNKNOWN";
                break;
    }

      copied += snprintf(m+copied, s-copied, "%15s: %s\r\n",
          pin_names[oks[i].name],  c);
      if ( copied >= MAX_OUTPUT_LENGTH ) break;
    }
    return pdFALSE;
  }
  else {
    snprintf(m, s, "power_ctl: invalid argument %s received\r\n", p1);
    return pdFALSE;
  }
  // Send a message to the power supply task, if needed
  xQueueSendToBack(xPwrQueue, &message, pdMS_TO_TICKS(10));
  m[0] = '\0'; // no output from this command

  return pdFALSE;
}

// takes one argument
static BaseType_t alarm_ctl(char *m, size_t s, const char *mm)
{
  int8_t *p1, *p2;
  BaseType_t p1l, p2l;
  p1 = FreeRTOS_CLIGetParameter(mm, 1, &p1l);
  p2 = FreeRTOS_CLIGetParameter(mm, 2, &p2l);
  if ( p1 == NULL ) {
    snprintf(m, s, "%s: need one or more arguments\r\n", __func__);
    return pdFALSE;
  }
  p1[p1l] = 0x00; // terminate strings

  uint32_t message;
  if ( strcmp(p1, "clear") == 0 ) {
    message = TEMP_ALARM_CLEAR_ALL; // turn on power supply
    xQueueSendToBack(xAlmQueue, &message, pdMS_TO_TICKS(10));
    m[0] = '\0'; // no output from this command

    return pdFALSE;
  }
  else if ( strcmp(p1, "status") == 0 ) { // report status to UART
    int copied = 0;
    copied += snprintf(m+copied,s-copied, "%s: ALARM status\r\n", __func__);
    int32_t stat = getAlarmStatus();
    float val = getAlarmTemperature();
    int tens = val; int frac = ABS((tens-val))*100;
    copied += snprintf(m+copied, s-copied, "Temperature threshold: %02d.%02d\n\r",
        tens,frac);
    copied += snprintf(m+copied, s-copied, "Raw: 0x%08x\r\n", stat);
    copied += snprintf(m+copied, s-copied, "TEMP TM4C: %s\r\n",
        stat&ALM_STAT_TM4C_OVERTEMP?"ALARM":"GOOD");
    copied += snprintf(m+copied, s-copied, "TEMP FPGA: %s\r\n",
        stat&ALM_STAT_FPGA_OVERTEMP?"ALARM":"GOOD");
    copied += snprintf(m+copied, s-copied, "TEMP FFLY: %s\r\n",
        stat&ALM_STAT_FIREFLY_OVERTEMP?"ALARM":"GOOD");
    copied += snprintf(m+copied, s-copied, "TEMP DCDC: %s\r\n",
        stat&ALM_STAT_DCDC_OVERTEMP?"ALARM":"GOOD");
    return pdFALSE;
  }
  else if ( strcmp(p1, "settemp") == 0 ) {
    p2[p2l] = 0x00; // terminate strings
    char *ptr;
    float newtemp = strtol((const char*)p2,&ptr,10);
    setAlarmTemperature(newtemp);
    snprintf(m,s, "%s: set alarm temperature to %s\r\n", __func__, p2);
    return pdFALSE;
  }
  else {
    snprintf(m, s, "%s: invalid argument %s received\r\n", __func__, p1);
    return pdFALSE;
  }

  return pdFALSE;
}




static BaseType_t i2c_scan(char *m, size_t s, const char *mm)
{
  // takes no arguments
  int copied = 0;
  copied += snprintf(m, s, "i2c bus scan\r\n");
  copied += snprintf(m+copied,s-copied,
      "     0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f\r\n00:         ");
  for (uint8_t i = 0x3; i < 0x78; ++i ) {
    uint8_t data;
    if ( i%16 ==0 ) copied += snprintf(m+copied,s-copied,"\r\n%2x:", i);
    tSMBusStatus r = SMBusMasterI2CRead(p_sMaster, i, &data, 1);
    if ( r != SMBUS_OK ) {
      Print("i2c_scan: Probe failed 1\r\n");
    }
    while ( SMBusStatusGet(p_sMaster) == SMBUS_TRANSFER_IN_PROGRESS) {
      vTaskDelay( pdMS_TO_TICKS( 10 )); // wait
    }
    if ( *p_eStatus == SMBUS_OK )
      copied += snprintf(m+copied, s-copied, " %2x", i);
    else
      copied += snprintf(m+copied, s-copied, " --");
    configASSERT(copied < s);
  }
  copied += snprintf(m+copied, s-copied,"\r\n");
  return pdFALSE;
}

// send LED commands
static BaseType_t led_ctl(char *m, size_t s, const char *mm)
{
  int8_t *p1;
  BaseType_t p1l;
  p1 = FreeRTOS_CLIGetParameter(mm, 1, &p1l);
  p1[p1l] = 0x00; // terminate strings
  BaseType_t i1 = strtol(p1, NULL, 10);

  uint32_t message = HUH; // default: message not understood
  if ( i1 == 0 ) { // ToDo: make messages less clunky. break out color.
    message = RED_LED_OFF;
  }
  else if (i1 == 1 ) {
    message = RED_LED_ON;
  }
  else if ( i1 == 2 ) {
    message = RED_LED_TOGGLE;
  }
  else if ( i1 == 3 ) {
    message = RED_LED_TOGGLE3;
  }
  else if ( i1 == 4 ) {
    message = RED_LED_TOGGLE4;
  }
  // Send a message to the LED task
  xQueueSendToBack(xLedQueue, &message, pdMS_TO_TICKS(10));
  m[0] = '\0'; // no output from this command

  return pdFALSE;
}

// dump monitor information
static BaseType_t mon_ctl(char *m, size_t s, const char *mm)
{
  int8_t *p1;
  BaseType_t p1l;
  p1 = FreeRTOS_CLIGetParameter(mm, 1, &p1l);
  p1[p1l] = 0x00; // terminate strings
  BaseType_t i1 = strtol(p1, NULL, 10);

  if ( i1 < 0 || i1 >= dcdc_args.n_commands ) {
    snprintf(m, s, "%s: Invalid argument, must be between 0 and %d\r\n", __func__,
        dcdc_args.n_commands-1);
    return pdFALSE;
  }
  // update times, in seconds
  TickType_t now = pdTICKS_TO_MS( xTaskGetTickCount())/1000;
  TickType_t last = pdTICKS_TO_MS(dcdc_args.updateTick)/1000;
  int copied = 0;
  if ( (now-last) > 60 ) {
    int mins = (now-last)/60;
    copied += snprintf(m+copied, s-copied, "%s: stale data, last update %d minutes ago\r\n", __func__, mins);
  }
  copied += snprintf(m+copied, s-copied, "%s\r\n", dcdc_args.commands[i1].name);
  for (int ps = 0; ps < dcdc_args.n_devices; ++ps) {
    copied += snprintf(m+copied, s-copied, "SUPPLY %s\r\n",
        dcdc_args.devices[ps].name);
    for (int page = 0; page < dcdc_args.n_pages; ++page ) {
      float val = dcdc_args.pm_values[ps*(dcdc_args.n_commands*dcdc_args.n_pages)
                                      +page*dcdc_args.n_commands+i1];
      int tens = val;
      int frac = ABS((val - tens)*100.0);

      copied += snprintf(m+copied, s-copied, "VALUE %02d.%02d\t", tens, frac );
    }
    copied += snprintf(m+copied, s-copied, "\r\n");
  }


  return pdFALSE;
}


// this command takes no arguments
static BaseType_t adc_ctl(char *m, size_t s, const char *mm)
{
  int copied = 0;

  static int whichadc = 0;
  if ( whichadc == 0 ) {
    copied += snprintf(m+copied, s-copied, "ADC outputs\r\n");
  }
  for ( ; whichadc < 21; ++whichadc ) {
    float val = getADCvalue(whichadc);
    int tens = val;
    int frac = ABS((val-tens)*100.);
    copied += snprintf(m+copied, s-copied, "%14s: %02d.%02d\r\n", getADCname(whichadc), tens, frac);
    if ( (s-copied) < 20 ) {
      ++whichadc;
      return pdTRUE;
    }
  }
  whichadc = 0;
  return pdFALSE;
}

// this command takes no arguments and never returns.
static BaseType_t bl_ctl(char *m, size_t s, const char *mm)
{
  Print("Jumping to boot loader.\r\n");
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

// this command takes no arguments
static BaseType_t ver_ctl(char *m, size_t s, const char *mm)
{
  int copied = 0;
  copied += snprintf(m+copied, s-copied, "Version %s built at %s.\r\n",
      gitVersion(), buildTime()) ;
  return pdFALSE;
}


// this command takes up to two arguments
static BaseType_t ff_ctl(char *m, size_t s, const char *mm)
{
  // argument handling
  int8_t *p1, *p2;
  BaseType_t p1l, p2l;
  int argc = 0;
  p1 = FreeRTOS_CLIGetParameter(mm, 1, &p1l);
  p2 = FreeRTOS_CLIGetParameter(mm, 2, &p2l);
  if ( p1 != NULL ) {
    ++argc;
    p1[p1l] = 0x00; // terminate strings
  }
  if ( p2 != NULL ) {
    ++argc;
    p2[p2l] = 0x00; // terminate strings
  }
  int copied = 0;
  static int whichff = 0;

  if ( whichff == 0 ) {
    // check for stale data
    TickType_t now =  pdTICKS_TO_MS( xTaskGetTickCount())/1000;
    TickType_t last = pdTICKS_TO_MS(getFFupdateTick())/1000;
    if ( (now-last) > 60 ) {
      int mins = (now-last)/60;
      copied += snprintf(m+copied, s-copied, "%s: stale data, last update %d minutes ago\r\n", __func__, mins);
    }

  }

  if ( argc == 0 ) { // default command: temps

    if ( whichff == 0 ) {
      copied += snprintf(m+copied, s-copied, "FF temperatures\r\n");
    }
    for ( ; whichff < NFIREFLIES; ++whichff ) {
      int8_t val = getFFvalue(whichff);
      const char *name = getFFname(whichff);
      if ( val > 0 )
        copied += snprintf(m+copied, s-copied, "%17s: %2d", name, val);
      else // dummy value
        copied += snprintf(m+copied, s-copied, "%17s: %2s", name, "--");
      bool isTx = (strstr(name, "Tx") != NULL);
      if ( isTx )
        copied += snprintf(m+copied, s-copied, "\t");
      else
        copied += snprintf(m+copied, s-copied, "\r\n");
      if ( (s-copied ) < 20 ) {
        ++whichff;
        return pdTRUE;
      }

    }
    if ( whichff%2 ==1 ) {
      m[copied++] = '\r';
      m[copied++] = '\n';
      m[copied] = '\0';
    }
    whichff = 0;
  }
  else { // more than one argument, check which command
    if ( argc == 1 ) {
      copied += snprintf(m+copied, s-copied, "%s: command %s needs an argument\r\n",
          __func__, p1);
      return pdFALSE;
    }
    char *c;
    int message;
    if ( strncmp(p1, "cdr",3) == 0 ) {
      c = "off";
      message = FFLY_DISABLE_CDR; // default: disable
      if ( strncmp(p2, "on", 2) == 0 ) {
        message = FFLY_ENABLE_CDR;
        c = "on";
      }
    }
    else if (strncmp(p1, "xmit",4) == 0 ) {
      c = "off";
      message = FFLY_DISABLE_TRANSMITTERS;
      if ( strncmp(p2, "on", 2) == 0 ) {
        message = FFLY_ENABLE_TRANSMITTERS;
        c = "on";
      }
    }
    else {
      copied += snprintf(m+copied,s-copied, "%s: command %s not recognized\r\n",
          __func__, p1);
      return pdFALSE;
    }
    xQueueSendToBack(xFFlyQueue, &message, pdMS_TO_TICKS(10));
    copied += snprintf(m+copied,s-copied, "%s: command %s %s sent.\r\n",
        __func__, p1,c);

  } // end commands with arguments
  return pdFALSE;
}

// this command takes no arguments since there is only one command
// right now.
static BaseType_t fpga_ctl(char *m, size_t s, const char *mm)
{
  int copied = 0;
  static int whichfpga = 0;
  int howmany = fpga_args.n_devices*fpga_args.n_pages;
  if ( whichfpga == 0 ) {
    TickType_t now =  pdTICKS_TO_MS( xTaskGetTickCount())/1000;
    TickType_t last = pdTICKS_TO_MS(getFFupdateTick())/1000;
    if ( (now-last) > 60 ) {
      int mins = (now-last)/60;
      copied += snprintf(m+copied, s-copied, "%s: stale data, last update %d minutes ago\r\n", __func__, mins);
    }

    copied += snprintf(m+copied, s-copied, "FPGA monitors\r\n");
    copied += snprintf(m+copied, s-copied, "%s\r\n", fpga_args.commands[0].name);
  }

  for ( ; whichfpga < howmany; ++whichfpga ) {
    float val = fpga_args.pm_values[whichfpga];
    int tens = val;
    int frac = ABS((val - tens)*100.0);

    copied += snprintf(m+copied, s-copied, "%5s: %02d.%02d",
        fpga_args.devices[whichfpga].name, tens, frac);
    if ( whichfpga%2 == 1 )
      copied += snprintf(m+copied, s-copied, "\r\n");
    else
      copied += snprintf(m+copied, s-copied, "\t");
    if ( (s-copied ) < 20 ) {
      ++whichfpga;
      return pdTRUE;
    }

  }
  if ( whichfpga%2 ==1 ) {
    m[copied++] = '\r';
    m[copied++] = '\n';
    m[copied] = '\0';
  }
  whichfpga = 0;
  return pdFALSE;
}

// this command takes no arguments since there is only one command
// right now.
static BaseType_t sensor_summary(char *m, size_t s, const char *mm)
{
  int copied = 0;
  // collect all sensor information
  // highest temperature for each
  // Firefly
  // FPGA
  // DCDC
  // TM4C
  float tm4c_temp = getADCvalue(ADC_INFO_TEMP_ENTRY);
  int tens = tm4c_temp;
  int frac = ABS((tm4c_temp-tens))*100.;
  copied += snprintf(m+copied, s-copied, "MCU %02d.%02d\r\n", tens, frac);
  // Fireflies. These are reported as ints but we are asked
  // to report a float.
  int8_t imax_temp = -99.0;
  for ( int i = 0; i < NFIREFLIES; ++i ) {
    int8_t v = getFFvalue(i);
    if ( v > imax_temp )
      imax_temp = v;
  }
  copied += snprintf(m+copied, s-copied, "FIREFLY %02d.0\r\n", imax_temp);
  // FPGAs. This is gonna bite me in the @#$#@ someday
  float max_fpga = MAX(fpga_args.pm_values[0], fpga_args.pm_values[1]);
  tens = max_fpga;
  frac = ABS((tens-max_fpga))*100.;
  copied += snprintf(m+copied, s-copied, "FPGA %02d.%02d\r\n", tens, frac);

  // DCDC. The first command is READ_TEMPERATURE_1.
  // I am assuming it stays that way!!!!!!!!
  float max_temp = -99.0;
  for (int ps = 0; ps < dcdc_args.n_devices; ++ps ) {
    for ( int page = 0; page < dcdc_args.n_pages; ++page ) {
      float thistemp = dcdc_args.pm_values[ps*(dcdc_args.n_commands*dcdc_args.n_pages)
                                           +page*dcdc_args.n_commands+0];
      if ( thistemp > max_temp )
        max_temp = thistemp;
    }
  }
  tens = max_temp;
  frac = ABS((max_temp-tens))*100.0;
  copied += snprintf(m+copied, s-copied, "REG %02d.%02d\r\n", tens, frac);

  return pdFALSE;
}

// This command takes no arguments
static BaseType_t restart_mcu(char *m, size_t s, const char *mm)
{
  int copied = 0;
  copied += snprintf(m+copied, s-copied, "Restarting MCU\r\n");
  SysCtlReset();	// This function does not return
  return pdFALSE;
}

// This command takes 1 argument, either k or v
static BaseType_t fpga_reset(char *m, size_t s, const char *mm)
{
  int copied = 0;
  int8_t *p1;
  BaseType_t p1l;
  p1 = FreeRTOS_CLIGetParameter(mm, 1, &p1l);
  p1[p1l] = 0x00; // terminate strings
  const TickType_t delay = 1 / portTICK_PERIOD_MS;  // 1 ms delay

  if ( strcmp(p1, "v") == 0 ) {
	  write_gpio_pin(V_FPGA_PROGRAM, 0x1);
	  vTaskDelay(delay);
	  write_gpio_pin(V_FPGA_PROGRAM, 0x0);
	  copied += snprintf(m+copied, s-copied, "VU7P has been reset\r\n");
    }
  if ( strcmp(p1, "k") == 0 ) {
	  write_gpio_pin(K_FPGA_PROGRAM, 0x1);
	  vTaskDelay(delay);
	  write_gpio_pin(K_FPGA_PROGRAM, 0x0);
	  copied += snprintf(m+copied, s-copied, "KU15P has been reset\r\n");

    }
  return pdFALSE;
}

// This command takes 1 arg, the address
static BaseType_t eeprom_read(char *m, size_t s, const char *mm)
{
  int copied = 0;
  int8_t *p1;
  BaseType_t p1l;
  p1 = FreeRTOS_CLIGetParameter(mm, 1, &p1l); // address
  p1[p1l] = 0x00; // terminate strings

  uint32_t addr;
  uint64_t data;
  addr = strtol(p1,NULL,16);
  uint32_t block = EEPROMBlockFromAddr(addr);
  data = read_eeprom_multi(addr);

  copied += snprintf(m+copied, s-copied, "Data read from EEPROM block %d: %08x%08x \r\n",block,data);

  return pdFALSE;
}

// This command takes 2 args, the address and 4 bytes of data to be written
static BaseType_t eeprom_write(char *m, size_t s, const char *mm)
{
  int copied = 0;
  int8_t *p1, *p2;
  BaseType_t p1l, p2l;
  p1 = FreeRTOS_CLIGetParameter(mm, 1, &p1l); // address
  p2 = FreeRTOS_CLIGetParameter(mm, 2, &p2l); // data
  p1[p1l] = 0x00; // terminate strings
  p2[p2l] = 0x00; // terminate strings

  uint32_t data, addr;
  data = strtoul(p2,NULL,16);
  addr = strtoul(p1,NULL,16);
  uint32_t block = EEPROMBlockFromAddr(addr);
  write_eeprom_single(data,addr);
  copied += snprintf(m+copied, s-copied, "Data written to EEPROM block %d: %08x \r\n",block,data);

  return pdFALSE;
}

// Takes 0 arguments
static BaseType_t eeprom_info(char *m, size_t s, const char *mm)
{
  int copied = 0;

  copied += snprintf(m+copied, s-copied, "EEPROM has 96 blocks of 64 bytes each. \r\n");
  copied += snprintf(m+copied, s-copied, "Block 1 \t 0x0040-0x007c \t r \t Apollo ID Information. Password: 0x12345678 \r\n");

  return pdFALSE;
}

// Takes 3 arguments
static BaseType_t set_board_id(char *m, size_t s, const char *mm)
{
  int copied = 0;
  int8_t *p1, *p2, *p3;
  BaseType_t p1l, p2l, p3l;
  p1 = FreeRTOS_CLIGetParameter(mm, 1, &p1l); // password
  p2 = FreeRTOS_CLIGetParameter(mm, 2, &p2l); // address
  p3 = FreeRTOS_CLIGetParameter(mm, 3, &p3l); // input data
  p1[p1l] = 0x00; // terminate strings
  p2[p2l] = 0x00; // terminate strings
  p3[p3l] = 0x00; // terminate strings

  uint32_t pass, addr, data;
  pass = strtoul(p1,NULL,16);
  addr = strtoul(p2,NULL,16);
  data = strtoul(p3,NULL,16);
  uint32_t block = EEPROMBlockFromAddr(addr);
  if (block!=1){
	  copied += snprintf(m+copied, s-copied, "Please input address in Block 1\r\n");
	  return pdFALSE;
  }
  uint32_t *pPassword = &pass;
  uint32_t *dataptr = &data;
  uint32_t dlen = 4;
  uint32_t err = 0;

  err += EEPROMBlockUnlock(1, pPassword, 1);
  err += EEPROMProgram(dataptr,addr,dlen);
  err += EEPROMBlockLock(1);
  uint32_t success = 1;
  if(err!=success){
	  copied += snprintf(m+copied, s-copied, "Write unsuccessful \r\n");
	  return pdFALSE;
  }
  copied += snprintf(m+copied, s-copied, "Successfully wrote to 0x%x in ID Block: %08x \r\n",addr,data);
  return pdFALSE;
}

// one-time use, has one function and takes 0 arguments
static BaseType_t set_board_id_password(char *m, size_t s, const char *mm)
{
  int copied = 0;
  uint32_t pass = 0x12345678;
  uint32_t *passptr = &pass;
  EEPROMBlockProtectSet(1, EEPROM_PROT_RW_LRO_URW);
  EEPROMBlockPasswordSet(1, passptr, 1);
  EEPROMBlockLock(1);

  copied += snprintf(m+copied, s-copied, "Block locked\r\n");

  return pdFALSE;
}

static BaseType_t board_id_info(char *m, size_t s, const char *mm)
{
  int copied = 0;
  uint32_t wordsize = 0x0004;
  uint32_t sn_addr = 0x0040;
  uint32_t ff_addr = sn_addr + wordsize;

  uint32_t sn = read_eeprom_single(sn_addr);	// last byte is revision, first 3 are serial number
  uint32_t num = sn >> 16;
  uint32_t rev = sn&0xff;

  uint32_t ff = read_eeprom_single(ff_addr);
  copied += snprintf(m+copied, s-copied, "ID:%08x\r\n",sn);

  copied += snprintf(m+copied, s-copied, "Board number: %x\r\n",num);
  copied += snprintf(m+copied, s-copied, "Revision: %x\r\n",rev);
  copied += snprintf(m+copied, s-copied, "Firefly config: %x\r\n",ff);
  // TODO: Figure out the best way to organize firefly information

  return pdFALSE;
}

static
void TaskGetRunTimeStats( char *pcWriteBuffer, size_t bufferLength )
{
  TaskStatus_t *pxTaskStatusArray;
  volatile UBaseType_t uxArraySize, x;
  uint32_t ulTotalRunTime, ulStatsAsPercentage;

  // Make sure the write buffer does not contain a string.
  *pcWriteBuffer = 0x00;

  // Take a snapshot of the number of tasks in case it changes while this
  // function is executing.
  uxArraySize = uxTaskGetNumberOfTasks();

  // Allocate a TaskStatus_t structure for each task.  An array could be
  // allocated statically at compile time.
  pxTaskStatusArray = pvPortMalloc( uxArraySize * sizeof( TaskStatus_t ) );

  if( pxTaskStatusArray != NULL )
  {
    // Generate raw status information about each task.
    uxArraySize = uxTaskGetSystemState( pxTaskStatusArray, uxArraySize, &ulTotalRunTime );

    // For percentage calculations.
    ulTotalRunTime /= 100UL;

    // Avoid divide by zero errors.
    if( ulTotalRunTime > 0 )
    {
      // For each populated position in the pxTaskStatusArray array,
      // format the raw data as human readable ASCII data
      for( x = 0; x < uxArraySize; x++ )
      {
        // What percentage of the total run time has the task used?
        // This will always be rounded down to the nearest integer.
        // ulTotalRunTimeDiv100 has already been divided by 100.
        ulStatsAsPercentage = pxTaskStatusArray[ x ].ulRunTimeCounter / ulTotalRunTime;

        if( ulStatsAsPercentage > 0UL )
        {
          snprintf( pcWriteBuffer, bufferLength, "%s\t\t%12u\t\t%2u%%\r\n",
              pxTaskStatusArray[ x ].pcTaskName, pxTaskStatusArray[ x ].ulRunTimeCounter, ulStatsAsPercentage );
        }
        else
        {
          // If the percentage is zero here then the task has
          // consumed less than 1% of the total run time.
          snprintf( pcWriteBuffer, bufferLength, "%s\t\t%12u\t\t<1%%\r\n", pxTaskStatusArray[ x ].pcTaskName, pxTaskStatusArray[ x ].ulRunTimeCounter );
        }
        size_t added = strlen( ( char * ) pcWriteBuffer );
        pcWriteBuffer += added;
        bufferLength  -= added;
      }
    }

    // The array is no longer needed, free the memory it consumes.
    vPortFree( pxTaskStatusArray );
  }
}

void vGetTaskHandle(char *key, TaskHandle_t  *t);

static BaseType_t task_ctl(char *m, size_t s, const char *mm)
{
  int8_t *p1, *p2;
  BaseType_t p1l, p2l;
  p1 = FreeRTOS_CLIGetParameter(mm, 1, &p1l); // task
  p2 = FreeRTOS_CLIGetParameter(mm, 2, &p2l); // command
  p1[p1l] = 0x00; // terminate strings
  p2[p2l] = 0x00; // terminate strings

  TaskHandle_t t = 0;
  vGetTaskHandle(p1,&t);
  if ( t == NULL ) {
    snprintf(m,s, "%s: invalid task %s requested\r\n", __func__, p1);
    return pdFALSE;
  }
  if (strncmp(p2,  "susp", 4) == 0 ) {
    vTaskSuspend(t);
    snprintf(m,s, "%s: suspended task %s\r\n", __func__, p1);
  }
  else if ( strncmp(p2, "resu",4) == 0 ) {
    vTaskResume(t);
    snprintf(m,s, "%s: resumed task %s\r\n", __func__, p1);
  }
  else { // unrecognized command
    snprintf(m,s,"%s: command %s not recognized. Valid commands are 'suspend' and 'resume'.\r\n",
        __func__, p2);
  }
  return pdFALSE;
}


#pragma GCC diagnostic pop
// WARNING: this command easily leads to stack overflows. It does not correctly
// ensure that there are no overwrites to pcCommandString.
static
BaseType_t TaskStatsCommand( char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString )
{
  const char *const pcHeader = "     State   Time  Fraction    #\r\n************************************************\r\n";
  BaseType_t xSpacePadding;


  ( void ) pcCommandString;
  configASSERT( pcWriteBuffer );
  int copied = 0;

  /* Generate a table of task stats. */
  strncpy( pcWriteBuffer, "Task", xWriteBufferLen);
  pcWriteBuffer += strlen( pcWriteBuffer );
  copied += strlen(pcWriteBuffer);

  /* Minus three for the null terminator and half the number of characters in
  "Task" so the column lines up with the centre of the heading. */
  configASSERT( configMAX_TASK_NAME_LEN > 3 );
  for( xSpacePadding = strlen( "Task" ); xSpacePadding < ( configMAX_TASK_NAME_LEN - 3 ); xSpacePadding++ )
  {
    /* Add a space to align columns after the task's name. */
    *pcWriteBuffer = ' ';
    pcWriteBuffer++; ++copied;

    /* Ensure always terminated. */
    *pcWriteBuffer = 0x00;
  }
  strncpy( pcWriteBuffer, pcHeader,xWriteBufferLen-copied );
  copied += strlen(pcHeader);
  TaskGetRunTimeStats( pcWriteBuffer + strlen( pcHeader ), xWriteBufferLen-copied );

  /* There is no more data to return after this single string, so return
  pdFALSE. */
  return pdFALSE;
}


static const char * const pcWelcomeMessage =
		"FreeRTOS command server.\r\nType \"help\" to view a list of registered commands.\r\n";

static
CLI_Command_Definition_t i2c_set_dev_command = {
    .pcCommand="i2c_base",
    .pcHelpString="i2c_base <device>\r\n Set I2C controller number. Value between 0-9.\r\n",
    .pxCommandInterpreter = i2c_ctl_set_dev,
    1
};
static
CLI_Command_Definition_t i2c_read_command = {
    .pcCommand="i2cr",
    .pcHelpString="i2cr <address> <number of bytes>\r\n Read I2C controller. Addr in hex.\r\n",
    .pxCommandInterpreter = i2c_ctl_r,
    2
};
static
CLI_Command_Definition_t i2c_read_reg_command = {
    .pcCommand="i2crr",
    .pcHelpString="i2crr <address> <reg> <number of bytes>\r\n Read I2C controller. Addr in hex\r\n",
    .pxCommandInterpreter = i2c_ctl_reg_r,
    3
};
static
CLI_Command_Definition_t i2c_write_command = {
    .pcCommand="i2cw",
    .pcHelpString="i2cw <address> <number of bytes> <value>\r\n Write I2C controller.\r\n",
    .pxCommandInterpreter = i2c_ctl_w,
    3
};
static
CLI_Command_Definition_t i2c_write_reg_command = {
    .pcCommand="i2cwr",
    .pcHelpString="i2cwr <address> <reg> <number of bytes>\r\n Write I2C controller.\r\n",
    .pxCommandInterpreter = i2c_ctl_reg_w,
    4
};
static
CLI_Command_Definition_t i2c_scan_command = {
    .pcCommand="i2c_scan",
    .pcHelpString="i2c_scan\r\n Scan current I2C bus.\r\n",
    .pxCommandInterpreter = i2c_scan,
    0
};

static
CLI_Command_Definition_t pwr_ctl_command = {
    .pcCommand="pwr",
    .pcHelpString="pwr (on|off|status)\r\n Turn on or off all power.\r\n",
    .pxCommandInterpreter = power_ctl,
    1
};
static
CLI_Command_Definition_t alm_ctl_command = {
    .pcCommand="alm",
    .pcHelpString="alm (clear|status|settemp #)\r\n Get or clear status of alarm task.\r\n",
    .pxCommandInterpreter = alarm_ctl,
    -1 // variable number of commands
};


static
CLI_Command_Definition_t led_ctl_command = {
    .pcCommand="led",
    .pcHelpString="led (0-4)\r\n Manipulate red LED.\r\n",
    .pxCommandInterpreter = led_ctl,
    1
};
static
CLI_Command_Definition_t task_stats_command = {
    .pcCommand="task-stats",
    .pcHelpString="task-stats\r\n Displays a table showing the state of each FreeRTOS task\r\n",
    .pxCommandInterpreter = TaskStatsCommand,
    0
};


static
CLI_Command_Definition_t monitor_command = {
    .pcCommand="mon",
    .pcHelpString="mon <#>\r\n Displays a table showing the state of power supplies.\r\n",
    .pxCommandInterpreter = mon_ctl,
    1
};
static
CLI_Command_Definition_t adc_command = {
    .pcCommand="adc",
    .pcHelpString="adc\r\n Displays a table showing the state of ADC inputs.\r\n",
    .pxCommandInterpreter = adc_ctl,
    0
};

static
CLI_Command_Definition_t task_command = {
    .pcCommand="task",
    .pcHelpString="task <name> <command>\r\n Manipulate task <name>. Options are suspend and restart.\r\n",
    .pxCommandInterpreter = task_ctl,
    2
};

static
CLI_Command_Definition_t ff_command = {
    .pcCommand="ff",
    .pcHelpString="ff\r\n Displays a table showing the state of FF modules.\r\n",
    .pxCommandInterpreter = ff_ctl,
    -1
};

static
CLI_Command_Definition_t fpga_command = {
    .pcCommand="fpga",
    .pcHelpString="fpga\r\n Displays a table showing the state of FPGAs.\r\n",
    .pxCommandInterpreter = fpga_ctl,
    0
};
static
CLI_Command_Definition_t sensor_summary_command = {
    .pcCommand="simple_sensor",
    .pcHelpString="simple_sensor\r\n Displays a table showing the state of temps.\r\n",
    .pxCommandInterpreter = sensor_summary,
    0
};

static
CLI_Command_Definition_t version_command = {
    .pcCommand="version",
    .pcHelpString="version\r\n Displays information about MCU firmware\r\n",
    .pxCommandInterpreter = ver_ctl,
    0
};

static
CLI_Command_Definition_t bootloader_command = {
    .pcCommand="bootloader",
    .pcHelpString="bootloader\r\n Call the boot loader\r\n",
    .pxCommandInterpreter = bl_ctl,
    0
};

static
CLI_Command_Definition_t restart_command = {
    .pcCommand="restart_mcu",
    .pcHelpString="restart_mcu\r\n Restart mcu\r\n",
    .pxCommandInterpreter = restart_mcu,
    0
};

static
CLI_Command_Definition_t fpga_reset_command = {
    .pcCommand="fpga_reset",
    .pcHelpString="fpga_reset (k|v)\r\n Resets either the KU15P or VU7P FPGA according to argument\r\n",
    .pxCommandInterpreter = fpga_reset,
    1
};

static
CLI_Command_Definition_t eeprom_read_command = {
    .pcCommand="eeprom_read",
    .pcHelpString="eeprom_read <address> \r\n Reads 4 bytes from EEPROM. Address should be a multiple of 4.\r\n",
    .pxCommandInterpreter = eeprom_read,
    1
};

static
CLI_Command_Definition_t eeprom_write_command = {
    .pcCommand="eeprom_write",
    .pcHelpString="eeprom_write <address> <data>\r\n Writes <data> to <address> in EEPROM. <address> should be a multiple of 4.\r\n",
    .pxCommandInterpreter = eeprom_write,
    2
};

static
CLI_Command_Definition_t eeprom_info_command = {
    .pcCommand="eeprom_info",
    .pcHelpString="eeprom_info\r\n Prints information about the EEPROM.\r\n",
    .pxCommandInterpreter = eeprom_info,
    0
};

static
CLI_Command_Definition_t set_id_command = {
    .pcCommand="set_id",
    .pcHelpString="set_id <password> <address> <data>\r\n Allows the user to set the board id information.\r\n",
    .pxCommandInterpreter = set_board_id,
    3
};

static
CLI_Command_Definition_t set_id_password_command = {
    .pcCommand="set_id_password",
    .pcHelpString="set_id_password \r\n One-time use: sets password for ID block.\r\n",
    .pxCommandInterpreter = set_board_id_password,
    0
};

static
CLI_Command_Definition_t id_command = {
    .pcCommand="id",
    .pcHelpString="id \r\n Prints board ID information.\r\n",
    .pxCommandInterpreter = board_id_info,
    0
};

void vCommandLineTask( void *pvParameters )
{
  uint8_t cRxedChar, cInputIndex = 0;
  BaseType_t xMoreDataToFollow;
  /* The input and output buffers are declared static to keep them off the stack. */
  static char pcOutputString[ MAX_OUTPUT_LENGTH ], pcInputString[ MAX_INPUT_LENGTH ];

  configASSERT(pvParameters != 0);

  CommandLineTaskArgs_t *args = pvParameters;
  StreamBufferHandle_t uartStreamBuffer = args->UartStreamBuffer;
  uint32_t uart_base = args->uart_base;

  // register the commands
  FreeRTOS_CLIRegisterCommand(&adc_command      );
  FreeRTOS_CLIRegisterCommand(&alm_ctl_command  );
  FreeRTOS_CLIRegisterCommand(&ff_command       );
  FreeRTOS_CLIRegisterCommand(&bootloader_command  );
  FreeRTOS_CLIRegisterCommand(&eeprom_read_command	);
  FreeRTOS_CLIRegisterCommand(&eeprom_write_command	);
  FreeRTOS_CLIRegisterCommand(&eeprom_info_command	);
  FreeRTOS_CLIRegisterCommand(&fpga_command       );
  FreeRTOS_CLIRegisterCommand(&fpga_reset_command	);
  FreeRTOS_CLIRegisterCommand(&id_command );
  FreeRTOS_CLIRegisterCommand(&i2c_read_command );
  FreeRTOS_CLIRegisterCommand(&i2c_read_reg_command );
  FreeRTOS_CLIRegisterCommand(&i2c_set_dev_command );
  FreeRTOS_CLIRegisterCommand(&i2c_write_command);
  FreeRTOS_CLIRegisterCommand(&i2c_write_reg_command);
  FreeRTOS_CLIRegisterCommand(&i2c_scan_command );
  FreeRTOS_CLIRegisterCommand(&led_ctl_command  );
  FreeRTOS_CLIRegisterCommand(&monitor_command  );
  FreeRTOS_CLIRegisterCommand(&pwr_ctl_command  );
  FreeRTOS_CLIRegisterCommand(&restart_command  );
  FreeRTOS_CLIRegisterCommand(&sensor_summary_command);
  FreeRTOS_CLIRegisterCommand(&set_id_command);
  FreeRTOS_CLIRegisterCommand(&set_id_password_command);
  FreeRTOS_CLIRegisterCommand(&task_stats_command );
  FreeRTOS_CLIRegisterCommand(&task_command  );
  FreeRTOS_CLIRegisterCommand(&version_command  );


  /* Send a welcome message to the user knows they are connected. */
  UARTPrint(uart_base, pcWelcomeMessage);
  UARTPrint(uart_base, "% ");

  for( ;; ) {
    /* This implementation reads a single character at a time.  Wait in the
        Blocked state until a character is received. */
    xStreamBufferReceive(uartStreamBuffer, &cRxedChar, 1, portMAX_DELAY);
    UARTCharPut(uart_base, cRxedChar); // TODO this should use the Mutex
    // ugh there has to be a better way of handling this
    if ( cRxedChar == '\177') {
      UARTCharPut(uart_base, '\b');
      UARTCharPut(uart_base, ' ');
      UARTCharPut(uart_base, '\b'); // I hate you screen
    }
    if( cRxedChar == '\n' || cRxedChar == '\r' ) {
      UARTCharPut(uart_base, '\n');
      if ( cInputIndex != 0 ) { // empty command -- skip

        snprintf(pcOutputString, MAX_OUTPUT_LENGTH, "Calling command >%s<\r\n",
            pcInputString);

        DPRINT(pcOutputString);
        if ( pcInputString[0] != '#' ) { // process unless comment char
          /* The command interpreter is called repeatedly until it returns
            pdFALSE.  See the "Implementing a command" documentation for an
            explanation of why this is. */
          do {
            /* Send the command string to the command interpreter.  Any
                output generated by the command interpreter will be placed in the
                pcOutputString buffer. */
            xMoreDataToFollow = FreeRTOS_CLIProcessCommand
                (
                    (const char*)pcInputString,   /* The command string.*/
                    (char*)pcOutputString,  /* The output buffer. */
                    MAX_OUTPUT_LENGTH/* The size of the output buffer. */
                );

            /* Write the output generated by the command interpreter to the
                console. */
            if ( pcOutputString[0] != '\0')
              UARTPrint(uart_base,pcOutputString);

          } while( xMoreDataToFollow != pdFALSE );
        }
        /* All the strings generated by the input command have been sent.
            Processing of the command is complete.  Clear the input string ready
            to receive the next command. */
        cInputIndex = 0;
        memset( pcInputString, 0x00, MAX_INPUT_LENGTH );
      }
      UARTPrint(uart_base,"% ");
    }
    else {
      /* The if() clause performs the processing after a newline character
            is received.  This else clause performs the processing if any other
            character is received. */
      if( cRxedChar == '\b' || cRxedChar == '\177' ) {
        /* Backspace was pressed.  Erase the last character in the input
                buffer - if there are any. */
        if( cInputIndex > 0 ) {
          cInputIndex--;
          pcInputString[ cInputIndex ] = '\0';
        }
      }
      else {
        /* A character was entered.  It was not a new line, backspace
                or carriage return, so it is accepted as part of the input and
                placed into the input buffer.  When a \n is entered the complete
                string will be passed to the command interpreter. */
        if( cInputIndex < MAX_INPUT_LENGTH ) {
          pcInputString[ cInputIndex ] = cRxedChar;
          cInputIndex++;
        }
      }
    }
  }
}

