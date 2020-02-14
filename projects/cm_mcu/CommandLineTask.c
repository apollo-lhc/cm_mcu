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
#include "microrl.h"

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

#define SCRATCH_SIZE 512
char m[SCRATCH_SIZE];


// Ugly hack for now -- I don't understand how to reconcile these
// two parts of the FreeRTOS-Plus code w/o casts-o-plenty
//#pragma GCC diagnostic push
//#pragma GCC diagnostic ignored "-Wpointer-sign"
//#pragma GCC diagnostic ignored "-Wdiscarded-qualifiers"
#pragma GCC diagnostic ignored "-Wformat=" // because of our mini-sprintf

static BaseType_t i2c_ctl_set_dev(int argc, char ** argv)
{
  int s = SCRATCH_SIZE;
  BaseType_t i = strtol(argv[1], NULL, 10); // device number
  if ( ! ((i == 1)||(i==2)||(i==3)||(i==4)||(i==6))) {
    snprintf(m, s, "Invalid i2c device %d (%s), only 1,2,3, 4 and 6 supported\r\n", i, argv[1]);
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
      snprintf(m, s, "%s: huh? line %d\r\n", argv[0], __LINE__);
      return pdFALSE;
      break;
  }
  snprintf(m, s,"Setting i2c device to %d \r\n", i);
  return pdFALSE;
}

static BaseType_t i2c_ctl_r(int argc, char ** argv)
{
  int s = SCRATCH_SIZE;

  BaseType_t address, nbytes;
  address = strtol(argv[1], NULL, 16);
  nbytes = strtol(argv[2], NULL, 10);
  const int MAX_BYTES=4;
  uint8_t data[MAX_BYTES];
  memset(data,0,MAX_BYTES*sizeof(data[0]));
  if ( nbytes > MAX_BYTES )
    nbytes = MAX_BYTES;

  snprintf(m, s, "i2c_ctl_r: Read %d bytes from I2C address 0x%x\r\n", nbytes, address);
  DPRINT(m);

  tSMBusStatus r = SMBusMasterI2CRead(p_sMaster, address, data, nbytes);
  if (r != SMBUS_OK) {
    snprintf(m,s, "%s: operation failed (1)\r\n", argv[0]);
    return pdFALSE;
  }
  while (SMBusStatusGet(p_sMaster) == SMBUS_TRANSFER_IN_PROGRESS) {
    vTaskDelay(pdMS_TO_TICKS(10));
  }
  if ( *p_eStatus != SMBUS_OK) {
    snprintf(m,s, "%s: operation failed (2, value=%d)\r\n", argv[0], *p_eStatus);
    return pdFALSE;
  }

  snprintf(m, s, "%s: add: 0x%02x: value 0x%02x %02x %02x %02x\r\n", argv[0],
           address, data[3], data[2], data[1], data[0]);
  return pdFALSE;
}
static BaseType_t i2c_ctl_reg_r(int argc, char ** argv)
{
  int s = SCRATCH_SIZE;

  BaseType_t address, reg_address, nbytes;
  address = strtol(argv[1], NULL, 16);
  reg_address = strtol(argv[2], NULL, 16);
  nbytes = strtol(argv[3], NULL, 10);
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
    snprintf(m,s, "%s: operation failed (1)\r\n", argv[0]);
    return pdFALSE;
  }
  while (SMBusStatusGet(p_sMaster) == SMBUS_TRANSFER_IN_PROGRESS) {
    vTaskDelay(pdMS_TO_TICKS(10));
  }
  if ( *p_eStatus != SMBUS_OK) {
    snprintf(m,s, "%s: operation failed (2, value=%d)\r\n", argv[0], *p_eStatus);
    return pdFALSE;
  }


  snprintf(m, s, "i2cr: add: 0x%02x, reg 0x%02x: value 0x%02x %02x %02x %02x\r\n",
           address, reg_address, data[3], data[2], data[1], data[0]);
  return pdFALSE;
}

static BaseType_t i2c_ctl_reg_w(int argc, char ** argv)
{
  int s = SCRATCH_SIZE;

  // first byte is the register, others are the data
  BaseType_t address, reg_address, nbytes, packed_data;
  address = strtol(argv[1], NULL, 16); // address
  reg_address = strtol(argv[2], NULL, 16); // register
  nbytes = strtol(argv[3], NULL, 16); // number of bytes
  packed_data = strtol(argv[4], NULL, 16); // data
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
  snprintf(m, s, "%s: write 0x%08x to address 0x%02x, register 0x%02x (%d bytes including reg addr byte)\r\n", argv[0],
           packed_data, address, reg_address, nbytes);
  DPRINT(m);
  tSMBusStatus r = SMBusMasterI2CWrite(p_sMaster, address, data, nbytes);
  if (r != SMBUS_OK) {
    snprintf(m,s, "%s: operation failed (1)\r\n", argv[0]);
    return pdFALSE;
  }
  while (SMBusStatusGet(p_sMaster) == SMBUS_TRANSFER_IN_PROGRESS) {
    vTaskDelay(pdMS_TO_TICKS(10));
  }
  if ( *p_eStatus != SMBUS_OK) {
    snprintf(m,s, "%s: operation failed (2)\r\n", argv[0]);
    return pdFALSE;
  }

  snprintf(m, s, "%s: Wrote to address 0x%x, register 0x%x, value 0x%08x (%d bytes)\r\n", argv[0],
           address, reg_address, packed_data, nbytes-1);
  return pdFALSE;
}


static BaseType_t i2c_ctl_w(int argc, char **argv)
{
  int s = SCRATCH_SIZE;

  BaseType_t address, nbytes, value;
  address = strtol(argv[1], NULL, 16);
  nbytes = strtol(argv[2], NULL, 16);
  value = strtol(argv[3], NULL, 16);
  const int MAX_BYTES=4;
  uint8_t data[MAX_BYTES];
  for (int i = 0; i < MAX_BYTES; ++i ) {
    data[i] = (value >> i*8) & 0xFFUL;
  }
  if ( nbytes > MAX_BYTES )
    nbytes = MAX_BYTES;
  snprintf(m, s, "%s: write 0x%x to address 0x%x  (%d bytes)\r\n", argv[0],
           value, address, nbytes);
  DPRINT(m);

  tSMBusStatus r = SMBusMasterI2CWrite(p_sMaster, address, data, nbytes);
  if (r != SMBUS_OK) {
    snprintf(m,s, "%s: write failed (1)\r\n", argv[0]);
    return pdFALSE;
  }
  while (SMBusStatusGet(p_sMaster) == SMBUS_TRANSFER_IN_PROGRESS) {
    vTaskDelay(pdMS_TO_TICKS(10));
  }
  if ( *p_eStatus != SMBUS_OK) {
    snprintf(m,s, "%s: write failed (2)\r\n", argv[0]);
    return pdFALSE;
  }

  snprintf(m, s, "i2cwr: Wrote to address 0x%x, value 0x%08x (%d bytes)\r\n",
           address, value, nbytes);
  return pdFALSE;
}


extern struct gpio_pin_t oks[];

// send power control commands
static BaseType_t power_ctl(int argc, char ** argv)
{
  int s = SCRATCH_SIZE;

  uint32_t message;
  if ( strcmp(argv[1], "on") == 0 ) {
    message = PS_ON; // turn on power supply
  }
  else if ( strcmp(argv[1], "off")  == 0 ) {
    message = PS_OFF; // turn off power supply
  }
  else if ( strcmp(argv[1], "status") == 0 ) { // report status to UART
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
    snprintf(m, s, "power_ctl: invalid argument %s received\r\n", argv[1]);
    return pdFALSE;
  }
  // Send a message to the power supply task, if needed
  xQueueSendToBack(xPwrQueue, &message, pdMS_TO_TICKS(10));
  m[0] = '\0'; // no output from this command

  return pdFALSE;
}

// takes one argument
static BaseType_t alarm_ctl(int argc, char ** argv)
{
  int s = SCRATCH_SIZE;
  if ( argc < 2 ) {
    snprintf(m, s, "%s: need one or more arguments\r\n", argv[0]);
    return pdFALSE;
  }

  uint32_t message;
  if ( strcmp(argv[1], "clear") == 0 ) {
    message = TEMP_ALARM_CLEAR_ALL; // turn on power supply
    xQueueSendToBack(xAlmQueue, &message, pdMS_TO_TICKS(10));
    m[0] = '\0'; // no output from this command

    return pdFALSE;
  }
  else if ( strcmp(argv[1], "status") == 0 ) { // report status to UART
    int copied = 0;
    copied += snprintf(m+copied,s-copied, "%s: ALARM status\r\n", argv[0]);
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
  else if ( strcmp(argv[1], "settemp") == 0 ) {
    char *ptr;
    float newtemp = strtol(argv[2],&ptr,10);
    setAlarmTemperature(newtemp);
    snprintf(m,s, "%s: set alarm temperature to %s\r\n", argv[0], argv[2]);
    return pdFALSE;
  }
  else {
    snprintf(m, s, "%s: invalid argument %s received\r\n", argv[0], argv[1]);
    return pdFALSE;
  }

  return pdFALSE;
}




static BaseType_t i2c_scan(int argc, char ** argv)
{
  // takes no arguments
  int copied = 0, s = SCRATCH_SIZE;
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
static BaseType_t led_ctl(int argc, char ** argv)
{

  BaseType_t i1 = strtol(argv[1], NULL, 10);

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
static BaseType_t mon_ctl(int argc, char ** argv)
{
  int s = SCRATCH_SIZE;
  BaseType_t i1 = strtol(argv[1], NULL, 10);

  if ( i1 < 0 || i1 >= dcdc_args.n_commands ) {
    snprintf(m, s, "%s: Invalid argument, must be between 0 and %d\r\n", argv[0],
        dcdc_args.n_commands-1);
    return pdFALSE;
  }
  // update times, in seconds
  TickType_t now = pdTICKS_TO_MS( xTaskGetTickCount())/1000;
  TickType_t last = pdTICKS_TO_MS(dcdc_args.updateTick)/1000;
  int copied = 0;
  if ( (now-last) > 60 ) {
    int mins = (now-last)/60;
    copied += snprintf(m+copied, s-copied, "%s: stale data, last update %d minutes ago\r\n", argv[0], mins);
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
static BaseType_t adc_ctl(int argc, char ** argv)
{
  int copied = 0, s = SCRATCH_SIZE;

  static int whichadc = 0;
  if ( whichadc == 0 ) {
    copied += snprintf(m+copied, s-copied, "ADC outputs\r\n");
  }
  for ( ; whichadc < 21; ++whichadc ) {
    float val = getADCvalue(whichadc);
    int tens = val;
    int frac = ABS((val-tens)*100.);
    copied += snprintf(m+copied, s-copied, "%14s: %02d.%02d\r\n", getADCname(whichadc), tens, frac);
    if ( (s-copied) < 20  && (whichadc < 20)) {
      ++whichadc;
      return pdTRUE;
    }
  }
  whichadc = 0;
  return pdFALSE;
}

// this command takes no arguments and never returns.
static BaseType_t bl_ctl(int argc, char ** argv)
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
static BaseType_t ver_ctl(int argc, char ** argv)
{
  int copied = 0, s = SCRATCH_SIZE;
  copied += snprintf(m+copied, s-copied, "Version %s built at %s.\r\n",
      gitVersion(), buildTime()) ;
  return pdFALSE;
}


// this command takes up to two arguments
static BaseType_t ff_ctl(int argc, char ** argv)
{
  int s = SCRATCH_SIZE;
  // argument handling
  int copied = 0;
  static int whichff = 0;

  if ( whichff == 0 ) {
    // check for stale data
    TickType_t now =  pdTICKS_TO_MS( xTaskGetTickCount())/1000;
    TickType_t last = pdTICKS_TO_MS(getFFupdateTick())/1000;
    if ( (now-last) > 60 ) {
      int mins = (now-last)/60;
      copied += snprintf(m+copied, s-copied, "%s: stale data, last update %d minutes ago\r\n", argv[0], mins);
    }

  }

  if ( argc == 1 ) { // default command: temps

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
    if ( argc == 2 ) {
      copied += snprintf(m+copied, s-copied, "%s: command %s needs an argument\r\n",
          argv[0], argv[1]);
      return pdFALSE;
    }
    char *c;
    int message;
    if ( strncmp(argv[1], "cdr",3) == 0 ) {
      c = "off";
      message = FFLY_DISABLE_CDR; // default: disable
      if ( strncmp(argv[2], "on", 2) == 0 ) {
        message = FFLY_ENABLE_CDR;
        c = "on";
      }
    }
    else if (strncmp(argv[1], "xmit",4) == 0 ) {
      c = "off";
      message = FFLY_DISABLE_TRANSMITTERS;
      if ( strncmp(argv[2], "on", 2) == 0 ) {
        message = FFLY_ENABLE_TRANSMITTERS;
        c = "on";
      }
    }
    else {
      copied += snprintf(m+copied,s-copied, "%s: command %s not recognized\r\n",
          argv[0], argv[1]);
      return pdFALSE;
    }
    xQueueSendToBack(xFFlyQueue, &message, pdMS_TO_TICKS(10));
    copied += snprintf(m+copied,s-copied, "%s: command %s %s sent.\r\n",
        argv[0], argv[1],c);

  } // end commands with arguments
  return pdFALSE;
}


// this command takes up to one argument
static BaseType_t fpga_ctl(int argc, char ** argv)
{
  int s = SCRATCH_SIZE;
  if ( argc == 2 ) {
    if ( strncmp(argv[1], "done",4) == 0 ) { // print out value of done pins
      int ku_done_ = read_gpio_pin(_K_FPGA_DONE);
      int vu_done_ = read_gpio_pin(_V_FPGA_DONE);
      //int copied =
      snprintf(m, s, "KU_DONE* = %d\r\nVU_DONE* = %d\r\n", ku_done_, vu_done_);
      return pdFALSE;
    }
    else {
      snprintf(m, s, "%s: invalid command %s\r\n", argv[0], argv[1]);
      return pdFALSE;
    }

  }
  else if (argc != 1 ) {
    // error, invalid
    snprintf(m,s, "%s: invalid argument count %d\r\n", argv[0]);
    return pdFALSE;
  }
  else {
    int copied = 0;
    static int whichfpga = 0;
    int howmany = fpga_args.n_devices*fpga_args.n_pages;
    if ( whichfpga == 0 ) {
      TickType_t now =  pdTICKS_TO_MS( xTaskGetTickCount())/1000;
      TickType_t last = pdTICKS_TO_MS(getFFupdateTick())/1000;
      if ( (now-last) > 60 ) {
        int mins = (now-last)/60;
        copied += snprintf(m+copied, s-copied, "%s: stale data, last update %d minutes ago\r\n", argv[0], mins);
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
}

// this command takes no arguments since there is only one command
// right now.
static BaseType_t sensor_summary(int argc, char ** argv)
{
  int copied = 0, s = SCRATCH_SIZE;
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
static BaseType_t restart_mcu(int argc, char ** argv)
{
  int copied = 0, s = SCRATCH_SIZE;
  copied += snprintf(m+copied, s-copied, "Restarting MCU\r\n");
  SysCtlReset();	// This function does not return
  return pdFALSE;
}

// This command takes 1 argument, either k or v
static BaseType_t fpga_reset(int argc, char ** argv)
{
  int copied = 0, s = SCRATCH_SIZE;
  const TickType_t delay = 1 / portTICK_PERIOD_MS;  // 1 ms delay

  if ( strcmp(argv[1], "v") == 0 ) {
    write_gpio_pin(V_FPGA_PROGRAM, 0x1);
    vTaskDelay(delay);
    write_gpio_pin(V_FPGA_PROGRAM, 0x0);
    copied += snprintf(m+copied, s-copied, "VU7P has been reset\r\n");
  }
  if ( strcmp(argv[1], "k") == 0 ) {
    write_gpio_pin(K_FPGA_PROGRAM, 0x1);
    vTaskDelay(delay);
    write_gpio_pin(K_FPGA_PROGRAM, 0x0);
    copied += snprintf(m+copied, s-copied, "KU15P has been reset\r\n");

  }
  return pdFALSE;
}

// This command takes 1 arg, the address
static BaseType_t eeprom_read(int argc, char ** argv)
{
  int copied = 0, s = SCRATCH_SIZE;

  uint32_t addr;
  addr = strtol(argv[1],NULL,16);
  uint32_t block = EEPROMBlockFromAddr(addr);

  uint64_t data = read_eeprom_multi(addr);
  copied += snprintf(m+copied, s-copied, "Data read from EEPROM block %d: %08x%08x \r\n",block,data);

  return pdFALSE;
}

// This command takes 2 args, the address and 4 bytes of data to be written
static BaseType_t eeprom_write(int argc, char ** argv)
{
  int copied = 0, s = SCRATCH_SIZE;

  uint32_t data, addr;
  data = strtoul(argv[2],NULL,16);
  addr = strtoul(argv[1],NULL,16);
  uint32_t block = EEPROMBlockFromAddr(addr);
  if((block==1)|((EBUF_MINBLK<=block)&&(block<=EBUF_MAXBLK))){
	  copied += snprintf(m+copied, s-copied, "Please choose available block\r\n");
	  return pdFALSE;
  }
  write_eeprom(data,addr);
  copied += snprintf(m+copied, s-copied, "Data written to EEPROM block %d: %08x \r\n",block,data);

  return pdFALSE;
}

// Takes 0 arguments
static BaseType_t eeprom_info(int argc, char ** argv)
{
  int copied = 0, s = SCRATCH_SIZE;

  copied += snprintf(m+copied, s-copied, "EEPROM has 96 blocks of 64 bytes each. \r\n");
  copied += snprintf(m+copied, s-copied, "Block 0 \t 0x0000-0x0040 \t Free. \r\n");
  copied += snprintf(m+copied, s-copied, "Block 1 \t 0x0040-0x007c \t Apollo ID Information. Password: 0x12345678 \r\n");
  copied += snprintf(m+copied, s-copied, "Blocks %u-%u \t 0x%04x-0x%04x \t Error buffer. \r\n",EBUF_MINBLK, EBUF_MAXBLK,EEPROMAddrFromBlock(EBUF_MINBLK),EEPROMAddrFromBlock(EBUF_MAXBLK+1)-4);

  return pdFALSE;
}

// Takes 3 arguments
static BaseType_t set_board_id(int argc, char ** argv)
{
  int copied = 0, s = SCRATCH_SIZE;

  uint64_t pass, addr, data;
  pass = strtoul(argv[1],NULL,16);
  addr = strtoul(argv[2],NULL,16);
  data = strtoul(argv[3],NULL,16);
  uint64_t block = EEPROMBlockFromAddr(addr);
  if (block!=1){
	  copied += snprintf(m+copied, s-copied, "Please input address in Block 1\r\n");
	  return pdFALSE;
  }

  uint64_t unlock = EPRMMessage((uint64_t)EPRM_UNLOCK_BLOCK,block,pass);
  xQueueSendToBack(xEPRMQueue_in, &unlock, portMAX_DELAY);

  uint64_t message = EPRMMessage((uint64_t)EPRM_WRITE_SINGLE,addr,data);
  xQueueSendToBack(xEPRMQueue_in, &message, portMAX_DELAY);

  uint64_t lock = EPRMMessage((uint64_t)EPRM_LOCK_BLOCK,block<<32,0);
  xQueueSendToBack(xEPRMQueue_in, &lock, portMAX_DELAY);

  if(pass!=0x12345678){
	  copied += snprintf(m+copied, s-copied, "Wrong password. Type eeprom_info to get password.");
  }	// data not printing correctly?

  return pdFALSE;
}

// one-time use, has one function and takes 0 arguments
static BaseType_t set_board_id_password(int argc, char ** argv)
{
  int copied = 0, s = SCRATCH_SIZE;
  uint32_t pass = 0x12345678;
  uint32_t *passptr = &pass;

  // DOES NOT GO THROUGH GATEKEEPER TASK
  EEPROMBlockProtectSet(1, EEPROM_PROT_RW_LRO_URW);
  EEPROMBlockPasswordSet(1, passptr, 1);
  EEPROMBlockLock(1);

  copied += snprintf(m+copied, s-copied, "Block locked\r\n");

  return pdFALSE;
}

static BaseType_t board_id_info(int argc, char ** argv)
{
  int copied = 0;
  uint64_t sn_addr = 0x0040;
  uint64_t ff_addr = sn_addr + 0x4;
  uint64_t sn,ff;

  uint64_t sn_message = ((uint64_t)EPRM_READ_SINGLE<<48)|(sn_addr<<32);
  xQueueSendToBack(xEPRMQueue_in, &sn_message, portMAX_DELAY);
  xQueueReceive(xEPRMQueue_out, &sn, portMAX_DELAY);

  uint64_t ff_message = ((uint64_t)EPRM_READ_SINGLE<<48)|(ff_addr<<32);
  xQueueSendToBack(xEPRMQueue_in, &ff_message, portMAX_DELAY);
  xQueueReceive(xEPRMQueue_out, &ff, portMAX_DELAY);

  uint32_t num = (uint32_t)sn >> 16;
  uint32_t rev = ((uint32_t)sn)&0xff;

  copied += snprintf(m+copied, s-copied, "ID:%08x\r\n",(uint32_t)sn);

  copied += snprintf(m+copied, s-copied, "Board number: %x\r\n",num);
  copied += snprintf(m+copied, s-copied, "Revision: %x\r\n",rev);
  copied += snprintf(m+copied, s-copied, "Firefly config: %x\r\n",ff);
  // TODO: Figure out the best way to organize firefly information

  return pdFALSE;
}

// This command takes 1 arg, the data to be written to the buffer
static BaseType_t errbuff_in(char *m, size_t s, const char *mm)
{
  int copied = 0;
  int8_t *p1;
  BaseType_t p1l;
  p1 = FreeRTOS_CLIGetParameter(mm, 1, &p1l); // data
  p1[p1l] = 0x00; // terminate strings

  uint32_t data;
  data = strtoul(p1,NULL,16);
  errbuffer_put(ebuf,data,0);
  copied += snprintf(m+copied, s-copied, "Data written to EEPROM buffer: %x \r\n",data);

  return pdFALSE;
}

static BaseType_t errbuff_out(char *m, size_t s, const char *mm)
{
  int copied = 0;
  uint32_t arr[EBUF_NGET];
  uint32_t (*arrptr)[EBUF_NGET]=&arr;
  errbuffer_get(ebuf,arrptr);

  copied += snprintf(m+copied, s-copied, "Entries in EEPROM buffer:\r\n");

  int i=0, max=EBUF_NGET;
  while(i<max){
	  uint32_t word = (*arrptr)[i];

	  uint16_t entry = (uint16_t)word;
	  uint16_t errcode = (entry&ERRCODE_MASK)>>ERRDATA_OFFSET;
	  uint16_t errdata = entry&ERRDATA_MASK;
	  uint16_t counter = entry>>(16-COUNTER_OFFSET);
	  uint16_t realcount = counter*4+1;

	  uint16_t timestamp = (uint16_t)(word>>16);
	  uint16_t days = timestamp/0x5a0;
	  uint16_t hours = (timestamp%0x5a0)/0x3c;
	  uint16_t minutes = timestamp%0x3c;
	  switch(errcode){
	  case RESTART:
		  copied += snprintf(m+copied, s-copied, "%02u %02u:%02u \t %x RESTART \r\n",days, hours, minutes, counter);
		  break;
	  case RESET_BUFFER:
		  copied += snprintf(m+copied, s-copied, "%02u %02u:%02u \t %x RESET BUFFER \r\n",days, hours, minutes,counter);
		 break;
	  default:
		  copied += snprintf(m+copied, s-copied, "%02u %02u:%02u \t %x %x %02x \r\n",days, hours, minutes, realcount, errcode,errdata);
	  }
	  i++;
  }
  return pdFALSE;
}

static BaseType_t errbuff_info(char *m, size_t s, const char *mm)
{
  int copied = 0;
  uint32_t cap, minaddr, maxaddr, head;
  uint16_t last, counter;

  cap = errbuffer_capacity(ebuf);
  minaddr = errbuffer_minaddr(ebuf);
  maxaddr = errbuffer_maxaddr(ebuf);
  head = errbuffer_head(ebuf);
  last = errbuffer_last(ebuf);
  counter = errbuffer_counter(ebuf);

  copied += snprintf(m+copied, s-copied, "Capacity: %8x words \r\n",cap);
  copied += snprintf(m+copied, s-copied, "Min address: %8x \r\n",minaddr);
  copied += snprintf(m+copied, s-copied, "Max address: %8x \r\n",maxaddr);
  copied += snprintf(m+copied, s-copied, "Head address: %8x \r\n",head);
  copied += snprintf(m+copied, s-copied, "Last entry: %x \r\n",last);
  copied += snprintf(m+copied, s-copied, "Message counter: %x \r\n",counter);

  return pdFALSE;
}
// Takes no arguments
static BaseType_t errbuff_reset(char *m, size_t s, const char *mm)
{
  errbuffer_reset(ebuf);
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
          snprintf( pcWriteBuffer, bufferLength, "%s\t%12u\t%2u%%\r\n",
              pxTaskStatusArray[ x ].pcTaskName, pxTaskStatusArray[ x ].ulRunTimeCounter, ulStatsAsPercentage );
        }
        else
        {
          // If the percentage is zero here then the task has
          // consumed less than 1% of the total run time.
          snprintf( pcWriteBuffer, bufferLength, "%s\t%12u\t<1%%\r\n", pxTaskStatusArray[ x ].pcTaskName, pxTaskStatusArray[ x ].ulRunTimeCounter );
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

void vGetTaskHandle(const char *key, TaskHandle_t  *t);

// argument 1 is task, argument 2 is command
static BaseType_t task_ctl(int argc, char ** argv)
{
  int s = SCRATCH_SIZE;

  TaskHandle_t t = 0;
  vGetTaskHandle(argv[1],&t);
  if ( t == NULL ) {
    snprintf(m,s, "%s: invalid task %s requested\r\n", argv[0], argv[1]);
    return pdFALSE;
  }
  if (strncmp(argv[2],  "susp", 4) == 0 ) {
    vTaskSuspend(t);
    snprintf(m,s, "%s: suspended task %s\r\n", argv[0], argv[1]);
  }
  else if ( strncmp(argv[2], "resu",4) == 0 ) {
    vTaskResume(t);
    snprintf(m,s, "%s: resumed task %s\r\n", argv[0], argv[1]);
  }
  else { // unrecognized command
    snprintf(m,s,"%s: command %s not recognized. Valid commands are 'suspend' and 'resume'.\r\n",
        argv[0], argv[2]);
  }
  return pdFALSE;
}

static BaseType_t uptime(int argc, char ** argv)
{
  int s = SCRATCH_SIZE;
  TickType_t now =  pdTICKS_TO_MS( xTaskGetTickCount())/1000/60; // time in minutes
  snprintf(m,s, "%s: MCU uptime %d minutes\r\n", argv[0], now);
  return pdFALSE;
}


#pragma GCC diagnostic pop
// WARNING: this command easily leads to stack overflows. It does not correctly
// ensure that there are no overwrites to pcCommandString.
static
BaseType_t TaskStatsCommand( int argc, char ** argv )
{
  int s = SCRATCH_SIZE;
  const char *const pcHeader = "            Time     %\r\n"
                               "********************************\r\n";
  BaseType_t xSpacePadding;
  int copied = 0;
  char * mm = m;
  /* Generate a table of task stats. */
  strncpy( mm, "Task", s);
  mm += strlen( m );
  copied += strlen(m);

  /* Minus three for the null terminator and half the number of characters in
  "Task" so the column lines up with the centre of the heading. */
  configASSERT( configMAX_TASK_NAME_LEN > 3 );
  for( xSpacePadding = strlen( "Task" ); xSpacePadding < ( configMAX_TASK_NAME_LEN - 3 ); xSpacePadding++ )
  {
    /* Add a space to align columns after the task's name. */
    *mm = ' ';
    mm++; ++copied;

    /* Ensure always terminated. */
    *mm = 0x00;
  }
  strncpy( mm, pcHeader,s-copied );
  copied += strlen(pcHeader);
  TaskGetRunTimeStats( mm + strlen( pcHeader ), s-copied );

  /* There is no more data to return after this single string, so return
  pdFALSE. */
  return pdFALSE;
}


static const char * const pcWelcomeMessage =
		"CLI based on microrl.\r\nType \"help\" to view a list of registered commands.\r\n";



struct command_t {
  const char * commandstr;
  BaseType_t (*interpreter)(int argc, char **);
  const char * helpstr;
  const int num_args;
};
static
BaseType_t help_command_fcn(int argc, char **);


#define NUM_COMMANDS (sizeof(commands)/sizeof(commands[0]))
static
struct command_t commands[] = {
    {
        "help",
        help_command_fcn,
        "help\r\n This help command\r\n",
        0
    },
    {"ff", ff_ctl, "ff\r\n firefly monitoring command\r\n", -1},
    {"alm", alarm_ctl, "alm (clear|status|settemp #)\r\n Get or clear status of alarm task.\r\n", -1},
    {"i2c_base", i2c_ctl_set_dev, "i2c_base <device>\r\n Set I2C controller number. Value between 0-9.\r\n", 1},
    {"i2cr", i2c_ctl_r, "i2cr <address> <number of bytes>\r\n Read I2C controller. Addr in hex.\r\n", 2},
    {
        "i2crr",
        i2c_ctl_reg_r,
        "i2crr <address> <reg> <number of bytes>\r\n Read I2C controller. Addr in hex\r\n",
        3
    },
    {
        "i2cw",
        i2c_ctl_w,
        "i2cw <address> <number of bytes> <value>\r\n Write I2C controller.\r\n",
        3
    },
    {
        "i2cwr",
        i2c_ctl_reg_w,
        "i2cwr <address> <reg> <number of bytes>\r\n Write I2C controller.\r\n",
        4
    },
    {
        "i2c_scan",
        i2c_scan,
        "i2c_scan\r\n Scan current I2C bus.\r\n",
        0,
    },
    {
        "pwr",
        power_ctl,
        "pwr (on|off|status)\r\n Turn on or off all power.\r\n",
        1
    },
    {
        "led",
        led_ctl,
        "led (0-4)\r\n Manipulate red LED.\r\n",
        1
    },
    {
        "task-stats",
        TaskStatsCommand,
        "task-stats\r\n Displays a table showing the state of each FreeRTOS task\r\n",
        0
    },
    {
        "mon",
        mon_ctl,
        "mon <#>\r\n Displays a table showing the state of power supplies.\r\n",
        1
    },
    {
        "adc",
        adc_ctl,
        "adc\r\n Displays a table showing the state of ADC inputs.\r\n",
        0

    },
    {
        "task",
        task_ctl,
        "task <name> <command>\r\n Manipulate task <name>. Options are suspend and restart.\r\n",
        2
    },
    {
        "fpga",
        fpga_ctl,
        "fpga\r\n Displays a table showing the state of FPGAs.\r\n",
        -1
    },
    {
        "simple_sensor",
        sensor_summary,
        "simple_sensor\r\n Displays a table showing the state of temps.\r\n",
        0
    },
    {
        "uptime",
        uptime,
        "uptime\r\n Display uptime in minutes\r\n",
        0
    },
    {
      "version",
      ver_ctl,
      "version\r\n Display information about MCU firmware version\r\n",
      0
    },
    {
        "bootloader",
        bl_ctl,
        "bootloader\r\n Call the boot loader\r\n",
        0
    },
    {
        "restart_mcu",
        restart_mcu,
        "restart_mcu\r\n Restart the microcontroller\r\n",
        0
    },
    {
        "fpga_reset",
        fpga_reset,
        "fpga_reset (k|v)\r\n Reset Kintex (k) or Virtex (V) FPGA\r\n",
      1
    },
    {
        "eeprom_read",
        eeprom_read,
        "eeprom_read <address>\r\n Reads 4 bytes from EEPROM. Address should be a multiple of 4.\r\n",
        1
    },
    {
        "eeprom_write",
        eeprom_write,
        "eeprom_write <address> <data>\r\n Writes <data> to <address> in EEPROM. <address> should be a multiple of 4.\r\n",
        2
    },
    {
        "eeprom_info",
        eeprom_info,
        "eeprom_info\r\n Prints information about the EEPROM.\r\n",
        0
    },
    {
      "set_id",
      set_board_id,
      "set_id <password> <address> <data>\r\n Allows the user to set the board id information.\r\n",
      3
    },
    {
        "set_id_password",
        set_board_id_password,
        "set_id_password\r\n One-time use: sets password for ID block.\r\n",
        0
    },
    {
      "id",
      board_id_info,
      "id\r\n Prints board ID information.\r\n",
      0
    },
    {
     "buffer_in",
     errbuff_in,
     "buffer_in <data> \r\n Manual entry of 2-byte code into the eeprom buffer.\r\n",
     1
    },
    {
     "buffer_out",
     errbuff_out,
     "buffer_out <data> \r\n Prints last 5 entries in the eeprom buffer.\r\n",
     0
    },
    {
     "buffer_info",
     errbuff_info,
     "buffer_info <data> \r\n Prints information about the eeprom buffer.\r\n",
     0
    },
  {
   "buffer_reset",
   errbuff_reset,
   "buffer_reset <data> \r\n Resets the eeprom buffer.\r\n",
   0
  },
};



static
void U4Print(const char* str)
{
  UARTPrint(UART4_BASE, str);
}
static
void U1Print(const char * str)
{
  UARTPrint(UART1_BASE, str);
}

struct microrl_user_data_t {
  uint32_t uart_base;
};

static
BaseType_t help_command_fcn(int argc, char ** argv)
{
  int s = SCRATCH_SIZE, copied = 0;
  static int i = 0;
  for ( ; i < NUM_COMMANDS; ++i ) {
    if ( (s-copied)<strlen(commands[i].helpstr) ) {
      ++i;
      return pdTRUE;
    }
    copied += snprintf(m+copied, s-copied, "%s", commands[i].helpstr);
  }
  i = 0;
  return pdFALSE;
}

static
int execute (void * p, int argc, char ** argv)
{
  struct microrl_user_data_t * userdata = p;

  UARTPrint(userdata->uart_base, "\r\n"); // the microrl does not terminate the active command TODO check if linefeed issue

  // find the command in the list
  for ( int i = 0; i < NUM_COMMANDS; ++i ) {
    if ( strncmp(commands[i].commandstr, argv[0],256) == 0 ) {
      if ( (argc == commands[i].num_args+1) || commands[i].num_args<0) {
        int retval = commands[i].interpreter(argc, argv);
        if (m[0] != '\0') UARTPrint(userdata->uart_base, m);
        while ( retval == pdTRUE) {
          retval = commands[i].interpreter(argc, argv);
          if (m[0] != '\0') UARTPrint(userdata->uart_base, m);
        }
        m[0] = '\0';
        return 0;
      }
      else {
        snprintf(m, SCRATCH_SIZE,
            "Wrong number of arguments for command %s: %d expected, got %d\r\n",
            argv[0], commands[i].num_args, argc );
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
void vCommandLineTask( void *pvParameters )
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
  if ( uart_base == UART1_BASE) {
    rl_config.print = U1Print; // switch to Zynq
  }
  microrl_t rl;
  microrl_init(&rl, &rl_config);
  microrl_set_execute_callback(&rl, execute);
  microrl_insert_char(&rl, ' '); // this seems to be necessary?

//  microrl_init(&rl, U4Print); // TODO: this should print to the relevant UART
//  microrl_set_execute_callback(&rl, execute);

  for( ;; ) {
    /* This implementation reads a single character at a time.  Wait in the
        Blocked state until a character is received. */
    xStreamBufferReceive(uartStreamBuffer, &cRxedChar, 1, portMAX_DELAY);
    //UARTCharPut(uart_base,cRxedChar);
    microrl_insert_char(&rl, cRxedChar);

  }
}

