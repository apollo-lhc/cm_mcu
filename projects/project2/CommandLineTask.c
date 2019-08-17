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

// local includes
#include "common/i2c_reg.h"
#include "common/uart.h"
#include "common/power_ctl.h"
#include "common/pinsel.h"
#include "common/smbus.h"

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

#ifdef DEBUG_CON
// prototype of mutex'd print
# define DPRINT(x) Print(x)
#else // DEBUG_CON
# define DPRINT(x)
#endif // DEBUG_CON

void Print(const char* str);

// local sprintf prototype
int snprintf( char *buf, unsigned int count, const char *format, ... );

// external definition
extern QueueHandle_t xPwrQueue;
extern QueueHandle_t xLedQueue;


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
    snprintf(m, s, "Invalid i2c device %d (%s), only 1,2,3, 4 and 6 supported\n", i, p1);
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
      snprintf(m, s, "%s: huh? line %d\n", __func__, __LINE__);
      return pdFALSE;
      break;
  }
  snprintf(m, s,"Setting i2c device to %d \n", i);
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

  snprintf(m, s, "i2c_ctl_r: Read %d bytes from I2C address 0x%x\n", nbytes, address);
  DPRINT(m);

  tSMBusStatus r = SMBusMasterI2CRead(p_sMaster, address, data, nbytes);
  if (r != SMBUS_OK) {
    snprintf(m,s, "%s: operation failed (1)\n", __func__);
    return pdFALSE;
  }
  while (SMBusStatusGet(p_sMaster) == SMBUS_TRANSFER_IN_PROGRESS) {
    vTaskDelay(pdMS_TO_TICKS(10));
  }
  if ( *p_eStatus != SMBUS_OK) {
    snprintf(m,s, "%s: operation failed (2, value=%d)\n", __func__, *p_eStatus);
    return pdFALSE;
  }

  snprintf(m, s, "%s: add: 0x%02x: value 0x%02x %02x %02x %02x\n", __func__,
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
  snprintf(m, s, "i2c_ctl_reg_r: Read %d bytes from I2C address 0x%x, reg 0x%x\n", nbytes, address, reg_address);
  DPRINT(m);

  tSMBusStatus r = SMBusMasterI2CWriteRead(p_sMaster,address,&txdata,1,data,nbytes);
  if (r != SMBUS_OK) {
    snprintf(m,s, "%s: operation failed (1)\n", __func__);
    return pdFALSE;
  }
  while (SMBusStatusGet(p_sMaster) == SMBUS_TRANSFER_IN_PROGRESS) {
    vTaskDelay(pdMS_TO_TICKS(10));
  }
  if ( *p_eStatus != SMBUS_OK) {
    snprintf(m,s, "%s: operation failed (2, value=%d)\n", __func__, *p_eStatus);
    return pdFALSE;
  }


  snprintf(m, s, "i2cr: add: 0x%02x, reg 0x%02x: value 0x%02x %02x %02x %02x\n",
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
  for (int i = 1; i < MAX_BYTES+1; ++i ) {
    data[i] = (packed_data >> i*8) & 0xFFUL;
  }
  if ( nbytes > MAX_BYTES )
    nbytes = MAX_BYTES;
  snprintf(m, s, "%s: write 0x%08x to address 0x%02x, register 0x%02x (%d bytes)\n", __func__,
           packed_data, address, reg_address, nbytes);
  DPRINT(m);

  tSMBusStatus r = SMBusMasterI2CWrite(p_sMaster, address, data, nbytes);
  if (r != SMBUS_OK) {
    snprintf(m,s, "%s: operation failed (1)\n", __func__);
    return pdFALSE;
  }
  while (SMBusStatusGet(p_sMaster) == SMBUS_TRANSFER_IN_PROGRESS) {
    vTaskDelay(pdMS_TO_TICKS(10));
  }
  if ( *p_eStatus != SMBUS_OK) {
    snprintf(m,s, "%s: operation failed (2)\n", __func__);
    return pdFALSE;
  }

  snprintf(m, s, "%s: Wrote to address 0x%x, register 0x%x, value 0x%08x (%d bytes)\n", __func__,
           address, reg_address, packed_data, nbytes);
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
  snprintf(m, s, "%s: write 0x%x to address 0x%x  (%d bytes)\n", __func__,
           value, address, nbytes);
  DPRINT(m);

  tSMBusStatus r = SMBusMasterI2CWrite(p_sMaster, address, data, nbytes);
  if (r != SMBUS_OK) {
    snprintf(m,s, "%s: write failed (1)\n", __func__);
    return pdFALSE;
  }
  while (SMBusStatusGet(p_sMaster) == SMBUS_TRANSFER_IN_PROGRESS) {
    vTaskDelay(pdMS_TO_TICKS(10));
  }
  if ( *p_eStatus != SMBUS_OK) {
    snprintf(m,s, "%s: write failed (2)\n", __func__);
    return pdFALSE;
  }

  snprintf(m, s, "i2cwr: Wrote to address 0x%x, value 0x%08x (%d bytes)\n", address, value, nbytes);
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
  else if ( strcmp(p1, "status") == 0 ) {
    int copied = 0;
    copied += snprintf(m+copied, s-copied, "power_ctl:\nLowest ena: %d\n",
        getLowestEnabledPSPriority());
    for ( int i = 0; i < N_PS_OKS; ++i ) {
      int j = getPSStatus(i);
      char *c;
      switch (j) {
              case 0:
                c = "UNKNOWN";
                break;
              case 1:
                c = "PWR_ON";
                break;
              case 2:
                c = "PWR_OFF";
                break;
              case 3:
              default:
                c = "UNKNOWN";
                break;
    }

      copied += snprintf(m+copied, s-copied, "%15s: %s\n",
          pin_names[oks[i].name],  c);
      if ( copied >= MAX_OUTPUT_LENGTH ) break;
    }
    return pdFALSE;
  }
  else {
    snprintf(m, s, "power_ctl: invalid argument %s received\n", p1);
    return pdFALSE;
  }
  // Send a message to the power supply task, if needed
  xQueueSendToBack(xPwrQueue, &message, pdMS_TO_TICKS(10));
  m[0] = '\0'; // no output from this command

  return pdFALSE;
}



static BaseType_t i2c_scan(char *m, size_t s, const char *mm)
{
  // takes no arguments
  int copied = 0;
  copied += snprintf(m, s, "i2c bus scan\n");
  copied += snprintf(m+copied,s-copied,
      "     0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f\n00:         ");
  for (uint8_t i = 0x3; i < 0x78; ++i ) {
    uint8_t data;
    if ( i%16 ==0 ) copied += snprintf(m+copied,s-copied,"\n%2x:", i);
    tSMBusStatus r = SMBusMasterI2CRead(p_sMaster, i, &data, 1);
    if ( r != SMBUS_OK ) {
      Print("i2c_scan: Probe failed 1\n");
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
  copied += snprintf(m+copied, s-copied,"\n");
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
    message = RED_LED_TOGGLE; // turn on power supply
  }
  else if ( i1 == 3 ) {
    message = RED_LED_TOGGLE3; // turn off power supply
  }
  else if ( i1 == 4 ) {
    message = RED_LED_TOGGLE4; // turn off power supply
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

  if ( i1 < 0 || i1 >= NCOMMANDS_PS ) {
    snprintf(m, s, "%s: Invalid argument, must be between 0 and %d\n", __func__,
        NCOMMANDS_PS-1);
    return pdFALSE;
  }

  int copied = 0;
  copied += snprintf(m+copied, s-copied, "%s\n", pm_command_dcdc[i1].name);
  for (int ps = 0; ps < NSUPPLIES_PS; ++ps) {
    copied += snprintf(m+copied, s-copied, "SUPPLY %d\n", ps);
    for (int page = 0; page < NPAGES_PS; ++page ) {
      float val = pm_values[ps*(NCOMMANDS_PS*NPAGES_PS)+page*NCOMMANDS_PS+i1];
      int tens = val;
      int frac = ABS((val - tens)*100.0);

      copied += snprintf(m+copied, s-copied, "VALUE %02d.%02d\t", tens, frac );
    }
    copied += snprintf(m+copied, s-copied, "\n");
  }


  return pdFALSE;
}

const char* getADCname(const int i);
float getADCvalue(const int i);


// this command takes no arguments
static BaseType_t adc_ctl(char *m, size_t s, const char *mm)
{
  int copied = 0;
  static int whichadc = 0;
  if ( whichadc == 0 ) {
    copied += snprintf(m+copied, s-copied, "ADC outputs\n");
  }
  for ( ; whichadc < 21; ++whichadc ) {
    float val = getADCvalue(whichadc);
    int tens = val;
    int frac = ABS((val-tens)*100.);
    copied += snprintf(m+copied, s-copied, "%14s: %02d.%02d\n", getADCname(whichadc), tens, frac);
    if ( (s-copied) < 20 ) {
      ++whichadc;
      return pdTRUE;
    }
  }
  whichadc = 0;
  return pdFALSE;
}

const char* getFFname(const uint8_t i);
int8_t getFFvalue(const uint8_t i);


// this command takes no arguments
static BaseType_t ff_ctl(char *m, size_t s, const char *mm)
{
  int copied = 0;
  static int whichff = 0;
  if ( whichff == 0 ) {
    copied += snprintf(m+copied, s-copied, "FF temperatures\n");
  }
  for ( ; whichff < 25; ++whichff ) {
    int8_t val = getFFvalue(whichff);
    copied += snprintf(m+copied, s-copied, "%17s: %3d", getFFname(whichff), val);
    if ( whichff%2 == 1 )
      copied += snprintf(m+copied, s-copied, "\n");
    else
      copied += snprintf(m+copied, s-copied, "\t");
    if ( (s-copied ) < 20 ) {
      ++whichff;
      return pdTRUE;
    }

  }
  if ( whichff%2 ==1 ) {
    m[copied++] = '\n';
    m[copied] = '\0';
  }
  whichff = 0;
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
    snprintf(m,s, "%s: invalid task %s requested\n", __func__, p1);
    return pdFALSE;
  }
  if (strncmp(p2,  "susp", 4) == 0 ) {
    vTaskSuspend(t);
    snprintf(m,s, "%s: suspended task %s\n", __func__, p1);
  }
  else if ( strncmp(p2, "resu",4) == 0 ) {
    vTaskResume(t);
    snprintf(m,s, "%s: resumed task %s\n", __func__, p1);
  }
  else { // unrecognized command
    snprintf(m,s,"%s: command %s not recognized. Valid commands are 'suspend' and 'resume'.\n",
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
    .pcHelpString="i2c_base <device>\n Set I2C controller number. Value between 0-9.\r\n",
    .pxCommandInterpreter = i2c_ctl_set_dev,
    1
};
static
CLI_Command_Definition_t i2c_read_command = {
    .pcCommand="i2cr",
    .pcHelpString="i2cr <address> <number of bytes>\n Read I2C controller. Addr in hex.\r\n",
    .pxCommandInterpreter = i2c_ctl_r,
    2
};
static
CLI_Command_Definition_t i2c_read_reg_command = {
    .pcCommand="i2crr",
    .pcHelpString="i2crr <address> <reg> <number of bytes>\n Read I2C controller. Addr in hex\r\n",
    .pxCommandInterpreter = i2c_ctl_reg_r,
    3
};
static
CLI_Command_Definition_t i2c_write_command = {
    .pcCommand="i2cw",
    .pcHelpString="i2cw <address> <number of bytes> <value>\n Write I2C controller.\r\n",
    .pxCommandInterpreter = i2c_ctl_w,
    3
};
static
CLI_Command_Definition_t i2c_write_reg_command = {
    .pcCommand="i2cwr",
    .pcHelpString="i2cwr <address> <reg> <number of bytes>\n Write I2C controller.\r\n",
    .pxCommandInterpreter = i2c_ctl_reg_w,
    4
};
static
CLI_Command_Definition_t i2c_scan_command = {
    .pcCommand="i2c_scan",
    .pcHelpString="i2c_scan\n Scan current I2C bus.\r\n",
    .pxCommandInterpreter = i2c_scan,
    0
};

static
CLI_Command_Definition_t pwr_ctl_command = {
    .pcCommand="pwr",
    .pcHelpString="pwr (on|off|status)\n Turn on or off all power.\r\n",
    .pxCommandInterpreter = power_ctl,
    1
};
static
CLI_Command_Definition_t led_ctl_command = {
    .pcCommand="led",
    .pcHelpString="led (0-4)\n Manipulate red LED.\r\n",
    .pxCommandInterpreter = led_ctl,
    1
};
static
CLI_Command_Definition_t task_stats_command = {
    .pcCommand="task-stats",
    .pcHelpString="task-stats\n Displays a table showing the state of each FreeRTOS task\r\n",
    .pxCommandInterpreter = TaskStatsCommand,
    0
};


static
CLI_Command_Definition_t monitor_command = {
    .pcCommand="mon",
    .pcHelpString="mon <#>\n Displays a table showing the state of power supplies.\r\n",
    .pxCommandInterpreter = mon_ctl,
    1
};
static
CLI_Command_Definition_t adc_command = {
    .pcCommand="adc",
    .pcHelpString="adc\n Displays a table showing the state of ADC inputs.\r\n",
    .pxCommandInterpreter = adc_ctl,
    0
};

static
CLI_Command_Definition_t task_command = {
    .pcCommand="task",
    .pcHelpString="task <name> <command>\n Manipulate task <name>. Options are suspend and restart.\r\n",
    .pxCommandInterpreter = task_ctl,
    2
};

static
CLI_Command_Definition_t ff_command = {
    .pcCommand="ff",
    .pcHelpString="ff\n Displays a table showing the state of FF modules.\r\n",
    .pxCommandInterpreter = ff_ctl,
    0
};

extern StreamBufferHandle_t xUARTStreamBuffer;


void vCommandLineTask( void *pvParameters )
{
  uint8_t cRxedChar, cInputIndex = 0;
  BaseType_t xMoreDataToFollow;
  /* The input and output buffers are declared static to keep them off the stack. */
  static char pcOutputString[ MAX_OUTPUT_LENGTH ], pcInputString[ MAX_INPUT_LENGTH ];


  // register the commands
  FreeRTOS_CLIRegisterCommand(&task_stats_command );
  FreeRTOS_CLIRegisterCommand(&task_command  );
  FreeRTOS_CLIRegisterCommand(&i2c_read_command );
  FreeRTOS_CLIRegisterCommand(&i2c_set_dev_command );
  FreeRTOS_CLIRegisterCommand(&i2c_read_reg_command );
  FreeRTOS_CLIRegisterCommand(&i2c_write_command);
  FreeRTOS_CLIRegisterCommand(&i2c_write_reg_command);
  FreeRTOS_CLIRegisterCommand(&i2c_scan_command );
  FreeRTOS_CLIRegisterCommand(&pwr_ctl_command  );
  FreeRTOS_CLIRegisterCommand(&led_ctl_command  );
  FreeRTOS_CLIRegisterCommand(&monitor_command  );
  FreeRTOS_CLIRegisterCommand(&adc_command      );
  FreeRTOS_CLIRegisterCommand(&ff_command       );



  /* Send a welcome message to the user knows they are connected. */
  Print(pcWelcomeMessage);
  Print("% ");

  for( ;; ) {
    /* This implementation reads a single character at a time.  Wait in the
        Blocked state until a character is received. */
    xStreamBufferReceive(xUARTStreamBuffer, &cRxedChar, 1, portMAX_DELAY);
    UARTCharPut(CLI_UART, cRxedChar); // TODO this should use the Mutex

    // TODO: on lnx231 the terminal _only_ sends a \r which I did not think was possible.
    // on some platforms (Mac) I think this will cause the command to be sent 2x.
    // this should be set in the terminal client
    if( cRxedChar == '\n' || cRxedChar == '\r' ) {
      /* A newline character was received, so the input command string is
            complete and can be processed.  Transmit a line separator, just to
            make the output easier to read. */
      //Print("\r\n");
      if ( cInputIndex != 0 ) { // empty command -- skip

        snprintf(pcOutputString, MAX_OUTPUT_LENGTH, "Calling command >%s<\n\r",
            pcInputString);

        DPRINT(pcOutputString);
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
            Print(pcOutputString);

        } while( xMoreDataToFollow != pdFALSE );

        /* All the strings generated by the input command have been sent.
            Processing of the command is complete.  Clear the input string ready
            to receive the next command. */
        cInputIndex = 0;
        memset( pcInputString, 0x00, MAX_INPUT_LENGTH );
      }
      Print("% ");
    }
    else {
      /* The if() clause performs the processing after a newline character
            is received.  This else clause performs the processing if any other
            character is received. */

      if( cRxedChar == '\r' ) {
        /* Ignore carriage returns. */
      }
      else if( cRxedChar == '\b' ) {
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

