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

// FreeRTOS includes
#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "stream_buffer.h"
#include "queue.h"

#include "FreeRTOS_CLI.h"

// strlen, strtol, and strncpy
#include "string.h"
#include <stdlib.h>

// TivaWare includes
#include "driverlib/uart.h"

#include "MonitorTask.h"


// prototype of mutex'd print
void Print(const char* str);

// local sprintf prototype
int snprintf( char *buf, unsigned int count, const char *format, ... );

// external definition
extern QueueHandle_t xPwrQueue;
extern QueueHandle_t xLedQueue;


#define MAX_INPUT_LENGTH    50
#define MAX_OUTPUT_LENGTH   256


// Ugly hack for now -- I don't understand how to reconcile these
// two parts of the FreeRTOS-Plus code w/o casts-o-plenty
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpointer-sign"
#pragma GCC diagnostic ignored "-Wdiscarded-qualifiers"
#pragma GCC diagnostic ignored "-Wformat=" // because of our mini-sprintf

static BaseType_t i2c1_ctl_r(char *m, size_t s, const char *mm)
{

  int8_t *p1, *p2;
  BaseType_t p1l, p2l;
  p1 = FreeRTOS_CLIGetParameter(mm, 1, &p1l); // address
  p2 = FreeRTOS_CLIGetParameter(mm, 2, &p2l); // number of bytes
  p1[p1l] = 0x00; // terminate strings
  p2[p2l] = 0x00; // terminate strings

  BaseType_t i1, i2;
  i1 = strtol(p1, NULL, 16);
  i2 = strtol(p2, NULL, 10);
  const int MAX_BYTES=4;
  uint8_t data[MAX_BYTES];
  memset(data,0,MAX_BYTES*sizeof(data[0]));
  if ( i2 > MAX_BYTES )
    i2 = MAX_BYTES;

  snprintf(m, s, "i2c1_ctl_r: Read %d bytes from I2C address 0x%x\n", i2, i1);
  Print(m);
  readI2C(I2C1_BASE, i1, data, i2);
  snprintf(m, s, "i2cr: add: 0x%02x: value 0x%02x %02x %02x %02x\n",
           i1, data[3], data[2], data[1], data[0]);
  return pdFALSE;
}
static BaseType_t i2c1_ctl_reg_r(char *m, size_t s, const char *mm)
{

  int8_t *p1, *p2, *p3;
  BaseType_t p1l, p2l, p3l;
  p1 = FreeRTOS_CLIGetParameter(mm, 1, &p1l); // address
  p2 = FreeRTOS_CLIGetParameter(mm, 2, &p2l); // register
  p3 = FreeRTOS_CLIGetParameter(mm, 3, &p3l); // number of bytes
  p1[p1l] = 0x00; // terminate strings
  p2[p2l] = 0x00; // terminate strings
  p3[p3l] = 0x00; // terminate strings

  BaseType_t i1, i2, i3;
  i1 = strtol(p1, NULL, 16);
  i2 = strtol(p2, NULL, 16);
  i3 = strtol(p3, NULL, 10);
  const int MAX_BYTES=4;
  uint8_t data[MAX_BYTES];
  memset(data,0,MAX_BYTES*sizeof(data[0]));
  if ( i3 > MAX_BYTES )
    i3 = MAX_BYTES;
  snprintf(m, s, "i2c1_ctl_reg_r: Read %d bytes from I2C address 0x%x, reg 0x%x\n", i3, i1, i2);
  Print(m);
  readI2Creg(I2C1_BASE, i1, i2, data, i3);

  snprintf(m, s, "i2cr: add: 0x%02x, reg 0x%02x: value 0x%02x %02x %02x %02x\n",
           i1, i2, data[3], data[2], data[1], data[0]);
  return pdFALSE;
}

static BaseType_t i2c1_ctl_reg_w(char *m, size_t s, const char *mm)
{

  int8_t *p1, *p2, *p3, *p4;
  BaseType_t p1l, p2l, p3l, p4l;
  p1 = FreeRTOS_CLIGetParameter(mm, 1, &p1l); // address
  p2 = FreeRTOS_CLIGetParameter(mm, 2, &p2l); // register
  p3 = FreeRTOS_CLIGetParameter(mm, 3, &p3l); // value(s)
  p4 = FreeRTOS_CLIGetParameter(mm, 4, &p4l); // byte to write
  p1[p1l] = 0x00; // terminate strings
  p2[p2l] = 0x00; // terminate strings
  p3[p3l] = 0x00; // terminate strings
  p4[p4l] = 0x00; // terminate strings

  BaseType_t i1, i2, i3, i4;
  i1 = strtol(p1, NULL, 16);
  i2 = strtol(p2, NULL, 16);
  i3 = strtol(p3, NULL, 16);
  i4 = strtol(p4, NULL, 16);
  const int MAX_BYTES=4;
  uint8_t data[MAX_BYTES];
  for (int i = 0; i < MAX_BYTES; ++i ) {
    data[i] = (i4 >> i*8) & 0xFFUL;
  }
  if ( i3 > MAX_BYTES )
    i3 = MAX_BYTES;
  snprintf(m, s, "i2c1_ctl_reg_w: write 0x%08x to address 0x%02x, register 0x%02x (%d bytes)\n",
           i4, i1, i2, i3);
  Print(m);
  writeI2Creg(I2C1_BASE, i1, i2, data, i3);

  snprintf(m, s, "i2cwr: Wrote to address 0x%x, register 0x%x, value 0x%08x (%d bytes)\n", i1, i2, i3, i4);
  return pdFALSE;
}

static BaseType_t i2c1_ctl_w(char *m, size_t s, const char *mm)
{

  int8_t *p1, *p3, *p4;
  BaseType_t p1l, p3l, p4l;
  p1 = FreeRTOS_CLIGetParameter(mm, 1, &p1l); // address
  p3 = FreeRTOS_CLIGetParameter(mm, 2, &p3l); // value(s)
  p4 = FreeRTOS_CLIGetParameter(mm, 3, &p4l); // byte to write
  p1[p1l] = 0x00; // terminate strings
  p3[p3l] = 0x00; // terminate strings
  p4[p4l] = 0x00; // terminate strings

  BaseType_t i1, i3, i4;
  i1 = strtol(p1, NULL, 16);
  i3 = strtol(p3, NULL, 16);
  i4 = strtol(p4, NULL, 16);
  const int MAX_BYTES=4;
  uint8_t data[MAX_BYTES];
  for (int i = 0; i < MAX_BYTES; ++i ) {
    data[i] = (i4 >> i*8) & 0xFFUL;
  }
  if ( i3 > MAX_BYTES )
    i3 = MAX_BYTES;
  snprintf(m, s, "i2c1_ctl_w: write 0x%x to address 0x%x  (%d bytes)\n",
           i4, i1, i3);
  Print(m);
  writeI2C(I2C1_BASE, i1, data, i3);

  snprintf(m, s, "i2cwr: Wrote to address 0x%x, value 0x%08x (%d bytes)\n", i1, i4, i3);
  return pdFALSE;
}


// send power control commands
static BaseType_t power_ctl(char *m, size_t s, const char *mm)
{
  int8_t *p1;
  BaseType_t p1l;
  p1 = FreeRTOS_CLIGetParameter(mm, 1, &p1l);
  p1[p1l] = 0x00; // terminate strings
  BaseType_t i1 = strtol(p1, NULL, 10);

  uint32_t message;
  if ( !(i1 == 1 || i1 == 0 )) {
    snprintf(m, s, "power_ctl: invalid argument %d received\n", i1);
    return pdFALSE;
  }
  else if ( i1 == 1 ) {
    message = PS_ON; // turn on power supply
  }
  else if ( i1 == 0 ) {
    message = PS_OFF; // turn off power supply
  }
  // Send a message to the power supply task
  xQueueSendToBack(xPwrQueue, &message, pdMS_TO_TICKS(10));
  m[0] = '\0'; // no output from this command

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
  int copied = 0;
  copied += snprintf(m+copied, s-copied, "%s\n", pm_common[i1].name);
  for (int ps = 0; ps < NSUPPLIES; ++ps) {
    copied += snprintf(m+copied, s-copied, "SUPPLY %d\n", ps);
    for (int page = 0; page < NPAGES; ++page ) {
      float val = pm_values[ps*(NCOMMANDS*NPAGES)+page*NCOMMANDS+i1];
      int tens = val;
      int frac = ABS((val - tens)*100.0);

      copied += snprintf(m+copied, s-copied, "VALUE %d.%d\t", tens, frac );
    }
    copied += snprintf(m+copied, s-copied, "\n");
  }


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
          snprintf( pcWriteBuffer, bufferLength, "%s\t\t%u\t\t%u%%\r\n",
              pxTaskStatusArray[ x ].pcTaskName, pxTaskStatusArray[ x ].ulRunTimeCounter, ulStatsAsPercentage );
        }
        else
        {
          // If the percentage is zero here then the task has
          // consumed less than 1% of the total run time.
          snprintf( pcWriteBuffer, bufferLength, "%s\t\t%u\t\t<1%%\r\n", pxTaskStatusArray[ x ].pcTaskName, pxTaskStatusArray[ x ].ulRunTimeCounter );
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

CLI_Command_Definition_t i2c_read_command = {
    .pcCommand="i2cr",
    .pcHelpString="i2cr <address> <number of bytes>\n Read no1 I2C controller. Addr in hex.\r\n",
    .pxCommandInterpreter = i2c1_ctl_r,
    2
};
CLI_Command_Definition_t i2c_read_reg_command = {
    .pcCommand="i2crr",
    .pcHelpString="i2crr <address> <reg> <number of bytes>\n Read no1 I2C controller. Addr in hex\r\n",
    .pxCommandInterpreter = i2c1_ctl_reg_r,
    3
};

CLI_Command_Definition_t i2c_write_command = {
    .pcCommand="i2cw",
    .pcHelpString="i2cw <address> <number of bytes> <value>\n Write no1 I2C controller.\r\n",
    .pxCommandInterpreter = i2c1_ctl_w,
    3
};

CLI_Command_Definition_t i2c_write_reg_command = {
    .pcCommand="i2cwr",
    .pcHelpString="i2cwr <address> <reg> <number of bytes>\n Write no1 I2C controller.\r\n",
    .pxCommandInterpreter = i2c1_ctl_reg_w,
    4
};

CLI_Command_Definition_t pwr_ctl_command = {
    .pcCommand="pwr",
    .pcHelpString="pwr (0|1)\n Turn on or off all power.\r\n",
    .pxCommandInterpreter = power_ctl,
    1
};

CLI_Command_Definition_t led_ctl_command = {
    .pcCommand="led",
    .pcHelpString="led (0|1)\n Turn on red LED (for now).\r\n",
    .pxCommandInterpreter = led_ctl,
    1
};

CLI_Command_Definition_t task_stats_command = {
    .pcCommand="task-stats",
    .pcHelpString="task-stats\n Displays a table showing the state of each FreeRTOS task\r\n",
    .pxCommandInterpreter = TaskStatsCommand,
    0
};



CLI_Command_Definition_t monitor_command = {
    .pcCommand="mon",
    .pcHelpString="mon\n Displays a table showing the state of power supplies\r\n",
    .pxCommandInterpreter = mon_ctl,
    1
};

extern StreamBufferHandle_t xStreamBuffer;


void vCommandLineTask( void *pvParameters )
{
  uint8_t cRxedChar, cInputIndex = 0;
  BaseType_t xMoreDataToFollow;
  /* The input and output buffers are declared static to keep them off the stack. */
  static char pcOutputString[ MAX_OUTPUT_LENGTH ], pcInputString[ MAX_INPUT_LENGTH ];


  // register the commands
  FreeRTOS_CLIRegisterCommand(&task_stats_command );
  FreeRTOS_CLIRegisterCommand(&i2c_read_command );
  FreeRTOS_CLIRegisterCommand(&i2c_read_reg_command );
  FreeRTOS_CLIRegisterCommand(&i2c_write_command);
  FreeRTOS_CLIRegisterCommand(&i2c_write_reg_command);
  FreeRTOS_CLIRegisterCommand(&pwr_ctl_command  );
  FreeRTOS_CLIRegisterCommand(&led_ctl_command  );
  FreeRTOS_CLIRegisterCommand(&monitor_command  );



  /* Send a welcome message to the user knows they are connected. */
  Print(pcWelcomeMessage);
  Print("% ");

  for( ;; ) {
    /* This implementation reads a single character at a time.  Wait in the
        Blocked state until a character is received. */
    xStreamBufferReceive(xStreamBuffer, &cRxedChar, 1, portMAX_DELAY);
    UARTCharPut(CLI_UART, cRxedChar); // TODO this should use the Mutex

    if( cRxedChar == '\n' ) {
      /* A newline character was received, so the input command string is
            complete and can be processed.  Transmit a line separator, just to
            make the output easier to read. */
      //Print("\r\n");
      if ( cInputIndex != 0 ) { // empty command -- skip

        snprintf(pcOutputString, MAX_OUTPUT_LENGTH, "Calling command >%s<\n",
            pcInputString);

        Print(pcOutputString);
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

