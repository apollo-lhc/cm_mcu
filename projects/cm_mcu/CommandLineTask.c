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

// Include commands
#include "commands/BufferCommands.h"
#include "commands/EEPROMCommands.h"
#include "commands/I2CCommands.h"
#include "commands/SensorControl.h"

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

//extern tSMBus g_sMaster1;
//extern tSMBusStatus eStatus1;
//extern tSMBus g_sMaster2;
//extern tSMBusStatus eStatus2;
//extern tSMBus g_sMaster3;
//extern tSMBusStatus eStatus3;
//extern tSMBus g_sMaster4;
//extern tSMBusStatus eStatus4;
//extern tSMBus g_sMaster6;
//extern tSMBusStatus eStatus6;
//static tSMBus *p_sMaster = &g_sMaster4;
//static tSMBusStatus *p_eStatus = &eStatus4;

#define SCRATCH_SIZE 512
static char m[SCRATCH_SIZE];

// Ugly hack for now -- I don't understand how to reconcile these
// two parts of the FreeRTOS-Plus code w/o casts-o-plenty
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wformat" // because of our mini-sprintf

// dump monitor information
static BaseType_t psmon_ctl(int argc, char **argv, char m)
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

// this command takes no arguments and never returns.
static BaseType_t bl_ctl(int argc, char **argv, char m)
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
static BaseType_t clock_ctl(int argc, char **argv, char m)
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
static BaseType_t ver_ctl(int argc, char **argv, char m)
{
  int copied = 0;
  copied += snprintf(m + copied, SCRATCH_SIZE - copied, "Version %s built at %s.\r\n", gitVersion(),
                     buildTime());
  configASSERT(copied < SCRATCH_SIZE);

  return pdFALSE;
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

static BaseType_t snapshot(int argc, char **argv, char m)
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

// takes no arguments
static BaseType_t stack_ctl(int argc, char **argv, char m)
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
      message = SOFTUART_ENABLE_TRANSMIT;
      copied += snprintf(m + copied, SCRATCH_SIZE - copied, "%s: transmit on\r\n", argv[0]);
    }
    else if (strncmp(argv[1], "off", 3) == 0) {
      message = SOFTUART_DISABLE_TRANSMIT;
      copied += snprintf(m + copied, SCRATCH_SIZE - copied, "%s: transmit off\r\n", argv[0]);
    }
#ifdef SUART_TEST_MODE
    else if (strncmp(argv[1], "debug1", 6) == 0) {
      message = SOFTUART_TEST_SINGLE;
      copied +=
          snprintf(m + copied, SCRATCH_SIZE - copied, "%s: debug mode 1 (single)\r\n", argv[0]);
    }
    else if (strncmp(argv[1], "debugraw", 8) == 0) {
      message = SOFTUART_TEST_RAW;
      copied += snprintf(m + copied, SCRATCH_SIZE - copied, "%s: debug raw (single)\r\n", argv[0]);
    }
    else if (strncmp(argv[1], "debug2", 6) == 0) {
      message = SOFTUART_TEST_INCREMENT;
      copied +=
          snprintf(m + copied, SCRATCH_SIZE - copied, "%s: debug mode 2 (increment)\r\n", argv[0]);
    }
    else if (strncmp(argv[1], "normal", 5) == 0) {
      message = SOFTUART_TEST_OFF;
      copied += snprintf(m + copied, SCRATCH_SIZE - copied, "%s: regular mode\r\n", argv[0]);
    }
    else if (strncmp(argv[1], "sendone", 7) == 0) {
      message = SOFTUART_TEST_SEND_ONE;
      copied += snprintf(m + copied, SCRATCH_SIZE - copied, "%s: send one\r\n", argv[0]);
    }
    else if (strncmp(argv[1], "status", 5) == 0) {
      uint8_t mode = getSUARTTestMode();
      uint8_t sensor = getSUARTTestSensor();
      uint16_t data = getSUARTTestData();
      copied += snprintf(m + copied, SCRATCH_SIZE - copied,
                         "%s: test mode = %s, sensor = 0x%x, data = 0x%x\r\n", argv[0],
                         mode == 0 ? "single" : "increment", sensor, data);
    }
#endif // SUART_TEST_MODE
    else {
      understood = false;
    }
  }
#ifdef SUART_TEST_MODE
  else if (argc == 4) {
    if (strncmp(argv[1], "settest", 7) == 0) {
      uint8_t sensor = strtol(argv[2], NULL, 16);
      uint16_t data = strtol(argv[3], NULL, 16);
      setSUARTTestData(sensor, data);
      copied += snprintf(m + copied, SCRATCH_SIZE - copied,
                         "%s: set test sensor, data to 0x%x, 0x%x\r\n", argv[0], sensor, data);
    }
    else {
      understood = false;
    }
  }
#endif // SUART_TEST_MODE
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
    xQueueSendToBack(xSoftUartQueue, &message, pdMS_TO_TICKS(10));
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
#ifdef SUART_TEST_MODE
        "suart (on|off|status|debug1|debug2|debugraw|normal|sendone|(settest <sensor> <val>))\r\n"
#else
        "suart (on|off)\r\n"
#endif // SUART_TEST_MODE
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
        int retval = commands[i].interpreter(argc, argv, m);
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
