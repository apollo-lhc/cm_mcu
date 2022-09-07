/*
 * CommandLineTask.c
 *
 *  Created on: Apr 7, 2019
 *      Author: wittich, rzou
 */

// Include commands

#include <strings.h>
#include <assert.h>

#include "commands/BoardCommands.h"
#include "commands/BufferCommands.h"
#include "commands/EEPROMCommands.h"
#include "commands/I2CCommands.h"
#include "commands/SensorControl.h"
#include "common/smbus_units.h"
#include "common/printf.h"
#include "common/log.h"

static char m[SCRATCH_SIZE];

// this command takes no arguments and never returns.
static BaseType_t bl_ctl(int argc, char **argv, char *m)
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
static BaseType_t clock_ctl(int argc, char **argv, char *m)
{
  int copied = 0;
  int status = -1; // shut up clang compiler warning
  BaseType_t i = strtol(argv[1], NULL, 10);
  if (!((i == 1) || (i == 2))) {
    copied +=
        snprintf(m + copied, SCRATCH_SIZE - copied,
                 "Invalid mode %ld for clock, only 1 (reset) and 2 (program) supported\r\n", i);
    return pdFALSE;
  }
  copied += snprintf(m + copied, SCRATCH_SIZE - copied, "%s mode set to %ld. \r\n", argv[0], i);
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

#ifdef REV2
// this command takes one argument (from triplet version but will take two argument to include an input from config versions for octlet eeprom)
static BaseType_t init_load_clock_ctl(int argc, char **argv, char *m)
{
  int copied = 0;
  char *clk_ids[5] = {"r0a", "r0b", "r1a", "r1b", "r1c"};
  BaseType_t i = strtol(argv[1], NULL, 10);
  if (i < 0 || i > 4) {
    snprintf(m + copied, SCRATCH_SIZE - copied,
             "Invalid clock chip %ld , the clock id options are r0a:0, r0b:1, r1a:2, "
             "r1b:3 and r1c:4 \r\n",
             i);
    return pdFALSE;
  }
  copied += snprintf(m + copied, SCRATCH_SIZE - copied, "%s is programming clock %s. \r\n", argv[0], clk_ids[i]);
  int status = -1;           // shut up clang compiler warning
  status = init_load_clk(i); // status is 0 if all registers can be written to a clock chip. otherwise, it implies that some write registers fail in a certain list.
  if (status == 0) {
    snprintf(m + copied, SCRATCH_SIZE - copied,
             "clock synthesizer with id %s successfully programmed. \r\n", clk_ids[i]);
  }
  else {
    snprintf(m + copied, SCRATCH_SIZE - copied, "%s operation failed \r\n", argv[0]);
  }
  return pdFALSE;
}
#endif // REV2

// this command takes no arguments
static BaseType_t ver_ctl(int argc, char **argv, char *m)
{
  int copied = 0;
  copied += snprintf(m + copied, SCRATCH_SIZE - copied, "Version %s built at %s.\r\n", gitVersion(),
                     buildTime());
  configASSERT(copied < SCRATCH_SIZE);

  return pdFALSE;
}

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

static BaseType_t snapshot(int argc, char **argv, char *m)
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
  if (which < 0 || which > (NSUPPLIES_PS - 1)) {
    copied += snprintf(m + copied, SCRATCH_SIZE - copied, "%s: device %d must be between 0-%d\r\n",
                       argv[0], which, (NSUPPLIES_PS - 1));
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
static BaseType_t stack_ctl(int argc, char **argv, char *m)
{
  int copied = 0;
  int i = SystemStackWaterHighWaterMark();
  copied += snprintf(m + copied, SCRATCH_SIZE - copied, "stack: %d of %d untouched\r\n", i,
                     SYSTEM_STACK_SIZE);
  return pdFALSE;
}

static BaseType_t mem_ctl(int argc, char **argv, char *m)
{
  size_t heapSize = xPortGetFreeHeapSize();
  snprintf(m, SCRATCH_SIZE, "heap: %d bytes\r\n", heapSize);
  return pdFALSE;
}

static void TaskGetRunTimeStats(char *pcWriteBuffer, size_t bufferLength)
{
  TaskStatus_t *pxTaskStatusArray;
  volatile UBaseType_t uxArraySize;
  uint32_t ulTotalRunTime;

  // Make sure the write buffer does not contain a string.
  *pcWriteBuffer = '\0';

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
      for (UBaseType_t x = 0; x < uxArraySize; x++) {
        // What percentage of the total run time has the task used?
        // This will always be rounded down to the nearest integer.
        // ulTotalRunTimeDiv100 has already been divided by 100.
        uint32_t ulStatsAsPercentage = pxTaskStatusArray[x].ulRunTimeCounter / ulTotalRunTime;

        if (ulStatsAsPercentage > 0UL) {
          snprintf(pcWriteBuffer, bufferLength, "%s\t%12lu\t%2lu%%\r\n",
                   pxTaskStatusArray[x].pcTaskName, pxTaskStatusArray[x].ulRunTimeCounter,
                   ulStatsAsPercentage);
        }
        else {
          // If the percentage is zero here then the task has
          // consumed less than 1% of the total run time.
          snprintf(pcWriteBuffer, bufferLength, "%s\t%12lu\t<1%%\r\n",
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

static BaseType_t uptime(int argc, char **argv, char *m)
{
  int s = SCRATCH_SIZE;
  TickType_t now = xTaskGetTickCount() / (configTICK_RATE_HZ * 60); // time in minutes
  snprintf(m, s, "%s: MCU uptime %ld minutes\r\n", argv[0], now);
  return pdFALSE;
}

// WARNING: this command easily leads to stack overflows. It does not correctly
// ensure that there are no overwrites to pcCommandString.
static BaseType_t TaskStatsCommand(int argc, char **argv, char *m)
{
  int s = SCRATCH_SIZE;
  const char *const pcHeader = "            Time     %\r\n"
                               "********************************\r\n";
  BaseType_t xSpacePadding;
  unsigned int copied = 0;
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
  TaskGetRunTimeStats(m + copied, SCRATCH_SIZE - copied);
  unsigned int len = strlen(m);
  configASSERT(len < SCRATCH_SIZE);

  /* There is no more data to return after this single string, so return
  pdFALSE. */
  return pdFALSE;
}

static BaseType_t help_command_fcn(int argc, char **, char *m);

static BaseType_t watchdog_ctl(int argc, char **argv, char *m)
{
  int copied = 0;
  uint16_t stat = task_watchdog_get_status();
  copied = snprintf(m + copied, SCRATCH_SIZE - copied, "%s: status 0x%08x\r\n", argv[0], stat);
  return pdFALSE;
}

static BaseType_t first_mcu_ctl(int argc, char **argv, char *m);

static BaseType_t zmon_ctl(int argc, char **argv, char *m)
{
  int copied = 0;
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
      uint8_t mode = getZYNQMonTestMode();
      uint8_t sensor = getZYNQMonTestSensor();
      uint16_t data = getZYNQMonTestData();
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
      setZYNQMonTestData(sensor, data);
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
    copied += snprintf(m + copied, SCRATCH_SIZE - copied, "%s: message not understood >", argv[0]);
    for (int i = 0; i < argc; ++i) {
      copied += snprintf(m + copied, SCRATCH_SIZE - copied, "%s ", argv[i]);
    }
    snprintf(m + copied, SCRATCH_SIZE - copied, "<\r\n");
    return pdFALSE;
  }

  if (message) {
    copied +=
        snprintf(m + copied, SCRATCH_SIZE - copied, "%s: Sending message %s\r\n", argv[0], argv[1]);
    // Send a message to the zmon task
    xQueueSendToBack(xZynqMonQueue, &message, pdMS_TO_TICKS(10));
  }
  return pdFALSE;
}

// this command takes up to two arguments
static BaseType_t log_ctl(int argc, char **argv, char *m)
{
  int copied = 0;
  if (argc == 2) {
    if (strncmp(argv[argc - 1], "toggle", 6) == 0) {
      bool newval = !log_get_quiet();
      log_set_quiet(newval);
      snprintf(m, SCRATCH_SIZE, "%s: quiet set to %d\r\n", argv[0], newval);
    }
    else if (strncmp(argv[argc - 1], "status", 6) == 0) {
      copied += snprintf(m + copied, SCRATCH_SIZE - copied, "%s: status\r\n", argv[0]);
      for (enum log_facility_t i = 0; i < NUM_LOG_FACILITIES; ++i) {
        int level = log_get_current_level(i);
        copied += snprintf(m + copied, SCRATCH_SIZE - copied, "%s: %-7s = %s\r\n", argv[0], log_facility_string(i),
                           log_level_string(level));
      }
    }
    else if (strncmp(argv[argc - 1], "dump", 4) == 0) {
      // note that this is different from other functions because it does not
      // use the intermediate buffer and just prints directy to the callback function.
      log_dump(Print);
    }
    else {
      snprintf(m, SCRATCH_SIZE, "%s: command %s not understood\r\n", argv[0], argv[1]);
    }
  }
  else if (argc == 3) {
    int j = 0;
    size_t len = strlen(argv[1]);
    bool success = false;
    const char *f, *l;
    for (; j < NUM_LOG_FACILITIES; ++j) {
      f = log_facility_string(j);
      if (strncasecmp(argv[1], f, len) == 0) {
        break;
      }
    }
    len = strlen(argv[2]);
    int i = 0;
    for (; i < NUM_LOG_LEVELS && j < NUM_LOG_FACILITIES; ++i) {
      l = log_level_string(i);
      if (strncasecmp(argv[2], l, len) == 0) {
        log_set_level(i, j);
        success = true;
        break;
      }
    }
    if (success) {
      snprintf(m, SCRATCH_SIZE, "%s: set logging level for facility %s to %s\r\n", argv[0], f, l);
    }
    else {
      snprintf(m, SCRATCH_SIZE, "%s: facility %s not recognized\r\n", argv[0], argv[1]);
    }
  }
  else {
    copied += snprintf(m + copied, SCRATCH_SIZE - copied, "%s: argument(s)", argv[0]);
    for (int i = 1; i < argc; ++i) {
      copied += snprintf(m + copied, SCRATCH_SIZE - copied, "%s ", argv[i]);
    }
    copied += snprintf(m + copied, SCRATCH_SIZE - copied, "not understood\r\n");
  }

  return pdFALSE;
}

////////////////////////////////////////////////////////////////////////
/*-----------------------------------------------------------*/
#define tskRUNNING_CHAR   ('X')
#define tskBLOCKED_CHAR   ('B')
#define tskREADY_CHAR     ('R')
#define tskDELETED_CHAR   ('D')
#define tskSUSPENDED_CHAR ('S')

static portBASE_TYPE taskInfo(int argc, char *argv[], char *m)
{
  const char *const pcHeader = "Task   State  Priority  Stack  #\r\n*********************************\r\n";

  /* Generate a table of task stats. */
  static_assert(sizeof(m) >= sizeof(pcHeader), "m too small");
  strcpy(m, pcHeader);
  unsigned int copied = strlen(m);

  /* Take a snapshot of the number of tasks in case it changes while this
    function is executing. */
  UBaseType_t uxArraySize = uxTaskGetNumberOfTasks();

  /* Allocate an array index for each task.  NOTE!  if
    configSUPPORT_DYNAMIC_ALLOCATION is set to 0 then pvPortMalloc() will
    equate to NULL. */
  TaskStatus_t *pxTaskStatusArray = pvPortMalloc(uxArraySize * sizeof(TaskStatus_t)); /*lint !e9079 All values returned by pvPortMalloc() have at least the alignment required by the MCU's stack and this allocation allocates a struct that has the alignment requirements of a pointer. */

  if (pxTaskStatusArray != NULL) {
    /* Generate the (binary) data. */
    uxTaskGetSystemState(pxTaskStatusArray, uxArraySize, NULL);
    char cStatus;

    /* Create a human readable table from the binary data. */
    for (UBaseType_t x = 0; x < uxArraySize; x++) {
      switch (pxTaskStatusArray[x].eCurrentState) {
        case eRunning:
          cStatus = tskRUNNING_CHAR;
          break;
        case eReady:
          cStatus = tskREADY_CHAR;
          break;
        case eBlocked:
          cStatus = tskBLOCKED_CHAR;
          break;
        case eSuspended:
          cStatus = tskSUSPENDED_CHAR;
          break;
        case eDeleted:
          cStatus = tskDELETED_CHAR;
          break;
        case eInvalid:
        default:
          cStatus = (char)0x00;
          break;
      }
      copied += snprintf(m + copied, SCRATCH_SIZE - copied, "%-6s",
                         pxTaskStatusArray[x].pcTaskName);

      // note that the Stack high water mark shows the smallest the
      // stack has ever been. Smaller == closer to overflow.
      copied += snprintf(m + copied, SCRATCH_SIZE - copied, "\t%c\t%u\t%u\t%u\r\n",
                         cStatus, (unsigned int)pxTaskStatusArray[x].uxCurrentPriority,
                         (unsigned int)pxTaskStatusArray[x].usStackHighWaterMark,
                         (unsigned int)pxTaskStatusArray[x].xTaskNumber);
    }

    /* Free the array again.  NOTE!  If configSUPPORT_DYNAMIC_ALLOCATION
      is 0 then vPortFree() will be #defined to nothing. */
    vPortFree(pxTaskStatusArray);
  }

  unsigned int len = strlen(m);
  configASSERT(len < SCRATCH_SIZE);
  /* There is no more data to return after this single string, so return
  pdFALSE. */
  return pdFALSE;
}
/*-----------------------------------------------------------*/
////////////////////////////////////////////////////////////////////////

static const char *const pcWelcomeMessage =
    "CLI based on microrl.\r\nType \"help\" to view a list of registered commands.\r\n";

struct command_t {
  const char *commandstr;
  BaseType_t (*interpreter)(int argc, char **, char *m);
  const char *helpstr;
  const int num_args;
};

#define NUM_COMMANDS (sizeof(commands) / sizeof(commands[0]))
static struct command_t commands[] = {
    {"adc", adc_ctl, "Displays a table showing the state of ADC inputs.\r\n", 0},
    {"alm", alarm_ctl, "args: (clear|status|settemp #)\r\nGet or clear status of alarm task.\r\n",
     -1},
    {"bootloader", bl_ctl, "Call the boot loader\r\n", 0},
    {"clkmon", clkmon_ctl, "Displays a table showing the clock chips' statuses given the clock chip id option\r\n", 1},
    {"clock", clock_ctl,
     "args: (1|2)\r\nReset (1) or program the clock synthesizer to 156.25 MHz (2).\r\n", 1},
    {"eeprom_info", eeprom_info, "Prints information about the EEPROM.\r\n", 0},
    {"eeprom_read", eeprom_read,
     "args: <address>\r\nReads 4 bytes from EEPROM. Address should be a multiple of 4.\r\n",
     1},
    {"eeprom_write", eeprom_write,
     "args: <address> <data>\r\nWrites <data> to <address> in EEPROM. <address> should be "
     "a multiple of 4.\r\n",
     2},
    {"errorlog_entry", errbuff_in,
     "args: <data>\r\nManual entry of 2-byte code into the eeprom error logger.\r\n", 1},
    {"errorlog", errbuff_out,
     "args: <n>\r\nPrints last n entries in the eeprom error logger.\r\n", 1},
    {"errorlog_info", errbuff_info,
     "Prints information about the eeprom error logger.\r\n", 0},
    {"errorlog_reset", errbuff_reset,
     "Resets the eeprom error logger.\r\n", 0},
    {"first_mcu", first_mcu_ctl, "args: <board #> <revision #>\r\n Detect first-time setup of MCU and prompt loading internal EEPROM configuration\r\n", 4},
    {"fpga_reset", fpga_reset, "Reset Kintex (k) or Virtex (V) FPGA\r\n", 1},
    {"ff", ff_ctl,
     "args: (xmit|cdr on/off (0-23|all)) | regw reg# val (0-23|all) | regr reg# (0-23)\r\n"
     " Firefly controlling and monitoring commands\r\n",
     -1},
    {
        "ff_status",
        ff_status,
        "Displays a table showing the status of the fireflies.\r\n",
        0,
    },
    {
        "ff_los",
        ff_los_alarm,
        "Displays a table showing the loss of signal alarms of the fireflies.\r\n",
        0,
    },
    {
        "ff_cdr_lol",
        ff_cdr_lol_alarm,
        "Displays a table showing the CDR loss of lock alarms of the fireflies.\r\n",
        0,
    },
    {
        "ff_temp",
        ff_temp,
        "Displays a table showing the temperature of the I2C fireflies.\r\n",
        -1,
    },
    {"fpga", fpga_ctl, "Displays a table showing the state of FPGAs.\r\n",
     -1},
    {
        "gpio",
        gpio_ctl,
        "Get or set any GPIO pin.\r\n",
        -1,
    },
    {"help", help_command_fcn, "This help command\r\n", -1},
    {"id", board_id_info, "Prints board ID information.\r\n", 0},
    {"i2cr", i2c_ctl_r,
     "args: <dev> <address> <number of bytes>\r\nRead I2C controller. Addr in hex.\r\n", 3},
    {"i2crr", i2c_ctl_reg_r,
     "i2crr <dev> <address> <n reg addr bytes> <reg addr> <n data bytes> \r\n Read I2C controller. Addr in hex\r\n", 5},
    {"i2cw", i2c_ctl_w, "i2cw <dev> <address> <number of bytes> <value>\r\n Write I2C controller.\r\n",
     4},
    {"i2cwr", i2c_ctl_reg_w,
     "args: <dev> <address> <number of reg bytes> <reg> <number of bytes>\r\nWrite I2C controller.\r\n", 6},
    {
        "i2c_scan",
        i2c_scan,
        "Scan current I2C bus.\r\n",
        1,
    },
#ifdef REV2
    {
        "jtag_sm",
        jtag_sm_ctl,
        "(on|off) set the JTAG from SM or not\r\n",
        -1,
    },
    {
        "loadclock",
        init_load_clock_ctl,
        "args: the clock id options to program are r0a:0, r0b:1, r1a:2, r1b:3 and r1c:4.\r\n",
        1,
    },
#endif // REV2
    {
        "log",
        log_ctl,
        "args: (<fac> debug|toggle|info|warn|fatal|trace)(status|quiet)\r\nManipulate logger levels\r\n",
        -1,
    },
    {"led", led_ctl, "Manipulate red LED.\r\n", 1},
    {"mem", mem_ctl, "Size of heap.\r\n", 0},
    {
        "pwr",
        power_ctl,
        "args: (on|off|status|clearfail)\r\nTurn on or off all power, get status or clear "
        "failures.\r\n",
        1,
    },
    {"psmon", psmon_ctl, "Displays a table showing the state of power supplies.\r\n", 1},
    {"psreg", psmon_reg, "<which> <reg>. which: LGA80D (10*dev+page), reg: reg address in hex\r\n", 2},
    {"restart_mcu", restart_mcu, "Restart the microcontroller\r\n", 0},
    {"snapshot", snapshot,
     "args:# (0|1)\r\nDump snapshot register. #: which of 5 LGA80D (10*dev+page). 0|1 decide "
     "if to reset snapshot.\r\n",
     2},
    {
        "set_id",
        set_board_id,
        "args: <passwd> <addr> <data>\r\nAllows the user to set the board id "
        "information.\r\n",
        3,
    },
    {
        "set_id_password",
        set_board_id_password,
        "One-time use: sets password for ID block.\r\n",
        0,
    },
    {
        "simple_sensor",
        sensor_summary,
        "Displays a table showing the state of temps.\r\n",
        0,
    },
    {
        "stack_usage",
        stack_ctl,
        "Print out system stack high water mark.\r\n",
        0,
    },
    {
        "taskinfo",
        taskInfo,
        "Info about FreeRTOS tasks\r\n",
        0,
    },
    {"taskstats",
     TaskStatsCommand,
     "Displays a table showing the state of each FreeRTOS task\r\n", 0},
#ifdef REV2
    {
        "time",
        time_ctl,
        "(set HH:MM:SS MM/DD/YYYY|<none)\r\nRTC set and display\r\n",
        -1,
    },
#endif // REV2
    {"uptime", uptime, "Display uptime in minutes\r\n", 0},
    {"version", ver_ctl, "Display information about MCU firmware version\r\n", 0},
    {"watchdog", watchdog_ctl, "Display status of the watchdog task\r\n", 0},
    {
        "zmon",
        zmon_ctl,
#ifdef ZYNQMON_TEST_MODE
        "args:(on|off|status|debug1|debug2|debugraw|normal|sendone|(settest <sensor> <val>))\r\n"
#else
        "args:(on|off)\r\n"
#endif // ZYNQMON_TEST_MODE
        " Control ZynqMon task.\r\n",
        -1,
    },

};

#ifdef REV1
static void U4Print(const char *str)
{
  UARTPrint(UART4_BASE, str);
}
static void U1Print(const char *str)
{
  UARTPrint(UART1_BASE, str);
}
#elif defined(REV2) // REV1
static void U0Print(const char *str)
{
  UARTPrint(UART0_BASE, str);
}
#endif

struct microrl_user_data_t {
  uint32_t uart_base;
};

static BaseType_t help_command_fcn(int argc, char **argv, char *m)
{
  int copied = 0;
  if (argc == 1) {
    static int i = 0;
    for (; i < NUM_COMMANDS; ++i) {
      int left = SCRATCH_SIZE - copied;
      // need room for command string, help string, newlines, etc, and trailing \0
      unsigned int len = strlen(commands[i].helpstr) + strlen(commands[i].commandstr) + 7;
      if (left < len) {
        return pdTRUE;
      }
      copied += snprintf(m + copied, SCRATCH_SIZE - copied, "%s:\r\n %s",
                         commands[i].commandstr, commands[i].helpstr);
    }
    i = 0;
    return pdFALSE;
  }
  else { // help on a specific command.
    // help for any command that matches the entered command
    for (int j = 0; j < NUM_COMMANDS; ++j) {
      if (strncmp(commands[j].commandstr, argv[1], strlen(argv[1])) == 0) {
        copied += snprintf(m + copied, SCRATCH_SIZE - copied, "%s:\r\n %s",
                           commands[j].commandstr, commands[j].helpstr);
        // return pdFALSE;
      }
    }
  }
  if (copied == 0) {
    snprintf(m + copied, SCRATCH_SIZE - copied,
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
          retval = commands[i].interpreter(argc, argv, m);
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

static BaseType_t first_mcu_ctl(int argc, char **argv, char *m)
{

  if (read_eeprom_single(EEPROM_ID_SN_ADDR) == 0xffffffff) {
    uint32_t board_id, rev, ps_mask, data;
    uint64_t pass, addr_id, addr_ff, addr_ps;
    board_id = strtoul(argv[1], NULL, 16);
    rev = strtoul(argv[2], NULL, 16);
    ff_USER_mask = strtoul(argv[3], NULL, 16);
    ps_mask = strtoul(argv[4], NULL, 16);
    snprintf(m, SCRATCH_SIZE, "Registering board_id %lx revision %lx, USER ff mass %lx and PS ignore mask %lx \r\n", board_id, rev, ff_USER_mask, ps_mask);

    pass = 0x12345678;
    addr_id = 0x40; // internal eeprom block for board id
    addr_ff = 0x44; // internal eeprom block for ps ignore fail
    addr_ps = 0x48; // internal eeprom block for ps ignore fail

    data = (board_id << 16) + rev;
    uint64_t block = EEPROMBlockFromAddr(addr_id);

    uint64_t unlock = EPRMMessage((uint64_t)EPRM_UNLOCK_BLOCK, block, pass);
    xQueueSendToBack(xEPRMQueue_in, &unlock, portMAX_DELAY);

    uint64_t message = EPRMMessage((uint64_t)EPRM_WRITE_SINGLE, addr_id, data);
    xQueueSendToBack(xEPRMQueue_in, &message, portMAX_DELAY);

    uint64_t lock = EPRMMessage((uint64_t)EPRM_LOCK_BLOCK, block << 32, 0);
    xQueueSendToBack(xEPRMQueue_in, &lock, portMAX_DELAY);

    data = ff_USER_mask;
    block = EEPROMBlockFromAddr(addr_ff);

    unlock = EPRMMessage((uint64_t)EPRM_UNLOCK_BLOCK, block, pass);
    xQueueSendToBack(xEPRMQueue_in, &unlock, portMAX_DELAY);

    message = EPRMMessage((uint64_t)EPRM_WRITE_SINGLE, addr_ff, data);
    xQueueSendToBack(xEPRMQueue_in, &message, portMAX_DELAY);

    lock = EPRMMessage((uint64_t)EPRM_LOCK_BLOCK, block << 32, 0);
    xQueueSendToBack(xEPRMQueue_in, &lock, portMAX_DELAY);

    data = ps_mask;
    block = EEPROMBlockFromAddr(addr_ps);

    unlock = EPRMMessage((uint64_t)EPRM_UNLOCK_BLOCK, block, pass);
    xQueueSendToBack(xEPRMQueue_in, &unlock, portMAX_DELAY);

    message = EPRMMessage((uint64_t)EPRM_WRITE_SINGLE, addr_ps, data);
    xQueueSendToBack(xEPRMQueue_in, &message, portMAX_DELAY);

    lock = EPRMMessage((uint64_t)EPRM_LOCK_BLOCK, block << 32, 0);
    xQueueSendToBack(xEPRMQueue_in, &lock, portMAX_DELAY);
  }
  else {
    uint32_t sn = read_eeprom_single(EEPROM_ID_SN_ADDR);

    uint32_t num = (uint32_t)sn >> 16;
    uint32_t rev = ((uint32_t)sn) & 0xff;
    snprintf(m, SCRATCH_SIZE, "This is not the first-time loading MCU FW to board #%lx (rev %lx) \r\n", num, rev);
  }

  return pdFALSE;
}

extern struct dev_moni2c_addr_t ff_moni2c_addrs[NFIREFLIES];

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

#ifdef REV1
  void (*printer)(const char *) = U4Print;
#elif defined(REV2)
  void (*printer)(const char *) = U0Print;
#endif

  struct microrl_config rl_config = {
      .print = printer, // default to front panel
      // set callback for execute
      .execute = execute,
      .prompt_str = "% ",
      .prompt_length = 2,
      .userdata = &rl_userdata,
  };
#ifdef REV1
  // this is a hack
  if (uart_base == UART1_BASE) {
    rl_config.print = U1Print; // switch to Zynq
  }
#endif // REV1
  microrl_t rl;
  microrl_init(&rl, &rl_config);
  microrl_set_execute_callback(&rl, execute);
  microrl_insert_char(&rl, ' '); // this seems to be necessary?

  for (;;) {
    /* This implementation reads a single character at a time.  Wait in the
       Blocked state until a character is received. */
    xStreamBufferReceive(uartStreamBuffer, &cRxedChar, 1, portMAX_DELAY);
    microrl_insert_char(&rl, cRxedChar);

    // monitor stack usage for this task
    CHECK_TASK_STACK_USAGE(args->stack_size);
  }
}
