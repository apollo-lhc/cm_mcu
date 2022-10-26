#include <assert.h>
#include <strings.h>

#include "commands/SoftwareCommands.h"
#include "FreeRTOS.h"
#include "Tasks.h"
#include "portmacro.h"
#include "Semaphore.h"
#include "projdefs.h"
#define SCRATCH_SIZE 512

// takes no arguments
BaseType_t stack_ctl(int argc, char **argv, char *m)
{
  int copied = 0;
  int i = SystemStackWaterHighWaterMark();
  copied += snprintf(m + copied, SCRATCH_SIZE - copied, "stack: %d of %d untouched\r\n", i,
                     SYSTEM_STACK_SIZE);
  return pdFALSE;
}

BaseType_t mem_ctl(int argc, char **argv, char *m)
{
  size_t heapSize = xPortGetFreeHeapSize();
  snprintf(m, SCRATCH_SIZE, "heap: %d bytes\r\n", heapSize);
  return pdFALSE;
}

void TaskGetRunTimeStats(char *pcWriteBuffer, size_t bufferLength)
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

BaseType_t uptime(int argc, char **argv, char *m)
{
  int s = SCRATCH_SIZE;
  TickType_t now = xTaskGetTickCount() / (configTICK_RATE_HZ * 60); // time in minutes
  snprintf(m, s, "%s: MCU uptime %ld minutes\r\n", argv[0], now);
  return pdFALSE;
}

// WARNING: this command easily leads to stack overflows. It does not correctly
// ensure that there are no overwrites to pcCommandString.
BaseType_t TaskStatsCommand(int argc, char **argv, char *m)
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

BaseType_t watchdog_ctl(int argc, char **argv, char *m)
{
  int copied = 0;
  uint16_t stat = task_watchdog_get_status();
  copied = snprintf(m + copied, SCRATCH_SIZE - copied, "%s: status 0x%08x\r\n", argv[0], stat);
  return pdFALSE;
}

BaseType_t zmon_ctl(int argc, char **argv, char *m)
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
BaseType_t log_ctl(int argc, char **argv, char *m)
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

// this command takes no arguments
BaseType_t ver_ctl(int argc, char **argv, char *m)
{
  int copied = 0;
  copied += snprintf(m + copied, SCRATCH_SIZE - copied, "Version %s built at %s.\r\n", gitVersion(),
                     buildTime());
  configASSERT(copied < SCRATCH_SIZE);

  return pdFALSE;
}

////////////////////////////////////////////////////////////////////////
/*-----------------------------------------------------------*/
#define tskRUNNING_CHAR   ('X')
#define tskBLOCKED_CHAR   ('B')
#define tskREADY_CHAR     ('R')
#define tskDELETED_CHAR   ('D')
#define tskSUSPENDED_CHAR ('S')

portBASE_TYPE taskInfo(int argc, char *argv[], char *m)
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


// take or release one of the i2c semaphores
BaseType_t sem_ctl(int argc, char **argv, char *m)
{
  // which semaphore
  int number = atoi(argv[1]);
  SemaphoreHandle_t s = 0;
  switch (number) {
    case 1:
      s = i2c1_sem;
      break;
    case 2:
      s = i2c2_sem;
      break;
    case 3:
      s = i2c3_sem;
      break;
    case 4:
      s = i2c4_sem;
      break;
    case 5:
      s = i2c5_sem;
      break;
    case 6:
      s = i2c6_sem;
      break;
    default:
      snprintf(m, SCRATCH_SIZE, "%s: value must be between 1-6 (got %s)\r\n", argv[0], argv[1]);
      return pdFALSE;
      break;
  }
  // options are
  // take or release
  if ( strncmp(argv[2], "release", 5) == 0 ) {
    // check if we have the semaphore
    if ( xSemaphoreGetMutexHolder(s) == xTaskGetCurrentTaskHandle()) {
      xSemaphoreGive(s);
      snprintf(m, SCRATCH_SIZE, "%s: releasing semaphore %d\r\n", argv[0], number);
    }
    else { // we don't hold this semaphore
      snprintf(m, SCRATCH_SIZE, "%s: trying to release a semaphore %d we don't hold\r\n", argv[0], number);
    }
  }
  else if (strncmp(argv[2], "take", 5) == 0) {
    // try to acquire the semaphore. Wait a finite amount of time.
    if ( xSemaphoreTake(i2c1_sem, (TickType_t)50) == pdTRUE ) {
      snprintf(m, SCRATCH_SIZE, "%s: taking semaphore %d\r\n", argv[0], number);
    }
    else {
      snprintf(m, SCRATCH_SIZE, "%s: failed to acquire semaphore %d in time\r\n", argv[0], number);
    }
  }
  else {
    snprintf(m, SCRATCH_SIZE, "%s: argument must be 'release' or 'take' (got %s)\r\n", argv[0], argv[2]);
  }
  return pdFALSE;
}
