/*
 * ZynqCommands.c
 *
 * Zynq monitor CLI command handler, extracted from SoftwareCommands.c.
 */

#include <strings.h>

#include "commands/ZynqCommands.h"
#include "commands/parameters.h"
#include "Tasks.h"
#include "projdefs.h"

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
    snprintf(m + copied, SCRATCH_SIZE - copied, "%s: Sending message %s\r\n", argv[0], argv[1]);
    // Send a message to the zmon task
    xQueueSendToBack(xZynqMonQueue, &message, pdMS_TO_TICKS(10));
  }
  return pdFALSE;
}
