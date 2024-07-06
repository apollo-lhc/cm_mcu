/*
 * GenericAlarmTask.c
 *
 *  Created on: August 11, 2020
 *      Author: pw94
 *
 *  Generic alarm task that uses a callback and dispatches alarms if it deems fit.
 */
#include "FreeRTOS.h"
#include "Tasks.h"
#include "common/log.h"
#include "common/power_ctl.h"

#include "AlarmUtilities.h"

#define X_MACRO_ALM_STATES \
    X(ALM_INIT) \
    X(ALM_NORMAL) \
    X(ALM_WARN) \
    X(ALM_ERROR)

enum alarm_task_state {
#define X(name) name,
  X_MACRO_ALM_STATES
#undef X
};

// alarm state names
static const char *alarm_task_state_names[] = {
#define X(name) #name,
    X_MACRO_ALM_STATES
#undef X
};

// ALARM TASK STATE MACHINE
// +------+
// | INIT |
// +---+--+
//     |
//     |
//  +--+--------+      +---------+      +-------+
//  |   NORMAL  +----->+  WARN   +----->+ ERROR |
//  +-----+-----+      +-+-------+      +---+---+
//        ^              |                  |
//        +--------------+                  |
//        ^                                 |
//        +---------------------------------+
// once we get into the ERROR state the only way to clear an error is via a message
// sent to the CLI.
//


void GenericAlarmTask(void *parameters)
{
  struct GenericAlarmParams_t *params = parameters;

  char *taskName = pcTaskGetTaskName(NULL); // get the name of the task

  // initialize to the current tick time
  TickType_t xLastWakeTime = xTaskGetTickCount();
  uint32_t message; // this must be in a semi-permanent scope
  bool alarming;

  enum alarm_task_state currentState = ALM_INIT;
  for (;;) {
    vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(50));
    if (xQueueReceive(params->xAlmQueue, &message, 0)) {
      log_debug(LOG_ALM, "%s: received message %d (%s)\r\n", taskName, message, 
          msgqueue_message_text[message]);
      switch (message) {
        case ALM_CLEAR_ALL: // clear all alarms
          alarming = false;
          break;
        default:
          break;
      }
      continue; // we break out of the loop because we want data
      // to refresh
    }

    int status = params->checkStatus();

    // state machine
    enum alarm_task_state nextState;
    switch (currentState) {
      case ALM_INIT: {
        nextState = ALM_NORMAL;
        break;
      }
      case ALM_NORMAL: {
        if (status) {
          params->errorlog_registererror();
          alarming = true;
          nextState = ALM_WARN;
        }
        else {
          alarming = false;
          nextState = ALM_NORMAL;
        }
        break;
      }
      case ALM_WARN: {
        if (!status) {
          // we are back to normal
          params->errorlog_clearerror();
          nextState = ALM_NORMAL;
          alarming = false;
        }
        else if (status > 1) {
          // log alarm, send message to turn off power and move to error state
          if (params->errorlog_registererror)
            params->errorlog_registererror();
          message = TEMP_ALARM;
          // this message always goes to the power queue, for all
          // alarms.
          xQueueSendToFront(xPwrQueue, &message, 100);
          log_debug(LOG_ALM, "sent message %d (%s) to power queue\r\n", TEMP_ALARM, 
              msgqueue_message_text[TEMP_ALARM]);
          nextState = ALM_ERROR;
        }
        else {
          nextState = ALM_WARN;
        }
        break;
      }
      case ALM_ERROR: {
        if (!alarming && !status) {
          // error has cleared, log and move to normal state
          if (params->errorlog_clearerror)
            params->errorlog_clearerror();
          message = TEMP_ALARM_CLEAR;
          // this message always goes to the power queue, for all
          // alarms.
          xQueueSendToFront(xPwrQueue, &message, 100);
          log_debug(LOG_ALM, "sent message %d (%s) to power queue\r\n", TEMP_ALARM_CLEAR,
                    msgqueue_message_text[TEMP_ALARM_CLEAR]);
          nextState = ALM_NORMAL;
        }
        else {
          nextState = ALM_ERROR;
        }
        break;
      }
      default:
        nextState = ALM_ERROR;
        break;
    }
    if (currentState != nextState) {
      log_debug(LOG_ALM, "%s: change from state %s to %s\r\n", taskName,
                alarm_task_state_names[currentState], alarm_task_state_names[nextState]);
    }

    currentState = nextState;

    // monitor stack usage for this task
    CHECK_TASK_STACK_USAGE(params->stack_size);
  }
  return;
}
