/*
 * AlarmTask.c
 *
 *  Created on: Aug 26, 2019
 *      Author: pw94
 *
 *  This task monitors the temperatures (and maybe other quantities in the
 *  future) and dispatches alarms if it deems fit.
 *
 *  The alarms currently are not cleared except by restarting the MCU or
 *  by sending a message from the console.
 */
#include "Tasks.h"
#include "MonitorTask.h"
#include "common/power_ctl.h"
#include "common/utils.h"

// this queue is used to receive messages
QueueHandle_t xAlmQueue;

enum temp_state { TEMP_UNKNOWN, TEMP_GOOD, TEMP_BAD };
enum alarm_state { ALM_UNKNOWN, ALM_GOOD, ALM_BAD };

enum alarm_task_state { ALM_INIT, ALM_NORMAL, ALM_WARN, ALM_ERROR };

// Status of the alarm task
static uint32_t status_T = 0x0;
uint32_t oldstatus;

// read-only, so no need to use queue
uint32_t getAlarmStatus()
{
  return status_T;
}

#define INITIAL_ALARM_TEMP_FF   45.0f // in Celsius duh
#define INITIAL_ALARM_TEMP_DCDC 70.0f
#define INITIAL_ALARM_TEMP_TM4C 70.0f
#define INITIAL_ALARM_TEMP_FPGA 70.0f
#define ALM_OVERTEMP_THRESHOLD  5.0f // if device is this many degrees over alarm temp, turn off power

static float alarm_temp_ff = INITIAL_ALARM_TEMP_FF;
static float alarm_temp_dcdc = INITIAL_ALARM_TEMP_DCDC;
static float alarm_temp_tm4c = INITIAL_ALARM_TEMP_TM4C;
static float alarm_temp_fpga = INITIAL_ALARM_TEMP_FPGA;

float getAlarmTemperature(enum device device_name)
{
  switch (device_name) {
  case TM4C:
    return alarm_temp_tm4c;
  case DCDC:
    return alarm_temp_dcdc;
  case FPGA:
    return alarm_temp_fpga;
  case FF:
    return alarm_temp_ff;
  default:
    return 0;
  }
}

void setAlarmTemperature(enum device device_name, const float newtemp)
{
  switch (device_name) {
  case TM4C:
    alarm_temp_tm4c = newtemp;
    return;
  case DCDC:
    alarm_temp_dcdc = newtemp;
    return;
  case FPGA:
    alarm_temp_fpga = newtemp;
    return;
  case FF:
    alarm_temp_ff = newtemp;
    return;
  default:
    return;
  }
}

void AlarmTask(void *parameters)
{
  // initialize to the current tick time
  TickType_t xLastWakeTime = xTaskGetTickCount();
  uint32_t message; // this must be in a semi-permanent scope
  enum temp_state current_temp_state = TEMP_UNKNOWN;
  enum alarm_task_state currentState, nextState;
  currentState = INIT;
  float temp_over_ff, temp_over_fpga, temp_over_tm4c, temp_over_dcdc;
  float largest_over_temp = 0, temp_over = 0, temp_over_old = 0;

  bool alarming = false;
  bool alarming_temp = false;
  bool alarming_voltage = false;
  bool alarming_current = false;

  vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(2500));  // let things settle, ugh

  for (;;) {
    vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(25));

    if (xQueueReceive(xAlmQueue, &message, 0)) {
      switch (message) {
      case ALM_CLEAR_ALL: // clear all alarms
        alarming = false;
        alarming_temp = false;
        alarming_voltage = false;
        alarming_current = false;
        break;
      case ALM_CLEAR_TEMP: // clear temperature alarm
        alarming_temp = false;
        break;
      case ALM_CLEAR_CURRENT:
        alarming_current = false;
        break;
      case ALM_CLEAR_VOLTAGE:
        alarming_voltage = false;
        break;
      default:
        break;
      }
      continue; // we break out of the loop because we want data
      // to refresh
    }
    // -----------------------------------------
    // temperature alarms
    // -----------------------------------------
    temp_over = 0;
    largest_over_temp = 0;
    status_T = 0;

    // microcontroller
    float tm4c_temp = getADCvalue(ADC_INFO_TEMP_ENTRY);
    if (tm4c_temp > alarm_temp_tm4c) {
      status_T |= ALM_STAT_TM4C_OVERTEMP;
      temp_over_tm4c = tm4c_temp - alarm_temp_tm4c;
      largest_over_temp = temp_over_tm4c;
      temp_over = temp_over_tm4c;
    }

    // FPGA
    float max_fpga;
    if (fpga_args.n_devices == 2) {
      max_fpga = MAX(fpga_args.pm_values[0], fpga_args.pm_values[1]);
    }
    else {
      max_fpga = fpga_args.pm_values[0];
    }
    if (max_fpga > alarm_temp_fpga) {
      status_T |= ALM_STAT_FPGA_OVERTEMP;
      temp_over_fpga = max_fpga - alarm_temp_fpga;
      temp_over += temp_over_fpga;
      if (temp_over_fpga > largest_over_temp) {
        largest_over_temp = temp_over_fpga;
      }
    }

    // DCDC. The first command is READ_TEMPERATURE_1.
    // I am assuming it stays that way!!!!!!!!
    float max_dcdc_temp = -99.0f;
    for (int ps = 0; ps < dcdc_args.n_devices; ++ps) {
      for (int page = 0; page < dcdc_args.n_pages; ++page) {
        size_t index = ps * (dcdc_args.n_commands * dcdc_args.n_pages) +
                       page * dcdc_args.n_commands + 0;
        float thistemp = dcdc_args.pm_values[index];
        if (thistemp > max_dcdc_temp)
          max_dcdc_temp = thistemp;
      }
    }
    if (max_dcdc_temp > alarm_temp_dcdc) {
      status_T |= ALM_STAT_DCDC_OVERTEMP;
      temp_over_dcdc = max_dcdc_temp - alarm_temp_dcdc;
      temp_over += temp_over_dcdc;
      if (temp_over_dcdc > largest_over_temp) {
        largest_over_temp = temp_over_dcdc;
      }
    }

    // Fireflies. These are reported as ints but we are asked
    // to report a float.
    int8_t imax_ff_temp = -99;
    for (int i = 0; i < NFIREFLIES; ++i) {
      int8_t v = getFFvalue(i);
      if (v > imax_ff_temp)
        imax_ff_temp = v;
    }
    if ((float)imax_ff_temp > alarm_temp_ff) {
      status_T |= ALM_STAT_FIREFLY_OVERTEMP;
      temp_over_ff = imax_ff_temp - alarm_temp_ff;
      temp_over += temp_over_ff;
      if (temp_over_ff > largest_over_temp) {
        largest_over_temp = temp_over_ff;
      }
    }
    // -----------------------------------------
    // Current Alarms
    // -----------------------------------------
    // Does Nothing Yet (TM)
    // -----------------------------------------
    // Voltage Alarms
    // -----------------------------------------
    uint32_t status_V;
    if ( getPowerControlState() == POWER_ON ) {
      // only check if the power control state machine says we are in normal operation mode
      // check ADC outputs 
      for (size_t i = 0; i < ADC_CHANNEL_COUNT; ++i) {
        float voltage = getADCvalue(i);
        if ((voltage < .9f * getADCtargetValue(i)) ||
            (voltage > 1.1f * getADCtargetValue(i))) {
          alarming_voltage = true;
          status_V |= 1U<<i;
        }
      } // end loop over ADC channels
      
    }


    // set the overall alarm flag
    alarming = alarming_temp || alarming_voltage || alarming_current;
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
#if 0
    // if temp is over max by ALM_OVERTEMP_THRESHOLD, then turn off power
    if (largest_over_temp > ALM_OVERTEMP_THRESHOLD) {
      message = TEMP_ALARM;
      xQueueSendToFront(xPwrQueue, &message, pdMS_TO_TICKS(100));
    }
    // if temp returns to normal, send buffer message
    if ((temp_over == 0) && (temp_over_old > 0)) {
      errbuffer_put(EBUF_TEMP_NORMAL, 0);
      temp_over_old = temp_over;
    }
    if (status_T && (current_temp_state != TEMP_BAD)) {
      // If temp is bad, turn on alarm, send error message to buffer
      if ((temp_over > temp_over_old) || (status_T != oldstatus)) {
        // only send message when status_T has changed or temp has risen since
        // last entry, to avoid filling up buffer
        errbuffer_temp_high((uint8_t)tm4c_temp, (uint8_t)max_fpga,
                            (uint8_t)imax_ff_temp, (uint8_t)max_dcdc_temp);
        oldstatus = status_T;
        temp_over_old = temp_over;
      }
      current_temp_state = TEMP_BAD;
    }
    else if ((!status_T) && (current_temp_state == TEMP_BAD)) {
      // If status_T is cleared (from cli), turn off alarm, send message to buffer
      message = TEMP_ALARM_CLEAR;
      xQueueSendToFront(xPwrQueue, &message, pdMS_TO_TICKS(100));
      current_temp_state = TEMP_GOOD;
    }
    else {
      // If no change in temp state
      current_temp_state = TEMP_GOOD;
    }
#endif // 0
    switch (currentState) {
    case ALM_INIT: {
      nextState = ALM_NORMAL;
      break;
    }
    case ALM_NORMAL: {
      if (status_T) {
        errbuffer_temp_high((uint8_t)tm4c_temp, (uint8_t)max_fpga,
                            (uint8_t)imax_ff_temp, (uint8_t)max_dcdc_temp);

        alarming_temp = true;
        nextState = ALM_WARN;
      }
      else {
        alarming_temp = false;
        nextState = ALM_NORMAL;
      }
      break;
    }
    case ALM_WARN: {
      if (!status_T && ! status_V) {
        // we are back to normal
        errbuffer_put(EBUF_TEMP_NORMAL, 0);
        nextState = ALM_NORMAL;
        alarming_temp = false;
      }
      else if (largest_over_temp > ALM_OVERTEMP_THRESHOLD) {
        // log alarm, send message to turn off power and move to error state
        errbuffer_temp_high((uint8_t)tm4c_temp, (uint8_t)max_fpga,
                            (uint8_t)imax_ff_temp, (uint8_t)max_dcdc_temp);
        message = TEMP_ALARM;
        xQueueSendToFront(xPwrQueue, &message, 100);
        nextState = ALM_ERROR;
      }
      else {
        nextState = ALM_WARN;
      }
      break;
    }
    case ALM_ERROR: {
      if (!alarming && !status_T) {
        // error has cleared, log and move to normal state
        errbuffer_put(EBUF_TEMP_NORMAL, 0);
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
    currentState = nextState;
  } // end infinite loop
  return;
}
