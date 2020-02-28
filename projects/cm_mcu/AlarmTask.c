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

enum temp_state {TEMP_UNKNOWN, TEMP_GOOD, TEMP_BAD};

enum alarm_state {ALM_UNKNOWN, ALM_GOOD, ALM_BAD};



// Status of the alarm task
static uint32_t status = 0x0;
uint32_t oldstatus;

// read-only, so no need to use queue
uint32_t getAlarmStatus()
{
  return status;
}

#define INITIAL_ALARM_TEMP 65.0 // in Celsius duh
static float alarm_temp = INITIAL_ALARM_TEMP;

float getAlarmTemperature()
{
  return alarm_temp;
}

void setAlarmTemperature(const float newtemp)
{
  alarm_temp = newtemp;
  return;
}


void AlarmTask(void *parameters)
{
  // initialize to the current tick time
  TickType_t xLastWakeTime = xTaskGetTickCount();
  uint32_t message; // this must be in a semi-permanent scope
  uint16_t errbuf_data,errbuf_olddata;
  enum temp_state current_temp_state = TEMP_UNKNOWN;
  float buffer_maxtemp = INITIAL_ALARM_TEMP;

  vTaskDelayUntil( &xLastWakeTime, pdMS_TO_TICKS( 2500 ) );


  for (;;) {
    vTaskDelayUntil( &xLastWakeTime, pdMS_TO_TICKS( 25 ) );

    if ( xQueueReceive(xAlmQueue, &message, 0) ) {
      switch (message) {
      case TEMP_ALARM_CLEAR_ALL:
        status = 0;
        break;
      case TEMP_ALARM_CLEAR_FPGA: // this is an example
        status &= ~ALM_STAT_FPGA_OVERTEMP;
        break;
      default:
        break;
      }
      continue; // we break out of the loop because we want data
      // to refresh
    }


    // microcontroller
    float tm4c_temp = getADCvalue(ADC_INFO_TEMP_ENTRY);
    if ( tm4c_temp > alarm_temp ) status |= ALM_STAT_TM4C_OVERTEMP;
    buffer_maxtemp=tm4c_temp;
    // FPGA
    float max_fpga = MAX(fpga_args.pm_values[0], fpga_args.pm_values[1]);
    if ( max_fpga > alarm_temp) status |= ALM_STAT_FPGA_OVERTEMP;
    if (max_fpga>buffer_maxtemp){ buffer_maxtemp=max_fpga;}

    // DCDC. The first command is READ_TEMPERATURE_1.
    // I am assuming it stays that way!!!!!!!!
    float max_dcdc_temp = -99.0;
    for (int ps = 0; ps < dcdc_args.n_devices; ++ps ) {
      for ( int page = 0; page < dcdc_args.n_pages; ++page ) {
        float thistemp = dcdc_args.pm_values[ps*(dcdc_args.n_commands*dcdc_args.n_pages)
                                             +page*dcdc_args.n_commands+0];
        if ( thistemp > max_dcdc_temp )
          max_dcdc_temp = thistemp;
      }
    }
    if ( max_dcdc_temp > alarm_temp ) status |= ALM_STAT_DCDC_OVERTEMP;
    if (max_dcdc_temp>buffer_maxtemp){ buffer_maxtemp=max_dcdc_temp;}
    // Fireflies. These are reported as ints but we are asked
    // to report a float.
    int8_t imax_ff_temp = -99;
    for ( int i = 0; i < NFIREFLIES; ++i ) {
      int8_t v = getFFvalue(i);
      if ( v > imax_ff_temp )
        imax_ff_temp = v;
    }
    if ( (float)imax_ff_temp > alarm_temp ) status |= ALM_STAT_FIREFLY_OVERTEMP;
    if (imax_ff_temp>buffer_maxtemp){ buffer_maxtemp=imax_ff_temp;}

    if ( status && current_temp_state != TEMP_BAD ) {
    	// If temp goes from good to bad, turn on alarm, send error message to buffer
      errbuf_data=(0x0FFFFFFFU)&(uint8_t)buffer_maxtemp;
      if ((errbuf_data!=errbuf_olddata)||(status!=oldstatus)){
    	  // only send message when status or temp have changed, to avoid filling up buffer
    	  errbuffer_put(ebuf, TEMP_HIGH(status),errbuf_data);
      	  errbuf_olddata=errbuf_data;
      	  oldstatus=status;}
      message = TEMP_ALARM;
      xQueueSendToFront(xPwrQueue, &message, pdMS_TO_TICKS(100));
      current_temp_state = TEMP_BAD;
    }
    else if ( !status && current_temp_state == TEMP_BAD ) {
    	// If temp goes from bad to good, turn off alarm, send message to buffer
      errbuf_data=(0x0FFFFFFFU&(uint8_t)buffer_maxtemp);
      errbuffer_put(ebuf, TEMP_NORMAL, errbuf_data);
      buffer_maxtemp = INITIAL_ALARM_TEMP;
      message = TEMP_ALARM_CLEAR;
      xQueueSendToFront(xPwrQueue, &message, pdMS_TO_TICKS(100));
      current_temp_state = TEMP_GOOD;
    }
    else {
    	// If no change in temp state
      current_temp_state = TEMP_GOOD;
    }

  }
  return;
}
