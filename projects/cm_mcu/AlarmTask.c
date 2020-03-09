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

#define INITIAL_ALARM_TEMP_FF 45.0 // in Celsius duh
#define INITIAL_ALARM_TEMP_DCDC 70.0
#define INITIAL_ALARM_TEMP_TM4C 70.0
#define INITIAL_ALARM_TEMP_FPGA 70.0
#define TEMP_WARNING_DIFF 5.0 // if device is 5 degrees over alarm temp, turn off power

static float alarm_temp_ff = INITIAL_ALARM_TEMP_FF;
static float alarm_temp_dcdc = INITIAL_ALARM_TEMP_DCDC;
static float alarm_temp_tm4c = INITIAL_ALARM_TEMP_TM4C;
static float alarm_temp_fpga = INITIAL_ALARM_TEMP_FPGA;

float getAlarmTemperature(enum device device_name)
{
	switch(device_name){
		case TM4C: return alarm_temp_tm4c;
		case DCDC: return alarm_temp_dcdc;
		case FPGA: return alarm_temp_fpga;
		case FF: return alarm_temp_ff;
		default: return 0;
	}
}

void setAlarmTemperature(enum device device_name,const float newtemp)
{
	switch(device_name){
		case TM4C: alarm_temp_tm4c = newtemp; return;
		case DCDC: alarm_temp_dcdc = newtemp; return;
		case FPGA:   alarm_temp_fpga = newtemp; return;
		case FF: alarm_temp_ff = newtemp; return;
		default: return;
	}
}


void AlarmTask(void *parameters)
{
  // initialize to the current tick time
  TickType_t xLastWakeTime = xTaskGetTickCount();
  uint32_t message; // this must be in a semi-permanent scope
  enum temp_state current_temp_state = TEMP_UNKNOWN;
  float temp_over_ff, temp_over_fpga, temp_over_tm4c, temp_over_dcdc;
  float temp_over_device=0, temp_over=0, temp_over_old=0;

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

    temp_over=0;
    temp_over_device=0;

    // microcontroller
    float tm4c_temp = getADCvalue(ADC_INFO_TEMP_ENTRY);
    if ( tm4c_temp > alarm_temp_tm4c ) {
    	status |= ALM_STAT_TM4C_OVERTEMP;
        temp_over_tm4c = tm4c_temp-alarm_temp_tm4c;
        temp_over_device += temp_over_tm4c;
        temp_over += temp_over_tm4c;
    }

    // FPGA
    float max_fpga = MAX(fpga_args.pm_values[0], fpga_args.pm_values[1]);
    if ( max_fpga > alarm_temp_fpga) {
    	status |= ALM_STAT_FPGA_OVERTEMP;
        temp_over_fpga = max_fpga-alarm_temp_fpga;
        temp_over += temp_over_fpga;
        if (temp_over_fpga>temp_over_device){
        	temp_over_device = temp_over_fpga;
        	}
    }


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
    if ( max_dcdc_temp > alarm_temp_dcdc ) {
    	status |= ALM_STAT_DCDC_OVERTEMP;
    	temp_over_dcdc = max_dcdc_temp-alarm_temp_dcdc;
    	temp_over += temp_over_dcdc;
    	if (temp_over_dcdc>temp_over_device){
    	    temp_over_device = temp_over_dcdc;
    	    }
    }

    // Fireflies. These are reported as ints but we are asked
    // to report a float.
    int8_t imax_ff_temp = -99;
    for ( int i = 0; i < NFIREFLIES; ++i ) {
      int8_t v = getFFvalue(i);
      if ( v > imax_ff_temp )
        imax_ff_temp = v;
    }
    if ( (float)imax_ff_temp > alarm_temp_ff ) {
    	status |= ALM_STAT_FIREFLY_OVERTEMP;
        temp_over_ff = imax_ff_temp-alarm_temp_ff;
        temp_over += temp_over_ff;
        if (temp_over_ff>temp_over_device){
        	temp_over_device=temp_over_ff;
        	}
    }


    // if temp is over max by TEMP_WARNING_DIFF, then turn off power
    if (temp_over_device>TEMP_WARNING_DIFF){
  	  message = TEMP_ALARM;
  	  xQueueSendToFront(xPwrQueue, &message, pdMS_TO_TICKS(100));
    }
    // if temp returns to normal, send buffer message
    if ((temp_over==0) && (temp_over_old>0)){
        errbuffer_put(ebuf, EBUF_TEMP_NORMAL, 0);
        temp_over_old = temp_over;
    }
    if ( status && (current_temp_state != TEMP_BAD )) {
    	// If temp is bad, turn on alarm, send error message to buffer
      if ((temp_over>temp_over_old)||(status!=oldstatus)){
    	  // only send message when status has changed or temp has risen since last entry, to avoid filling up buffer
    	  errbuffer_temp_high((uint8_t)tm4c_temp,(uint8_t)max_fpga,(uint8_t)imax_ff_temp,(uint8_t)max_dcdc_temp);
		  oldstatus=status;
          temp_over_old=temp_over;
      }
      current_temp_state = TEMP_BAD;
    }
    else if ( (!status) && (current_temp_state == TEMP_BAD )) {
    	// If status is cleared (from cli), turn off alarm, send message to buffer
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
