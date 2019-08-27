/*
 * Tasks.h
 *
 *  Created on: Aug 26, 2019
 *      Author: pw94
 *
 *      Header file for tasks and associated functions, for those tasks
 *      where a dedicated header file would be overkill.
 */

#ifndef PROJECTS_PROJECT2_TASKS_H_
#define PROJECTS_PROJECT2_TASKS_H_

#include "FreeRTOS.h"
#include "FreeRTOSConfig.h"
#include "queue.h"

// ADC task
#define ADC_CHANNEL_COUNT 21
#define ADC_INFO_TEMP_ENTRY 20 // this needs to be manually kept correct.

const char* getADCname(const int i);
float getADCvalue(const int i);

// Holds the handle of the created queue for the LED task.
extern QueueHandle_t xLedQueue;

// control the LED
void LedTask(void *parameters);

// Holds the handle of the created queue for the power supply task.
extern QueueHandle_t xPwrQueue;


// monitor and control the power supplies
void PowerSupplyTask(void *parameters);
void MonitorTask(void *parameters);

// firefly monitoring
#define NFIREFLIES_KU15P 11
#define NFIREFLIES_VU7P 14
#define NFIREFLIES (NFIREFLIES_KU15P+NFIREFLIES_VU7P)

void FireFlyTask(void *parameters);



// Monitoring using the ADC inputs
void ADCMonitorTask(void *parameters);

#endif /* PROJECTS_PROJECT2_TASKS_H_ */
