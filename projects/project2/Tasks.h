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

 #define MAX(a,b) (a)>(b)?(a):(b)


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

// --- Power Supply management task
void PowerSupplyTask(void *parameters);
extern QueueHandle_t xPwrQueue;

// --- Semi-generic PMBUS based I2C task
void MonitorTask(void *parameters);

// --- Firefly monitoring
#define NFIREFLIES_KU15P 11
#define NFIREFLIES_VU7P 14
#define NFIREFLIES (NFIREFLIES_KU15P+NFIREFLIES_VU7P)

void FireFlyTask(void *parameters);

const char* getFFname(const uint8_t i);
int8_t getFFvalue(const uint8_t i);


// ---- ALARMS

// status register bits
#define ALM_STAT_TM4C_OVERTEMP    0x1
#define ALM_STAT_FIREFLY_OVERTEMP 0x2
#define ALM_STAT_FPGA_OVERTEMP    0x4
#define ALM_STAT_DCDC_OVERTEMP    0x8
// Alarm Queue
extern QueueHandle_t xAlmQueue;
// messages
#define TEMP_ALARM_CLEAR_ALL 1
#define TEMP_ALARM_CLEAR_FPGA 2 // ...


void AlarmTask(void *parameters);
float getAlarmTemperature();
void setAlarmTemperature(const float);
uint32_t getAlarmStatus();


// Monitoring using the ADC inputs
void ADCMonitorTask(void *parameters);

#endif /* PROJECTS_PROJECT2_TASKS_H_ */
