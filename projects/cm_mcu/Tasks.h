/*
 * Tasks.h
 *
 *  Created on: Aug 26, 2019
 *      Author: pw94
 *
 *      Header file for tasks and associated functions, for those tasks
 *      where a dedicated header file would be overkill.
 */

#ifndef PROJECTS_CM_MCU_TASKS_H_
#define PROJECTS_CM_MCU_TASKS_H_

#include "FreeRTOS.h"
#include "FreeRTOSConfig.h"
#include "queue.h"
#include "semphr.h"

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

// this should go elsewhere
#define RED_LED_OFF     (25)
#define RED_LED_ON      (26)
#define RED_LED_TOGGLE  (27)
#define RED_LED_TOGGLE3 (28)
#define RED_LED_TOGGLE4 (29)

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
extern QueueHandle_t xFFlyQueue;

const char* getFFname(const uint8_t i);
int8_t getFFvalue(const uint8_t i);
TickType_t getFFupdateTick();


int disable_xcvr_cdr(const char *name);

// messages for FF task
#define FFLY_DISABLE_TRANSMITTERS (1)
#define FFLY_ENABLE_TRANSMITTERS  (2)
#define FFLY_ENABLE_CDR        (3)
#define FFLY_DISABLE_CDR       (4)
//// control access to I2C
//extern SemaphoreHandle_t xI2C1Mutex;
//extern SemaphoreHandle_t xI2C2Mutex;
//extern SemaphoreHandle_t xI2C3Mutex;
//extern SemaphoreHandle_t xI2C4Mutex;
//extern SemaphoreHandle_t xI2C6Mutex;

// ---- version info
const char* buildTime();
const char* gitVersion();

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

#endif /* PROJECTS_CM_MCU_TASKS_H_ */
