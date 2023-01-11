#ifndef PROJECTS_CM_MCU_ALARMUTILITIES_H_
#define PROJECTS_CM_MCU_ALARMUTILITIES_H_

#include "Tasks.h"

struct GenericAlarmParams_t {
  // this queue is used to receive messages
  QueueHandle_t xAlmQueue;
  int (*checkStatus)(void); // return 0 for normal, 1 for warn, >1 for error
  void (*errorlog_registererror)(void);
  void (*errorlog_clearerror)(void);
  UBaseType_t stack_size; // stack size of task
};

extern struct GenericAlarmParams_t tempAlarmTask;
extern struct GenericAlarmParams_t voltAlarmTask;
extern struct GenericAlarmParams_t currAlarmTask;

// temperature alarms
//    first some commands for setting/getting the thresholds
float getAlarmTemperature(enum device theDevice);
void setAlarmTemperature(enum device theDevice, float temperature);
void getAlarmTemperatureStatus(void);
//    callback functions
int TempStatus(void);
void TempErrorLog(void);
void TempClearErrorLog(void);

// voltage alarms
//    first some commands for setting/getting the thresholds
float getAlarmVoltage(enum device theDevice);
void setAlarmVoltage(enum device theDevice, float voltage);
void getAlarmVoltageStatus(void);
//    callback functions
int VoltStatus(void);
void VoltErrorLog(void);
void VoltClearErrorLog(void);

// current alarms
//    first some commands for setting/getting the thresholds
/*
float getAlarmCurrent(enum device theDevice);
void setAlarmCurrent(enum device theDevice, float current);
void getAlarmCurrentStatus(void);
//    callback functions
int CurrStatus(void);
void CurrErrorLog(void);
void CurrClearErrorLog(void);
*/

#endif // PROJECTS_CM_MCU_ALARMUTILITIES_H_
