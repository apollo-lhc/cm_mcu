#ifndef PROJECTS_CM_MCU_ALARMUTILITIES_H_
#define PROJECTS_CM_MCU_ALARMUTILITIES_H_

#include "Tasks.h"

struct GenericAlarmParams_t {
  // this queue is used to receive messages
  QueueHandle_t xAlmQueue;
  int (*checkStatus)(void); // return 0 for normal, 1 for warn, >1 for error
  void (*errorlog_registererror)(void);
  void (*errorlog_clearerror)(void);
};

extern struct GenericAlarmParams_t tempAlarmTask;

// temperature alarms
//    first some commands for setting/getting the thresholds
float getAlarmTemperature(enum device theDevice);
void setAlarmTemperature(enum device theDevice, float temperature);
void getAlarmTemperatureStatus(void);
//    callback functions
int TempStatus();
void TempErrorLog();
void TempClearErrorLog();

// voltage alarms



#endif // PROJECTS_CM_MCU_ALARMUTILITIES_H_
