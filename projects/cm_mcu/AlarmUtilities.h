#ifndef PROJECTS_CM_MCU_ALARMUTILITIES_H_
#define PROJECTS_CM_MCU_ALARMUTILITIES_H_

#include "Tasks.h"

// Compile-time default temperature alarm thresholds (integer degrees Celsius)
#define INITIAL_ALARM_TEMP_FF   55
#define INITIAL_ALARM_TEMP_DCDC 70
#define INITIAL_ALARM_TEMP_TM4C 70
#define INITIAL_ALARM_TEMP_FPGA 81

struct GenericAlarmParams_t {
  int (*checkStatus)(void); // return 0 for normal, 1 for warn, >1 for error
  void (*errorlog_registererror)(void);
  void (*errorlog_clearerror)(void);
  QueueHandle_t xAlmQueue;
  UBaseType_t stack_size; // stack size of task
};

extern struct GenericAlarmParams_t tempAlarmTask;
extern struct GenericAlarmParams_t voltAlarmTask;

// temperature alarms
//    first some commands for setting/getting the thresholds
int16_t getAlarmTemperature(enum device theDevice);
void setAlarmTemperature(enum device theDevice, int16_t temperature);
void loadAlarmTemperaturesFromEEPROM(void);
void getAlarmTemperatureStatus(void);
//    callback functions
int TempStatus(void);
void TempErrorLog(void);
void TempClearErrorLog(void);

// voltage alarms
//    first some commands for setting/getting the thresholds
float getAlarmVoltageThres(void);
void setAlarmVoltageThres(float voltthres);
void getAlarmVoltageStatus(void);
//    callback functions
int VoltStatus(void);
void VoltErrorLog(void);
void VoltClearErrorLog(void);

#endif // PROJECTS_CM_MCU_ALARMUTILITIES_H_
