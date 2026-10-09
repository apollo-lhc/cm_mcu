#ifndef PROJECTS_CM_MCU_ALARMUTILITIES_H_
#define PROJECTS_CM_MCU_ALARMUTILITIES_H_

#include <stdbool.h>

#include "Tasks.h"

// Compile-time default temperature alarm thresholds (integer degrees Celsius)
#define INITIAL_ALARM_TEMP_FF   55
#define INITIAL_ALARM_TEMP_DCDC 70
#define INITIAL_ALARM_TEMP_TM4C 70
#define INITIAL_ALARM_TEMP_FPGA 81

struct GenericAlarmParams_t {
  int (*checkStatus)(void); // return 0 for normal, 1 for warn, >1 for error
  void (*errorlog_registererror)(void);
  // fault_latched: false on WARN->NORMAL (truly back to normal); true on
  // FAULT_ERRORING->FAULT_ERROR_CLEARED (condition gone, power-off still latched)
  void (*errorlog_clearerror)(bool fault_latched);
  void (*clearHysteresis)(void); // called on ALM_CLEAR_ALL; NULL = no-op
  QueueHandle_t xAlmQueue;
  UBaseType_t stack_size;                 // stack size of task
  const LedMsg_t *led_warn_msg;           // LED state on WARN entry (NULL = no update)
  const LedMsg_t *led_alarm_msg;          // LED state on FAULT_ERRORING entry (NULL = no update)
  const LedMsg_t *led_normal_msg;         // LED state on return to NORMAL (NULL = no update)
  enum alarm_task_state *published_state; // task writes its current state here every
                                          // iteration; NULL = no publication
};

extern struct GenericAlarmParams_t tempAlarmTask;
extern struct GenericAlarmParams_t voltAlarmTask;

// temperature alarms
//    first some commands for setting/getting the thresholds
int16_t getAlarmTemperature(enum device theDevice);
void setAlarmTemperature(enum device theDevice, int16_t temperature);
// Non-blocking variant for ProgCom: false (and no change) if the EEPROM queue is full.
bool setAlarmTemperatureTry(enum device theDevice, int16_t temperature);
void loadAlarmTemperaturesFromEEPROM(void);
void getAlarmTemperatureStatus(void);
//    callback functions
int TempStatus(void);
void TempErrorLog(void);
void TempClearErrorLog(bool fault_latched);
void clearWarnLatch(void);

// voltage alarms
//    first some commands for setting/getting the thresholds
// in centi-percent (500 = +/-5.00 %)
uint16_t getAlarmVoltageThresCpct(void);
void setAlarmVoltageThresCpct(uint16_t cpct);
// Non-blocking variant for ProgCom: false (and no change) if the EEPROM queue
// is full.
bool setAlarmVoltageThresCpctTry(uint16_t cpct);
void loadAlarmVoltageFromEEPROM(void);
void getAlarmVoltageStatus(void);
//    callback functions
int VoltStatus(void);
void VoltErrorLog(void);
void VoltClearErrorLog(bool fault_latched);
void clearVoltAlarmLatch(void);
// Worst rail of the most recent voltage alarm, latched until ALM_CLEAR_ALL.
// Returns false (outputs untouched) if nothing has been latched.
bool getVoltAlarmLatch(int *ch, float *now, float *target, float *pct);

// name of a GenericAlarmTask FSM state
const char *getAlarmTaskStateName(enum alarm_task_state state);

#endif // PROJECTS_CM_MCU_ALARMUTILITIES_H_
