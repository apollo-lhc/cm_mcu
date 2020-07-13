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
#define ABS(x) ((x)<0?(-(x)):(x))

// INIT task
void InitTask(void *parameters);

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
#define RED_LED_OFF       (25)
#define RED_LED_ON        (26)
#define RED_LED_TOGGLE    (27)
#define RED_LED_TOGGLE3   (28)
#define RED_LED_TOGGLE4   (29)
#define BLUE_LED_OFF      (30)
#define BLUE_LED_ON       (31)
#define BLUE_LED_TOGGLE   (32)
#define BLUE_LED_TOGGLE3  (33)
#define BLUE_LED_TOGGLE4  (34)
#define GREEN_LED_OFF     (35)
#define GREEN_LED_ON      (36)
#define GREEN_LED_TOGGLE  (37)
#define GREEN_LED_TOGGLE3 (38)
#define GREEN_LED_TOGGLE4 (39)
// Holds the handle of the created queue for the power supply task.

// --- Power Supply management task
#define SET_PS_RETRY 5	// time interval between each set_ps() attempt
void PowerSupplyTask(void *parameters);
extern QueueHandle_t xPwrQueue;

// --- Semi-generic PMBUS based I2C task
void MonitorTask(void *parameters);

// --- Firefly monitoring
#define NFIREFLIES_KU15P 11
#define NFIREFLIES_VU7P 14
#define NFIREFLIES (NFIREFLIES_KU15P+NFIREFLIES_VU7P)

void FireFlyTask(void *parameters);
extern QueueHandle_t xFFlyQueueIn;
extern QueueHandle_t xFFlyQueueOut;

const char* getFFname(const uint8_t i);
int8_t getFFvalue(const uint8_t i);
TickType_t getFFupdateTick();

int disable_xcvr_cdr(const char *name);

// messages for FF task
#define FFLY_DISABLE_TRANSMITTER (1)
#define FFLY_ENABLE_TRANSMITTER  (2)
#define FFLY_ENABLE_CDR          (3)
#define FFLY_DISABLE_CDR         (4)
#define FFLY_WRITE_REGISTER      (5)
#define FFLY_READ_REGISTER       (6)

// FF Task message format
// two fields, a task code and task data.
#define FF_MESSAGE_DATA_SZ 26
#define FF_MESSAGE_DATA_OFFSET 0
#define FF_MESSAGE_DATA_MASK ((1<<FF_MESSAGE_DATA_SZ)-1)
#define FF_MESSAGE_CODE_SZ 6
#define FF_MESSAGE_CODE_OFFSET FF_MESSAGE_DATA_SZ
#define FF_MESSAGE_CODE_MASK ((1<<FF_MESSAGE_CODE_SZ)-1)

// FF register read/write task
// the 26 bits are split into three fields
// 11 bits of register (top two bits are page)
// 10 bits of data 
// 5 bits of which firefly 
#define FF_MESSAGE_CODE_REG_REG_SZ 11
#define FF_MESSAGE_CODE_REG_REG_OFFSET 0
#define FF_MESSAGE_CODE_REG_DAT_SZ 10
#define FF_MESSAGE_CODE_REG_DAT_OFFSET FF_MESSAGE_CODE_REG_REG_SZ
#define FF_MESSAGE_CODE_REG_FF_SZ 10
#define FF_MESSAGE_CODE_REG_FF_OFFSET (FF_MESSAGE_CODE_REG_REG_SZ+FF_MESSAGE_CODE_REG_DAT_SZ)
// derived masks
#define FF_MESSAGE_CODE_REG_REG_MASK ((1 << FF_MESSAGE_CODE_REG_REG_SZ) - 1)
#define FF_MESSAGE_CODE_REG_DAT_MASK ((1 << FF_MESSAGE_CODE_REG_DAT_SZ) - 1)
#define FF_MESSAGE_CODE_REG_FF_MASK ((1 << FF_MESSAGE_CODE_REG_FF_SZ) - 1)

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
#define TEMP_ALARM_CLEAR_ALL  1
#define TEMP_ALARM_CLEAR_FPGA 2 // ...

enum device {FF,DCDC,TM4C,FPGA};



void AlarmTask(void *parameters);
float getAlarmTemperature(enum device device_name);
void  setAlarmTemperature(enum device device_name, const float newtemp);
uint32_t getAlarmStatus();

// Monitoring using the ADC inputs
void ADCMonitorTask(void *parameters);

// I2C Slave
void I2CSlaveTask(void *parameters);

// EEPROM

extern QueueHandle_t xEPRMQueue_in;
extern QueueHandle_t xEPRMQueue_out;

#define EPRM_WRITE_SINGLE 1
#define EPRM_READ_SINGLE 2
#define EPRM_READ_DOUBLE 3
#define EPRM_LOCK_BLOCK 4
#define EPRM_UNLOCK_BLOCK 5
#define EPRM_PASS_SET 6

uint64_t EPRMMessage(uint64_t action,uint64_t addr,uint64_t data);
void EEPROMTask(void *parameters);

// Soft UART Task
// SoftUart queue messages
//#define SUART_TEST_MODE

#define SOFTUART_ENABLE_TRANSMIT  0x1
#define SOFTUART_DISABLE_TRANSMIT 0x2
#define SOFTUART_TEST_SINGLE      0x3
#define SOFTUART_TEST_INCREMENT   0x4
#define SOFTUART_TEST_OFF         0x5
#define SOFTUART_TEST_SEND_ONE    0x6
#define SOFTUART_TEST_RAW         0x7

extern QueueHandle_t xSoftUartQueue;
void SoftUartTask(void *parameters);

#ifdef SUART_TEST_MODE
void setSUARTTestData(uint8_t sensor, uint16_t value);
uint8_t getSUARTTestMode();
uint8_t getSUARTTestSensor();
uint16_t getSUARTTestData();
#endif // SUART_TEST_MODE

// utility functions
const uint32_t *getSystemStack();
int SystemStackWaterHighWaterMark();

// Xilinx MonitorTask
int get_ku_index();
int get_vu_index();
// void set_ku_index(int index);
// void set_vu_index(int index);
void initFPGAMon();

#endif /* PROJECTS_CM_MCU_TASKS_H_ */
