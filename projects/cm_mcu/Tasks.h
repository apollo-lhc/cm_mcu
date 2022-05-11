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

#include "common/printf.h"

#ifdef __INTELLISENSE__
#define __fp16 float
#endif // __INTELLISENSE

// INIT task
void InitTask(void *parameters);

// ADC task
#define ADC_CHANNEL_COUNT   21
#define ADC_INFO_TEMP_ENTRY 20 // this needs to be manually kept correct.

const char *getADCname(const int i);
float getADCvalue(const int i);
float getADCtargetValue(const int i);

// Holds the handle of the created queue for the LED task.
extern QueueHandle_t xLedQueue;

// control the LED
void LedTask(void *parameters);

#define RED_LED_OFF       (10)
#define RED_LED_ON        (11)
#define RED_LED_TOGGLE    (12)
#define RED_LED_TOGGLE3   (13)
#define RED_LED_TOGGLE4   (14)
#define BLUE_LED_OFF      (20)
#define BLUE_LED_ON       (21)
#define BLUE_LED_TOGGLE   (22)
#define BLUE_LED_TOGGLE3  (23)
#define BLUE_LED_TOGGLE4  (24)
#define GREEN_LED_OFF     (30)
#define GREEN_LED_ON      (31)
#define GREEN_LED_TOGGLE  (32)
#define GREEN_LED_TOGGLE3 (33)
#define GREEN_LED_TOGGLE4 (34)
// Holds the handle of the created queue for the power supply task.

// --- Power Supply management task
void PowerSupplyTask(void *parameters);
extern QueueHandle_t xPwrQueue;
enum power_system_state {
  POWER_FAILURE,
  POWER_INIT,
  POWER_OFF,
  POWER_L1ON,
  POWER_L2ON,
  POWER_L3ON,
  POWER_L4ON,
  POWER_L5ON,
  POWER_ON,
};
enum power_system_state getPowerControlState();
const char *getPowerControlStateName(enum power_system_state);
const bool getPowerControlExternalAlarmState();

void LGA80D_init(void);

// --- Semi-generic PMBUS based I2C task
void MonitorTask(void *parameters);

// Firefly task
// --- Firefly monitoring
// REV1
#ifndef REV2
#define NFIREFLIES_F1 11
#define NFIREFLIES_F2 14
#else // REV2
// REV 2
#define NFIREFLIES_F1    10
#define NFIREFLIES_F2    10 // Placeholders
//#error "Fix placeholder values"
#endif // REV 2
#define NFIREFLIES (NFIREFLIES_F1 + NFIREFLIES_F2)

void FireFlyTask(void *parameters);
extern QueueHandle_t xFFlyQueueIn;
extern QueueHandle_t xFFlyQueueOut;
SemaphoreHandle_t getFFMutex();

const char *getFFname(const uint8_t i);
int8_t *test_read(const uint8_t i);
bool isEnabledFF(int ff);
int8_t getFFtemp(const uint8_t i);
uint8_t getFFstatus(const uint8_t i);
bool getFFlos(int i, int channel);
bool getFFlol(int i, int channel);
TickType_t getFFupdateTick();
// FFLY I/O Expander initialization
void init_registers_ff();

int disable_xcvr_cdr(const char *name);

// messages for FF task
#define FFLY_DISABLE_TRANSMITTER (1)
#define FFLY_ENABLE_TRANSMITTER  (2)
#define FFLY_ENABLE_CDR          (3)
#define FFLY_DISABLE_CDR         (4)
#define FFLY_DISABLE             (5)
#define FFLY_ENABLE	             (6)
#define FFLY_WRITE_REGISTER      (7)
#define FFLY_READ_REGISTER       (8)
#define FFLY_TEST_READ           (9)
#define FFLY_SUSPEND             (10)
#define FFLY_RESUME              (11)

// FF Task message format
// two fields, a task code and task data.
#define FF_MESSAGE_DATA_SZ     26
#define FF_MESSAGE_DATA_OFFSET 0
#define FF_MESSAGE_DATA_MASK   ((1 << FF_MESSAGE_DATA_SZ) - 1)
#define FF_MESSAGE_CODE_SZ     6
#define FF_MESSAGE_CODE_OFFSET FF_MESSAGE_DATA_SZ
#define FF_MESSAGE_CODE_MASK   ((1 << FF_MESSAGE_CODE_SZ) - 1)

// FF register read/write task
// the 26 bits are split into three fields
// 11 bits of register (top two bits are page)
// 10 bits of data
// 5 bits of which firefly
#define FF_MESSAGE_CODE_REG_REG_SZ     11
#define FF_MESSAGE_CODE_REG_REG_OFFSET 0
#define FF_MESSAGE_CODE_REG_DAT_SZ     10
#define FF_MESSAGE_CODE_REG_DAT_OFFSET FF_MESSAGE_CODE_REG_REG_SZ
#define FF_MESSAGE_CODE_REG_FF_SZ      10
#define FF_MESSAGE_CODE_REG_FF_OFFSET  (FF_MESSAGE_CODE_REG_REG_SZ + FF_MESSAGE_CODE_REG_DAT_SZ)
// derived masks
#define FF_MESSAGE_CODE_REG_REG_MASK ((1 << FF_MESSAGE_CODE_REG_REG_SZ) - 1)
#define FF_MESSAGE_CODE_REG_DAT_MASK ((1 << FF_MESSAGE_CODE_REG_DAT_SZ) - 1)
#define FF_MESSAGE_CODE_REG_FF_MASK  ((1 << FF_MESSAGE_CODE_REG_FF_SZ) - 1)

// FF test register
#define FF_MESSAGE_CODE_TEST_REG_SZ      8
#define FF_MESSAGE_CODE_TEST_REG_OFFSET  0
#define FF_MESSAGE_CODE_TEST_SIZE_SZ     5
#define FF_MESSAGE_CODE_TEST_SIZE_OFFSET FF_MESSAGE_CODE_TEST_REG_SZ
#define FF_MESSAGE_CODE_TEST_FF_SZ       5
#define FF_MESSAGE_CODE_TEST_FF_OFFSET   (FF_MESSAGE_CODE_TEST_REG_SZ + FF_MESSAGE_CODE_TEST_REG_SZ)
// derived masks
#define FF_MESSAGE_CODE_TEST_REG_MASK  ((1 << FF_MESSAGE_CODE_TEST_REG_SZ) - 1)
#define FF_MESSAGE_CODE_TEST_SIZE_MASK ((1 << FF_MESSAGE_CODE_TEST_SIZE_SZ) - 1)
#define FF_MESSAGE_CODE_TEST_FF_MASK   ((1 << FF_MESSAGE_CODE_TEST_FF_SZ) - 1)

// ---- version info
const char *buildTime();
const char *gitVersion();

// ---- ALARMS

// status register bits
#define ALM_STAT_TM4C_OVERTEMP    0x1
#define ALM_STAT_FIREFLY_OVERTEMP 0x2
#define ALM_STAT_FPGA_OVERTEMP    0x4
#define ALM_STAT_DCDC_OVERTEMP    0x8
// messages
#define ALM_CLEAR_ALL     1
#define ALM_CLEAR_TEMP    2
#define ALM_CLEAR_CURRENT 3
#define ALM_CLEAR_VOLTAGE 4

enum device { FF, DCDC, TM4C, FPGA };

void GenericAlarmTask(void *parameters);

float getAlarmTemperature(enum device device_name);
void setAlarmTemperature(enum device device_name, const float newtemp);
uint32_t getAlarmStatus();

// Monitoring using the ADC inputs
void ADCMonitorTask(void *parameters);

// I2C Slave
void I2CSlaveTask(void *parameters);

// EEPROM

extern QueueHandle_t xEPRMQueue_in;
extern QueueHandle_t xEPRMQueue_out;

#define EPRM_WRITE_SINGLE 1
#define EPRM_READ_SINGLE  2
#define EPRM_READ_DOUBLE  3
#define EPRM_LOCK_BLOCK   4
#define EPRM_UNLOCK_BLOCK 5
#define EPRM_PASS_SET     6

uint64_t EPRMMessage(uint64_t action, uint64_t addr, uint64_t data);
void EEPROMTask(void *parameters);

// -- ZynqMon
//#define ZYNQMON_TEST_MODE
// ZynqMon queue messages
#define ZYNQMON_ENABLE_TRANSMIT  0x1
#define ZYNQMON_DISABLE_TRANSMIT 0x2
#define ZYNQMON_TEST_SINGLE      0x3
#define ZYNQMON_TEST_INCREMENT   0x4
#define ZYNQMON_TEST_OFF         0x5
#define ZYNQMON_TEST_SEND_ONE    0x6
#define ZYNQMON_TEST_RAW         0x7

extern QueueHandle_t xZynqMonQueue;
void ZynqMonTask(void *parameters);
// data for zynqmon task to be sent to Zynq 
#define ZM_NUM_ENTRIES 256
struct zynqmon_data_t {
  uint8_t sensor;
  union convert_16_t {
    uint16_t us;
    uint8_t uc[2];
    char c[2];
    int16_t i;
    __fp16 f;
  } data;
};

extern struct zynqmon_data_t zynqmon_data[ZM_NUM_ENTRIES];

#ifdef ZYNQMON_TEST_MODE
void setZYNQMonTestData(uint8_t sensor, uint16_t value);
uint8_t getZYNQMonTestMode();
uint8_t getZYNQMonTestSensor();
uint16_t getZYNQMonTestData();
#endif // ZYNQMON_TEST_MODE

// utility functions
const uint32_t *getSystemStack();
int SystemStackWaterHighWaterMark();

// clock IO expander initalization
void init_registers_clk();
#ifdef REV2

#define CLOCK_CHIP_COMMON_I2C_ADDR  0x6b
#define CLOCK_CHIP_R0A_I2C_ADDR     0x77
#define CLOCK_I2C_DEV               2
#define CLOCK_I2C_MUX_ADDR          0x70
#define CLOCK_I2C_EEPROM_ADDR       0x50

// configuring clock initalization
int init_load_clk(int clk_n);


// hibernate/RTC
void InitRTC();
#endif // REV2

struct dev_i2c_addr_t; // forward reference
void snapdump(struct dev_i2c_addr_t *add, uint8_t page, uint8_t snapshot[32], bool reset);

// Xilinx MonitorTask
int get_f1_index();
int get_f2_index();
void initFPGAMon();

// Watchdog Task
void WatchdogTask(void* parameters);
enum WatchdogTaskLabel {
  kWatchdogTaskID_FireFly,
  kWatchdogTaskID_XiMon,
  kWatchdogTaskID_PSMon,
};
void task_watchdog_register_task(uint16_t task_id);
void task_watchdog_unregister_task(uint16_t task_id);
void task_watchdog_feed_task(uint16_t task_id);
uint16_t task_watchdog_get_status();

// general
// monitor stack usage for this task
#define CHECK_TASK_STACK_USAGE(vv)                                                   \
  {                                                                                  \
    UBaseType_t val = uxTaskGetStackHighWaterMark(NULL);                             \
    if (val < vv) {                                                                  \
      log_info(LOG_SERVICE, "stack (%s) = %d(was %d)\r\n", pcTaskGetName(NULL), val, \
               vv);                                                                  \
    }                                                                                \
    vv = val;                                                                        \
  }

#endif /* PROJECTS_CM_MCU_TASKS_H_ */
