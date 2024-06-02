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

#include "FreeRTOS.h" // IWYU pragma: keep
#include "queue.h"
#include "semphr.h"
#include "common/log.h"

#include <sys/_types.h>

#include "clocksynth.h"

#ifdef __INTELLISENSE__
#define __fp16 float
#endif // __INTELLISENSE

struct dev_moni2c_addr_t {
  char *name;
  uint8_t mux_addr;             // I2C address of the Mux
  uint8_t mux_bit;              // port of the mux; write value 0x1U<<mux_bit to the mux register
  uint8_t dev_addr;             // I2C address of device.
  uint16_t eeprom_progname_reg; // register on eeprom for reading program version
};

// Local task
typedef union {
  uint8_t us;
  int8_t s;
} convert_8_t;

// INIT task
void InitTask(void *parameters);

// ADC task
#define ADC_CHANNEL_COUNT   21
#define ADC_INFO_TEMP_ENTRY 20 // this needs to be manually kept correct.
#ifdef REV1
#define ADC_INFO_GEN_VCC_INIT_CH  0
#define ADC_INFO_GEN_VCC_FIN_CH   5
#define ADC_INFO_FPGA_VCC_INIT_CH 6
#define ADC_INFO_FPGA_VCC_FIN_CH  7
#elif defined(REV2) // REV2
#define ADC_INFO_GEN_VCC_INIT_CH  0
#define ADC_INFO_GEN_VCC_4V0_CH   3
#define ADC_INFO_GEN_VCC_FIN_CH   4
#define ADC_INFO_FPGA_VCC_INIT_CH 5
#define ADC_INFO_FPGA_VCC_FIN_CH  12
#define ADC_INFO_CUR_INIT_CH      13
#define ADC_INFO_CUR_FIN_CH       17
#endif

const char *const getADCname(const int i);
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
  POWER_DOWN,
  POWER_OFF,
  POWER_L1ON,
  POWER_L2ON,
  POWER_L3ON,
  POWER_L4ON,
  POWER_L5ON,
  POWER_L6ON,
  POWER_ON,
};
enum power_system_state getPowerControlState(void);
const char *getPowerControlStateName(enum power_system_state);
const bool getPowerControlExternalAlarmState(void);
const uint16_t getPowerControlIgnoreMask(void);

void LGA80D_init(void);

// --- Semi-generic PMBUS based I2C task
void MonitorTask(void *parameters);
void MonitorI2CTask(void *parameters);
void MonitorTaskI2C(void *parameters);
#ifdef REV1
#define N_PM_ADDRS_DCDC 5
#elif defined(REV2) // REV2
#define N_PM_ADDRS_DCDC 7
#endif
#define N_EXTRA_CMDS 7

// MonitorI2C task
// --- Firefly and Clock monitoring via I2C
// I2C information -- which device on the MCU is for the FF for each FPGA and clock chips SI5341 and SI5395
// this is what corresponds to I2C_BASE variables in the MCU
#ifdef REV1
#define I2C_DEVICE_F1 4
#define I2C_DEVICE_F2 3
#elif defined(REV2)
#define I2C_DEVICE_F1 4
#define I2C_DEVICE_F2 3
#endif

// pilfered and adapted from http://billauer.co.il/blog/2018/01/c-pmbus-xilinx-fpga-kc705/
enum pm_type { PM_VOLTAGE,
               PM_NONVOLTAGE,
               PM_STATUS,
               PM_LINEAR11,
               PM_LINEAR16U,
               PM_LINEAR16S };

struct clk_program_t {
  char progname_clkdesgid[CLOCK_PROGNAME_REG_NAME];     // program name from DESIGN_ID register of clock chip
  char progname_eeprom[CLOCK_EEPROM_PROGNAME_REG_NAME]; // program name from eeprom
};

extern struct clk_program_t clkprog_args[5]; // NSUPPLIES_CLK + NSUPPLIES_CLKR0A = 5

#define ADDR_ID 0x40 // internal eeprom block for board number & rev
#define ADDR_FF 0x44 // internal eeprom block for ff mask
#define ADDR_PS 0x48 // internal eeprom block for ps ignore fail
#define PASS    0x12345678

// Enable or disable the 3.8V power supplies for the SamTec Fireflies
int enable_3v8(UBaseType_t ffmask[2], bool turnOff);

// ff_ctl end
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

// ---- version info
const char *buildTime(void);
const char *gitVersion(void);

// ---- ALARMS

extern QueueHandle_t xAlmQueue;

// status register bits
#define ALM_STAT_TM4C_OVERTEMP    0x1
#define ALM_STAT_FIREFLY_OVERTEMP 0x2
#define ALM_STAT_FPGA_OVERTEMP    0x4
#define ALM_STAT_DCDC_OVERTEMP    0x8
// status register bits
#define ALM_STAT_GEN_OVERVOLT   0x5
#define ALM_STAT_FPGA1_OVERVOLT 0x6
#define ALM_STAT_FPGA2_OVERVOLT 0x7
// messages
#define ALM_CLEAR_ALL     1
#define ALM_CLEAR_TEMP    2
#define ALM_CLEAR_CURRENT 3
#define ALM_CLEAR_VOLTAGE 4

enum device { FF,
              DCDC,
              TM4C,
              FPGA };

enum powdevice { GEN,
                 FPGA1,
                 FPGA2 };

void GenericAlarmTask(void *parameters);

float getAlarmTemperature(enum device device_name);
void setAlarmTemperature(enum device device_name, const float newtemp);
uint32_t getTempAlarmStatus(void);
float getAlarmVoltageThres(void);
void setAlarmVoltageThres(const float voltthres);
uint32_t getVoltAlarmStatus(void);

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
// #define ZYNQMON_TEST_MODE
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
#define ZM_NUM_ENTRIES 1024
struct zynqmon_data_t {
  uint16_t sensor;
  union convert_16_t {
    uint16_t us;   // cppcheck-suppress unusedStructMember
    uint8_t uc[2]; // cppcheck-suppress unusedStructMember
    char c[2];     // cppcheck-suppress unusedStructMember
    int16_t i;     // cppcheck-suppress unusedStructMember
    __fp16 f;
  } data;
};

extern struct zynqmon_data_t zynqmon_data[ZM_NUM_ENTRIES];

void zm_fill_structs(void);
void zm_set_firefly_temps(struct zynqmon_data_t data[], int start);
void zm_set_gitversion(struct zynqmon_data_t data[], int start);
void zm_set_uptime(struct zynqmon_data_t data[], int start);
void zm_set_firefly_temps(struct zynqmon_data_t data[], int start);
void zm_set_firefly_bits(struct zynqmon_data_t data[], int start);
void zm_set_clkconfigversion(struct zynqmon_data_t data[], int start, int n);
void zm_set_firefly_info(struct zynqmon_data_t data[], int start);
void zm_set_adcmon(struct zynqmon_data_t data[], int start);
void zm_set_psmon_legacy(struct zynqmon_data_t data[], int start);
void zm_set_psmon(struct zynqmon_data_t data[], int start);
void zm_set_clock(struct zynqmon_data_t data[], int start);
void zm_set_fpga(struct zynqmon_data_t data[], int start);
void zm_set_allclk(struct zynqmon_data_t data[], int start);

#ifdef ZYNQMON_TEST_MODE
void setZYNQMonTestData(uint8_t sensor, uint16_t value);
uint8_t getZYNQMonTestMode(void);
uint8_t getZYNQMonTestSensor(void);
uint16_t getZYNQMonTestData(void);
#endif // ZYNQMON_TEST_MODE

// utility functions
const uint32_t *getSystemStack(void);
int SystemStackWaterHighWaterMark(void);

// clock IO expander initalization
int init_registers_clk(void);
#ifdef REV2

#define CLOCK_CHIP_COMMON_I2C_ADDR 0x6b
#define CLOCK_CHIP_R0A_I2C_ADDR    0x77
#define CLOCK_I2C_DEV              2
#define CLOCK_I2C_MUX_ADDR         0x70
#define CLOCK_I2C_EEPROM_ADDR      0x50

#define CLOCK_NUM_SI5341 1
#define CLOCK_NUM_SI5395 4

// configuring clock initalization
int init_load_clk(int clk_n);

// hibernate/RTC
void InitRTC(void);
#endif // REV2

struct dev_i2c_addr_t; // forward reference
void snapdump(struct dev_i2c_addr_t *add, uint8_t page, uint8_t snapshot[32], bool reset);

// Xilinx MonitorTask
int get_f1_index(void);
int get_f2_index(void);
void initFPGAMon(void);

// Watchdog Task
void WatchdogTask(void *parameters);
enum WatchdogTaskLabel {
  kWatchdogTaskID_FireFly,
  kWatchdogTaskID_MonitorI2CTask,
  kWatchdogTaskID_XiMon,
  kWatchdogTaskID_PSMon,
};
void task_watchdog_register_task(uint16_t task_id);
void task_watchdog_unregister_task(uint16_t task_id);
void task_watchdog_feed_task(uint16_t task_id);
uint16_t task_watchdog_get_status(void);

// general
// monitor stack usage for this task
#define CHECK_TASK_STACK_USAGE(vv)                                                    \
  {                                                                                   \
    UBaseType_t val = uxTaskGetStackHighWaterMark(NULL);                              \
    if (val < (vv)) {                                                                 \
      log_debug(LOG_SERVICE, "stack (%s) = %d(was %d)\r\n", pcTaskGetName(NULL), val, \
                (vv));                                                                \
    }                                                                                 \
    (vv) = val;                                                                       \
  }

#endif /* PROJECTS_CM_MCU_TASKS_H_ */
