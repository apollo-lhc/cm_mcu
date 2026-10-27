#ifndef ZYNQMON_TASK_H_
#define ZYNQMON_TASK_H_

#include <stdint.h>
#include <stdbool.h>

#include "FreeRTOS.h" // IWYU pragma: keep
#include "task.h"
#include "queue.h"

#include "ZynqMon_addresses.h"

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
bool getZynqMonTransmitEnabled(void);
void ZynqMonTask(void *parameters);
// data for zynqmon task to be sent to Zynq
#define ZM_NUM_ENTRIES ZMON_VALID_ENTRIES
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
void zm_set_firefly_bits(struct zynqmon_data_t data[], int start);
void zm_set_clkconfigversion(struct zynqmon_data_t data[], int start, int n);
void zm_set_firefly_info(struct zynqmon_data_t data[], int start);
void zm_set_adcmon(struct zynqmon_data_t data[], int start);
void zm_set_psmon_legacy(struct zynqmon_data_t data[], int start);
void zm_set_psmon(struct zynqmon_data_t data[], int start);
void zm_set_clock(struct zynqmon_data_t data[], int start);
void zm_set_fpga(struct zynqmon_data_t data[], int start);
void zm_set_allclk(struct zynqmon_data_t data[], int start);
void zm_set_firefly_optpow12(struct zynqmon_data_t data[], int start);
void zm_set_firefly_optpow4(struct zynqmon_data_t data[], int start);

// Diagnostics block (REV2/REV3). Handles of the tasks whose stack high-water
// mark is published, filled in by xTaskCreate in cm_mcu.c. The order must
// match the names of the diag_stack block in PL_MEM_CM_rev{2,3}.yml.
enum zm_stack_task {
  ZM_STACK_POW,
  ZM_STACK_LED,
  ZM_STACK_CLIZY,
  ZM_STACK_ADC,
  ZM_STACK_FF_F1,
  ZM_STACK_FF_F2,
  ZM_STACK_CLK,
  ZM_STACK_PSMON,
  ZM_STACK_XIMON,
  ZM_STACK_I2CS0,
  ZM_STACK_EPRM,
  ZM_STACK_ZMON,
  ZM_STACK_TALM,
  ZM_STACK_VALM,
  ZM_STACK_PRGCM,
  ZM_STACK_NTASKS
};
extern TaskHandle_t zm_stack_tasks[ZM_STACK_NTASKS];

#if defined(REV2) || defined(REV3)
void zm_set_diag_hdr(struct zynqmon_data_t data[], int start);
void zm_set_diag_reset_cause(struct zynqmon_data_t data[], int start);
void zm_set_diag_prev_assert_pc(struct zynqmon_data_t data[], int start);
void zm_set_diag_prev_fatal(struct zynqmon_data_t data[], int start);
void zm_set_diag_pwr_fail(struct zynqmon_data_t data[], int start);
void zm_set_diag_i2c(struct zynqmon_data_t data[], int start);
void zm_set_diag_i2cs(struct zynqmon_data_t data[], int start);
void zm_set_diag_lga80d_badt(struct zynqmon_data_t data[], int start);
void zm_set_diag_pwr(struct zynqmon_data_t data[], int start);
void zm_set_diag_stack(struct zynqmon_data_t data[], int start);
#endif // REV2 or REV3

#ifdef ZYNQMON_TEST_MODE
void setZYNQMonTestData(uint8_t sensor, uint16_t value);
uint8_t getZYNQMonTestMode(void);
uint8_t getZYNQMonTestSensor(void);
uint16_t getZYNQMonTestData(void);
#endif // ZYNQMON_TEST_MODE

#endif // ZYNQMON_TASK_H_
