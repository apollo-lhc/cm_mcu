#pragma once

#include <stdbool.h>
// On Apollo the ADC range is from 0 - 2.5V.
// Some signals must be scaled to fit in this range.
#define ADC_MAX_VOLTAGE_RANGE 2.5f

// ADC task
#define ADC_CHANNEL_COUNT   21
#define ADC_INFO_TEMP_ENTRY 20     // this needs to be manually kept correct.
#if defined(REV2) || defined(REV3) // REV2
#define ADC_INFO_GEN_VCC_INIT_CH  0
#define ADC_INFO_GEN_VCC_4V0_CH   3
#define ADC_INFO_GEN_VCC_FIN_CH   4
#define ADC_INFO_FPGA_VCC_INIT_CH 5
#define ADC_INFO_FPGA_VCC_FIN_CH  12
#define ADC_INFO_CUR_INIT_CH      13
#define ADC_INFO_CUR_FIN_CH       17
#endif // REV2 or REV3

#define ADC_DIFF_TOLERANCE 0.05f // in percent

float getADCtargetValue(int i);
int check_ps_at_prio(int prio, bool f2_enable, bool f1_enable, float *delta);
