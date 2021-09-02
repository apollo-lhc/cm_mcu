/*
 * ADCMonitorTask.c
 *
 *  Created on: May 19, 2019
 *      Author: wittich
 *
 *      The TM4C1290NCPDT has two ADCs with 20 shared inputs plus the internal
 *      temperature monitor.
 *      The first ADC is configured to have 12 inputs and the second ADC is
 *      configured to have 8 inputs.
 *
 *      The TM4C sequencers can convert 8 (sequencer 0) or 4 (sequencer 1)
 *      values w/o processor intervention.
 *
 *      We always use sequencer 0 for ADC1 and sequencer 1 for ADC0. The end of
 *      conversion is signaled by an interrupt, which is handled here.
 *
 *      Todo:
 *      * handle errors but not with assert
 *      * change 2nd ADC to use sequence 0 (one fewer interrupt)
 */

// includes for types
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>

// memory mappings
#include "inc/hw_memmap.h"

// driverlib
#include "driverlib/rom.h"
#include "driverlib/adc.h"

#include "InterruptHandlers.h"
#include "Tasks.h"

// On Apollo the ADC range is from 0 - 2.5V.
// Some signals must be scaled to fit in this range.
#define ADC_MAX_VOLTAGE_RANGE 2.5f

// a struct to hold some information about the ADC channel.
struct ADC_Info_t {
  int channel;      // which of the 20 ADC channels on the TM4C1290NCPDT on Apollo CM v1
  const char *name; // name
  float scale;      // scaling, if needed, for signals bigger than 2.5V
  float target_value;
};

// parameters for how data is organized in the fADC arrays and how the
// sequencers are organized
#define ADCs_ADC1_START             0
#define ADCs_ADC1_FIRST_SEQ_LENGTH  8
#define ADCs_ADC1_SECOND_SEQ_LENGTH 5
#define ADCs_ADC1_ENTRIES           (ADCs_ADC1_FIRST_SEQ_LENGTH + ADCs_ADC1_SECOND_SEQ_LENGTH)
#define ADCs_ADC0_START             ADCs_ADC1_ENTRIES
#define ADCs_ADC0_ENTRIES           8
#define ADCs_ADC0_FIRST_SEQ_LENGTH  4
#define ADCs_ADC0_SECOND_SEQ_LENGTH 4

// -------------------------------------------------
//
// REV 1
//
// -------------------------------------------------

// This array holds the list of ADCs with the AIN channels on the TM4C1290NCPDT
// on Apollo CM v1, and whatever scaling is needed to get the value correct.
// We also read out the internal temperature sensor, which has a special
// channel sensor.
// clang-format off
#ifdef REV1
static
struct ADC_Info_t ADCs[] = {
    {ADC_CTL_CH12, "VCC_12V", 6.f, 12.f},
    {ADC_CTL_CH13, "VCC_2V5", 2.f, 2.5f},
    {ADC_CTL_CH14, "VCC_M3V3", 2.f, 3.3f},
    {ADC_CTL_CH16, "VCC_3V3", 2.f, 3.3f},
    {ADC_CTL_CH7,  "VCC_1V8", 1.f, 1.8f},
    {ADC_CTL_CH15, "VCC_M1V8", 1.f, 1.8f},
    {ADC_CTL_CH3,  "F2_VCCINT", 1.f, 0.85f},
    {ADC_CTL_CH8,  "F1_VCCINT", 1.f, 0.85f},
    {ADC_CTL_CH0,  "F2_MGTY1_AVTT", 1.f, 1.2f},
    {ADC_CTL_CH19, "F2_MGTY2_AVTT", 1.f, 1.2f},
    {ADC_CTL_CH11, "F1_MGTH_AVTT", 1.f, 1.2f},
    {ADC_CTL_CH4,  "F1_MGTY_AVTT", 1.f, 1.2f},
    {ADC_CTL_CH2,  "F2_MGTY1_VCCAUX", 1.f, 1.8f},
    {ADC_CTL_CH17, "F2_MGTY2_VCCAUX", 1.f, 1.8f},
    {ADC_CTL_CH6,  "F1_MGTY_VCCAUX", 1.f, 1.8f},
    {ADC_CTL_CH9,  "F1_MGTH_VCCAUX", 1.f, 1.8f},
    {ADC_CTL_CH1,  "F2_MGTY1_AVCC", 1.f, 0.90f},
    {ADC_CTL_CH18, "F2_MGTY2_AVCC", 1.f, 0.90f},
    {ADC_CTL_CH5,  "F1_MGTY_AVCC", 1.f, 0.90f},
    {ADC_CTL_CH10, "F1_MGTH_AVCC", 1.f, 0.90f},
    {ADC_CTL_TS,   "TM4C_TEMP", 1.f, 0.f}, // this one is special, temp in C
};
#elif defined(REV2)
static
struct ADC_Info_t ADCs[] = {
    {ADC_CTL_CH0, "VCC_12V", 6.f, 12.f},
    {ADC_CTL_CH1, "VCC_M3V3", 2.f, 3.3f},
    {ADC_CTL_CH2, "VCC_3V3", 2.f, 3.3f},
    {ADC_CTL_CH3, "VCC_4V0", 2.f, 4.0f},
    {ADC_CTL_CH4,  "VCC_1V8", 1.f, 1.8f},
    {ADC_CTL_CH5,  "F1_VCCINT", 1.f, 0.85f},
    {ADC_CTL_CH6,  "F1_AVCC", 1.f, 0.90f},
    {ADC_CTL_CH7, "F1_AVTT", 1.f, 1.2f},
	  {ADC_CTL_CH8,  "F1_VCCAUX", 1.f, 1.8f},
    {ADC_CTL_CH9,  "F2_VCCINT", 1.f, 0.85f},
    {ADC_CTL_CH10,  "F2_AVCC", 1.f, 0.90f},
    {ADC_CTL_CH11,  "F2_AVTT", 1.f, 1.2f},
    {ADC_CTL_CH12,  "F2_VCCAUX", 1.f, 1.8f},
    {ADC_CTL_CH13, "CUR_V_12V", 2.f, 2.5f}, // TODO: sf
    {ADC_CTL_CH14, "CUR_V_M3V3", 2.f, 2.5f}, // TODO: sf
    {ADC_CTL_CH15, "CUR_V_4V0", 2.f, 2.5f}, // TODO: sf
    {ADC_CTL_CH16, "CUR_V_F1VCCAUX", 2.f, 2.5f}, // TODO: sf
    {ADC_CTL_CH17, "CUR_V_F1VCCAUX", 2.f, 2.5f}, // TODO: sf
    {ADC_CTL_CH18, "F1_TEMP", (1.004f/1.026f)/0.004f, 35.f}, // degrees K
    {ADC_CTL_CH19, "F2_TEMP", (1.004f/1.026f)/0.004f, 35.f}, // degrees K
    {ADC_CTL_TS,   "TM4C_TEMP", 1.f, 0.f}, // this one is special, temp in C
};
#error need ADC input mapping
#else
#error Need to define either Rev1 or Rev2
#endif
// clang-format on

// -------------------------------------------------
//
// REV 2
//
// -------------------------------------------------
// To be added here 

static __fp16 fADCvalues[ADC_CHANNEL_COUNT]; // ADC values in volts

// read-only accessor functions for ADC names and values.

const char *getADCname(const int i)
{
  configASSERT(i >= 0 && i < ADC_CHANNEL_COUNT);
  return ADCs[i].name;
}

float getADCvalue(const int i)
{
  configASSERT(i >= 0 && i < ADC_CHANNEL_COUNT);
  return fADCvalues[i];
}

float getADCtargetValue(const int i)
{
  configASSERT(i >= 0 && i < ADC_CHANNEL_COUNT);
  return ADCs[i].target_value;
}

// there is a lot of copy-paste in the following functions,
// but it makes it very clear what's going on here.
static void initADC1FirstSequence()
{

  ROM_ADCSequenceDisable(ADC1_BASE, 0);

  ROM_ADCSequenceConfigure(ADC1_BASE, 0, ADC_TRIGGER_PROCESSOR, 0);

  ROM_ADCSequenceStepConfigure(ADC1_BASE, 0, 0, ADCs[0].channel);
  ROM_ADCSequenceStepConfigure(ADC1_BASE, 0, 1, ADCs[1].channel);
  ROM_ADCSequenceStepConfigure(ADC1_BASE, 0, 2, ADCs[2].channel);
  ROM_ADCSequenceStepConfigure(ADC1_BASE, 0, 3, ADCs[3].channel);
  ROM_ADCSequenceStepConfigure(ADC1_BASE, 0, 4, ADCs[4].channel);
  ROM_ADCSequenceStepConfigure(ADC1_BASE, 0, 5, ADCs[5].channel);
  ROM_ADCSequenceStepConfigure(ADC1_BASE, 0, 6, ADCs[6].channel);
  ROM_ADCSequenceStepConfigure(ADC1_BASE, 0, 7, ADCs[7].channel | ADC_CTL_IE | ADC_CTL_END);

  ROM_ADCSequenceEnable(ADC1_BASE, 0);

  ROM_ADCIntClear(ADC1_BASE, 0);
}

static void initADC1SecondSequence()
{
  ROM_ADCSequenceDisable(ADC1_BASE, 0);

  ROM_ADCSequenceConfigure(ADC1_BASE, 0, ADC_TRIGGER_PROCESSOR, 0);
  ROM_ADCSequenceStepConfigure(ADC1_BASE, 0, 0, ADCs[8].channel);
  ROM_ADCSequenceStepConfigure(ADC1_BASE, 0, 1, ADCs[9].channel);
  ROM_ADCSequenceStepConfigure(ADC1_BASE, 0, 2, ADCs[10].channel);
  ROM_ADCSequenceStepConfigure(ADC1_BASE, 0, 3, ADCs[11].channel);
  ROM_ADCSequenceStepConfigure(ADC1_BASE, 0, 4, ADCs[12].channel | ADC_CTL_IE | ADC_CTL_END);
  ROM_ADCSequenceEnable(ADC1_BASE, 0);
  ROM_ADCIntClear(ADC1_BASE, 0);
}

static void initADC0FirstSequence()
{
  ROM_ADCSequenceDisable(ADC0_BASE, 1);
  ROM_ADCSequenceConfigure(ADC0_BASE, 1, ADC_TRIGGER_PROCESSOR, 0);
  ROM_ADCSequenceStepConfigure(ADC0_BASE, 1, 0, ADCs[13].channel);
  ROM_ADCSequenceStepConfigure(ADC0_BASE, 1, 1, ADCs[14].channel);
  ROM_ADCSequenceStepConfigure(ADC0_BASE, 1, 2, ADCs[15].channel);
  ROM_ADCSequenceStepConfigure(ADC0_BASE, 1, 3, ADCs[16].channel | ADC_CTL_IE | ADC_CTL_END);
  ROM_ADCSequenceEnable(ADC0_BASE, 1);
  ROM_ADCIntClear(ADC0_BASE, 1);
}

static void initADC0SecondSequence()
{
  ROM_ADCSequenceDisable(ADC0_BASE, 1);
  ROM_ADCSequenceConfigure(ADC0_BASE, 1, ADC_TRIGGER_PROCESSOR, 0);
  ROM_ADCSequenceStepConfigure(ADC0_BASE, 1, 0, ADCs[17].channel);
  ROM_ADCSequenceStepConfigure(ADC0_BASE, 1, 1, ADCs[18].channel);
  ROM_ADCSequenceStepConfigure(ADC0_BASE, 1, 2, ADCs[19].channel);
  ROM_ADCSequenceStepConfigure(ADC0_BASE, 1, 3, ADCs[20].channel | ADC_CTL_IE | ADC_CTL_END);
  ROM_ADCSequenceEnable(ADC0_BASE, 1);
  ROM_ADCIntClear(ADC0_BASE, 1);
}

// ADC monitoring task.
void ADCMonitorTask(void *parameters)
{
  // initialize to the current tick time
  TickType_t xLastWakeTime = xTaskGetTickCount();
  uint32_t iADCvalues[ADC_CHANNEL_COUNT]; // raw adc outputs

  const TickType_t xMaxBlockTime = pdMS_TO_TICKS(20000);

  for (;;) {
    // First sequence for ADC1
    initADC1FirstSequence();

    // Set up the task notification and start the conversion.
    TaskNotifyADC = xTaskGetCurrentTaskHandle();
    ROM_ADCProcessorTrigger(ADC1_BASE, 0);

    // Wait to be notified that the transmission is complete.
    unsigned long ulNotificationValue = ulTaskNotifyTake(pdTRUE, xMaxBlockTime);

    if (ulNotificationValue == 1) {
      uint32_t got = ROM_ADCSequenceDataGet(ADC1_BASE, 0, iADCvalues);
      configASSERT(got == ADCs_ADC1_FIRST_SEQ_LENGTH);
    }
    else {
      // handle error here
      configASSERT(0);
    }
    ROM_ADCSequenceDisable(ADC1_BASE, 0);

    // ADC0,first sequence
    initADC0FirstSequence();
    TaskNotifyADC = xTaskGetCurrentTaskHandle();
    ROM_ADCProcessorTrigger(ADC0_BASE, 1);
    ulNotificationValue = ulTaskNotifyTake(pdTRUE, xMaxBlockTime);

    if (ulNotificationValue == 1) {
      uint32_t got = ROM_ADCSequenceDataGet(ADC0_BASE, 1, iADCvalues + ADCs_ADC0_START);
      configASSERT(got == ADCs_ADC0_FIRST_SEQ_LENGTH);
    }
    else {
      // handle error here
      configASSERT(0);
    }
    ROM_ADCSequenceDisable(ADC0_BASE, 1);

    // second sequence for ADC0
    initADC0SecondSequence();
    TaskNotifyADC = xTaskGetCurrentTaskHandle();
    ROM_ADCProcessorTrigger(ADC0_BASE, 1);
    ulNotificationValue = ulTaskNotifyTake(pdTRUE, xMaxBlockTime);

    if (ulNotificationValue == 1) {
      uint32_t got = ROM_ADCSequenceDataGet(
          ADC0_BASE, 1, iADCvalues + ADCs_ADC0_START + ADCs_ADC0_FIRST_SEQ_LENGTH);
      configASSERT(got == ADCs_ADC0_SECOND_SEQ_LENGTH);
    }
    else {
      // handle error here
      configASSERT(0);
    }
    ROM_ADCSequenceDisable(ADC0_BASE, 1);

    initADC1SecondSequence();
    TaskNotifyADC = xTaskGetCurrentTaskHandle();
    ROM_ADCProcessorTrigger(ADC1_BASE, 0);

    // Wait to be notified that the transmission is complete.
    ulNotificationValue = ulTaskNotifyTake(pdTRUE, xMaxBlockTime);

    if (ulNotificationValue == 1) {
      uint32_t got = ROM_ADCSequenceDataGet(ADC1_BASE, 0, iADCvalues + ADCs_ADC1_FIRST_SEQ_LENGTH);
      configASSERT(got == ADCs_ADC1_SECOND_SEQ_LENGTH);
    }
    else {
      // handle error here
      configASSERT(0);
    }
    ROM_ADCSequenceDisable(ADC1_BASE, 0);

    // convert data to float values
    for (int i = 0; i < ADC_CHANNEL_COUNT; ++i) {
      fADCvalues[i] = iADCvalues[i] / 4096.f * ADC_MAX_VOLTAGE_RANGE * ADCs[i].scale;
    }
    // special: temperature of Tiva die. Tiva manu 15.3.6, last equation.
    fADCvalues[ADC_INFO_TEMP_ENTRY] =
        147.5f - (75.f * ADC_MAX_VOLTAGE_RANGE * iADCvalues[ADC_INFO_TEMP_ENTRY]) / 4096.f;

    // wait x ms for next iteration
    vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(1000));
  }
}
