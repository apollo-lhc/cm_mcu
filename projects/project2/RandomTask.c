/*
 * RandomTask.c
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
 */

// includes for types
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>

// memory mappings
#include "inc/hw_memmap.h"

// driverlib
#include "driverlib/adc.h"

// FreeRTOS
#include "FreeRTOS.h"
#include "FreeRTOSConfig.h"
#include "task.h"

// On Apollo the ADC range is from 0 - 2.5V.
// Some signals must be scaled to fit in this range.
#define ADC_MAX_VOLTAGE_RANGE 2.5


// a struct to hold some information about the ADC channel.
struct ADC_Info_t {
  int channel; // which of the 20 ADC channels on the TM4C1290NCPDT on Apollo CM v1
  const char* name; // name
  float scale; // scaling, if needed, for signals bigger than 2.5V
};

// These are not in order of channel number but instead grouped by
// which ADC they have been assigned to in the TI PinMux tool. I could
// probably consider reassigning them in the PinMUX tool to make it more
// logical.
#define ADCs_ADC1_START 0
#define ADCs_ADC1_ENTRIES 13
#define ADCs_ADC1_FIRST_SEQ_LENGTH 8
#define ADCs_ADC0_START 13
#define ADCs_ADC0_ENTRIES 8
#define ADCs_ADC0_FIRST_SEQ_LENGTH 4
static
struct ADC_Info_t ADCs[] = {
    {0, "V_MGTY1_AVTT", 1.}, // ADC1
    {1, "V_MGTY1_AVCC", 1.},
    {2, "V_MGTY1_VCCAUX", 1.},
    {3, "V_VCCINT", 1.},
    {12, "VCC_12V", 6.},
    {13, "VCC_2V5", 2.},
    {14, "VCC_M3V3", 2.},
    {15, "VCC_M1V8", 1.},
    {16, "VCC_3V3", 2.},
    {17, "V_MGTY2_VCCAUX", 1.},
    {18, "V_MGTY2_AVCC", 1.},
    {19, "V_MGTY2_AVTT", 1.},
    {20, "TM4C_TEMP", 1.}, // this one is special, temp in C
    {4, "K_MGTY_AVTT", 1.}, // ADC0
    {5, "K_MGTY_AVCC", 1.},
    {6, "K_MGTY_VCCAUX", 1.},
    {7, "VCC_1V8", 1.},
    {8, "K_VCCINT", 1.},
    {9, "K_MGTH_VCCAUX", 1.},
    {10, "K_MGTH_AVCC", 1.},
    {11, "K_MGTH_AVTT", 1.},
};

static float fADCvalues[21]; // ADC values in volts

// read-only accessor functions for ADC names and values.

const char* getADCname(const int i)
{
  configASSERT(i>=0&&i<21);
  return ADCs[i].name;
}

float getADCvalue(const int i)
{
  configASSERT(i>=0&&i<21);
  return fADCvalues[i];
}



// Stores the handle of the task that will be notified when the
// ADC conversion is complete.
static TaskHandle_t TaskNotifyADC = NULL;

void ADCSeq0Interrupt()
{
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;

  ADCIntClear(ADC1_BASE, 0);

  /* At this point xTaskToNotify should not be NULL as a transmission was
      in progress. */
  configASSERT( TaskNotifyADC != NULL );

  /* Notify the task that the transmission is complete. */
  vTaskNotifyGiveFromISR( TaskNotifyADC, &xHigherPriorityTaskWoken );

  /* There are no transmissions in progress, so no tasks to notify. */
  TaskNotifyADC = NULL;

  /* If xHigherPriorityTaskWoken is now set to pdTRUE then a context switch
      should be performed to ensure the interrupt returns directly to the highest
      priority task.  The macro used for this purpose is dependent on the port in
      use and may be called portEND_SWITCHING_ISR(). */
  portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
  return;
}


void ADCSeq1Interrupt()
{
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;

  ADCIntClear(ADC0_BASE, 1);

  /* At this point xTaskToNotify should not be NULL as a transmission was
      in progress. */
  configASSERT( TaskNotifyADC != NULL );

  /* Notify the task that the transmission is complete. */
  vTaskNotifyGiveFromISR( TaskNotifyADC, &xHigherPriorityTaskWoken );

  /* There are no transmissions in progress, so no tasks to notify. */
  TaskNotifyADC = NULL;

  /* If xHigherPriorityTaskWoken is now set to pdTRUE then a context switch
      should be performed to ensure the interrupt returns directly to the highest
      priority task.  The macro used for this purpose is dependent on the port in
      use and may be called portEND_SWITCHING_ISR(). */
  portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
  return;
}

// there is a lot of copy-paste in the following functions,
// but it makes it very clear what's going on here.
static
void initADC1FirstSequence()
{
  ADCSequenceConfigure(ADC1_BASE, 0, ADC_TRIGGER_PROCESSOR, 0);

  ADCSequenceStepConfigure(ADC1_BASE, 0, 0, ADCs[0].channel);
  ADCSequenceStepConfigure(ADC1_BASE, 0, 1, ADCs[1].channel);
  ADCSequenceStepConfigure(ADC1_BASE, 0, 2, ADCs[2].channel);
  ADCSequenceStepConfigure(ADC1_BASE, 0, 3, ADCs[3].channel);
  ADCSequenceStepConfigure(ADC1_BASE, 0, 4, ADCs[4].channel);
  ADCSequenceStepConfigure(ADC1_BASE, 0, 5, ADCs[5].channel);
  ADCSequenceStepConfigure(ADC1_BASE, 0, 6, ADCs[6].channel);
  ADCSequenceStepConfigure(ADC1_BASE, 0, 7, ADCs[7].channel | ADC_CTL_IE | ADC_CTL_END);

  ADCSequenceEnable(ADC1_BASE, 0);


  ADCIntClear(ADC1_BASE, 0);


}

static
void initADC1SecondSequence()
{
  ADCSequenceConfigure(ADC1_BASE, 0, ADC_TRIGGER_PROCESSOR, 0);
  ADCSequenceStepConfigure(ADC1_BASE, 0, 0, ADCs[8].channel);
  ADCSequenceStepConfigure(ADC1_BASE, 0, 1, ADCs[9].channel);
  ADCSequenceStepConfigure(ADC1_BASE, 0, 2, ADCs[10].channel);
  ADCSequenceStepConfigure(ADC1_BASE, 0, 3, ADCs[11].channel);
  ADCSequenceStepConfigure(ADC1_BASE, 0, 4, ADC_CTL_TS| ADC_CTL_IE | ADC_CTL_END);
  ADCSequenceEnable(ADC1_BASE, 0);
  ADCIntClear(ADC1_BASE, 0);

}


static
void initADC0FirstSequence()
{
  ADCSequenceConfigure(ADC0_BASE, 1, ADC_TRIGGER_PROCESSOR, 0);
  ADCSequenceStepConfigure(ADC0_BASE, 1, 0, ADCs[13].channel);
  ADCSequenceStepConfigure(ADC0_BASE, 1, 1, ADCs[14].channel);
  ADCSequenceStepConfigure(ADC0_BASE, 1, 2, ADCs[15].channel);
  ADCSequenceStepConfigure(ADC0_BASE, 1, 3, ADCs[16].channel| ADC_CTL_IE | ADC_CTL_END);
  ADCSequenceEnable(ADC0_BASE, 1);
  ADCIntClear(ADC0_BASE, 1);

}

static
void initADC0SecondSequence()
{
  ADCSequenceConfigure(ADC0_BASE, 1, ADC_TRIGGER_PROCESSOR, 0);
  ADCSequenceStepConfigure(ADC0_BASE, 1, 0, ADCs[17].channel);
  ADCSequenceStepConfigure(ADC0_BASE, 1, 1, ADCs[18].channel);
  ADCSequenceStepConfigure(ADC0_BASE, 1, 2, ADCs[19].channel);
  ADCSequenceStepConfigure(ADC0_BASE, 1, 3, ADCs[20].channel| ADC_CTL_IE | ADC_CTL_END);
  ADCSequenceEnable(ADC0_BASE, 1);
  ADCIntClear(ADC0_BASE, 1);

}


// playground to test various things
void RandomTask(void *parameters)
{
  // initialize to the current tick time
  TickType_t xLastWakeTime = xTaskGetTickCount();
  uint32_t iADCvalues[21]; // raw adc outputs


  const TickType_t xMaxBlockTime = pdMS_TO_TICKS( 20000 );

  for (;;) {
    // First sequence for both ADCs
    initADC1FirstSequence();

    // Set up the task notification and start the conversion.
    TaskNotifyADC = xTaskGetCurrentTaskHandle();
    ADCProcessorTrigger(ADC1_BASE, 0);

    // Wait to be notified that the transmission is complete.
    unsigned long ulNotificationValue = ulTaskNotifyTake( pdTRUE, xMaxBlockTime );

    if( ulNotificationValue == 1 ) {
      ADCSequenceDataGet(ADC1_BASE, 0, iADCvalues);
    }
    else {
      // handle error here
      configASSERT(0);
    }
    // ADC0,first sequence
    initADC0FirstSequence();
    TaskNotifyADC = xTaskGetCurrentTaskHandle();
    ADCProcessorTrigger(ADC0_BASE, 1);
    ulNotificationValue = ulTaskNotifyTake( pdTRUE, xMaxBlockTime );

    if( ulNotificationValue == 1 ) {
      ADCSequenceDataGet(ADC0_BASE, 1, iADCvalues+13); // check offset
    }
    else {
      // handle error here
      configASSERT(0);
    }


    // second sequence
    initADC0SecondSequence();
    TaskNotifyADC = xTaskGetCurrentTaskHandle();
    ADCProcessorTrigger(ADC0_BASE, 1);
    ulNotificationValue = ulTaskNotifyTake( pdTRUE, xMaxBlockTime );

    if( ulNotificationValue == 1 ) {
      ADCSequenceDataGet(ADC0_BASE, 1, iADCvalues+ADCs_ADC1_ENTRIES+ADCs_ADC0_FIRST_SEQ_LENGTH); // check offset
    }
    else {
      // handle error here
      configASSERT(0);
    }
    initADC1SecondSequence();
    TaskNotifyADC = xTaskGetCurrentTaskHandle();
    ADCProcessorTrigger(ADC1_BASE, 0);

    // Wait to be notified that the transmission is complete.
    ulNotificationValue = ulTaskNotifyTake( pdTRUE, xMaxBlockTime );

    if( ulNotificationValue == 1 ) {
      ADCSequenceDataGet(ADC1_BASE, 0, iADCvalues+ADCs_ADC1_FIRST_SEQ_LENGTH);
    }
    else {
      // handle error here
      configASSERT(0);
    }

    // convert data to float values
    for ( int i = 0; i < 21; ++i ) {
      fADCvalues[i] = iADCvalues[i]/4096.*ADC_MAX_VOLTAGE_RANGE * ADCs[i].scale;
    }
    // special: temperature of Tiva die. Tiva manu 15.3.6, last equation.
    fADCvalues[12] = 147.5 - ( 75 * ADC_MAX_VOLTAGE_RANGE * iADCvalues[12])/4096;


    // wait x ms for next iteration
    vTaskDelayUntil( &xLastWakeTime, pdMS_TO_TICKS( 1000 ) );

  }

}
