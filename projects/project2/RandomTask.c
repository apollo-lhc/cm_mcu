/*
 * RandomTask.c
 *
 *  Created on: May 19, 2019
 *      Author: wittich
 */

// includes for types
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>

// memory mappings
#include "inc/hw_types.h"
#include "inc/hw_memmap.h"

// driverlib
#include "driverlib/adc.h"
#include "driverlib/sysctl.h" // --> systeminit
#include "driverlib/gpio.h"   // --> systeminit

// FreeRTOS
#include "FreeRTOS.h"
#include "FreeRTOSConfig.h"
#include "task.h"

//// local includes
//#include "common/i2c_reg.h"
//#include "common/smbus.h"
//#include "common/smbus_units.h"

struct ADC_Info_t {
  int channel;
  const char* name;
  float scale;
};

struct ADC_Info_t ADCs[] = {
    {0, "V_MGTY1_AVTT", 1.}, // ADC1
    {1, "V_MGTY1_AVCC", 1.},
    {2, "V_MGTY1_VCCAUX", 1.},
    {3, "V_VCCINT", 1.},
    {12, "VCC_12V", 8.},
    {13, "VCC_2V5", 2.},
    {14, "VCC_M3V3", 2.},
    {15, "VCC_M1V8", 1.},
    {16, "VCC_3V3", 1.},
    {17, "V_MGTY2_VCCAUX", 1.},
    {18, "V_MGTY2_AVCC", 1.},
    {19, "V_MGTY2_AVTT", 1.},
    {20, "TM4C_TEMP", 1.}, // this one is special: (1475 - ((2475 * ui32TempAvg)) / 4096) / 10;
    {4, "K_MGTY_AVTT", 1.}, // ADC0
    {5, "K_MGTY_AVCC", 1.},
    {6, "K_MGTY_VCCAUX", 1.},
    {7, "VCC_1V8", 1.},
    {8, "K_VCCINT", 1.},
    {9, "K_MGTH_VCCAUX", 1.},
    {10, "K_MGTH_AVCC", 1.},
    {11, "K_MGTH_AVTT", 1.},
};

static uint32_t iADCvalues[21]; //
static uint32_t fADCvalues[21]; //


static int currentADC1Sequence = 0;
static int currentADC0Sequence = 0;

// Stores the handle of the task that will be notified when the
// transmission is complete.
static TaskHandle_t TaskNotifyADC = NULL;

void ADC0Interrupt()
{
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;

  ADCIntClear(ADC0_BASE, currentADC0Sequence);

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


void ADC1Interrupt()
{
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;

  ADCIntClear(ADC1_BASE, currentADC1Sequence);

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

  currentADC1Sequence = 0;

}

static
void initADC1SecondSequence()
{
  ADCSequenceConfigure(ADC1_BASE, 0, ADC_TRIGGER_PROCESSOR, 0);
  ADCSequenceStepConfigure(ADC1_BASE, 0, 0, ADCs[8].channel);
  ADCSequenceStepConfigure(ADC1_BASE, 0, 0, ADCs[9].channel);
  ADCSequenceStepConfigure(ADC1_BASE, 0, 0, ADCs[10].channel);
  ADCSequenceStepConfigure(ADC1_BASE, 0, 0, ADCs[11].channel);
  ADCSequenceStepConfigure(ADC1_BASE, 0, 0, ADC_CTL_TS| ADC_CTL_IE | ADC_CTL_END);
  ADCSequenceEnable(ADC1_BASE, 0);
  ADCIntClear(ADC1_BASE, 0);

  currentADC1Sequence = 0;
}


static
void initADC0FirstSequence()
{
  ADCSequenceConfigure(ADC0_BASE, 1, ADC_TRIGGER_PROCESSOR, 0);
  ADCSequenceStepConfigure(ADC0_BASE, 1, 0, ADCs[13].channel);
  ADCSequenceStepConfigure(ADC0_BASE, 1, 0, ADCs[14].channel);
  ADCSequenceStepConfigure(ADC0_BASE, 1, 0, ADCs[15].channel);
  ADCSequenceStepConfigure(ADC0_BASE, 1, 0, ADCs[16].channel| ADC_CTL_IE | ADC_CTL_END);
  ADCSequenceEnable(ADC0_BASE, 1);
  ADCIntClear(ADC0_BASE, 1);

}

static
void initADC0SecondSequence()
{
  ADCSequenceConfigure(ADC0_BASE, 1, ADC_TRIGGER_PROCESSOR, 0);
  ADCSequenceStepConfigure(ADC0_BASE, 1, 0, ADCs[17].channel);
  ADCSequenceStepConfigure(ADC0_BASE, 1, 0, ADCs[18].channel);
  ADCSequenceStepConfigure(ADC0_BASE, 1, 0, ADCs[19].channel);
  ADCSequenceStepConfigure(ADC0_BASE, 1, 0, ADCs[20].channel| ADC_CTL_IE | ADC_CTL_END);
  ADCSequenceEnable(ADC0_BASE, 1);
  ADCIntClear(ADC0_BASE, 1);

}


// playground to test various things
void RandomTask(void *parameters)
{
  // initialize to the current tick time
  TickType_t xLastWakeTime = xTaskGetTickCount();

  // initialize the ADCs. This should all go into SystemInit()
  SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
  GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_3);


  const TickType_t xMaxBlockTime = pdMS_TO_TICKS( 200 );

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
    }
    // ADC0,first sequence
    initADC0FirstSequence();
    TaskNotifyADC = xTaskGetCurrentTaskHandle();
    ADCProcessorTrigger(ADC0_BASE, 1);
    ulNotificationValue = ulTaskNotifyTake( pdTRUE, xMaxBlockTime );

    if( ulNotificationValue == 1 ) {
      ADCSequenceDataGet(ADC1_BASE, 0, iADCvalues+14); // check offset
    }
    else {
      // handle error here
    }


    // second sequence
    initADC0SecondSequence();
    TaskNotifyADC = xTaskGetCurrentTaskHandle();
    ADCProcessorTrigger(ADC0_BASE, 1);
    ulNotificationValue = ulTaskNotifyTake( pdTRUE, xMaxBlockTime );

    if( ulNotificationValue == 1 ) {
      ADCSequenceDataGet(ADC1_BASE, 0, iADCvalues+14+4); // check offset
    }
    else {
      // handle error here
    }
    initADC1SecondSequence();
    TaskNotifyADC = xTaskGetCurrentTaskHandle();
    ADCProcessorTrigger(ADC1_BASE, 0);

    // Wait to be notified that the transmission is complete.
    ulNotificationValue = ulTaskNotifyTake( pdTRUE, xMaxBlockTime );

    if( ulNotificationValue == 1 ) {
      ADCSequenceDataGet(ADC1_BASE, 0, iADCvalues+8);
    }
    else {
      // handle error here
    }

    // convert data to float values
    for ( int i = 0; i < 21; ++i ) {
      fADCvalues[i] = iADCvalues[i] * ADCs[i].scale;
    }
    // special: temperature of Tiva die
    fADCvalues[13] = (1475 - ((2475 * iADCvalues[13] )) / 4096) / 10.;


    // wait x ms for next iteration
    vTaskDelayUntil( &xLastWakeTime, pdMS_TO_TICKS( 1000 ) );

  }

}
