/*
 * InterruptHandlers.c
 *
 *  Created on: Aug 23, 2019
 *      Author: wittich
 */

// includes for types
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>

#include <string.h>

#include "InterruptHandlers.h"

// local includes
#include "common/uart.h"
#include "common/utils.h"
#include "common/power_ctl.h"
#include "common/i2c_reg.h"
#include "common/pinout.h"
#include "common/pinsel.h"
#include "common/smbus.h"
#include "common/smbus.h"
#include "common/softuart.h"

// TI Includes
#include "inc/hw_types.h"
#include "inc/hw_memmap.h"
#include "inc/hw_ints.h"
#include "driverlib/pin_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/rom.h"
#include "driverlib/adc.h"
#include "driverlib/rom_map.h"
#include "driverlib/uart.h"
#include "driverlib/timer.h"
#include "driverlib/interrupt.h"



#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "portmacro.h"



// Stream buffers for UART communication
StreamBufferHandle_t xUART4StreamBuffer, xUART1StreamBuffer;

void UART1IntHandler( void )
{
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  //
  // Get the interrupt status.
  //
  uint32_t ui32Status = ROM_UARTIntStatus(UART1_BASE, true);

  //
  // Clear the asserted interrupts.
  //
  ROM_UARTIntClear(UART1_BASE, ui32Status);

  //
  // Loop while there are characters in the receive FIFO.
  //
  uint8_t bytes[8];
  int received = 0;
  while(ROM_UARTCharsAvail(UART1_BASE)) {

    bytes[received] = (uint8_t)ROM_UARTCharGetNonBlocking(UART1_BASE);
    // Put byte in queue (ISR safe function) -- should probably send more than one byte at a time?
    if ( ++received == 8 ) {
      xStreamBufferSendFromISR(xUART1StreamBuffer, &bytes, 8, &xHigherPriorityTaskWoken);
      received = 0;
    }
  }
  if ( received )
    xStreamBufferSendFromISR(xUART1StreamBuffer, &bytes, received, &xHigherPriorityTaskWoken);

  /* If xHigherPriorityTaskWoken was set to pdTRUE inside
    xStreamBufferReceiveFromISR() then a task that has a priority above the
    priority of the currently executing task was unblocked and a context
    switch should be performed to ensure the ISR returns to the unblocked
    task.  In most FreeRTOS ports this is done by simply passing
    xHigherPriorityTaskWoken into taskYIELD_FROM_ISR(), which will test the
    variables value, and perform the context switch if necessary.  Check the
    documentation for the port in use for port specific instructions. */
  portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
}

void UART4IntHandler( void )
{
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  //
  // Get the interrupt status.
  //
  uint32_t ui32Status = ROM_UARTIntStatus(UART4_BASE, true);

  //
  // Clear the asserted interrupts.
  //
  ROM_UARTIntClear(UART4_BASE, ui32Status);

  //
  // Loop while there are characters in the receive FIFO.
  //
  uint8_t bytes[8];
  int received = 0;
  while(ROM_UARTCharsAvail(UART4_BASE)) {

    bytes[received] = (uint8_t)ROM_UARTCharGetNonBlocking(UART4_BASE);
    // Put byte in queue (ISR safe function) -- should probably send more than one byte at a time?
    if ( ++received == 8 ) {
      xStreamBufferSendFromISR(xUART4StreamBuffer, &bytes, 8, &xHigherPriorityTaskWoken);
      received = 0;
    }
  }
  if ( received )
    xStreamBufferSendFromISR(xUART4StreamBuffer, &bytes, received, &xHigherPriorityTaskWoken);

  /* If xHigherPriorityTaskWoken was set to pdTRUE inside
    xStreamBufferReceiveFromISR() then a task that has a priority above the
    priority of the currently executing task was unblocked and a context
    switch should be performed to ensure the ISR returns to the unblocked
    task.  In most FreeRTOS ports this is done by simply passing
    xHigherPriorityTaskWoken into taskYIELD_FROM_ISR(), which will test the
    variables value, and perform the context switch if necessary.  Check the
    documentation for the port in use for port specific instructions. */
  portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
}

//tSMBus g_sSlave0;  // for I2C #0

tSMBus g_sMaster1; // for I2C #1
tSMBus g_sMaster2; // for I2C #2
tSMBus g_sMaster3; // for I2C #3
tSMBus g_sMaster4; // for I2C #4
tSMBus g_sMaster6; // for I2C #6

volatile tSMBusStatus eStatus1 = SMBUS_OK;
volatile tSMBusStatus eStatus2 = SMBUS_OK;
volatile tSMBusStatus eStatus3 = SMBUS_OK;
volatile tSMBusStatus eStatus4 = SMBUS_OK;
volatile tSMBusStatus eStatus6 = SMBUS_OK;



// SMBUs specific handler for I2C
void
SMBusMasterIntHandler1(void)
{
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;

  //
  // Process the interrupt.
  //
  eStatus1 = SMBusMasterIntProcess(&g_sMaster1);
  // handle errors in the returning function
  portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
}

void
SMBusMasterIntHandler2(void)
{
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;

  //
  // Process the interrupt.
  //
  eStatus2 = SMBusMasterIntProcess(&g_sMaster2);
  // handle errors in the returning function
  portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
}

void
SMBusMasterIntHandler3(void)
{
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;

  //
  // Process the interrupt.
  //
  eStatus3 = SMBusMasterIntProcess(&g_sMaster3);
  // handle errors in the returning function
  portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
}

void
SMBusMasterIntHandler4(void)
{
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;

  //
  // Process the interrupt.
  //
  eStatus4 = SMBusMasterIntProcess(&g_sMaster4);
  // handle errors in the returning function
  portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
}

void
SMBusMasterIntHandler6(void)
{
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;

  //
  // Process the interrupt.
  //
  eStatus6 = SMBusMasterIntProcess(&g_sMaster6);
  // handle errors in the returning function
  portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
}



// Stores the handle of the task that will be notified when the
// ADC conversion is complete.
TaskHandle_t TaskNotifyADC = NULL;

void ADCSeq0Interrupt()
{
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;

  ROM_ADCIntClear(ADC1_BASE, 0);

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

  ROM_ADCIntClear(ADC0_BASE, 1);

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

// -----------------------------------------
TaskHandle_t TaskNotifyI2CSlave = NULL;


void I2CSlave0Interrupt()
{
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;

  // read the interrupt register
  uint32_t ui32InterruptStatus = ROM_I2CSlaveIntStatusEx(I2C0_BASE, true);

  // clear the interrupt register
  ROM_I2CSlaveIntClear(I2C0_BASE);
  //ROM_SysCtlDelay(100u);
  /* At this point xTaskToNotify should not be NULL as a transmission was
      in progress. */
  configASSERT( TaskNotifyI2CSlave != NULL );

  /* Notify the task that the transmission is complete. */
  xTaskNotifyFromISR(TaskNotifyI2CSlave, ui32InterruptStatus,
                     eSetValueWithOverwrite, &xHigherPriorityTaskWoken);

  /* There are no transmissions in progress, so no tasks to notify. */
  TaskNotifyI2CSlave = NULL;


  /* If xHigherPriorityTaskWoken is now set to pdTRUE then a context switch
      should be performed to ensure the interrupt returns directly to the highest
      priority task.  The macro used for this purpose is dependent on the port in
      use and may be called portEND_SWITCHING_ISR(). */
  portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
}

// Soft UART related
extern tSoftUART g_sUART;
//
// The transmit timer tick function.
//
void
Timer0AIntHandler(void)
{
  //
  // Clear the timer interrupt.
  //
  TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
  //
  // Call the software UART transmit timer tick function.
  //
  SoftUARTTxTimerTick(&g_sUART);
}

