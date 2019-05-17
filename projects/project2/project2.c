//*****************************************************************************
//
// FreeRTOS v2 -- wittich 3/2019
// initially based on the demo CORTEX_M4F_CEC_MEC_17xx_51xx_Keil_GCC
// chosen as one that has a CORTEX M4F and GCC
// that file had no makefile, so ...
//
//*****************************************************************************

// TODO: break this out into separate files. Too much clutter here.

// includes for types
#include <stdint.h>
#include <stdbool.h>

// local includes
#include "common/uart.h"
#include "common/utils.h"
#include "common/power_ctl.h"
#include "common/i2c_reg.h"
#include "common/pinout.h"
#include "common/pinsel.h"
#include "common/version.h"

// TI Includes
#include "inc/hw_types.h"
#include "inc/hw_memmap.h"
#include "inc/hw_ints.h"
#include "driverlib/pin_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "driverlib/uart.h"
#include "driverlib/interrupt.h"


// FreeRTOS includes
#include "FreeRTOS.h"
#include "FreeRTOSConfig.h"
#include "task.h"
#include "queue.h"
#include "stream_buffer.h"
#include "semphr.h"
#include "portmacro.h"
//*****************************************************************************
//
// Define pin to LED mapping.
//
//*****************************************************************************

#define USER_LED1_PIN  GPIO_PIN_0
#define USER_LED2_PIN  GPIO_PIN_1
#define USER_LED12_PORT GPIO_PORTJ_BASE // same port

#define USER_LED3_PIN  GPIO_PIN_0
#define USER_LED3_PORT GPIO_PORTP_BASE


//*****************************************************************************
//
// The error routine that is called if the driver library encounters an error.
//
//*****************************************************************************
#ifdef DEBUG
void
__error__(char *pcFilename, uint32_t ui32Line)
{
}
#endif



uint32_t g_ui32SysClock = 0;


static SemaphoreHandle_t xMutex = NULL;


void Print(const char* str)
{
    xSemaphoreTake( xMutex, portMAX_DELAY );
    {
      UARTPrint(CLI_UART, str);
    }
    xSemaphoreGive( xMutex );
  return;
}

// Alternate UART signal handler
/* A stream buffer that has already been created. */
StreamBufferHandle_t xStreamBuffer;

void UARTIntHandler( void )
{
  // TODO: figure out which UART caused the interrupt -- can I do this?
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  //
  // Get the interrupt status.
  //
  uint32_t ui32Status = ROM_UARTIntStatus(CLI_UART, true);

  //
  // Clear the asserted interrupts.
  //
  ROM_UARTIntClear(CLI_UART, ui32Status);

  //
  // Loop while there are characters in the receive FIFO.
  //
  uint8_t bytes[8];
  int received = 0;
  while(ROM_UARTCharsAvail(CLI_UART)) {

    bytes[received] = (uint8_t)ROM_UARTCharGetNonBlocking(CLI_UART);
    // Put byte in queue (ISR safe function) -- should probably send more than one byte at a time?
    if ( ++received == 8 ) {
      xStreamBufferSendFromISR(xStreamBuffer, &bytes, 8, &xHigherPriorityTaskWoken);
      received = 0;
    }
  }
  if ( received )
    xStreamBufferSendFromISR(xStreamBuffer, &bytes, received, &xHigherPriorityTaskWoken);

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




void SystemInit()
{
  //
  // Run from the PLL, internal oscillator, at the defined clock speed configCPU_CLOCK_HZ
  //
  g_ui32SysClock = SysCtlClockFreqSet((SYSCTL_OSC_INT|SYSCTL_USE_PLL |
                                       SYSCTL_CFG_VCO_480), configCPU_CLOCK_HZ);
  // Enable processor interrupts.
  //
  MAP_IntMasterEnable();

  // initialize all pins, using file setup by TI PINMUX tool
  PinoutSet();
  setupActiveLowPins();

#if (CLI_UART == UART4_BASE)
  UART4Init(g_ui32SysClock);
#else
#error "CLI UART not initialized"
#endif
  initI2C1(g_ui32SysClock);

  // SYSTICK timer -- this is already enabled in the portable layer
  return;
}

volatile uint32_t g_ui32SysTickCount;




// Holds the handle of the created queue for the LED task.
extern QueueHandle_t xLedQueue;

// control the LED
void LedTask(void *parameters);

// Holds the handle of the created queue for the power supply task.
extern QueueHandle_t xPwrQueue;


// monitor and control the power supplies
void PowerSupplyTask(void *parameters);

void vCommandLineTask(void *parameters);

// playground to test the ADCs
void RandomTask(void *parameters)
{
  // initialize to the current tick time
  TickType_t xLastWakeTime = xTaskGetTickCount();
  // TODO: this should go in system init
  MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);

  for (;;) {


    // wait here for the x msec, where x is 2nd argument below.
    vTaskDelayUntil( &xLastWakeTime, pdMS_TO_TICKS( 250 ) );
  }

}



// 
int main( void )
{
  // Set up the hardware ready to run the demo. 
  SystemInit();

  // mutex for the UART output
  xMutex = xSemaphoreCreateMutex();

  //  Create the stream buffer that sends data from the interrupt to the
  //  task, and create the task.
  // todo: handle sending more than one byte at a time, if needed
  xStreamBuffer = xStreamBufferCreate( 128, // length of stream buffer in bytes
                     1); // number of items before a trigger is sent

  // start the tasks here 
  xTaskCreate(PowerSupplyTask, "POW", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY+5, NULL);
  xTaskCreate(LedTask,         "LED", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY+2, NULL);
  xTaskCreate(vCommandLineTask,"CON", 256, NULL, tskIDLE_PRIORITY+1, NULL);

  // queue for the LED
  xLedQueue = xQueueCreate(5, // The maximum number of items the queue can hold.
      sizeof( uint32_t ));    // The size of each item.
  configASSERT(xLedQueue != NULL);

  xPwrQueue = xQueueCreate(5, sizeof(uint32_t)); // PWR queue
  configASSERT(xPwrQueue != NULL);
//  char tmp[128];
//  snprintf(tmp, 128, "queue pointers: LED: 0x%x, PWR: 0x%0x\n", &xLedQueue, &xPwrQueue);
//  Print(tmp);


  vQueueAddToRegistry(xLedQueue, "LedQueue");
  vQueueAddToRegistry(xPwrQueue, "PwrQueue");

  Print("\n----------------------------\n");
  Print("Staring Project2 " FIRMWARE_VERSION " (FreeRTOS scheduler about to start)\n");
  Print(  "----------------------------\n");
  // start the scheduler -- this function should not return
  vTaskStartScheduler();

  // should never get here 
  for( ;; );
}


/*-----------------------------------------------------------*/
#if  (configCHECK_FOR_STACK_OVERFLOW != 0)
void vApplicationStackOverflowHook( TaskHandle_t pxTask, char *pcTaskName )
{
  /* If configCHECK_FOR_STACK_OVERFLOW is set to either 1 or 2 then this
     function will automatically get called if a task overflows its stack. */
  ( void ) pxTask;
  ( void ) pcTaskName;
  for( ;; );
}
#endif
/*-----------------------------------------------------------*/

#if ( configUSE_IDLE_HOOK == 1 )
void vApplicationIdleHook( void )
{
  // not doing anything right now; this is just here in case I need to use it
}
#endif

