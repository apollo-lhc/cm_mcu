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


// Alternate UART signal handler
/* A stream buffer that has already been created. */
StreamBufferHandle_t xStreamBuffer;

void UARTIntHandler( void )
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
	  xStreamBufferSendFromISR(xStreamBuffer, &bytes, 8, &xHigherPriorityTaskWoken);
	  received = 0;
      }
  }
  if ( received )
    xStreamBufferSendFromISR(xStreamBuffer, &bytes, 8, &xHigherPriorityTaskWoken);

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


//*****************************************************************************
//
// The UART interrupt handler.
//
//*****************************************************************************
void
UARTIntHandler2(void)
{
  uint32_t ui32Status;

  //
  // Get the interrupt status.
  //
  ui32Status = ROM_UARTIntStatus(UART4_BASE, true);

  //
  // Clear the asserted interrupts.
  //
  ROM_UARTIntClear(UART4_BASE, ui32Status);

  //
  // Loop while there are characters in the receive FIFO.
  //
  while(ROM_UARTCharsAvail(UART4_BASE))
    {
      //
      // Read the next character from the UART and write it back to the UART.
      //
      ROM_UARTCharPutNonBlocking(UART4_BASE,
				 ROM_UARTCharGetNonBlocking(UART4_BASE));

      //
      // Blink the LED to show a character transfer is occurring.
      //
      MAP_GPIOPinWrite(USER_LED12_PORT, USER_LED2_PIN, USER_LED2_PIN);

      //
      // Delay for 1 millisecond.  Each SysCtlDelay is about 3 clocks.
      //
      SysCtlDelay(g_ui32SysClock / (1000 * 3));

      //
      // Turn off the LED
      //
      MAP_GPIOPinWrite(USER_LED12_PORT, USER_LED2_PIN, 0x0);

    }
}



void SystemInit()
{
  // initialize all pins, using file setup by TI PINMUX tool
  PinoutSet();

  //
  // Run from the PLL at 120 MHz.
  //
  g_ui32SysClock = SysCtlClockFreqSet((SYSCTL_OSC_INT |
                                       SYSCTL_USE_PLL |
                                       SYSCTL_CFG_VCO_480), 120000000);
  UART4Init(g_ui32SysClock);
  initI2C1(g_ui32SysClock);

  // SYSTICK timer
  MAP_SysTickPeriodSet(100); // period in Hz
  MAP_SysTickIntEnable(); // enable the interrupt
  MAP_SysTickEnable();
  return;
}

volatile uint32_t g_ui32SysTickCount;




// Holds the handle of the created queue.
static QueueHandle_t xLedQueue = NULL;

#define PS_BAD 0x03;
#define PS_GOOD 0x01;

// control the LED
void LedTask(void *parameters)
{
  // TODO: this should also handle blinking, by using a non-blocking queue check and a vTaskDelayUntil call
	TickType_t xLastWakeTime = xTaskGetTickCount();
  uint32_t result;
  // this function never returns
  for ( ;; ) {
	  // check for a new item in the queue but don't wait
	  xQueueReceive(xLedQueue, &result, 0);
	  switch (result ) {
	  case 0x00:
		  break;
	  case 0x01:
		  // do something
		  break;
	  default:
		  break;
	  }
	  // wait for next check
	  vTaskDelayUntil( &xLastWakeTime, pdMS_TO_TICKS( 250 ) );

  }
}
// monitor and control the power supplies
void PowerSupplyTask(void *parameters)
{
  // initialize to the current tick time
  TickType_t xLastWakeTime = xTaskGetTickCount();
  bool lastStateGood = false;

  // this function never returns
  for ( ;; ) {
	  uint32_t message;
	  bool good = check_ps();
	  if ( ! good ) {
		  message = PS_BAD;
		  lastStateGood = false;
	  }
	  else if ( !lastStateGood ) { // good now, was bad
		  message = PS_GOOD;
		  lastStateGood = true;
	  }
	  xQueueSendToBack(xLedQueue, &message, pdMS_TO_TICKS(10));

	  vTaskDelayUntil( &xLastWakeTime, pdMS_TO_TICKS( 250 ) );
  }
}

extern void vCommandLineTask(void *parameters);



// 
int main( void )
{
  // Set up the hardware ready to run the demo. 
  SystemInit();

  // semaphore for the UART

  // queue for the LED
  xLedQueue = xQueueCreate(5, // The maximum number of items the queue can hold.
			   sizeof( uint32_t )); 		// The size of each item.


  // start the tasks here 
  xTaskCreate(PowerSupplyTask, "POW", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY+5, NULL);
  xTaskCreate(LedTask,         "LED", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY+2, NULL);
  xTaskCreate(vCommandLineTask,"CON", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY+1, NULL);

  // start the scheduler -- this function should not return
  vTaskStartScheduler();

  // should never get here 
  for( ;; );
}


/*-----------------------------------------------------------*/

void vApplicationStackOverflowHook( TaskHandle_t pxTask, char *pcTaskName )
{
  /* If configCHECK_FOR_STACK_OVERFLOW is set to either 1 or 2 then this
     function will automatically get called if a task overflows its stack. */
  ( void ) pxTask;
  ( void ) pcTaskName;
  for( ;; );
}
/*-----------------------------------------------------------*/

void vApplicationIdleHook( void )
{
   // not doing anything right now; this is just here in case I need to use it
}


