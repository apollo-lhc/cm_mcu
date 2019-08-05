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
#include <stdio.h>

#include <string.h>

// local includes
#include "common/uart.h"
#include "common/utils.h"
#include "common/power_ctl.h"
#include "common/i2c_reg.h"
#include "common/pinout.h"
#include "common/pinsel.h"
#include "common/version.h"
#include "common/smbus.h"

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


static SemaphoreHandle_t xUARTMutex = NULL;


void Print(const char* str)
{
    xSemaphoreTake( xUARTMutex, portMAX_DELAY );
    {
      UARTPrint(CLI_UART, str);
    }
    xSemaphoreGive( xUARTMutex );
  return;
}

// Alternate UART signal handler
/* A stream buffer that has already been created. */
StreamBufferHandle_t xUARTStreamBuffer;

void UART1IntHandler( void )
{
  // TODO: figure out which UART caused the interrupt -- can I do this?
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
      xStreamBufferSendFromISR(xUARTStreamBuffer, &bytes, 8, &xHigherPriorityTaskWoken);
      received = 0;
    }
  }
  if ( received )
    xStreamBufferSendFromISR(xUARTStreamBuffer, &bytes, received, &xHigherPriorityTaskWoken);

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
  // TODO: figure out which UART caused the interrupt -- can I do this?
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
      xStreamBufferSendFromISR(xUARTStreamBuffer, &bytes, 8, &xHigherPriorityTaskWoken);
      received = 0;
    }
  }
  if ( received )
    xStreamBufferSendFromISR(xUARTStreamBuffer, &bytes, received, &xHigherPriorityTaskWoken);

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


#include "common/smbus.h"
tSMBus g_sMaster1; // for I2C #1
tSMBus g_sMaster2; // for I2C #2
tSMBus g_sMaster3; // for I2C #3
tSMBus g_sMaster4; // for I2C #4
tSMBus g_sMaster6; // for I2C #6
extern tSMBusStatus eStatus1;
extern tSMBusStatus eStatus2;
extern tSMBusStatus eStatus3;
extern tSMBusStatus eStatus4;
extern tSMBusStatus eStatus6;
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






// ARM DWT
#define DEMCR_TRCENA    0x01000000

/* Core Debug registers */
#define DEMCR           (*(volatile uint32_t *)0xE000EDFC)
#define DWT_CTRL        (*(volatile uint32_t *)0xe0001000)
#define CYCCNTENA       (1<<0)
#define DWT_CYCCNT      ((volatile uint32_t *)0xE0001004)
#define CPU_CYCLES      *DWT_CYCCNT

static uint32_t counter, prev_count;

void stopwatch_reset(void)
{
    /* Enable DWT */
    DEMCR |= DEMCR_TRCENA;
    *DWT_CYCCNT = 0;
    /* Enable CPU cycle counter */
    DWT_CTRL |= CYCCNTENA;
    counter = prev_count = 0U;
}

uint32_t stopwatch_getticks()
{
    uint32_t curr_count =  CPU_CYCLES;
    uint32_t diff = curr_count - prev_count;
    prev_count = curr_count;
    counter += diff>>12;
    return counter;
}



void SystemInit()
{
  //
  // Run from the PLL, internal oscillator, at the defined clock speed configCPU_CLOCK_HZ
  //
  g_ui32SysClock = MAP_SysCtlClockFreqSet((SYSCTL_OSC_INT|SYSCTL_USE_PLL |
                                           SYSCTL_CFG_VCO_320), configCPU_CLOCK_HZ);

  // initialize all pins, using file setup by TI PINMUX tool
  PinoutSet();


  // Set up the CLI
#if (CLI_UART == UART4_BASE) // front panel
  UART4Init(g_ui32SysClock);
#elif (CLI_UART == UART1_BASE) // zynq
  UART1Init(g_ui32SysClock);
#else 
#error "CLI UART not initialized"
#endif
  initI2C1(g_ui32SysClock); // controller for power supplies
  initI2C2(g_ui32SysClock); // controller for clocks
  initI2C3(g_ui32SysClock); // controller for V optics
  initI2C6(g_ui32SysClock); // controller for FPGAs
  initI2C4(g_ui32SysClock); // controller for K optics
  
  // initialize the ADCs.
  ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);
  ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC1);
  // Set the reference to external
  ROM_ADCReferenceSet(ADC0_BASE, ADC_REF_EXT_3V );
  ROM_ADCReferenceSet(ADC1_BASE, ADC_REF_EXT_3V );
  ROM_ADCIntEnable(ADC0_BASE, 1); // enable interrupt for sequence 1
  ROM_ADCIntEnable(ADC1_BASE, 0); // enable interrupt for sequence 0
  // clear the interrupts
  ROM_ADCIntClear(ADC0_BASE, 1);
  ROM_ADCIntClear(ADC1_BASE, 0);
  // FreeRTOS insists that the priority of interrupts be set up like this.
  ROM_IntPrioritySet( INT_ADC0SS1, configKERNEL_INTERRUPT_PRIORITY );
  ROM_IntPrioritySet( INT_ADC1SS0, configKERNEL_INTERRUPT_PRIORITY );

  ROM_IntEnable(INT_ADC0SS1);
  ROM_IntEnable(INT_ADC1SS0);

  //smbus
  // Initialize the master SMBus port.
  //
  SMBusMasterInit(&g_sMaster1, I2C1_BASE, g_ui32SysClock);
  SMBusMasterInit(&g_sMaster2, I2C2_BASE, g_ui32SysClock);
  SMBusMasterInit(&g_sMaster3, I2C3_BASE, g_ui32SysClock);
  SMBusMasterInit(&g_sMaster4, I2C4_BASE, g_ui32SysClock);
  SMBusMasterInit(&g_sMaster6, I2C6_BASE, g_ui32SysClock);
  //SMBusPECEnable(&g_sMaster);

  // FreeRTOS insists that the priority of interrupts be set up like this.
  ROM_IntPrioritySet( INT_I2C1, configKERNEL_INTERRUPT_PRIORITY );
  ROM_IntPrioritySet( INT_I2C2, configKERNEL_INTERRUPT_PRIORITY );
  ROM_IntPrioritySet( INT_I2C3, configKERNEL_INTERRUPT_PRIORITY );
  ROM_IntPrioritySet( INT_I2C4, configKERNEL_INTERRUPT_PRIORITY );
  ROM_IntPrioritySet( INT_I2C6, configKERNEL_INTERRUPT_PRIORITY );

  //
  // Enable master interrupts.
  //
  SMBusMasterIntEnable(&g_sMaster1);
  SMBusMasterIntEnable(&g_sMaster2);
  SMBusMasterIntEnable(&g_sMaster3);
  SMBusMasterIntEnable(&g_sMaster4);
  SMBusMasterIntEnable(&g_sMaster6);


  //
  // Enable processor interrupts.
  //
  // MAP_IntMasterEnable(); the FreeRTOS kernel does this at the appropriate moment

  setupActiveLowPins();

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
void MonitorTask(void *parameters);

// firefly monitoring
void FireFlyTask(void *parameters);

// Command line interface
void vCommandLineTask(void *parameters);


// Monitoring using the ADC inputs
void ADCMonitorTask(void *parameters);

void ShortDelay()
{
  vTaskDelay(pdMS_TO_TICKS(100));
}


struct TaskNamePair_t {
  char key[configMAX_TASK_NAME_LEN];
  TaskHandle_t value;
} ;

static struct TaskNamePair_t TaskNamePairs[6];

void vGetTaskHandle( char *key, TaskHandle_t *t)
{
  *t = NULL;
  for (int i = 0; i < 6; ++i) {
    if ( strncmp(key, TaskNamePairs[i].key,3) == 0)
      *t = TaskNamePairs[i].value;
  }
  return ;
}

// 
int main( void )
{

  // mutex for the UART output
  xUARTMutex = xSemaphoreCreateMutex();

  //  Create the stream buffer that sends data from the interrupt to the
  //  task, and create the task.
  // todo: handle sending more than one byte at a time, if needed
  xUARTStreamBuffer = xStreamBufferCreate( 128, // length of stream buffer in bytes
                                           1);  // number of items before a trigger is sent


  // start the tasks here 
  xTaskCreate(PowerSupplyTask, "POW", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY+5, &TaskNamePairs[0].value);
  xTaskCreate(LedTask,         "LED", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY+2, &TaskNamePairs[1].value);
  xTaskCreate(vCommandLineTask,"CON", 512,                      NULL, tskIDLE_PRIORITY+1, &TaskNamePairs[2].value);
  xTaskCreate(ADCMonitorTask,  "ADC", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY+4, &TaskNamePairs[3].value);
  xTaskCreate(MonitorTask,     "MON", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY+4, &TaskNamePairs[4].value);
  xTaskCreate(FireFlyTask,     "FLY", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY+4, &TaskNamePairs[5].value);

  snprintf(TaskNamePairs[0].key,configMAX_TASK_NAME_LEN,"POW");
  snprintf(TaskNamePairs[1].key,configMAX_TASK_NAME_LEN,"LED");
  snprintf(TaskNamePairs[2].key,configMAX_TASK_NAME_LEN,"CON");
  snprintf(TaskNamePairs[3].key,configMAX_TASK_NAME_LEN,"ADC");
  snprintf(TaskNamePairs[4].key,configMAX_TASK_NAME_LEN,"MON");
  snprintf(TaskNamePairs[5].key,configMAX_TASK_NAME_LEN,"FLY");

  // queue for the LED
  xLedQueue = xQueueCreate(5, // The maximum number of items the queue can hold.
      sizeof( uint32_t ));    // The size of each item.
  configASSERT(xLedQueue != NULL);

  xPwrQueue = xQueueCreate(10, sizeof(uint32_t)); // PWR queue
  configASSERT(xPwrQueue != NULL);


  vQueueAddToRegistry(xLedQueue, "LedQueue");
  vQueueAddToRegistry(xPwrQueue, "PwrQueue");

  // Set up the hardware ready to run the demo. Don't do this earlier as the interrupts
  // call some FreeRTOS tasks that need to be set up first.
  SystemInit();


  Print("\n----------------------------\n\r");
  Print("Staring Project2 " FIRMWARE_VERSION " (FreeRTOS scheduler about to start)\n\r");
  Print(  "----------------------------\n\r");
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

