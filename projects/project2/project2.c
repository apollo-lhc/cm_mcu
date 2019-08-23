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
#include "common/smbus.h"
#include "CommandLineTask.h"
#include "InterruptHandlers.h"

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


  // Set up the UARTs for the CLI
  UART1Init(g_ui32SysClock); // ZYNQ UART
  UART4Init(g_ui32SysClock); // front panel UART

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

  // Set up the I2C controllers
  initI2C1(g_ui32SysClock); // controller for power supplies
  initI2C2(g_ui32SysClock); // controller for clocks
  initI2C3(g_ui32SysClock); // controller for V optics
  initI2C4(g_ui32SysClock); // controller for K optics
  initI2C6(g_ui32SysClock); // controller for FPGAs

  //smbus
  // Initialize the master SMBus port.
  //
  SMBusMasterInit(&g_sMaster1, I2C1_BASE, g_ui32SysClock);
  SMBusMasterInit(&g_sMaster2, I2C2_BASE, g_ui32SysClock);
  SMBusMasterInit(&g_sMaster3, I2C3_BASE, g_ui32SysClock);
  SMBusMasterInit(&g_sMaster4, I2C4_BASE, g_ui32SysClock);
  SMBusMasterInit(&g_sMaster6, I2C6_BASE, g_ui32SysClock);

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



// Monitoring using the ADC inputs
void ADCMonitorTask(void *parameters);

void ShortDelay()
{
  vTaskDelay(pdMS_TO_TICKS(25));
}


struct TaskNamePair_t {
  char key[configMAX_TASK_NAME_LEN];
  TaskHandle_t value;
} ;

static struct TaskNamePair_t TaskNamePairs[7];

void vGetTaskHandle( char *key, TaskHandle_t *t)
{
  *t = NULL;
  for (int i = 0; i < 6; ++i) {
    if ( strncmp(key, TaskNamePairs[i].key,3) == 0)
      *t = TaskNamePairs[i].value;
  }
  return ;
}

CommandLineArgs_t cli_uart1;
CommandLineArgs_t cli_uart4;



// 
int main( void )
{

  // mutex for the UART output
  xUARTMutex = xSemaphoreCreateMutex();

  //  Create the stream buffers that sends data from the interrupt to the
  //  task, and create the task.
  // There are two buffers for the two CLIs (front panel and Zynq)
  xUART4StreamBuffer = xStreamBufferCreate( 128, // length of stream buffer in bytes
                                           1);  // number of items before a trigger is sent
  xUART1StreamBuffer = xStreamBufferCreate( 128, // length of stream buffer in bytes
                                           1);  // number of items before a trigger is sent

  cli_uart1.uart_base = UART1_BASE; cli_uart1.UartStreamBuffer = xUART1StreamBuffer;
  cli_uart4.uart_base = UART4_BASE; cli_uart4.UartStreamBuffer = xUART4StreamBuffer;

  // start the tasks here 
  xTaskCreate(PowerSupplyTask, "POW", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY+5, &TaskNamePairs[0].value);
  xTaskCreate(LedTask,         "LED", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY+2, &TaskNamePairs[1].value);
  xTaskCreate(vCommandLineTask,"CL1", 512,                &cli_uart1, tskIDLE_PRIORITY+1, &TaskNamePairs[2].value);
  xTaskCreate(vCommandLineTask,"CL4", 512,                &cli_uart4, tskIDLE_PRIORITY+1, &TaskNamePairs[3].value);
  xTaskCreate(ADCMonitorTask,  "ADC", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY+4, &TaskNamePairs[4].value);
  xTaskCreate(MonitorTask,     "MON", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY+4, &TaskNamePairs[5].value);
  xTaskCreate(FireFlyTask,     "FLY", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY+4, &TaskNamePairs[6].value);

  snprintf(TaskNamePairs[0].key,configMAX_TASK_NAME_LEN,"POW");
  snprintf(TaskNamePairs[1].key,configMAX_TASK_NAME_LEN,"LED");
  snprintf(TaskNamePairs[2].key,configMAX_TASK_NAME_LEN,"CL1");
  snprintf(TaskNamePairs[3].key,configMAX_TASK_NAME_LEN,"CL4");
  snprintf(TaskNamePairs[4].key,configMAX_TASK_NAME_LEN,"ADC");
  snprintf(TaskNamePairs[5].key,configMAX_TASK_NAME_LEN,"MON");
  snprintf(TaskNamePairs[6].key,configMAX_TASK_NAME_LEN,"FLY");

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

  Print("\n\r----------------------------\n\r");
  Print("Staring Project2 " FIRMWARE_VERSION " (FreeRTOS scheduler about to start)\n\r");
  Print("Built at " __TIME__", " __DATE__ "\n\r");
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

