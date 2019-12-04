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
#include "MonitorTask.h"
#include "Tasks.h"
#include "EEPROMTask.c"

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

// Mutex for UART -- should really have one for each UART
static SemaphoreHandle_t xUARTMutex = NULL;

// Mutex for I2C controllers
//SemaphoreHandle_t xI2C1Mutex = NULL;
//SemaphoreHandle_t xI2C2Mutex = NULL;
//SemaphoreHandle_t xI2C3Mutex = NULL;
//SemaphoreHandle_t xI2C4Mutex = NULL;
//SemaphoreHandle_t xI2C6Mutex = NULL;


void Print(const char* str)
{
    xSemaphoreTake( xUARTMutex, portMAX_DELAY );
    {
      UARTPrint(CLI_UART, str);
    }
    xSemaphoreGive( xUARTMutex );
  return;
}




// These register locations are defined by the ARM Cortex-M4F
// specification and do not depend on the TM4C1290NCPDT
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
    counter += diff>>12; // degrade counter a bit-- don't need this precision
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


 }
// This set of functions partially contains calls that require
// the FreeRTOS to be further configured.
void SystemInitInterrupts()
{
  // Set up the UARTs for the CLI
  // this also sets up the interrupts
  UART1Init(g_ui32SysClock); // ZYNQ UART
  UART4Init(g_ui32SysClock); // front panel UART

  // initialize the ADCs.
  ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);
  ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC1);
  // initialize the EEPROM.
  ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_EEPROM0);
  ROM_EEPROMInit();
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






void ShortDelay()
{
  vTaskDelay(pdMS_TO_TICKS(25));
}


struct TaskNamePair_t {
  char key[configMAX_TASK_NAME_LEN];
  TaskHandle_t value;
} ;

#define MAX_TASK_COUNT 10
static struct TaskNamePair_t TaskNamePairs[MAX_TASK_COUNT];

void vGetTaskHandle( char *key, TaskHandle_t *t)
{
  *t = NULL;
  for (int i = 0; i < MAX_TASK_COUNT; ++i) {
    if ( strncmp(key, TaskNamePairs[i].key,3) == 0)
      *t = TaskNamePairs[i].value;
  }
  return ;
}

CommandLineTaskArgs_t cli_uart1;
CommandLineTaskArgs_t cli_uart4;

struct dev_i2c_addr_t fpga_addrs[] = {
    {"VU7P", 0x70, 1, 0x36},
    {"KU15P", 0x70, 0, 0x36},
};

struct dev_i2c_addr_t fpga_addrs_kuonly[] = {
    {"KU15P", 0x70, 0, 0x36},
};

struct dev_i2c_addr_t fpga_addrs_vuonly[] = {
    {"VU7P", 0x70, 1, 0x36},
};



struct pm_command_t pm_command_fpga[] = {
    { 0x8d, 2, "READ_TEMPERATURE_1", "C", PM_LINEAR11 },
};

float pm_fpga[2] = {0.0,0.0};

struct MonitorTaskArgs_t fpga_args = {
    .name = "XIMON",
    .devices = fpga_addrs,
    .n_devices = 2,
    .commands = pm_command_fpga,
    .n_commands = 1,
    .pm_values = pm_fpga,
    .n_values = 2,
    .n_pages = 1,
    .smbus = &g_sMaster6,
    .smbus_status = &eStatus6,
};

// Supply Address | Voltages | Priority
// ---------------+----------|-----------
//       0x40     | 3.3 & 1.8|     2
//       0x44     | KVCCINT  |     1
//       0x43     | KVCCINT  |     1
//       0x46     | VVCCINT  |     1
//       0x45     | VVCCINT  |     1
struct dev_i2c_addr_t pm_addrs_dcdc[] = {
    {"3V3/1V8", 0x70, 0, 0x40},
    {"KVCCINT1", 0x70, 1, 0x44},
    {"KVCCINT2", 0x70, 2, 0x43},
    {"VVCCINT1", 0x70, 3, 0x46},
    {"VVCCINT2", 0x70, 4, 0x45},
};


struct pm_command_t pm_command_dcdc[] = {
        { 0x8d, 2, "READ_TEMPERATURE_1", "C", PM_LINEAR11 },
        { 0x8f, 2, "READ_TEMPERATURE_3", "C", PM_LINEAR11 },
        { 0x88, 2, "READ_VIN", "V", PM_LINEAR11 },
        { 0x8B, 2, "READ_VOUT", "V", PM_LINEAR16U },
        { 0x8c, 2, "READ_IOUT", "A", PM_LINEAR11 },
        //{ 0x4F, 2, "OT_FAULT_LIMIT", "C", PM_LINEAR11},
        { 0x79, 2, "STATUS_WORD", "", PM_STATUS },
        //{ 0xE7, 2, "IOUT_AVG_OC_FAULT_LIMIT", "A", PM_LINEAR11 },
        //{ 0x95, 2, "READ_FREQUENCY", "Hz", PM_LINEAR11},
      };
float dcdc_values[NSUPPLIES_PS*NPAGES_PS*NCOMMANDS_PS];
struct MonitorTaskArgs_t dcdc_args = {
    .name = "PSMON",
    .devices = pm_addrs_dcdc,
    .n_devices = NSUPPLIES_PS,
    .commands = pm_command_dcdc,
    .n_commands = NCOMMANDS_PS,
    .pm_values = dcdc_values,
    .n_values = NSUPPLIES_PS*NPAGES_PS*NCOMMANDS_PS,
    .n_pages = NPAGES_PS,
    .smbus = &g_sMaster1,
    .smbus_status = &eStatus1,
};

const char* buildTime()
{
  const char *btime = __TIME__ ", " __DATE__;
  return btime;
}

const char* gitVersion()
{
  const char *gitVersion = FIRMWARE_VERSION;
  return gitVersion;
}

// 
int main( void )
{
  SystemInit();
  // check if we are to include both FPGAs or not
  bool ku_enable = (read_gpio_pin(TM4C_DIP_SW_1) == 1);
  bool vu_enable = (read_gpio_pin(TM4C_DIP_SW_2) == 1);
  if ( ! ku_enable ) {
    fpga_args.devices = fpga_addrs_vuonly;
    fpga_args.n_devices = 1;
  }
  else if ( ! vu_enable ) {
    fpga_args.devices = fpga_addrs_kuonly;
    fpga_args.n_devices = 1;
  }

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

  // clear the various buffers
  for (int i =0; i < dcdc_args.n_values; ++i)
    dcdc_args.pm_values[i] = -999.;
  for (int i =0; i < fpga_args.n_values; ++i)
    fpga_args.pm_values[i] = -999.;



  // start the tasks here 
  xTaskCreate(PowerSupplyTask, "POW", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY+5, &TaskNamePairs[0].value);
  xTaskCreate(LedTask,         "LED", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY+2, &TaskNamePairs[1].value);
  xTaskCreate(vCommandLineTask,"CLIZY", 512,                &cli_uart1, tskIDLE_PRIORITY+1, &TaskNamePairs[2].value);
  xTaskCreate(vCommandLineTask,"CLIFP", 512,                &cli_uart4, tskIDLE_PRIORITY+1, &TaskNamePairs[3].value);
  xTaskCreate(ADCMonitorTask,  "ADC", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY+4, &TaskNamePairs[4].value);
  xTaskCreate(FireFlyTask,    "FFLY", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY+4, &TaskNamePairs[6].value);
  xTaskCreate(MonitorTask,   "PSMON", configMINIMAL_STACK_SIZE, &dcdc_args, tskIDLE_PRIORITY+4, &TaskNamePairs[5].value);
  xTaskCreate(MonitorTask,   "XIMON", configMINIMAL_STACK_SIZE, &fpga_args, tskIDLE_PRIORITY+4, &TaskNamePairs[7].value);
  xTaskCreate(AlarmTask,     "ALARM", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY+5, &TaskNamePairs[8].value);
  xTaskCreate(EEPROMTask,    "EPRM", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY+5, &TaskNamePairs[9].value);
    // TODO: Check that I added EEPROMTask correctly (priority, etc)

  snprintf(TaskNamePairs[0].key,configMAX_TASK_NAME_LEN,"POW");
  snprintf(TaskNamePairs[1].key,configMAX_TASK_NAME_LEN,"LED");
  snprintf(TaskNamePairs[2].key,configMAX_TASK_NAME_LEN,"CLIZY");
  snprintf(TaskNamePairs[3].key,configMAX_TASK_NAME_LEN,"CLIFP");
  snprintf(TaskNamePairs[4].key,configMAX_TASK_NAME_LEN,"ADC");
  snprintf(TaskNamePairs[5].key,configMAX_TASK_NAME_LEN,"PSMON");
  snprintf(TaskNamePairs[6].key,configMAX_TASK_NAME_LEN,"FFLY");
  snprintf(TaskNamePairs[7].key,configMAX_TASK_NAME_LEN,"XIMON");
  snprintf(TaskNamePairs[8].key,configMAX_TASK_NAME_LEN,"ALARM");
  snprintf(TaskNamePairs[9].key,configMAX_TASK_NAME_LEN,"EPRM");

  // -------------------------------------------------
  // Initialize all the queues
  // queue for the LED
  xLedQueue = xQueueCreate(5, // The maximum number of items the queue can hold.
      sizeof( uint32_t ));    // The size of each item.
  configASSERT(xLedQueue != NULL);

  xPwrQueue = xQueueCreate(10, sizeof(uint32_t)); // PWR queue
  configASSERT(xPwrQueue != NULL);

  xFFlyQueue = xQueueCreate(10, sizeof(uint32_t)); // PWR queue
  configASSERT(xFFlyQueue != NULL);

  xEPRMQueue = xQueueCreate(10, sizeof(uint64_t)); // EPRM queue
  configASSERT(xEPRMQueue != NULL);

  xAlmQueue = xQueueCreate(10, sizeof(uint32_t)); // ALARM queue
  configASSERT(xAlmQueue != NULL);

#ifdef DEBUGxx
  vQueueAddToRegistry(xLedQueue, "LedQueue");
  vQueueAddToRegistry(xPwrQueue, "PwrQueue");
#endif // DEBUG

  // Set up the hardware ready to run the firmware. Don't do this earlier as
  // the interrupts call some FreeRTOS tasks that need to be set up first.
  SystemInitInterrupts();

  // Say hello. The information below is only updated when the main()
  // function is recompiled.
  Print("\r\n----------------------------\r\n");
  Print("Staring Apollo CM MCU firmware " FIRMWARE_VERSION
        "\r\n\t\t (FreeRTOS scheduler about to start)\r\n");
  Print("Built on " __TIME__", " __DATE__ "\r\n");
  Print(  "----------------------------\r\n");
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
  char tmp[256];
  snprintf(tmp, 256, "Stack overflow: task %s\r\n", pcTaskName);
  Print(tmp);

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

