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
#include <stddef.h>

#include <string.h>

// local includes
#include "FreeRTOSConfig.h"
#include "common/LocalUart.h"
#include "common/utils.h"
#include "common/power_ctl.h"
#include "common/i2c_reg.h"
#include "common/pinout.h"
#include "common/pinsel.h"
#include "common/smbus.h"
#include "common/log.h"
#include "CommandLineTask.h"
#include "InterruptHandlers.h"
#include "MonitorI2CTask.h"
#include "MonitorTask.h"
#include "Tasks.h"
#include "I2CSlaveTask.h"
#include "AlarmUtilities.h"

// TI Includes
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "inc/hw_types.h"
#include "inc/hw_memmap.h"
#include "inc/hw_ints.h"
#include "driverlib/pin_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/i2c.h"
#include "driverlib/adc.h"
#include "driverlib/uart.h"
#include "driverlib/interrupt.h"

// FreeRTOS includes
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "stream_buffer.h"
#include "semphr.h"
#include "portmacro.h"
#include "Semaphore.h"

#define I2C0_SLAVE_ADDRESS 0x40

uint32_t g_ui32SysClock = 0;

void Print(const char *str)
{
  xSemaphoreTake(xUARTMutex, portMAX_DELAY);
  {
#ifdef REV1
    UARTPrint(FP_UART, str);
#endif // REV1
    UARTPrint(ZQ_UART, str);
  }
  xSemaphoreGive(xUARTMutex);
  return;
}

//*****************************************************************************
//
// The error routine that is called if the driver library encounters an error.
//
//*****************************************************************************
#ifdef DEBUG
void __error__(char *pcFilename, uint32_t ui32Line)
{
  char errstr[64];
  snprintf(errstr, 64, "driverlib error in %s:%u\r\n", pcFilename, (unsigned)ui32Line);
  Print(errstr);
}
#endif

void SystemInit(void)
{
  //
  // Run from the PLL, internal oscillator, at the defined clock speed configCPU_CLOCK_HZ
  // This function does not exist in the ROM
  g_ui32SysClock = MAP_SysCtlClockFreqSet((SYSCTL_OSC_INT | SYSCTL_USE_PLL | SYSCTL_CFG_VCO_320),
                                          configCPU_CLOCK_HZ);

  // initialize all pins, using file setup by TI PINMUX tool
  PinoutSet();
}
// This set of functions partially contains calls that require
// the FreeRTOS to be further configured.
void SystemInitInterrupts(void)
{
  // Set up the UARTs for the CLI
  // this also sets up the interrupts
#if defined(REV1)
  UART1Init(g_ui32SysClock); // ZYNQ UART
#elif defined(REV2)
  UART0Init(g_ui32SysClock); // ZYNQ UART
#endif
  UART4Init(g_ui32SysClock); // front panel UART in Rev1 and Zynq comms in Rev2

  // initialize the ADCs.
  ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);
  ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC1);
  // initialize the EEPROM.
  ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_EEPROM0);
  ROM_EEPROMInit();
  // Set the reference to external
  ROM_ADCReferenceSet(ADC0_BASE, ADC_REF_EXT_3V);
  ROM_ADCReferenceSet(ADC1_BASE, ADC_REF_EXT_3V);
  ROM_ADCIntEnable(ADC0_BASE, 1); // enable interrupt for sequence 1
  ROM_ADCIntEnable(ADC1_BASE, 0); // enable interrupt for sequence 0
  // clear the interrupts
  ROM_ADCIntClear(ADC0_BASE, 1);
  ROM_ADCIntClear(ADC1_BASE, 0);
  // FreeRTOS insists that the priority of interrupts be set up like this.
  ROM_IntPrioritySet(INT_ADC0SS1, configKERNEL_INTERRUPT_PRIORITY);
  ROM_IntPrioritySet(INT_ADC1SS0, configKERNEL_INTERRUPT_PRIORITY);

  ROM_IntEnable(INT_ADC0SS1);
  ROM_IntEnable(INT_ADC1SS0);

#if defined(REV1) || defined(REV2)
  // Set up the I2C controllers
  initI2C0(g_ui32SysClock); // Slave controller
  initI2C1(g_ui32SysClock); // controller for power supplies
  initI2C2(g_ui32SysClock); // controller for clocks
  initI2C3(g_ui32SysClock); // controller for V (F2) optics
  initI2C4(g_ui32SysClock); // controller for K (F1) optics
#endif
#if defined(REV1)
  initI2C6(g_ui32SysClock); // controller for FPGAs
#elif defined(REV2)
  initI2C5(g_ui32SysClock);  // controller for FPGAs
#endif

  // smbus
  // Initialize the master SMBus port.
  //
  SMBusMasterInit(&g_sMaster1, I2C1_BASE, g_ui32SysClock);
  SMBusMasterInit(&g_sMaster2, I2C2_BASE, g_ui32SysClock);
  SMBusMasterInit(&g_sMaster3, I2C3_BASE, g_ui32SysClock);
  SMBusMasterInit(&g_sMaster4, I2C4_BASE, g_ui32SysClock);
#if defined(REV1)
  SMBusMasterInit(&g_sMaster6, I2C6_BASE, g_ui32SysClock);
#elif defined(REV2)
  SMBusMasterInit(&g_sMaster5, I2C5_BASE, g_ui32SysClock);
#endif
  // FreeRTOS insists that the priority of interrupts be set up like this.
  ROM_IntPrioritySet(INT_I2C0, configKERNEL_INTERRUPT_PRIORITY);
  ROM_IntPrioritySet(INT_I2C1, configKERNEL_INTERRUPT_PRIORITY);
  ROM_IntPrioritySet(INT_I2C2, configKERNEL_INTERRUPT_PRIORITY);
  ROM_IntPrioritySet(INT_I2C3, configKERNEL_INTERRUPT_PRIORITY);
  ROM_IntPrioritySet(INT_I2C4, configKERNEL_INTERRUPT_PRIORITY);
#if defined(REV1)
  ROM_IntPrioritySet(INT_I2C6, configKERNEL_INTERRUPT_PRIORITY);
#elif defined(REV2)
  ROM_IntPrioritySet(INT_I2C5, configKERNEL_INTERRUPT_PRIORITY);
#endif
  //
  // Enable I2C master interrupts.
  //
  SMBusMasterIntEnable(&g_sMaster1);
  SMBusMasterIntEnable(&g_sMaster2);
  SMBusMasterIntEnable(&g_sMaster3);
  SMBusMasterIntEnable(&g_sMaster4);
#if defined(REV1)
  SMBusMasterIntEnable(&g_sMaster6);
#elif defined(REV2)
  SMBusMasterIntEnable(&g_sMaster5);
#endif

  // I2C slave
  ROM_I2CSlaveAddressSet(I2C0_BASE, 0, I2C0_SLAVE_ADDRESS);

  ROM_IntPrioritySet(INT_I2C0, configKERNEL_INTERRUPT_PRIORITY);

  // ignore I2C_SLAVE_INT_START, I2C_SLAVE_INT_STOP
  ROM_I2CSlaveIntEnableEx(I2C0_BASE, I2C_SLAVE_INT_DATA);
  ROM_IntEnable(INT_I2C0);

  //
  // Enable processor interrupts.
  //
  // MAP_IntMasterEnable(); the FreeRTOS kernel does this at the appropriate moment

  setupActiveLowPins();

#ifdef REV2
  // RTC in the Hibernation module
  InitRTC();
#endif // REV2

  // SYSTICK timer -- this is already enabled in the portable layer
  return;
}

volatile uint32_t g_ui32SysTickCount;

CommandLineTaskArgs_t cli_uart;
#ifdef REV1
CommandLineTaskArgs_t cli_uart4;
#endif // REV1

void ShortDelay(void)
{
  vTaskDelay(pdMS_TO_TICKS(25));
}

const char *buildTime(void)
{
  const char *btime = __TIME__ ", " __DATE__;
  return btime;
}

const char *gitVersion(void)
{
#ifdef DEBUG
#define BUILD_TYPE "DEBUG build"
#else
#define BUILD_TYPE "regular build"
#endif
  const char *gitVersion = FIRMWARE_VERSION "\r\n" BUILD_TYPE "\r\n";
  return gitVersion;
}

//
int main(void)
{
  SystemInit();

  initFPGAMon();

  // all facilities start at INFO
  for (enum log_facility_t i = 0; i < NUM_LOG_FACILITIES; ++i) {
    log_set_level(LOG_INFO, i);
  }
  log_set_level(LOG_ERROR, LOG_MON); // for now

  // Initialize all semaphores
  initSemaphores();
  dcdc_args.xSem = i2c1_sem;
  ffl12_f1_args.xSem = i2c4_sem;
  ffldaq_f1_args.xSem = i2c4_sem;
  ffl12_f2_args.xSem = i2c3_sem;
  ffldaq_f2_args.xSem = i2c3_sem;
  clock_args.xSem = i2c2_sem;
  clockr0a_args.xSem = i2c2_sem;
  //  Create the stream buffers that sends data from the interrupt to the
  //  task, and create the task.
#ifdef REV1
  // There are two buffers for the two CLIs (front panel and Zynq)
  xUART4StreamBuffer = xStreamBufferCreate(128, // length of stream buffer in bytes
                                           1);  // number of items before a trigger is sent
  cli_uart4.uart_base = FP_UART;
  cli_uart4.UartStreamBuffer = xUART4StreamBuffer;
  cli_uart4.stack_size = 4096U;
  xUART1StreamBuffer = xStreamBufferCreate(128, // length of stream buffer in bytes
                                           1);  // number of items before a trigger is sent
  cli_uart.uart_base = ZQ_UART;
  cli_uart.UartStreamBuffer = xUART1StreamBuffer;
#elif defined(REV2)
  // There is one buffer for the CLI (shared front panel and Zynq)
  xUART0StreamBuffer = xStreamBufferCreate(128, // length of stream buffer in bytes
                                           1);  // number of items before a trigger is sent

  cli_uart.uart_base = ZQ_UART;
  cli_uart.UartStreamBuffer = xUART0StreamBuffer;
  cli_uart.stack_size = 4096U;
#endif // REV1

  // clear the various buffers
  for (int i = 0; i < dcdc_args.n_values; ++i)
    dcdc_args.pm_values[i] = -999.f;
  for (int i = 0; i < fpga_args.n_values; ++i)
    fpga_args.pm_values[i] = -999.f;

  // start the tasks here
  xTaskCreate(PowerSupplyTask, "POW", 2 * configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 5, NULL);
  xTaskCreate(LedTask, "LED", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 2, NULL);
  xTaskCreate(vCommandLineTask, "CLIZY", 512, &cli_uart, tskIDLE_PRIORITY + 4, NULL);
#ifdef REV1
  xTaskCreate(vCommandLineTask, "CLIFP", 512, &cli_uart4, tskIDLE_PRIORITY + 4, NULL);
#endif // REV1
  xTaskCreate(ADCMonitorTask, "ADC", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 4, NULL);

  xTaskCreate(MonitorI2CTask, "FF12", 2 * configMINIMAL_STACK_SIZE, &ffl12_f1_args, tskIDLE_PRIORITY + 4,
              NULL);
  xTaskCreate(MonitorI2CTask, "FFDAQ", 2 * configMINIMAL_STACK_SIZE, &ffldaq_f1_args, tskIDLE_PRIORITY + 4,
              NULL);
  xTaskCreate(MonitorI2CTask, "FF12V", 2 * configMINIMAL_STACK_SIZE, &ffl12_f2_args, tskIDLE_PRIORITY + 4,
              NULL);
  xTaskCreate(MonitorI2CTask, "FFDAV", 2 * configMINIMAL_STACK_SIZE, &ffldaq_f2_args, tskIDLE_PRIORITY + 4,
              NULL);

#ifdef REV2
  xTaskCreate(MonitorI2CTask, "CLKSI", 2 * configMINIMAL_STACK_SIZE, &clock_args, tskIDLE_PRIORITY + 4,
              NULL);
  xTaskCreate(MonitorI2CTask, "CLKR0", 2 * configMINIMAL_STACK_SIZE, &clockr0a_args, tskIDLE_PRIORITY + 4,
              NULL);
#endif // REV2
  xTaskCreate(MonitorTask, "PSMON", 2 * configMINIMAL_STACK_SIZE, &dcdc_args, tskIDLE_PRIORITY + 4,
              NULL);
  xTaskCreate(MonitorTask, "XIMON", 2 * configMINIMAL_STACK_SIZE, &fpga_args, tskIDLE_PRIORITY + 4,
              NULL);
  xTaskCreate(I2CSlaveTask, "I2CS0", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 5, NULL);
  xTaskCreate(EEPROMTask, "EPRM", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 4, NULL);
  xTaskCreate(InitTask, "INIT", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 5, NULL);
  xTaskCreate(ZynqMonTask, "ZMON", 2 * configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 5, NULL);
  xTaskCreate(GenericAlarmTask, "TALM", configMINIMAL_STACK_SIZE, &tempAlarmTask,
              tskIDLE_PRIORITY + 5, NULL);
  xTaskCreate(GenericAlarmTask, "VALM", configMINIMAL_STACK_SIZE, &voltAlarmTask,
              tskIDLE_PRIORITY + 5, NULL);
  //  xTaskCreate(WatchdogTask, "WATCH", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY, NULL);

  // -------------------------------------------------
  // Initialize all the queues
  // queue for the LED

  xLedQueue = xQueueCreate(3,                 // The maximum number of items the queue can hold.
                           sizeof(uint32_t)); // The size of each item.
  configASSERT(xLedQueue != NULL);

  xPwrQueue = xQueueCreate(10, sizeof(uint32_t)); // PWR queue
  configASSERT(xPwrQueue != NULL);

  xEPRMQueue_in = xQueueCreate(5, sizeof(uint64_t)); // EPRM queues
  configASSERT(xEPRMQueue_in != NULL);
  xEPRMQueue_out = xQueueCreate(5, sizeof(uint64_t));
  configASSERT(xEPRMQueue_out != NULL);

  xALMQueue = xQueueCreate(10, sizeof(uint32_t)); // ALARM queue
  configASSERT(xALMQueue != NULL);

  xZynqMonQueue = xQueueCreate(10, sizeof(uint32_t)); // Soft UART queue
  configASSERT(xZynqMonQueue != NULL);

  // Set up the hardware ready to run the firmware. Don't do this earlier as
  // the interrupts call some FreeRTOS tasks that need to be set up first.
  SystemInitInterrupts();

  // Say hello. The information below is only updated when the main()
  // function is recompiled.
  Print("\r\n----------------------------\r\n");
  Print("Staring Apollo CM MCU firmware ");
  Print(gitVersion());
#ifdef REV1
  Print("\r\nRev1 build\r\n");
#elif defined(REV2)
  Print("\r\nRev2 build\r\n");
#endif
  Print("\t\t (FreeRTOS scheduler about to start)\r\n");
  Print("Built on " __TIME__ ", " __DATE__ "\r\n");
#ifdef ECN001
  Print("Includes ECN001 code mods\r\n");
#endif // ECN001
  Print("----------------------------\r\n");

  errbuffer_init(EBUF_MINBLK, EBUF_MAXBLK);

  // start the scheduler -- this function should not return
  vTaskStartScheduler();
  // should never get here
  Print("Scheduler start failed\r\n");
  configASSERT(1 == 0);
  __builtin_unreachable();
  return 0;
}

uintptr_t __stack_chk_guard = 0xdeadbeef;

void __stack_chk_fail(void)
{
  // fall back to lower-level routine since if we get here things are broken
  UARTPrint(ZQ_UART, "Stack smashing detected\r\n");
  while (MAP_UARTBusy(ZQ_UART))
    ;
  configASSERT(1 == 0);
}

int SystemStackWaterHighWaterMark(void)
{
  int i;
  const uint32_t *stack = getSystemStack();
  // we need to disable interrupts since we are looking at the system stack
  taskENTER_CRITICAL();
  for (i = 0; i < SYSTEM_STACK_SIZE; ++i) {
    if (stack[i] != 0xdeadbeefUL)
      break;
  }
  taskEXIT_CRITICAL();
  return i;
}

/*-----------------------------------------------------------*/
#if (configCHECK_FOR_STACK_OVERFLOW != 0)
void vApplicationStackOverflowHook(TaskHandle_t pxTask, char *pcTaskName)
{
  /* If configCHECK_FOR_STACK_OVERFLOW is set to either 1 or 2 then this
     function will automatically get called if a task overflows its stack. */
  taskDISABLE_INTERRUPTS();
  char tmp[256];
  snprintf(tmp, 256, "Stack overflow: task %s\r\n", pcTaskName);
  UARTPrint(ZQ_UART, tmp); // can't use Print() here -- this gets called
  // from an ISR-like context.
  while (MAP_UARTBusy(ZQ_UART))
    ;
  // log the error
  errbuffer_put_raw(EBUF_STACKOVERFLOW, 0);
#ifdef DEBUG
  // wait here for the debugger
  for (;;)
    ;
#else  // DEBUG
  ROM_SysCtlReset();
#endif // DEBUG
}
#endif
/*-----------------------------------------------------------*/

#if (configUSE_IDLE_HOOK == 1)
void vApplicationIdleHook(void)
{
  // monitor the state of the system stack (MSP)
  static int HW = 999;
  int nHW = SystemStackWaterHighWaterMark();
  if (nHW < HW) {
    log_info(LOG_SERVICE, "Stack canary now %d\r\n", nHW);
    HW = nHW;
#ifdef DUMP_STACK
    const uint32_t *p = getSystemStack();
    for (int i = 0; i < SYSTEM_STACK_SIZE; ++i) {
      if (i % 4 == 0) {
        snprintf(tmp, 96, "\r\n%x: ", p);
        Print(tmp);
      }
      snprintf(tmp, 96, "%08x\t", *p++);
      Print(tmp);
    }
    Print("\r\n");
#endif // DUMP_STACK
  }
}
#endif

#if (configUSE_MALLOC_FAILED_HOOK == 1)
void vApplicationMallocFailedHook(void)
{
  UARTPrint(ZQ_UART, "vApplicationMallocFailedHook\r\n");
  while (MAP_UARTBusy(ZQ_UART))
    ;

  for (;;)
    ;
}
#endif
