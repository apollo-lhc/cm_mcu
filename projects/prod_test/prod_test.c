//*****************************************************************************
// production testing firmware
//*****************************************************************************

#include <stdint.h>
#include <stdbool.h>

// TI includes
#include "driverlib/sysctl.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "inc/hw_ints.h"

// local includes
#include "common/pinout.h"
#include "prod_test.h"
#include "FreeRTOSConfig.h"
#include "common/i2c_reg.h"
#include "common/LocalUart.h"
#include "common/smbus.h"
#include "common/utils.h"

// FreeRTOS includes
#include "FreeRTOS.h" // IWYU pragma: keep
#include "task.h"
#include "stream_buffer.h"
#include "semphr.h"
#include "portmacro.h"
#include "InterruptHandlers.h"

SemaphoreHandle_t xUARTMutex = 0;
void vCommandLineTask(void *pvParameters);

//*****************************************************************************
//
// The error routine that is called if the driver library encounters an error.
//
//*****************************************************************************
#ifdef DEBUG
void __error__(char *pcFilename, uint32_t ui32Line)
{
  while (1)
    ;
}
#endif

uint32_t g_ui32SysClock;

void SystemInit(void)
{
  //
  // Run from the PLL, internal oscillator, at the defined clock speed configCPU_CLOCK_HZ
  // This function does not exist in the ROM
  g_ui32SysClock = MAP_SysCtlClockFreqSet((SYSCTL_OSC_INT | SYSCTL_USE_PLL | SYSCTL_CFG_VCO_320),
                                          configCPU_CLOCK_HZ);

  // initialize all pins, using file setup by TI PINMUX tool
  PinoutSet();

  // initialize interrupts
  UART0Init(g_ui32SysClock); // ZYNQ UART
  initI2C1(g_ui32SysClock);  // controller for power supplies
  SMBusMasterInit(&g_sMaster1, I2C1_BASE, g_ui32SysClock);
  ROM_IntPrioritySet(INT_I2C1, configKERNEL_INTERRUPT_PRIORITY);
  SMBusMasterIntEnable(&g_sMaster1);

  setupActiveLowPins();

  return;
}

void Print(const char *str)
{
  xSemaphoreTake(xUARTMutex, portMAX_DELAY);
  {
    UARTPrint(ZQ_UART, str);
  }
  xSemaphoreGive(xUARTMutex);
  return;
}

// Command line interface

typedef struct {
  StreamBufferHandle_t UartStreamBuffer;
  uint32_t uart_base;
  UBaseType_t stack_size;
} CommandLineTaskArgs_t;

CommandLineTaskArgs_t cli_uart;

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
  const char *gitVersion = BUILD_TYPE "\r\n" FIRMWARE_VERSION;
  return gitVersion;
}

//*****************************************************************************
//
// main entry point
//
//*****************************************************************************
__attribute__((noreturn)) int main(void)
{
  SystemInit();

  // There is one buffer for the CLI (shared front panel and Zynq)
  xUART0StreamBuffer = xStreamBufferCreate(128, // length of stream buffer in bytes
                                           1);  // number of items before a trigger is sent

  cli_uart.uart_base = ZQ_UART;
  cli_uart.UartStreamBuffer = xUART0StreamBuffer;
  cli_uart.stack_size = 4096U;
  xUARTMutex = xSemaphoreCreateMutex();

  xTaskCreate(vCommandLineTask, "CLIZY", 512, &cli_uart, tskIDLE_PRIORITY + 4, NULL);

  Print("\r\n----------------------------\r\n");
  Print("Staring Apollo CM MCU Production Test firmware ");
  Print(gitVersion());
  Print("\r\n");

  // start the scheduler -- this function should not return
  vTaskStartScheduler();
  // should never get here
  Print("Scheduler start failed\r\n");
  configASSERT(1 == 0); // capture in eeprom
  __builtin_unreachable();
}
