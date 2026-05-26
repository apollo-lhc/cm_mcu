/*
 * uart.c
 *
 *  Created on: May 9, 2019
 *      Author: wittich
 */
#include <string.h>

#include "common/LocalUart.h"

#include "inc/hw_memmap.h"
#include "inc/hw_ints.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include "driverlib/interrupt.h"

#ifdef USE_FREERTOS
#include "FreeRTOS.h"
#endif // USE_FREERTOS

static void UARTInitCommon(uint32_t periph, uint32_t base, uint32_t intNum,
                            uint32_t sysClock, bool enableRxInts)
{
  MAP_SysCtlPeripheralEnable(periph);
  MAP_UARTConfigSetExpClk(base, sysClock, 115200,
                          UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE);
  if (enableRxInts) {
#ifdef USE_FREERTOS
    MAP_IntPrioritySet(intNum, configKERNEL_INTERRUPT_PRIORITY);
#endif // USE_FREERTOS
    MAP_IntEnable(intNum);
    MAP_UARTIntEnable(base, UART_INT_RX | UART_INT_RT);
  }
}

void UART0Init(uint32_t ui32SysClock)
{
  UARTInitCommon(SYSCTL_PERIPH_UART0, UART0_BASE, INT_UART0, ui32SysClock, true);
}

void UART1Init(uint32_t ui32SysClock)
{
  UARTInitCommon(SYSCTL_PERIPH_UART1, UART1_BASE, INT_UART1, ui32SysClock, true);
}

// UART4 is the front panel in REV1, TX-only ZynqMon output in REV2/3.
// No RX interrupt handler is installed in REV2/3, so RX interrupts must not
// be enabled there or any received character would call IntDefaultHandler.
void UART4Init(uint32_t ui32SysClock)
{
#ifdef REV1
  UARTInitCommon(SYSCTL_PERIPH_UART4, UART4_BASE, INT_UART4, ui32SysClock, true);
#else
  UARTInitCommon(SYSCTL_PERIPH_UART4, UART4_BASE, INT_UART4, ui32SysClock, false);
#endif
}

void UART7Init(uint32_t ui32SysClock)
{
  UARTInitCommon(SYSCTL_PERIPH_UART7, UART7_BASE, INT_UART7, ui32SysClock, true);
}


//*****************************************************************************
//
// Send a string to the UART4
//
//*****************************************************************************

void UARTPrint(uint32_t uart_base, const char *str)
{
  size_t size = strlen(str);
  for (int i = 0; i < size; ++i) {
    MAP_UARTCharPut(uart_base, str[i]);
  }
}

void UART4Print(const char *str)
{
  UARTPrint(UART4_BASE, str);
}
