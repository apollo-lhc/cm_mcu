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

// Initialize the UART(s)
// It is hard to generalize these initialization functions as they use
// a bunch of #define's that are not iterable
// UART4 is the front panel
void UART4Init(uint32_t ui32SysClock)
{
  // Turn on the UART peripheral
  MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART4);

  //
  // Configure the UART for 115,200, 8-N-1 operation.
  //
  MAP_UARTConfigSetExpClk(UART4_BASE, ui32SysClock, 115200,
                          (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE));

  //
  // Enable the UART interrupt.
  //
#ifdef USE_FREERTOS
  MAP_IntPrioritySet(INT_UART4, configKERNEL_INTERRUPT_PRIORITY);
#endif // USE_FREERTOS
  MAP_IntEnable(INT_UART4);
  MAP_UARTIntEnable(UART4_BASE, UART_INT_RX | UART_INT_RT);

  return;
}

// UART1 is the Zynq
void UART1Init(uint32_t ui32SysClock)
{
  // Turn on the UART peripheral
  MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART1);

  //
  // Configure the UART for 115,200, 8-N-1 operation.
  //
  MAP_UARTConfigSetExpClk(UART1_BASE, ui32SysClock, 115200,
                          (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE));

  //
  // Enable the UART interrupt.
  //
#ifdef USE_FREERTOS
  MAP_IntPrioritySet(INT_UART1, configKERNEL_INTERRUPT_PRIORITY);
#endif // USE_FREERTOS
  MAP_IntEnable(INT_UART1);
  MAP_UARTIntEnable(UART1_BASE, UART_INT_RX | UART_INT_RT);

  return;
}

void UART0Init(uint32_t ui32SysClock)
{
  // Turn on the UART peripheral
  MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);

  //
  // Configure the UART for 115,200, 8-N-1 operation.
  //
  MAP_UARTConfigSetExpClk(UART0_BASE, ui32SysClock, 115200,
                          (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE));

  //
  // Enable the UART interrupt.
  //
#ifdef USE_FREERTOS
  MAP_IntPrioritySet(INT_UART0, configKERNEL_INTERRUPT_PRIORITY);
#endif // USE_FREERTOS
  MAP_IntEnable(INT_UART0);
  MAP_UARTIntEnable(UART0_BASE, UART_INT_RX | UART_INT_RT);

  return;
}



//*****************************************************************************
//
// Send a string to the UART4
//
//*****************************************************************************

void UART4Print(const char *str)
{
  size_t size = strlen(str);
  for (int i = 0; i < size; ++i) {
    //
    // Write the next character to the UART.
    //
    MAP_UARTCharPut(UART4_BASE, str[i]);
  }
}

void UARTPrint(uint32_t uart_base, const char *str)
{
  size_t size = strlen(str);
  for (int i = 0; i < size; ++i) {
    //
    // Write the next character to the UART.
    //
    MAP_UARTCharPut(uart_base, str[i]);
  }
}
