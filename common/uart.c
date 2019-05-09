/*
 * uart.c
 *
 *  Created on: May 9, 2019
 *      Author: wittich
 */
#include <string.h>

#include "common/uart.h"

#include "inc/hw_memmap.h"
#include "inc/hw_ints.h"
#include "driverlib/rom_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include "driverlib/interrupt.h"


// Initialize the UART
// based on uart_echo demo project
// we use UART4 (front panel)
void
UART4Init(uint32_t ui32SysClock)
{
  // Turn on the UART peripheral
  MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART4);


  //
  // Configure the UART for 115,200, 8-N-1 operation.
  //
  MAP_UARTConfigSetExpClk(UART4_BASE, ui32SysClock, 115200,
                         (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |
                          UART_CONFIG_PAR_NONE));

  //
  // Enable the UART interrupt.
  //
  MAP_IntEnable(INT_UART4);
  MAP_UARTIntEnable(UART4_BASE, UART_INT_RX | UART_INT_RT);

  return;
}



//*****************************************************************************
//
// Send a string to the UART4
//
//*****************************************************************************

void UART4Print(const char* str)
{
	int size = strlen(str);
	for ( int i = 0; i < size; ++i ) {
		//
		// Write the next character to the UART.
		//
		//MAP_UARTCharPutNonBlocking(UART4_BASE, str[i]);
		MAP_UARTCharPut(UART4_BASE, str[i]);

	}

}


void UARTPrint(uint32_t uart_base, const char* str)
{
	int size = strlen(str);
	for ( int i = 0; i < size; ++i ) {
		//
		// Write the next character to the UART.
		//
		//MAP_UARTCharPutNonBlocking(UART4_BASE, str[i]);
		MAP_UARTCharPut(uart_base, str[i]);

	}

}
