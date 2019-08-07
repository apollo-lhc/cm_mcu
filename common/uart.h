/*
 * uart.h
 *
 *  Created on: May 9, 2019
 *      Author: wittich
 */

#ifndef COMMON_UART_H_
#define COMMON_UART_H_

#include <stdint.h>
#include <stdbool.h>


// Initialize UART4 and 1. Assumes pin definitions were already run.
void UART1Init(uint32_t ui32SysClock);
void UART4Init(uint32_t ui32SysClock);


//*****************************************************************************
//
// Send a string to the UART4
//
//*****************************************************************************

void UART4Print(const char* str);

void UARTPrint(uint32_t uart_base, const char* str);


#endif /* COMMON_UART_H_ */
