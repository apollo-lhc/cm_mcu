//*****************************************************************************
//
// uart_echo.c - Example for reading data from and writing data to the UART in
//               an interrupt driven fashion.
//
// Copyright (c) 2013-2017 Texas Instruments Incorporated.  All rights reserved.
// Software License Agreement
// 
// Texas Instruments (TI) is supplying this software for use solely and
// exclusively on TI's microcontroller products. The software is owned by
// TI and/or its suppliers, and is protected under applicable copyright
// laws. You may not combine this software with "viral" open-source
// software in order to form a larger program.
// 
// THIS SOFTWARE IS PROVIDED "AS IS" AND WITH ALL FAULTS.
// NO WARRANTIES, WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT
// NOT LIMITED TO, IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE. TI SHALL NOT, UNDER ANY
// CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL, OR CONSEQUENTIAL
// DAMAGES, FOR ANY REASON WHATSOEVER.
// 
// This is part of revision 2.1.4.178 of the EK-TM4C1294XL Firmware Package.
//
//*****************************************************************************

#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "driverlib/debug.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"

#ifdef REV2
#define UART_FRONTPANEL      UART0_BASE
#define UARTx_BASE           UART0_BASE
#define LED_PERIPH           SYSCTL_PERIPH_GPIOP
#define LED_BASE             GPIO_PORTP_BASE
#define LED_PIN              GPIO_PIN_3
#else
#define LED_PERIPH SYSCTL_PERIPH_GPIOJ
#define LED_BASE GPIO_PORTJ_BASE
#define LED_PIN GPIO_PIN_1

#define UART_FRONTPANEL      UART4_BASE
#define UARTx_BASE           UART4_BASE
#endif

//*****************************************************************************
//
//! \addtogroup example_list
//! <h1>UART Echo (uart_echo)</h1>
//!
//! This example application utilizes the UART to echo text.  The first UART
//! (connected to the USB debug virtual serial port on the evaluation board)
//! will be configured in 115,200 baud, 8-n-1 mode.  All characters received on
//! the UART are transmitted back to the UART.
//
//*****************************************************************************

//****************************************************************************
//
// System clock rate in Hz.
//
//****************************************************************************
uint32_t g_ui32SysClock;

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

//*****************************************************************************
//
// The UART interrupt handler.
//
//*****************************************************************************
void
UARTIntHandler(void)
{
  uint32_t ui32Status;

  //
  // Get the interrupt status.
  //
  ui32Status = ROM_UARTIntStatus(UART_FRONTPANEL, true);

  //
  // Clear the asserted interrupts.
  //
  ROM_UARTIntClear(UART_FRONTPANEL, ui32Status);

  //
  // Loop while there are characters in the receive FIFO.
  //
  while(ROM_UARTCharsAvail(UART_FRONTPANEL))
    {
      //
      // Read the next character from the UART and write it back to the UART.
      //
      ROM_UARTCharPutNonBlocking(UART_FRONTPANEL, ROM_UARTCharGetNonBlocking(UART_FRONTPANEL));

      //
      // Blink the LED to show a character transfer is occurring.
      //
      ROM_GPIOPinWrite(LED_BASE, LED_PIN, LED_PIN);

      //
      // Delay for 1 millisecond.  Each SysCtlDelay is about 3 clocks.
      //
      SysCtlDelay(g_ui32SysClock / (1000 * 3));

      //
      // Turn off the LED
      //
      ROM_GPIOPinWrite(LED_BASE, LED_PIN, 0x0);
      
    }
}

//*****************************************************************************
//
// Send a string to the UART.
//
//*****************************************************************************
void
UARTSend(const uint8_t *pui8Buffer, uint32_t ui32Count)
{
  //
  // Loop while there are more characters to send.
  //
  while(ui32Count--)
    {
      //
      // Write the next character to the UART.
      //
      ROM_UARTCharPutNonBlocking(UART_FRONTPANEL, *pui8Buffer++);
    }
}

//*****************************************************************************
//
// This example demonstrates how to send a string of data to the UART.
//
//*****************************************************************************
int
main(void)
{
  //
  // Set the clocking to run directly from the crystal at 120MHz.
  //
  g_ui32SysClock =
      SysCtlClockFreqSet((SYSCTL_OSC_INT | SYSCTL_USE_PLL | SYSCTL_CFG_VCO_480), 120000000);
  //
  // Enable the GPIO port that is used for the on-board LED.
  //
  ROM_SysCtlPeripheralEnable(LED_PERIPH);

  //
  // Enable the GPIO pins for the LED (PJ1).
  //
  ROM_GPIOPinTypeGPIOOutput(LED_BASE, LED_PIN);

  //
  // Enable the peripherals used by this example.
  //
#if UARTx_BASE == UART4_BASE
  ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

  //
  // Configure the GPIO Pin Mux for PA2
  // for U4RX
  //
  ROM_GPIOPinConfigure(GPIO_PA2_U4RX);
  ROM_GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_2);

  //
  // Configure the GPIO Pin Mux for PA3
  // for U4TX
  //
  ROM_GPIOPinConfigure(GPIO_PA3_U4TX);
  ROM_GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_3);
  ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART4);
  // while ( ! ROM_SysCtlPeripheralReady(SYSCTL_PERIPH_UART4));

  //
  // Configure the UART for 115,200, 8-N-1 operation.
  //
  ROM_UARTConfigSetExpClk(UART4_BASE, g_ui32SysClock, 115200,
                          (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE));

  ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

#elif UARTx_BASE == UART1_BASE
  ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);

  //
  // Configure the GPIO Pin Mux for PB0
  // for U1RX
  //
  ROM_GPIOPinConfigure(GPIO_PB0_U1RX);
  ROM_GPIOPinTypeUART(GPIO_PORTB_BASE, GPIO_PIN_0);
  //
  // Configure the GPIO Pin Mux for PB1
  // for U1TX
  //
  ROM_GPIOPinConfigure(GPIO_PB1_U1TX);
  ROM_GPIOPinTypeUART(GPIO_PORTB_BASE, GPIO_PIN_1);

  // Turn on the UART peripheral
  ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART1);
  while (!ROM_SysCtlPeripheralReady(SYSCTL_PERIPH_UART1))
    ;

  //
  // Configure the UART for 115,200, 8-N-1 operation.
  //
  ROM_UARTConfigSetExpClk(UART1_BASE, g_ui32SysClock, 115200,
                          (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE));
#elif UARTx_BASE == UART0_BASE
  ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
  //
  // Configure the GPIO Pin Mux for PA0
  // for U0RX
  //
  MAP_GPIOPinConfigure(GPIO_PA0_U0RX);
  MAP_GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0);

  //
  // Configure the GPIO Pin Mux for PA1
  // for U0TX
  //
  MAP_GPIOPinConfigure(GPIO_PA1_U0TX);
  MAP_GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_1);

  // Turn on the UART peripheral
  ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
  while (!ROM_SysCtlPeripheralReady(SYSCTL_PERIPH_UART0))
    ;

  //
  // Configure the UART for 115,200, 8-N-1 operation.
  //
  ROM_UARTConfigSetExpClk(UART0_BASE, g_ui32SysClock, 115200,
                          (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE));
#else
#error "Not supported UART"
#endif


  //
  // Enable processor interrupts.
  //
  ROM_IntMasterEnable();

#ifdef REV1
  //
  // Set relevant GPIO pins as UART pins.
  //
  //
  // Configure the GPIO Pin Mux for PA2
  // for U4RX
  //
  ROM_GPIOPinConfigure(GPIO_PA2_U4RX);
  ROM_GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_2);

  //
  // Configure the GPIO Pin Mux for PA3
  // for U4TX
  //
  ROM_GPIOPinConfigure(GPIO_PA3_U4TX);
  ROM_GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_3);

  //
  // Configure the UART for 115,200, 8-N-1 operation.
  //
  ROM_UARTConfigSetExpClk(UART_FRONTPANEL, g_ui32SysClock, 115200,
                          (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE));
  //
  // Enable the UART interrupt.
  //
  ROM_IntEnable(INT_UART4);
  ROM_UARTIntEnable(UART_FRONTPANEL, UART_INT_RX | UART_INT_RT);
#else // REV2
  //
  // Enable the UART interrupt.
  //
  ROM_IntEnable(INT_UART0);
  ROM_UARTIntEnable(UART0_BASE, UART_INT_RX | UART_INT_RT);
#endif
  //
  // Prompt for text to be entered.
  //
  UARTSend((uint8_t *)"\033[2JEnter text: ", 16);

  //
  // Loop forever echoing data through the UART.
  //
  while(1)
    {
    }
}
