/*
 * bl_userhooks.c
 *
 *  Created on: Sep 26, 2019
 *      Author: pw94
 */
#include <stdint.h>
#include <stdbool.h>

#include "inc/hw_memmap.h"
#include "inc/hw_ints.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include "driverlib/interrupt.h"
#include "driverlib/gpio.h"

#include "boot_loader/bl_config.h"

#include "common/uart.h"

//*****************************************************************************
//
// A prototype for the function (in the startup code) for a predictable length
// delay.
//
//*****************************************************************************
extern void Delay(uint32_t ui32Count);


void
bl_user_init_fn(void)
{
  // ZYNQ UART for monitoring
  // Turn on the UART peripheral
  ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART1);


  //
  // Configure the UART for 115,200, 8-N-1 operation.
  //
  ROM_UARTConfigSetExpClk(UART1_BASE, CRYSTAL_FREQ, 115200,
                          (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |
                              UART_CONFIG_PAR_NONE));

  UARTPrint(UART4_BASE, "Bootloader starting\r\n");
  UARTPrint(UART4_BASE, FIRMWARE_VERSION "\r\n");

  // LEDs
  //
  MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOP);
  MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOJ);


  // Configure the GPIO Pin Mux for PJ0
  // for GPIO_PJ0
  //
  MAP_GPIOPinTypeGPIOOutput(GPIO_PORTJ_BASE, GPIO_PIN_0);

  //
  // Configure the GPIO Pin Mux for PJ1
  // for GPIO_PJ1
  //
  MAP_GPIOPinTypeGPIOOutput(GPIO_PORTJ_BASE, GPIO_PIN_1);
  // Red LED
  // Configure the GPIO Pin Mux for PP0
  // for GPIO_PP0
  //
  MAP_GPIOPinTypeGPIOOutput(GPIO_PORTP_BASE, GPIO_PIN_0);


  return;
}
#define LONG_DELAY 2500000

void bl_user_end_hook()
{
  MAP_GPIOPinWrite(GPIO_PORTJ_BASE, GPIO_PIN_1, 1);
  Delay(LONG_DELAY);
  MAP_GPIOPinWrite(GPIO_PORTJ_BASE, GPIO_PIN_1, 0);
  Delay(LONG_DELAY);
  MAP_GPIOPinWrite(GPIO_PORTJ_BASE, GPIO_PIN_1, 1);
  Delay(LONG_DELAY);
  return;
}

void bl_user_progress_hook(unsigned long ulCompleted, unsigned long ulTotal)
{
  int tens = (10*ulCompleted/ulTotal);
  MAP_GPIOPinWrite(GPIO_PORTJ_BASE, GPIO_PIN_0, tens%2);
  return;
}

void bl_user_flash_error()
{
//  MAP_GPIOPinWrite(GPIO_PORTP_BASE, GPIO_PIN_0, 1);
//  Delay(LONG_DELAY);

  return;
}
