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
#include "inc/hw_uart.h"
#include "inc/hw_types.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include "driverlib/interrupt.h"
#include "driverlib/gpio.h"

#include "boot_loader/bl_config.h"
#include "boot_loader/bl_uart.h"


#include "common/uart.h"

//*****************************************************************************
//
// A prototype for the function (in the startup code) for a predictable length
// delay.
//
//*****************************************************************************
extern void Delay(uint32_t ui32Count);

void ConfigureDevice(void);

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

  ConfigureDevice();

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
#if 0
// UART receive with timeout
static
int
bl_user_UARTReceive_timeout(uint8_t *pui8Data, uint32_t ui32Size, uint32_t timeout)
{
  timeout = (timeout<0?-timeout:timeout);
  //
  // Send out the number of bytes requested.
  //
  while(ui32Size--)
  {
    //
    // Wait for the FIFO to not be empty.
    //
    while ((HWREG(UARTx_BASE + UART_O_FR) & UART_FR_RXFE))
    {
      if ( timeout-- <=0 )
        break;
    }
    if ( timeout <0)
      return 0;

    //
    // Receive a byte from the UART.
    //
    *pui8Data++ = HWREG(UARTx_BASE + UART_O_DR);
  }
  return ui32Size;
}
#endif
// check the UART, if I receive the special command within
// some period of time I force an update
// return non-zero to force an update
#define BUFFER_SZ  4
unsigned long bl_user_checkupdate_hook(void)
{
  ConfigureDevice();
  //
  UARTPrint(UART4_BASE, __func__);

  int timeout = 100000;
  uint32_t ui32Size = BUFFER_SIZE;
  uint8_t ui8Data[BUFFER_SZ];
  uint8_t recvd = 0;
  //
  // Send out the number of bytes requested.
  //
  while(ui32Size--)
  {
    //
    // Wait for the FIFO to not be empty.
    //
    while ((HWREG(UARTx_BASE + UART_O_FR) & UART_FR_RXFE))
    {
      if ( timeout-- <=0 )
        break;
    }
    if ( timeout <=0)
      break;

    //
    // Receive a byte from the UART.
    //
    ui8Data[recvd++] = HWREG(UARTx_BASE + UART_O_DR);
  }
  if ( ! recvd )
    return 0; // got nothing on the UART
  // look for the special signal
  if ( ui8Data[0] == 'A'  &&
      ui8Data[1] == '5'  &&
      ui8Data[2] == 'A'  &&
      ui8Data[3] == '5'
      )
    return 1;
  return 0;
}
