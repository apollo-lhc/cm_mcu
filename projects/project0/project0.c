//*****************************************************************************
//
// project0.c - Example to demonstrate minimal TivaWare setup
//
// Copyright (c) 2012-2017 Texas Instruments Incorporated.  All rights reserved.
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
#include "inc/hw_types.h"
#include "inc/hw_memmap.h"
#include "inc/hw_ints.h"
#include "driverlib/pin_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "driverlib/uart.h"
#include "driverlib/interrupt.h"
#include "board_specific/pinout.h"
//#include "utils/uartstdio.h"

//*****************************************************************************
//
// Define pin to LED mapping.
//
//*****************************************************************************

//*****************************************************************************
//
//! \addtogroup example_list
//! <h1>Project Zero (project0)</h1>
//!
//! This example demonstrates the use of TivaWare to setup the clocks and
//! toggle GPIO pins to make the LED blink. This is a good place to start
//! understanding your launchpad and the tools that can be used to program it.
//
//*****************************************************************************

#define USER_LED1  GPIO_PIN_0
#define USER_LED2  GPIO_PIN_1


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


// Initialize the UART 
// based on uart_echo demo project
// we use UART4 (front panel) 
void 
UartInit(uint32_t ui32SysClock)
{
  //
  // Set relevant GPIO pins as UART pins.
  //
  //
  // Configure the GPIO Pin Mux for PA2
  // for U4RX
  //
  MAP_GPIOPinConfigure(GPIO_PA2_U4RX);
  MAP_GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_2);

  //
  // Configure the GPIO Pin Mux for PA3
  // for U4TX
  //
  MAP_GPIOPinConfigure(GPIO_PA3_U4TX);
  MAP_GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_3);

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
      MAP_UARTCharPutNonBlocking(UART4_BASE, *pui8Buffer++);
    }
}

uint32_t g_ui32SysClock = 0;


// data structures to hold PIN information for the supplies
struct supplies_t {
  char *name;
  int port, pin, priority;
};
  

struct supplies_t enables[] = {
  { "CTRL_K_VCCINT_PWR_EN", GPIO_PORTF_BASE, GPIO_PIN_2, 1},
  { "CTRL_V_VCCINT_PWR_EN", GPIO_PORTA_BASE, GPIO_PIN_4, 1},
  { "CTRL_VCC_1V8_PWR_EN",  GPIO_PORTA_BASE, GPIO_PIN_5, 2},
  { "CTRL_VCC_3V3_PWR_EN",  GPIO_PORTF_BASE, GPIO_PIN_1, 3},
  { "CTRL_V_MGTY1_VCCAUX_PWR_EN",GPIO_PORTN_BASE, GPIO_PIN_5, 4},
  { "CTRL_V_MGTY2_VCCAUX_PWR_EN",GPIO_PORTN_BASE, GPIO_PIN_4, 4},
  { "CTRL_K_MGTY_VCCAUX_PWR_EN", GPIO_PORTL_BASE, GPIO_PIN_2, 4},
  { "CTRL_K_MGTH_VCCAUX_PWR_EN", GPIO_PORTL_BASE, GPIO_PIN_3, 4},
  { "CTRL_V_MGTY1_AVCC_PWR_EN",GPIO_PORTN_BASE, GPIO_PIN_3, 5},
  { "CTRL_V_MGTY2_AVCC_PWR_EN",GPIO_PORTN_BASE, GPIO_PIN_2, 5},
  { "CTRL_K_MGTY_AVCC_PWR_EN", GPIO_PORTL_BASE, GPIO_PIN_4, 5},
  { "CTRL_K_MGTH_AVCC_PWR_EN", GPIO_PORTL_BASE, GPIO_PIN_5, 5},
  { "CTRL_K_MGTY_AVTT_PWR_EN", GPIO_PORTQ_BASE, GPIO_PIN_4, 6},
  { "CTRL_K_MGTH_AVTT_PWR_EN", GPIO_PORTP_BASE, GPIO_PIN_2, 6},
  { "CTRL_V_MGTY1_AVTT_PWR_EN",GPIO_PORTN_BASE, GPIO_PIN_1, 6},
  { "CTRL_V_MGTY2_AVTT_PWR_EN",GPIO_PORTN_BASE, GPIO_PIN_0, 6}
};
const int nenables = sizeof(enables)/sizeof(enables[0]);

struct supplies_t oks[] = {
  { "K_VCCINT_PG_A", GPIO_PORTK_BASE, GPIO_PIN_5, 1},
  { "K_VCCINT_PG_B", GPIO_PORTK_BASE, GPIO_PIN_6, 1},
  { "V_VCCINT_PG_A", GPIO_PORTH_BASE, GPIO_PIN_3, 1},
  { "V_VCCINT_PG_B", GPIO_PORTH_BASE, GPIO_PIN_2, 1},
  { "VCC_1V8_PG",  GPIO_PORTH_BASE, GPIO_PIN_0, 2},
  { "VCC_3V3_PG",  GPIO_PORTH_BASE, GPIO_PIN_1, 3},
  { "V_MGTY1_AVCC_OK",GPIO_PORTC_BASE, GPIO_PIN_6, 5},
  { "V_MGTY2_AVCC_OK",GPIO_PORTC_BASE, GPIO_PIN_5, 5},
  { "K_MGTY_AVCC_OK", GPIO_PORTM_BASE, GPIO_PIN_2, 5},
  { "K_MGTH_AVCC_OK", GPIO_PORTM_BASE, GPIO_PIN_3, 5},
  { "K_MGTY_AVTT_OK", GPIO_PORTM_BASE, GPIO_PIN_1, 6},
  { "K_MGTH_AVTT_OK", GPIO_PORTM_BASE, GPIO_PIN_4, 6},
  { "V_MGTY1_AVTT_OK",GPIO_PORTC_BASE, GPIO_PIN_7, 6},
  { "V_MGTY2_AVTT_OK",GPIO_PORTC_BASE, GPIO_PIN_4, 6}
};
const int noks = sizeof(oks)/sizeof(oks[0]);
const int num_priorities = 6;



// 
// check the power supplies and turn them on one by one
// 
bool set_ps(bool KU15P, bool VU7PMGT1, bool VU7PMGT2)
{
  bool success = true; // return value
  


  // data structure to turn on various power supplies. This should be ordered such
  // that the priority increases, though it's not necessary
  for ( int prio = 1; prio <= num_priorities; ++prio ) {
    // enable the supplies at the relevant priority
    for ( int e = 0; e < nenables; ++e ) {
      if ( enables[e].priority == prio ) 
	MAP_GPIOPinWrite(enables[e].port, enables[e].pin, 0x1);
    }
    //
    // Delay for a bit
    //
    SysCtlDelay(g_ui32SysClock/6);
    // check power good at this level or higher priority (lower number)
    bool all_good = true;
    int o = -1;
    for ( o = 0; o < noks; ++o ) {
      if ( enables[o].priority <= prio ) {
	int32_t val = MAP_GPIOPinRead(oks[o].port, oks[o].pin);
	if ( val == 0 ) {
	  all_good = false;
	  break;
	}
      }
    } // loop over 'ok' bits
    if (  ! all_good ) { 
      // o tells you which one died. should I print something on UART?
      // turn off all supplies at current priority level or lower
      // that is probably overkill since they should not all be 
      for ( int e = 0; e < nenables; ++e ) {
	  if ( enables[e].priority >= prio ) 
	    MAP_GPIOPinWrite(enables[e].port, enables[e].pin, 0x0);
	}
      success = false;
      break;
    }
  } // loop over priorities

  return success;
  
}

bool
check_ps(void)
{
  bool success = true;
  for ( int prio = 1; prio <= num_priorities; ++prio ) {
    // enable the supplies at the relevant priority
    bool all_good = true;
    int o = -1;
    for ( o = 0; o < noks; ++o ) {
      if ( enables[o].priority <= prio ) {
	int32_t val = MAP_GPIOPinRead(oks[o].port, oks[o].pin);
	if ( val == 0 ) {
	  all_good = false;
	  break;
	}
      }
    } // loop over 'ok' bits
    if (  ! all_good ) { 
      // o tells you which one died. should I print something on UART?
      // turn off all supplies at current priority level or lower
      for ( int e = 0; e < nenables; ++e ) {
	  if ( enables[e].priority >= prio ) 
	    MAP_GPIOPinWrite(enables[e].port, enables[e].pin, 0x0);
	}
      success = false;
      break;
    }
  } // loop over priorities

  return success;
}



//*****************************************************************************
//
// Main 'C' Language entry point.  Toggle an LED using TivaWare.
//
//*****************************************************************************
int
main(void)
{

    // initialize all pins, using file setup by TI PINMUX tool
    PinoutSet();


    //
    // Run from the PLL at 120 MHz.
    //
    g_ui32SysClock = SysCtlClockFreqSet((SYSCTL_XTAL_25MHZ |
                                       SYSCTL_OSC_MAIN |
                                       SYSCTL_USE_PLL |
                                       SYSCTL_CFG_VCO_480), 120000000);

    //
    // Enable and wait for the port to be ready for access
    //
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOJ);
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOJ))
    {
    }
    
    //
    // Configure the GPIO port for the LED operation.
    //
    GPIOPinTypeGPIOOutput(GPIO_PORTJ_BASE, (USER_LED1|USER_LED2));

    //
    // Say hello
    //
    UARTSend((uint8_t *)"Project0 starting", 16);

    if ( !set_ps(true,true,false) ) {
      UARTSend((uint8_t *)"set_ps failed!",16);
    }

    //
    // Loop Forever
    //
    while(1) {
      //
      // Turn on the LED 1
      //
      GPIOPinWrite(GPIO_PORTJ_BASE, (USER_LED1|USER_LED2), 0x1);
      
      //
      // Delay for a bit
      //
      SysCtlDelay(g_ui32SysClock/6);
      
      //
      // Turn on the LED 2
      //
      GPIOPinWrite(GPIO_PORTJ_BASE, (USER_LED1|USER_LED2), 0x2);

      //
      // Delay for a bit
      //
      SysCtlDelay(g_ui32SysClock/6);

      if ( !check_ps() ) {
	UARTSend((uint8_t *)"check_ps failed!",16);
      }
    }
}
