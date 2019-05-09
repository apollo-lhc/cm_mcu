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

#include "common/utils.h"
#include "common/pinout.h"
#include "common/pinsel.h"
#include "common/uart.h"
#include "common/power_ctl.h"

#include "inc/hw_types.h"
#include "inc/hw_memmap.h"
#include "inc/hw_ints.h"
#include "driverlib/pin_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "driverlib/interrupt.h"

//*****************************************************************************
//
// Define pin to LED mapping.
//
//*****************************************************************************

#define USER_LED1_PIN  GPIO_PIN_0
#define USER_LED2_PIN  GPIO_PIN_1
#define USER_LED12_PORT GPIO_PORTJ_BASE // same port

#define USER_LED3_PIN  GPIO_PIN_0
#define USER_LED3_PORT GPIO_PORTP_BASE


//*****************************************************************************
//
// The error routine that is called if the driver library encounters an error.
//
//*****************************************************************************
#ifdef DEBUG
void
__error__(char *pcFilename, uint32_t ui32Line)
{
	while (1) {

	}
}
#endif




uint32_t g_ui32SysClock = 0;




//*****************************************************************************
//
// Main 'C' Language entry point.
//
//*****************************************************************************
int
main(void)
{
  // initialize all pins, using file setup by TI PINMUX tool
  PinoutSet();
  //
  // Enable processor interrupts.
  //
  MAP_IntMasterEnable();

  //
  // Run from the PLL at 120 MHz, using the internal oscillator.
  //
  g_ui32SysClock = MAP_SysCtlClockFreqSet((SYSCTL_OSC_INT |
                                           SYSCTL_USE_PLL |
                                           SYSCTL_CFG_VCO_480), 120000000);
  UART4Init(g_ui32SysClock);
  //
  // Say hello
  //
  UART4Print("\nProject0 Starting\n");

  // turn off all the LEDs
  write_gpio_pin(TM4C_LED_RED,   0x0);
  write_gpio_pin(TM4C_LED_BLUE,  0x0);
  write_gpio_pin(TM4C_LED_GREEN, 0x0);
  write_gpio_pin(BLADE_POWER_OK, 0x0);



  bool ps_good = false;
  int cnt = 1; // to toggle LED for failed PS check
  //
  // Loop Forever
  //
  while(1) {
    //

    // turn on power supplies
    if ( ! ps_good ) {
      ps_good = set_ps(true,true,false) ;
      if ( ! ps_good )
        UART4Print("set_ps failed!\n");
      else
      	write_gpio_pin(BLADE_POWER_OK, 0x1);

    }
      

    //
    // Delay for a bit
    //
    MAP_SysCtlDelay(g_ui32SysClock/10);

    if ( !check_ps() ) {
    	UART4Print("check_ps failed!\n");
    	toggle_gpio_pin(TM4C_LED_RED);
    	write_gpio_pin(BLADE_POWER_OK, 0x0);

    	ps_good = false;
    }
    else {
    	++cnt;
    	// turn off red LED
    	MAP_GPIOPinWrite(USER_LED3_PORT, USER_LED3_PIN, 0);
    	//
    	//
    	// toggle green and blue LEDs
    	//
    	const uint8_t led_toggle[] = {USER_LED2_PIN, USER_LED1_PIN};
    	MAP_GPIOPinWrite(USER_LED12_PORT, (USER_LED1_PIN|USER_LED2_PIN), led_toggle[cnt%2]);
    	write_gpio_pin(BLADE_POWER_OK, 0x1); // redundant?

    }
  }

  return 0;
}
