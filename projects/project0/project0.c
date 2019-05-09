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
#include <string.h>
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
#include "common/pinout.h"
#include "common/pinsel.h"

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

// write by pin number or name
static inline
void write_gpio_pin(int pin, uint8_t value)
{
  uint32_t gport;
  uint8_t  gpin;
  pinsel(pin, &gport, &gpin);
  uint8_t pinval;
  if ( value == 1 )
	  pinval = gpin;
  else
	  pinval = 0;
  MAP_GPIOPinWrite(gport, gpin, pinval);
  return;
}

// write by pin number or name
static inline
uint8_t read_gpio_pin(int pin)
{
  uint32_t gport;
  uint8_t  gpin;
  uint8_t value;
  pinsel(pin, &gport, &gpin);
  value = MAP_GPIOPinRead(gport, gpin);
  return value;
}

// write by pin number or name
static inline
uint8_t toggle_gpio_pin(int pin)
{
  uint8_t val = read_gpio_pin(pin);
  if ( val ) val = 0;
  else
	  val = 1;
  write_gpio_pin(pin, val);
  return val;
}



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
// Send a string to the UART. From uart_echo example.
//
//*****************************************************************************
/*
void
UARTSend(const uint8_t *pui8Buffer, uint32_t ui32Count)
{
#ifdef USE_UART
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
#endif // USE_UART
}
*/


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


uint32_t g_ui32SysClock = 0;


// data structures to hold GPIO PIN information 
struct gpio_pin_t {
  int name;
  int priority;
};

struct gpio_pin_t enables[] = {
  {  CTRL_K_VCCINT_PWR_EN, 1},
  {  CTRL_V_VCCINT_PWR_EN, 1},
  {  CTRL_VCC_1V8_PWR_EN,  2},
  {  CTRL_VCC_3V3_PWR_EN,  3},
  {  CTRL_V_MGTY1_VCCAUX_PWR_EN, 4},
  {  CTRL_V_MGTY2_VCCAUX_PWR_EN, 4},
  {  CTRL_K_MGTY_VCCAUX_PWR_EN,  4},
  {  CTRL_K_MGTH_VCCAUX_PWR_EN,  4},
  {  CTRL_V_MGTY1_AVCC_PWR_EN, 5},
  {  CTRL_V_MGTY2_AVCC_PWR_EN, 5},
  {  CTRL_K_MGTY_AVCC_PWR_EN,  5},
  {  CTRL_K_MGTH_AVCC_PWR_EN,  5},
  {  CTRL_K_MGTY_AVTT_PWR_EN,  6},
  {  CTRL_K_MGTH_AVTT_PWR_EN,  6},
  {  CTRL_V_MGTY1_AVTT_PWR_EN, 6},
  {  CTRL_V_MGTY2_AVTT_PWR_EN, 6}
};
const int nenables = sizeof(enables)/sizeof(enables[0]);

struct gpio_pin_t oks[] = {
  { K_VCCINT_PG_A, 1},
  { K_VCCINT_PG_B, 1},
  { V_VCCINT_PG_A, 1},
  { V_VCCINT_PG_B, 1},
  { VCC_1V8_PG,    2},
  { VCC_3V3_PG,    3},
  { V_MGTY1_AVCC_OK, 5},
  { V_MGTY2_AVCC_OK, 5},
  { K_MGTY_AVCC_OK,  5},
  { K_MGTH_AVCC_OK,  5},
  { K_MGTY_AVTT_OK,  6},
  { K_MGTH_AVTT_OK,  6},
  { V_MGTY1_AVTT_OK, 6},
  { V_MGTY2_AVTT_OK, 6}
};
const int noks = sizeof(oks)/sizeof(oks[0]);
const int num_priorities = 6;



// 
// check the power supplies and turn them on one by one
// 
bool set_ps(bool KU15P, bool VU7PMGT1, bool VU7PMGT2)
{
  bool success = true; // return value

  // data structure to turn on various power supplies. This should be ordered
  // such that the priority increases, though it's not necessary
  for ( int prio = 1; prio <= num_priorities; ++prio ) {
	  // enable the supplies at the relevant priority
	  for ( int e = 0; e < nenables; ++e ) {
		  if ( enables[e].priority == prio ) {
			  write_gpio_pin(enables[e].name, 0x1);
		  }
	  }

	  //
	  // Delay for a bit
	  //
	  MAP_SysCtlDelay(g_ui32SysClock/6);
	  // check power good at this level or higher priority (lower number)
	  bool all_good = true;
	  int o = -1;
	  for ( o = 0; o < noks; ++o ) {
		  if ( oks[o].priority <= prio ) {
			  int8_t val = read_gpio_pin(oks[o].name);
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
				  write_gpio_pin(enables[e].name, 0x0);

		  }
		  // toggle RED leg
		  toggle_gpio_pin(TM4C_LED_RED);
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
        int8_t val = read_gpio_pin(oks[o].name);
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
        if ( enables[e].priority >= prio ) {
          write_gpio_pin(enables[e].name, 0x0);
        }
        success = false;
        // toggle on red LED
        toggle_gpio_pin(TM4C_LED_RED);

        break;
      }
    }
  } // loop over priorities

  return success;
}



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
    }
      

    //
    // Delay for a bit
    //
    MAP_SysCtlDelay(g_ui32SysClock/10);

    if ( !check_ps() ) {
    	UART4Print("check_ps failed!\n");
    	toggle_gpio_pin(TM4C_LED_RED);
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

    }
  }

  return 0;
}
