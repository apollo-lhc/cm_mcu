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
#include "board_specific/pinsel.h"

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
}
#endif

// write by pin number or name
static inline
void write_gpio_pin(int pin, uint8_t value)
{
  uint32_t gport;
  uint8_t  gpin;
  pinsel(pin, &gport, &gpin);
  MAP_GPIOPinWrite(gport, gpin, value);
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
// Send a string to the UART. From uart_echo example.
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
      if ( enables[e].priority == prio )
        write_gpio_pin(enables[e].name, 0x1);
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
        break;
      }
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
  UartInit(g_ui32SysClock);
  //
  // Say hello
  //
  UARTSend((uint8_t *)"Project0 starting", 16);


  bool ps_good = true;
  int bad_cnt = 1; // to toggle LED for failed PS check
  //
  // Loop Forever
  //
  while(1) {
    //
    // Turn on the LED 1, turn off LED 2
    //
    MAP_GPIOPinWrite(USER_LED12_PORT, (USER_LED1_PIN|USER_LED2_PIN), 0x1);
      
    //
    // Delay for a bit
    //
    SysCtlDelay(g_ui32SysClock/6);

    // turn on power supplies
    if ( ! ps_good ) {
      ps_good = set_ps(true,true,false) ;
      if ( ! ps_good )
        UARTSend((uint8_t *)"set_ps failed!",16);
    }
      
    //
    // Turn on the LED 2, turn off LED 1
    //
    MAP_GPIOPinWrite(USER_LED12_PORT, (USER_LED1_PIN|USER_LED2_PIN), 0x2);

    //
    // Delay for a bit
    //
    SysCtlDelay(g_ui32SysClock/6);

    if ( !check_ps() ) {
      UARTSend((uint8_t *)"check_ps failed!",16);
      MAP_GPIOPinWrite(USER_LED3_PORT, USER_LED3_PIN, bad_cnt%2);
      ++bad_cnt;
      ps_good = false;
    }
  }
  //_Static_assert (0, "assert1");

  return 0;
}
