/*
 * bl_userhooks.c
 *
 *  Created on: Sep 26, 2019
 *      Author: pw94
 */
#include <stdint.h>
#include <stdbool.h>

#include "inc/hw_memmap.h"
#include "inc/hw_sysctl.h"
#include "inc/hw_ints.h"
#include "inc/hw_uart.h"
#include "inc/hw_types.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include "driverlib/interrupt.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"

#include "boot_loader/bl_config.h"
#include "boot_loader/bl_uart.h"

#include "common/printf.h"
#include "common/microrl.h"
#include "boot_loader/rl_config.h"

#include "boot_loader/bl_userhooks.h"

#include "common/LocalUart.h"

#define LONG_DELAY 2500000

#ifdef REV1
#define RED_LED_BASE   GPIO_PORTP_BASE
#define RED_LED_PIN    GPIO_PIN_0
#define BLUE_LED_BASE  GPIO_PORTJ_BASE
#define BLUE_LED_PIN   GPIO_PIN_0
#define GREEN_LED_BASE GPIO_PORTJ_BASE
#define GREEN_LED_PIN  GPIO_PIN_1
#elif defined(REV2) || defined(REV3)
#define LED_BASE       GPIO_PORTP_BASE
#define RED_LED_BASE   LED_BASE
#define GREEN_LED_BASE LED_BASE
#define BLUE_LED_BASE  LED_BASE
#define RED_LED_PIN    GPIO_PIN_3
#define GREEN_LED_PIN  GPIO_PIN_4
#define BLUE_LED_PIN   GPIO_PIN_5
#endif

//*****************************************************************************
//
// A prototype for the function (in the startup code) for a predictable length
// delay.
//
//*****************************************************************************
extern void Delay(uint32_t ui32Count);

void toggleLed(enum color rgb)
{
  int gpio_port, gpio_pin;
  switch (rgb) {
    case red: // red
      gpio_port = RED_LED_BASE;
      gpio_pin = RED_LED_PIN;
      break;
    case green: // green
      gpio_port = GREEN_LED_BASE;
      gpio_pin = GREEN_LED_PIN;
      break;
    case blue: // blue
    default:
      gpio_port = BLUE_LED_BASE;
      gpio_pin = BLUE_LED_PIN;
      break;
  }

  int val = MAP_GPIOPinRead(gpio_port, gpio_pin);
  if (val)
    MAP_GPIOPinWrite(gpio_port, gpio_pin, 0);
  else
    MAP_GPIOPinWrite(gpio_port, gpio_pin, gpio_pin);
  return;
}

#ifdef BL_HW_INIT_FN_HOOK
void bl_user_init_hw_fn(void)
{
#ifdef REV1
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
#elif defined(REV2) || defined(REV3)
  // LEDs
  //
  MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOP);

  // Configure the GPIO Pin Mux for PP3
  // for GPIO_PP3
  //
  MAP_GPIOPinTypeGPIOOutput(GPIO_PORTP_BASE, GPIO_PIN_3);

  //
  // Configure the GPIO Pin Mux for PP4
  // for GPIO_PP4
  //
  MAP_GPIOPinTypeGPIOOutput(GPIO_PORTP_BASE, GPIO_PIN_4);
  // Red LED
  // Configure the GPIO Pin Mux for PP0
  // for GPIO_PP5
  //
  MAP_GPIOPinTypeGPIOOutput(GPIO_PORTP_BASE, GPIO_PIN_5);
#endif // LEDs for Rev1

  // CLOCK
  // Run from the PLL, internal oscillator, at the defined clock speed
  // CRYSTAL_FREQ
  uint32_t ui32SysClock =
      MAP_SysCtlClockFreqSet((SYSCTL_OSC_INT | SYSCTL_USE_PLL |
                              SYSCTL_CFG_VCO_320),
                             CRYSTAL_FREQ);

  // UART
  // Turn on the UART peripheral
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
  MAP_GPIOPinConfigure(GPIO_PA3_U4TX);
  MAP_GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_3);
  MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART4);
  // while ( ! ROM_SysCtlPeripheralReady(SYSCTL_PERIPH_UART4));

  //
  // Configure the UART for 115,200, 8-N-1 operation.
  //
  MAP_UARTConfigSetExpClk(UART4_BASE, ui32SysClock, 115200,
                          (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |
                           UART_CONFIG_PAR_NONE));

  ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

#elif UARTx_BASE == UART1_BASE
  ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);

  //
  // Configure the GPIO Pin Mux for PB0
  // for U1RX
  //
  MAP_GPIOPinConfigure(GPIO_PB0_U1RX);
  MAP_GPIOPinTypeUART(GPIO_PORTB_BASE, GPIO_PIN_0);
  //
  // Configure the GPIO Pin Mux for PB1
  // for U1TX
  //
  MAP_GPIOPinConfigure(GPIO_PB1_U1TX);
  MAP_GPIOPinTypeUART(GPIO_PORTB_BASE, GPIO_PIN_1);

  // Turn on the UART peripheral
  MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART1);
  while (!ROM_SysCtlPeripheralReady(SYSCTL_PERIPH_UART1))
    ;

  //
  // Configure the UART for 115,200, 8-N-1 operation.
  //
  MAP_UARTConfigSetExpClk(UART1_BASE, ui32SysClock, 115200,
                          (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |
                           UART_CONFIG_PAR_NONE));
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
  ROM_UARTConfigSetExpClk(UART0_BASE, ui32SysClock, 115200,
                          (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE));
#else
#error "Not supported UART"
#endif

  return;
}
#endif // BL_INIT_FN_HOOK

#ifdef BL_END_FN_HOOK

// Flash green LED 3 times

void bl_user_end_hook(void)
{
  MAP_GPIOPinWrite(RED_LED_BASE, RED_LED_PIN, RED_LED_PIN);
  Delay(LONG_DELAY);
  MAP_GPIOPinWrite(RED_LED_BASE, RED_LED_PIN, 0);
  Delay(LONG_DELAY);
  MAP_GPIOPinWrite(RED_LED_BASE, RED_LED_PIN, RED_LED_PIN);
  Delay(LONG_DELAY);
  MAP_GPIOPinWrite(RED_LED_BASE, RED_LED_PIN, 0);
  return;
}
#endif // BL_END_FN_HOOK

#ifdef BL_PROGRESS_FN_HOOK

void bl_user_progress_hook(unsigned long ulCompleted, unsigned long ulTotal)
{
  unsigned int tens = (10 * ulCompleted / ulTotal);
  MAP_GPIOPinWrite(RED_LED_BASE, RED_LED_PIN, tens % 2);
  return;
}
#endif // BL_PROGRESS_FN_HOOK

#ifdef BL_CHECK_UPDATE_FN_HOOK
// User hook to force an update
// print helper for CLI
void bl_user_uart_print(const char *str)
{
  UARTPrint(UARTx_BASE, str);
  return;
}

struct bl_user_data_t {
  bool done;
  int retval;
};

int bl_user_rl_execute(void *d, int argc, char **argv)
{
  const char *helpstr =
      "h\r\n This help.\r\n"
      "b\r\n Start normal boot process\r\n"
      "r\r\n Restart MCU\r\n"
      "f\r\n Force update\r\n";

  struct bl_user_data_t *data = d; // cast pointer to void

  UARTPrint(UARTx_BASE, "\r\n");

  if (argc > 1) {
    UARTPrint(UARTx_BASE, helpstr);
    data->done = false;
    return 0;
  }

  switch (argv[0][0]) {
    case 'b':
      UARTPrint(UARTx_BASE, "Booting application.\r\n");
      data->done = true;
      data->retval = 0;
      break;
    case 'f':
      UARTPrint(UARTx_BASE, "Force update\r\n");
      data->done = true;
      data->retval = 1;
      break;
    case 'r':
      UARTPrint(UARTx_BASE, "Hard reboot\r\n");
      ROM_SysCtlReset(); // this function never returns
      break;
    case 'h':
    default:
      UARTPrint(UARTx_BASE, helpstr);
      data->done = false;
      break;
  }
  while (ROM_UARTBusy(UARTx_BASE))
    ;

  // this return value is AFAIK ignored
  return 0;
}

// return non-zero to force an update
#define BL_USER_BUFFSZ 1
unsigned long bl_user_checkupdate_hook(void)
{
  UARTPrint(UARTx_BASE, "\r\n***** CM MCU BOOTLOADER *****\r\n");
  UARTPrint(UARTx_BASE, FIRMWARE_VERSION "\r\n");
  while (ROM_UARTBusy(UARTx_BASE))
    ;
  toggleLed(blue);

  struct bl_user_data_t rl_userdata = {
      .done = false,
      .retval = 0,
  };
  struct microrl_config rl_config = {
      .print = bl_user_uart_print,
      // set callback for execute
      .execute = bl_user_rl_execute,
      .prompt_str = "$ ",
      .prompt_length = 2,
      .userdata = &rl_userdata,
  };
  microrl_t rl;
  microrl_init(&rl, &rl_config);
  microrl_set_execute_callback(&rl, bl_user_rl_execute);
  microrl_insert_char(&rl, ' '); // this seems to be necessary?

  int cRxedChar;
  for (;;) {
    int timeleft = 20;
    cRxedChar = -1;

    while (timeleft) {
      if (MAP_UARTCharsAvail(UARTx_BASE)) {
        cRxedChar = MAP_UARTCharGetNonBlocking(UARTx_BASE);
        break;
      }
      else {
        ROM_SysCtlDelay(CRYSTAL_FREQ / 20);
      }
      timeleft--;
    }
    if (cRxedChar > 0)
      microrl_insert_char(&rl, cRxedChar);
    if (timeleft == 0) {
      UARTPrint(UARTx_BASE, "CLI timed out\r\n");
      break;
    }
    if (rl_userdata.done == true) {
      toggleLed(blue);
      return rl_userdata.retval;
    }
  }
  toggleLed(blue);

  return 0;
}
#endif // BL_CHECK_UPDATE_FN_HOOK
