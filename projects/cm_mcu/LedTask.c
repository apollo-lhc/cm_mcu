/*
 * LedTask.c
 *
 *  Created on: May 13, 2019
 *      Author: wittich
 */

// includes for types
#include <stdint.h>
#include <stdbool.h>

// local includes
#include "common/uart.h"
#include "common/utils.h"
#include "common/power_ctl.h"
#include "common/pinout.h"
#include "common/pinsel.h"
#include "Tasks.h"

// FreeRTOS includes
#include "FreeRTOS.h"
#include "FreeRTOSConfig.h"
#include "queue.h"

enum LEDpattern { OFF=0, ON=1, TOGGLE=2, TOGGLE3, TOGGLE4};

// Holds the handle of the created queue for the LED task.
// gets initialized in main()
QueueHandle_t xLedQueue = NULL;

// control the LED
// Todo: expand beyond just controlling the red LED, if needed, for the command line interface. Probably not needed.
void LedTask(void *parameters)
{
  errbuffer_init(ebuf,2,2);	// Ughhhh

  TickType_t xLastWakeTime = xTaskGetTickCount();
  uint32_t callcnt = 0;

  enum LEDpattern greenLedPattern = OFF;
  enum LEDpattern blueLedPattern = OFF;
  enum LEDpattern redLedPattern = OFF;

  // this function never returns. Loop here forever.
  for ( ;; ) {
    uint32_t message;
   // check for a new item in the queue but don't wait
    if ( xQueueReceive(xLedQueue, &message, 0) ) {
      switch (message ) {
      case PS_BAD:
        write_gpio_pin(BLADE_POWER_OK,0); // this should not be here
        break;
      case PS_GOOD:
        write_gpio_pin(BLADE_POWER_OK, 1); // this should not be here
        break;
      case RED_LED_ON:
        redLedPattern = ON;
        break;
      case RED_LED_OFF:
        redLedPattern = OFF;
        break;
      case RED_LED_TOGGLE:
        redLedPattern = TOGGLE;
        break;
      case RED_LED_TOGGLE3:
        redLedPattern = TOGGLE3;
        break;
      case RED_LED_TOGGLE4:
        redLedPattern = TOGGLE4;
        break;
      default:
        toggle_gpio_pin(TM4C_LED_BLUE); // message I don't understand? Toggle blue LED
        break;
      }
    }
    // Greed LED
    if ( greenLedPattern == OFF)
      write_gpio_pin(TM4C_LED_GREEN, 0x0);
    else if ( greenLedPattern == ON )
      write_gpio_pin(TM4C_LED_GREEN, 0x1);
    else if ( callcnt%greenLedPattern == 0 ) // toggle patterns
      toggle_gpio_pin(TM4C_LED_GREEN);
    // Red LED
    if ( redLedPattern == OFF)
      write_gpio_pin(TM4C_LED_RED, 0x0);
    else if ( redLedPattern == ON )
      write_gpio_pin(TM4C_LED_RED, 0x1);
    else if ( callcnt%redLedPattern == 0 ) // toggle patterns
      toggle_gpio_pin(TM4C_LED_RED);
    // Blue LED
    if ( blueLedPattern == OFF)
      write_gpio_pin(TM4C_LED_BLUE, 0x0);
    else if ( blueLedPattern == ON )
      write_gpio_pin(TM4C_LED_BLUE, 0x1);
    else if ( callcnt%blueLedPattern == 0 ) // toggle patterns
      toggle_gpio_pin(TM4C_LED_BLUE);
    ++callcnt;
    // wait for next check
    vTaskDelayUntil( &xLastWakeTime, pdMS_TO_TICKS( 250 ) );

  }
}
