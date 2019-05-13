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

// FreeRTOS includes
#include "FreeRTOS.h"
#include "FreeRTOSConfig.h"
#include "queue.h"

// Holds the handle of the created queue for the LED task.
QueueHandle_t xLedQueue = NULL;

// control the LED
void LedTask(void *parameters)
{
  TickType_t xLastWakeTime = xTaskGetTickCount();
  uint32_t message;

  enum LEDpattern { OFF=0, ON=1, TOGGLE=2, TOGGLE3, TOGGLE4};
  enum LEDpattern greenLedPattern = TOGGLE;
  enum LEDpattern blueLedPattern = OFF;
  enum LEDpattern redLedPattern = OFF;

  // this function never returns. Loop here forever.
  for ( ;; ) {
    // check for a new item in the queue but don't wait
    if ( xQueueReceive(xLedQueue, &message, 0) ) {
      switch (message ) {
      case PS_BAD:
        write_gpio_pin(BLADE_POWER_OK,0);
        break;
      case PS_GOOD:
        write_gpio_pin(BLADE_POWER_OK, 1);
        break;
      default:
        toggle_gpio_pin(TM4C_LED_RED); // message I don't understand? Toggle blue LED
        break;
      }
    }
    toggle_gpio_pin(TM4C_LED_GREEN);
    // wait for next check
    vTaskDelayUntil( &xLastWakeTime, pdMS_TO_TICKS( 250 ) );

  }
}
