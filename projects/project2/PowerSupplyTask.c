/*
 * PowerSupplyTask.c
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
#include "common/i2c_reg.h"
#include "common/pinout.h"
#include "common/pinsel.h"

// FreeRTOS includes
#include "FreeRTOS.h"
#include "FreeRTOSConfig.h"
#include "queue.h"



// Holds the handle of the created queue for the power supply task.
QueueHandle_t xPwrQueue = NULL;

extern QueueHandle_t xLedQueue;

void Print(const char* str);



// monitor and control the power supplies
void PowerSupplyTask(void *parameters)
{
  // initialize to the current tick time
  TickType_t xLastWakeTime = xTaskGetTickCount();
  enum ps_state oldState = UNKNOWN;
  //enum state desiredState = PWR_ON; // start with power-on right away

  // turn on the power supply at the start of the task
  set_ps(true,true,true) ;

  // this function never returns
  for ( ;; ) {
    // first check for message on the queue and act on any messages.
    // non-blocking call.
    uint32_t message;
    if ( xQueueReceive(xPwrQueue, &message, 0) ) { // TODO: what if I receive more than one message
      switch (message ) {
      case PS_OFF:
        disable_ps();
        //desiredState= PWR_OFF;
        break;
      case PS_ON:
        set_ps(true,true,true);
        //desiredState = PWR_ON;
        break;
      default:
        toggle_gpio_pin(TM4C_LED_RED); // message I don't understand? Toggle red LED
        break;
      }
    }
    // now check the actual state
    bool psgood = check_ps();
    enum ps_state newstate = psgood?PWR_ON:PWR_OFF;

    if ( newstate == PWR_OFF  && oldState != PWR_OFF) {
      Print("\nPowerSupplyTask: power supplies turned off.\n");
      message = PS_BAD;
    }
    else { // all good
      message = PS_GOOD;
    }
    // only send a message on state change.
    if ( oldState != newstate )
      xQueueSendToBack(xLedQueue, &message, pdMS_TO_TICKS(10));// todo: check on fail to send
    oldState = newstate;

    // wait here for the x msec, where x is 2nd argument below.
    vTaskDelayUntil( &xLastWakeTime, pdMS_TO_TICKS( 25 ) );
  }
}
