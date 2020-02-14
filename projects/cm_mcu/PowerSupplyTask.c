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
#include "Tasks.h"

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
  enum ps_state oldState = PWR_UNKNOWN;
  bool alarm = false;

  bool cli_powerdown_request = false;

  // turn on the power supply at the start of the task, if the power enable is sent by the
  // zynq
  if ( read_gpio_pin(BLADE_POWER_EN) == 1 )
	errbuffer_put(ebuf,RESTART,0);	//Does this look okay?
    set_ps() ;

  // this function never returns
  for ( ;; ) {
    // first check for message on the queue and act on any messages.
    // non-blocking call.
    uint32_t message;
    if ( xQueueReceive(xPwrQueue, &message, 0) ) { // TODO: what if I receive more than one message
      switch (message ) {
      case PS_OFF:
        cli_powerdown_request = true;
        disable_ps();
        break;
      case TEMP_ALARM:
        alarm = true;
        disable_ps();
        break;
      case TEMP_ALARM_CLEAR:
        alarm = false;
        break;
      case PS_ON:
        cli_powerdown_request = false;
        set_ps();
        break;
      default:
        message = RED_LED_TOGGLE;
        xQueueSendToBack(xLedQueue, &message, pdMS_TO_TICKS(10)); // message I don't understand? Toggle red LED
        break;
      }
    }
    enum ps_state newstate;
    // Check the state of BLADE_POWER_EN. The MCU will
    // run the disable command even if the power was already
    // off, just to be sure. check_ps() above can return PWR_OFF
    // even if some supplies are still on (it really is checking if _all__
    // expected supplies are good.)
    // Eventually this will be replaced by a message from an interrupt handler.
    bool blade_power_enable = (read_gpio_pin(BLADE_POWER_EN) == 1);
    if ( ! blade_power_enable  ) {
      disable_ps();
      newstate = PWR_OFF;
    }
    else if ( ! alarm && ! cli_powerdown_request ) { // blade_power_enable and not alarm
      set_ps();
      newstate = PWR_ON;
    }
    // now check the actual state
    bool psgood = check_ps();
    newstate = psgood?PWR_ON:PWR_OFF;


    if ( newstate == PWR_OFF  && oldState != PWR_OFF) {
      Print("\r\nPowerSupplyTask: power supplies turned off.\r\n");
      if ( ! blade_power_enable )
        Print("\r\nPowerSupplyTask: PWR_EN removed by SM\r\n");
      message = PS_BAD;
    }
    else { // either the PS is now good or there was no change.
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
