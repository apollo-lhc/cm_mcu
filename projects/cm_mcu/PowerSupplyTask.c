/*
 * PowerSupplyTask.c
 *
 *  Created on: May 13, 2019
 *      Author: wittich
 */

// includes for types
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <assert.h>

// local includes
#include "Tasks.h"
#include "common/i2c_reg.h"
#include "common/pinout.h"
#include "common/pinsel.h"
#include "common/power_ctl.h"
#include "common/uart.h"
#include "common/utils.h"

// FreeRTOS includes
#include "FreeRTOS.h"
#include "FreeRTOSConfig.h"
#include "queue.h"

// Holds the handle of the created queue for the power supply task.
QueueHandle_t xPwrQueue = NULL;

extern QueueHandle_t xLedQueue;

void Print(const char *str);
// local sprintf prototype
int snprintf( char *buf, unsigned int count, const char *format, ... );


enum power_system_state currentState = INIT; // start in INIT state

enum power_system_state getPowerControlState()
{
  return currentState;
}

extern const struct gpio_pin_t oks[];
static uint16_t check_ps_oks(void)
{
  uint16_t status = 0U;
  for (int i = 0; i < N_PS_OKS; ++i) {
    int val = read_gpio_pin(oks[i].name);
    if (val)
      status |= 1U << i;
  }
  return status;
}

// monitor and control the power supplies
void PowerSupplyTask(void *parameters)
{
  // compile-time sanity check
  static_assert(PS_ENS_MASK == (PS_ENS_GEN_MASK|PS_ENS_VU_MASK|PS_ENS_KU_MASK), "mask");
  static_assert(PS_OKS_MASK == (PS_OKS_GEN_MASK|PS_OKS_VU_MASK|PS_OKS_KU_MASK), "mask");

  // initialize to the current tick time
  TickType_t xLastWakeTime = xTaskGetTickCount();

  // alarm from outside this task
  bool external_alarm = false;
  // powerdown request from the CLI
  bool cli_powerdown_request = false;

  // masks to enable/check appropriate supplies
  uint16_t supply_ok_mask = PS_OKS_GEN_MASK;
  uint16_t supply_en_mask = PS_ENS_GEN_MASK;
  bool ku_enable = (read_gpio_pin(TM4C_DIP_SW_1) == 1);
  bool vu_enable = (read_gpio_pin(TM4C_DIP_SW_2) == 1);
  if (ku_enable) {
    supply_ok_mask |= PS_OKS_KU_MASK;
    supply_en_mask |= PS_ENS_KU_MASK;
  }
  if (vu_enable) {
    supply_ok_mask |= PS_OKS_VU_MASK;
    supply_en_mask |= PS_ENS_VU_MASK;
  }
  #ifdef APOLLO10_HACK
  // APOLLO 10 HACK
  supply_mask &= ~(1<<6);
  supply_mask &= ~(1<<7);
  supply_mask &= ~(1<<8);
  supply_mask &= ~(1<<9);
  supply_mask &= ~(1<<10);
  supply_mask &= ~(1<<11);
  supply_mask &= ~(1<<12);
  supply_mask &= ~(1<<13);
  char tmp[64];
  snprintf(tmp, 64, "PowerSupplyTask: hack mask is %x\r\n", supply_mask);
  Print(tmp);
  #endif // APOLLO10_HACK

  bool power_supply_alarm = false;
  uint16_t failed_mask = 0x0U;
  // this loop never exits
  for (;;) {
    // first check for message on the queue and collect all messages.
    // non-blocking call.
    uint32_t message;
    if (xQueueReceive(xPwrQueue, &message, 0)) { // TODO: more than one message
      switch (message) {
        case PS_OFF:
          cli_powerdown_request = true;
          break;
        case TEMP_ALARM:
          external_alarm = true;
          break;
        case TEMP_ALARM_CLEAR:
          external_alarm = false;
          break;
        case PS_ON:
          cli_powerdown_request = false;
          break;
        case PS_ANYFAIL_ALARM_CLEAR:
          power_supply_alarm = false;
          failed_mask = 0x0U;
          break;
        default:
          break;
      }
    }
    // Check the state of BLADE_POWER_EN.
    bool blade_power_enable = (read_gpio_pin(BLADE_POWER_EN) == 1);

    // now check the actual state of the power supplies
    uint16_t supply_bitset = check_ps_oks();
    bool supply_off = false; // are supplies off (besides the ones that are disabled)
    if ((supply_bitset&supply_ok_mask) != supply_ok_mask) {
      supply_off = true;
    }

    // MAIN POWER SUPPLY TASK STATE MACHINE
    // +------+
    // | INIT +--------------------------------------------+
    // +---+--+                                            |
    //     |                                               v
    //     |          +-----------+                  +-----+-----+
    //     |          |           +----------------> |           |
    //     +--------->+ POWER_ON  |                  | POWER_OFF |
    //                |           | <----------------+           |
    //                +----+------+                  +-------+---+
    //                     |                                 ^
    //                     |        +----------------+       |
    //                     |        |                |       |
    //                     +------->+  POWER_FAILURE +-------+
    //                              |                |
    //                              +----------------+
    // the state you are in is the current state, so for instance
    // there is no power_off transition in the power_off state; 
    // instead it's in the POWER_ON state (e.g.)
    enum power_system_state nextState;
    switch (currentState) {
      case INIT: {
        // only run on first boot
        if (blade_power_enable) {
          turn_on_ps(supply_en_mask);
          errbuffer_put(EBUF_POWER_ON, 0);
          nextState = POWER_ON;
          supply_off = false;
        }
        else {
          nextState = POWER_OFF;
        }
        break;
      }
      case POWER_ON: {
        if (supply_off) {
          // log erroring supplies
          failed_mask = (~supply_bitset) & supply_ok_mask;
          char tmp[64];
          snprintf(tmp, 64,
                   "failure set: fail, supply_mask,bitset =  %x,%x,%x\r\n",
                   failed_mask, supply_ok_mask, supply_bitset);
          Print(tmp);

          errbuffer_power_fail(failed_mask);
          // turn off all supplies
          disable_ps();
          power_supply_alarm = true;
          nextState = POWER_FAILURE;
        }
        else if (external_alarm) {
          errbuffer_put(EBUF_POWER_OFF_TEMP, 0);
          // turn off all supplies
          disable_ps();
          nextState = POWER_FAILURE;
        }
        else if (!blade_power_enable || cli_powerdown_request) {
          disable_ps();
          errbuffer_put(EBUF_POWER_OFF, 0);
          nextState = POWER_OFF;
        }
        else {
          nextState = POWER_ON;
        }
        break;
      }
      case POWER_OFF: {
        if (blade_power_enable && !cli_powerdown_request && !external_alarm &&
            !power_supply_alarm) {
          turn_on_ps(supply_en_mask);
          errbuffer_put(EBUF_POWER_ON, 0);
          nextState = POWER_ON;
        }
        else {
          nextState = POWER_OFF;
        }
        break;
      }
      case POWER_FAILURE: { // we go through POWER_OFF state before turning on.
        if (!power_supply_alarm && !external_alarm) { // errors cleared
          nextState = POWER_OFF;
        }
        else { // still in failed state
          nextState = POWER_FAILURE;
        }
        break;
      }
      default: {
        // configASSERT(1 == 0);
        nextState = INIT; // shut up debugger
        break;
      }
    }

    // update the ps_state variables, for external display
    // as well as for usage in ensuring the I2C pullups are on.
    supply_bitset = check_ps_oks();
    for (size_t i = 0; i < N_PS_OKS; i++) {
      if ((1U << i) & supply_bitset) {
        // OK bit is on -- PS is on
        setPSStatus(i, PWR_ON);
      }
      // OK bit is not on and ...
      else if (!((1U << i) & supply_ok_mask)) {
        // ... it should _not_ be on. Disabled intentionally
        setPSStatus(i, PWR_DISABLED);
      }
      else {
        // ... it _should_ be on based on the mask
        switch (currentState) {
        case POWER_OFF:
        case INIT:
          // ... but the power state is "off" or 
          // we are just initializing / turning on
          setPSStatus(i, PWR_OFF);
          break;
        case POWER_ON:
        case POWER_FAILURE:
          // ... but the power state is is "on," so ...
          if ((1U << i) & failed_mask) {
            // ... either the supply failed
            setPSStatus(i, PWR_FAILED);
          }
          else {
            // ... or it's just off because there was a power failure,
            // but this supply is not the root cause.
            setPSStatus(i, PWR_OFF);
          }
          break;
        default:
          break;
        }
      }
    }
    if (currentState != nextState) {
      char tmp[64];
      snprintf(tmp, 64, "PowerSupplyTask: change from state %d to %d\r\n",
               currentState, nextState);
      Print(tmp);
    }
    currentState = nextState;

    // wait here for the x msec, where x is 2nd argument below.
    vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(25));
  }
}
