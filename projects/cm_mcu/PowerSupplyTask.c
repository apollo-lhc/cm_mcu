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
#include "common/utils.h"
#include "common/log.h"

// FreeRTOS includes
#include "FreeRTOS.h"
#include "FreeRTOSConfig.h"
#include "queue.h"

// Holds the handle of the created queue for the power supply task.
QueueHandle_t xPwrQueue = NULL;


enum power_system_state currentState = POWER_INIT; // start in POWER_INIT state

enum power_system_state getPowerControlState()
{
  return currentState;
}

extern const struct gpio_pin_t oks[];
static uint16_t check_ps_oks(void)
{
  uint16_t status = 0U;
  for (int i = 0; i < N_PS_OKS; ++i) {
    int val = read_gpio_pin(oks[i].pin_number);
    if (val)
      status |= 1U << i;
  }
  return status;
}

void printfail(uint16_t failed_mask, uint16_t supply_ok_mask, uint16_t supply_bitset)
{
  log_error(LOG_PWRCTL, "psfail: fail, supply_mask, bitset =  %x,%x,%x\r\n", failed_mask,
      supply_ok_mask, supply_bitset);
}

static const char * const power_system_state_names[] = {
    "FAIL", "INIT", "OFF", "L1ON", "L2ON", "L3ON", "L4ON", "L5ON", "ON",
};

const char *getPowerControlStateName(enum power_system_state s)
{
  return power_system_state_names[s];
}

// alarm from outside this task
static bool external_alarm = false;
const bool getPowerControlExternalAlarmState()
{
  return external_alarm;
}

// monitor and control the power supplies
void PowerSupplyTask(void *parameters)
{


  // compile-time sanity check
  static_assert(PS_ENS_MASK == (PS_ENS_GEN_MASK | PS_ENS_F2_MASK | PS_ENS_F1_MASK), "mask");
  static_assert(PS_OKS_MASK == (PS_OKS_GEN_MASK | PS_OKS_F2_MASK | PS_OKS_F1_MASK), "mask");

  // powerdown request from the CLI
  bool cli_powerdown_request = false;

  // masks to enable/check appropriate supplies
  uint16_t supply_ok_mask = PS_OKS_GEN_MASK;
  uint16_t supply_en_mask = PS_ENS_GEN_MASK;
  uint16_t supply_ok_mask_L1 = 0U, supply_ok_mask_L2 = 0U, supply_ok_mask_L4 = 0U,
           supply_ok_mask_L5 = 0U;

  bool f1_enable = isFPGAF1_PRESENT();
  bool f2_enable = isFPGAF2_PRESENT();
  // HACK
  // setting the enables both to true for debugging blank bo
  if ( ! f1_enable && ! f2_enable ) {
    f1_enable = true;
    f2_enable = true;
  }
  // end HACK
  if (f1_enable) {
    supply_ok_mask |= PS_OKS_F1_MASK;
    supply_en_mask |= PS_ENS_F1_MASK;
    supply_ok_mask_L1 = PS_OKS_F1_MASK_L1;
    supply_ok_mask_L2 = supply_ok_mask_L1 | PS_OKS_F1_MASK_L2;
    supply_ok_mask_L4 = supply_ok_mask_L2 | PS_OKS_F1_MASK_L4;
    supply_ok_mask_L5 = supply_ok_mask_L4 | PS_OKS_F1_MASK_L5;
  }
  if (f2_enable) {
    supply_ok_mask |= PS_OKS_F2_MASK;
    supply_en_mask |= PS_ENS_F2_MASK;
    supply_ok_mask_L1 |= PS_OKS_F2_MASK_L1;
    supply_ok_mask_L2 |= supply_ok_mask_L1 | PS_OKS_F2_MASK_L2;
    supply_ok_mask_L4 |= supply_ok_mask_L2 | PS_OKS_F2_MASK_L4;
    supply_ok_mask_L5 |= supply_ok_mask_L4 | PS_OKS_F2_MASK_L5;
  }
#if defined( ECN001) || defined (REV2)
  // configure the LGA80D supplies. This call takes some time.
  LGA80D_init();
#endif // ECN001

  bool power_supply_alarm = false;
  uint16_t failed_mask = 0x0U;

  // initialize to the current tick time *after the initialization*
  TickType_t xLastWakeTime = xTaskGetTickCount();
  // this loop never exits
  for (;;) {
    // first check for message on the queue and collect all messages.
    // non-blocking call.
    uint32_t message;
    if (xQueueReceive(xPwrQueue, &message, 0)) { // TODO: what about > 1 message
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
    bool ignorefail =false; // HACK THIS NEEDS TO BE FIXED TODO FIXME
    // Check the state of BLADE_POWER_EN.
    bool blade_power_enable = (read_gpio_pin(BLADE_POWER_EN) == 1);

    // now check the actual state of the power supplies
    uint16_t supply_bitset = check_ps_oks();
    bool supply_off = false; // are supplies off (besides the ones that are disabled)
    if ((supply_bitset & supply_ok_mask) != supply_ok_mask) {
      supply_off = true;
    }

    // MAIN POWER SUPPLY TASK STATE MACHINE
    // ON1 .. ON5 are the five states of the turn-on sequence
    // OFF1 .. OFF5 are the five states of the turn-off sequence
    // in the transition to FAIL we turn off all the supplies in sequence,
    // even if they were not yet tured on (i.e., transition from ON3 -> FAIL)
    //                     +-------------------+
    //              +------+       FAIL        +<-----+
    // +-------+    |      +--+-------------+--+      |
    // | INIT  |    |         ^             ^         |
    // +---+---+    v         |             |         |
    //     |     ---+--+   +--+-+         +-+--+  +---+--+
    //     +---->+ OFF +---> ON1+-> ....  | ON5+->+  ON  |
    //           +--+--+   +----+         +----+  +---+--+
    //              |                                 |
    //              +---------------------------------+

    // Nota Bene:
    // ON3 does not have a FAIL transition because those supplies
    // do not have a PWR_GOOD in Rev 1 of the CM

    // the state you are in is the current state, so for instance
    // there is no power_off transition in the power_off state;
    // instead it's in the POWER_ON state (e.g.)
    enum power_system_state nextState;
    switch (currentState) {
      case POWER_INIT: {
        // only run on first boot
        nextState = POWER_OFF;
        break;
      }
      case POWER_ON: {
        if (supply_off && !ignorefail) {
          // log erroring supplies
          failed_mask = (~supply_bitset) & supply_ok_mask;
          printfail(failed_mask, supply_ok_mask, supply_bitset);
          errbuffer_power_fail(failed_mask);
          // turn off all supplies
          disable_ps();
          power_supply_alarm = true;
          nextState = POWER_FAILURE;
        }
        else if (external_alarm) {
          log_info(LOG_PWRCTL, "external alarm power down\r\n");
          errbuffer_put(EBUF_POWER_OFF_TEMP, 0);
          // turn off all supplies
          disable_ps();
          nextState = POWER_FAILURE;
        }
        else if (!blade_power_enable || cli_powerdown_request) {
          log_info(LOG_PWRCTL, "power-down requested\r\n");
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
        // start power-on sequence
        if (blade_power_enable && !cli_powerdown_request && !external_alarm &&
            !power_supply_alarm) {
          log_info(LOG_PWRCTL, "power-up requested\r\n");
          turn_on_ps_at_prio(f2_enable, f1_enable, 1);
          errbuffer_put(EBUF_POWER_ON, 0);
          nextState = POWER_L1ON;
        }
        else {
          nextState = POWER_OFF;
        }
        break;
      }
      case POWER_L1ON: {
        if (((supply_bitset & supply_ok_mask_L1) != supply_ok_mask_L1) && !ignorefail) {
          failed_mask = (~supply_bitset) & supply_ok_mask_L1;
          printfail(failed_mask, supply_ok_mask_L1, supply_bitset);
          errbuffer_power_fail(failed_mask);
          disable_ps();
          power_supply_alarm = true;
          nextState = POWER_FAILURE;
        }
        else {
          turn_on_ps_at_prio(f2_enable, f1_enable, 2);
          nextState = POWER_L2ON;
        }

        break;
      }
      case POWER_L2ON: {
        if (((supply_bitset & supply_ok_mask_L2) != supply_ok_mask_L2) && !ignorefail){
          failed_mask = (~supply_bitset) & supply_ok_mask_L2;
          printfail(failed_mask, supply_ok_mask_L2, supply_bitset);
          errbuffer_power_fail(failed_mask);
          disable_ps();
          power_supply_alarm = true;
          nextState = POWER_FAILURE;
        }
        else {
          turn_on_ps_at_prio(f2_enable, f1_enable, 3);
          nextState = POWER_L3ON;
        }

        break;
      }
      case POWER_L3ON: { // FIXME allow this transition to fail on Rev2
        // NO ENABLES AT L3. We always go to L4.
        turn_on_ps_at_prio(f2_enable, f1_enable, 4);
        nextState = POWER_L4ON;

        break;
      }
      case POWER_L4ON: {
        if (((supply_bitset & supply_ok_mask_L4) != supply_ok_mask_L4) && !ignorefail) {
          failed_mask = (~supply_bitset) & supply_ok_mask_L4;
          printfail(failed_mask, supply_ok_mask_L4, supply_bitset);
          errbuffer_power_fail(failed_mask);

          disable_ps();
          power_supply_alarm = true;
          nextState = POWER_FAILURE;
        }
        else {
          turn_on_ps_at_prio(f2_enable, f1_enable, 5);
          nextState = POWER_L5ON;
        }

        break;
      }
      case POWER_L5ON: {
        if (((supply_bitset & supply_ok_mask_L5) != supply_ok_mask_L5)&& !ignorefail) {
          failed_mask = (~supply_bitset) & supply_ok_mask_L5;
          printfail(failed_mask, supply_ok_mask_L5, supply_bitset);
          errbuffer_power_fail(failed_mask);

          disable_ps();
          power_supply_alarm = true;
          nextState = POWER_FAILURE;
        }
        else {
          blade_power_ok(true);
          nextState = POWER_ON;
        }

        break;
      }
      case POWER_FAILURE: { // we go through POWER_OFF state before turning on.
        if (!power_supply_alarm && !external_alarm) { // errors cleared
          nextState = POWER_OFF;
          errbuffer_power_fail_clear();
        }
        else { // still in failed state
          nextState = POWER_FAILURE;
        }
        break;
      }
      default: {
        // configASSERT(1 == 0);
        nextState = POWER_INIT; // shut up debugger
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
          case POWER_INIT:
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
      log_debug(LOG_PWRCTL, "%s: change from state %s to %s\r\n", pcTaskGetName(NULL),
                power_system_state_names[currentState], power_system_state_names[nextState]);
    }
    currentState = nextState;

    // monitor stack usage for this task
    static UBaseType_t vv = 4096;
    CHECK_TASK_STACK_USAGE(vv);

    // wait here for the x msec, where x is 2nd argument below.
    vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(25));
  }
}
