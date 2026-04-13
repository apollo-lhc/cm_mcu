/*
 * LedTask.c
 *
 *  Created on: May 13, 2019
 *      Author: wittich
 */

// includes for types
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

// local includes
#include "common/utils.h"
#include "common/pinsel.h"
#include "common/log.h"
#include "Tasks.h"

// FreeRTOS includes
#include "FreeRTOS.h"
#include "FreeRTOSConfig.h"
#include "queue.h"

// Predefined LED status states
const LedMsg_t LED_STATUS_INIT = {LED_PAT_OFF, LED_PAT_OFF, LED_PAT_ON};                // blue solid
const LedMsg_t LED_STATUS_PS_OFF = {LED_PAT_OFF, LED_PAT_OFF, LED_PAT_TOGGLE4};         // blue slow blink
const LedMsg_t LED_STATUS_NORMAL = {LED_PAT_OFF, LED_PAT_ON, LED_PAT_OFF};              // green solid
const LedMsg_t LED_STATUS_PS_LOADING = {LED_PAT_OFF, LED_PAT_TOGGLE4, LED_PAT_TOGGLE4}; // cyan slow blink (G+B)
const LedMsg_t LED_STATUS_WARN = {LED_PAT_TOGGLE3, LED_PAT_OFF, LED_PAT_OFF};           // red medium blink (temperature warning)
const LedMsg_t LED_STATUS_ALARM = {LED_PAT_ON, LED_PAT_OFF, LED_PAT_OFF};               // red solid
const LedMsg_t LED_STATUS_PS_FAULT = {LED_PAT_TOGGLE, LED_PAT_OFF, LED_PAT_OFF};        // red fast blink
const LedMsg_t LED_STATUS_FW_FAULT = {LED_PAT_TOGGLE, LED_PAT_OFF, LED_PAT_TOGGLE};     // magenta fast blink
const LedMsg_t LED_STATUS_BOOTLOADER = {LED_PAT_ON, LED_PAT_ON, LED_PAT_ON};            // white solid

// Holds the handle of the created queue for the LED task.
// gets initialized in main()
QueueHandle_t xLedQueue = NULL;

static void apply_pattern(enum LEDpattern pattern, uint32_t pin, uint32_t callcnt)
{
  if (pattern == LED_PAT_OFF)
    write_gpio_pin(pin, 0x0);
  else if (pattern == LED_PAT_ON)
    write_gpio_pin(pin, 0x1);
  else
    // Blink: first half-period ON, second half-period OFF.
    // All channels with the same period and callcnt are written the same
    // value, so multi-channel states (e.g. yellow = R+G) are always in phase.
    write_gpio_pin(pin, ((callcnt / (uint32_t)pattern) % 2 == 0) ? 0x1 : 0x0);
}

void LedTask(void *parameters)
{
  TickType_t xLastWakeTime = xTaskGetTickCount();
  uint32_t callcnt = 0;
  LedMsg_t cur = {LED_PAT_OFF, LED_PAT_OFF, LED_PAT_OFF};

  // this function never returns. Loop here forever.
  for (;;) {
    LedMsg_t msg;
    if (xQueueReceive(xLedQueue, &msg, 0)) {
      if (msg.red != cur.red || msg.green != cur.green || msg.blue != cur.blue) {
        cur = msg;
        callcnt = 0; // reset so toggling channels start in phase
      }
    }

    apply_pattern(cur.red, MCU_LED_RED, callcnt);
    apply_pattern(cur.green, MCU_LED_GREEN, callcnt);
    apply_pattern(cur.blue, MCU_LED_BLUE, callcnt);
    ++callcnt;

    // monitor stack usage for this task
    static UBaseType_t vv = 4096;
    CHECK_TASK_STACK_USAGE(vv);

    // wait for next check
    vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(250));
  }
}
