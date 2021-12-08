// watchdog
// Author: wittich
// Watchdog timer interface
// code adapted from a Memfault blog post (thanks!)
// 5/2021
// Not live as of now -- just prints to STDOUT

// includes for types
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

// driverlib
#include "driverlib/watchdog.h"
#include "inc/hw_memmap.h"

//
#include "Tasks.h"

// FreeRTOS
#include "FreeRTOS.h"
#include "task.h"

#include "common/log.h"


static uint16_t s_registered_tasks = 0;
static uint16_t s_fed_tasks = 0;

// feed the hardware watchdog
void hardware_watchdog_feed(void)
{
  const uint32_t reload_magic_value = 0x6E524635;
  ROM_WatchdogReloadSet(WATCHDOG0_BASE, reload_magic_value);
}

static void prv_task_watchdog_check(void)
{
  if ((s_fed_tasks & s_registered_tasks) == s_registered_tasks) {
    // all the tasks have been fed or are idle!
    //hardware_watchdog_feed();
    s_fed_tasks = 0;
  }
  else {
    log_trace(LOG_SERVICE, "%s: watchdog failed mask: 0x%x\r\n", __func__, s_registered_tasks);
  }
}

void task_watchdog_register_task(uint16_t task_id)
{
  taskDISABLE_INTERRUPTS();

  s_registered_tasks |= (1 << task_id);

  taskENABLE_INTERRUPTS();
}

void task_watchdog_unregister_task(uint16_t task_id)
{
  taskDISABLE_INTERRUPTS();

  s_registered_tasks &= ~(1 << task_id);
  s_fed_tasks &= ~(1 << task_id);

  taskENABLE_INTERRUPTS();
}

void task_watchdog_feed_task(uint16_t task_id)
{
  taskDISABLE_INTERRUPTS();

  s_fed_tasks |= (1 << task_id);

  taskENABLE_INTERRUPTS();
}

uint16_t task_watchdog_get_status()
{
  return s_fed_tasks & s_registered_tasks;
}

void WatchdogTask(void *parameters)
{
  //
  // initialize the update tick
  TickType_t wd_updateTick = xTaskGetTickCount();
  vTaskDelayUntil(&wd_updateTick, pdMS_TO_TICKS(5000));

  for (;;) {
    prv_task_watchdog_check();

    // monitor stack usage for this task
    UBaseType_t val = uxTaskGetStackHighWaterMark(NULL);
    static UBaseType_t vv = 4096;
    if (val < vv) {
      log_info(LOG_SERVICE, "stack (%s) = %d(was %d)\r\n", pcTaskGetName(NULL), val, vv);
    }
    vv = val;

    vTaskDelayUntil(&wd_updateTick, pdMS_TO_TICKS(1000));
  }
}
