/*
 * Semaphore.c
 *
 *  Created on: September 06, 2022
 *      Author: pkotamnives
 *      declare and create semaphores utilized across the cm_mcu project
 */

#include "Semaphore.h"

SemaphoreHandle_t xUARTMutex = 0;
SemaphoreHandle_t i2c1_sem = 0;
SemaphoreHandle_t i2c2_sem = 0;
SemaphoreHandle_t i2c3_sem = 0;
SemaphoreHandle_t i2c4_sem = 0;
SemaphoreHandle_t i2c5_sem = 0;
SemaphoreHandle_t i2c6_sem = 0;
SemaphoreHandle_t log_sem = 0;

void initSemaphores(void)
{
  xUARTMutex = xSemaphoreCreateMutex();
  i2c1_sem = xSemaphoreCreateMutex();
  i2c2_sem = xSemaphoreCreateMutex();
  i2c3_sem = xSemaphoreCreateMutex();
  i2c4_sem = xSemaphoreCreateMutex();
  i2c5_sem = xSemaphoreCreateMutex();
  i2c6_sem = xSemaphoreCreateMutex();
  log_sem = xSemaphoreCreateMutex();
}

SemaphoreHandle_t getSemaphore(int number)
{
  SemaphoreHandle_t s;
  switch (number) {
    case 1:
      s = i2c1_sem;
      break;
    case 2:
      s = i2c2_sem;
      break;
    case 3:
      s = i2c3_sem;
      break;
    case 4:
      s = i2c4_sem;
      break;
    case 5:
      s = i2c5_sem;
      break;
    case 6:
      s = i2c6_sem;
      break;
    default:
      s = 0;
      break;
  }
  return s;
}

// wait timeWaitms miliseconds to acquire the semaphore.
// block if timeWaitms is portMAX_DELAY. Return pdTRUE if acquired, pdFAIL if not.
int acquireI2CSemaphoreTime(SemaphoreHandle_t s, uint32_t timeWaitms)
{
  if (s == NULL) {
    return pdFAIL;
  }
  TickType_t ticks = (timeWaitms == portMAX_DELAY) ? portMAX_DELAY : pdMS_TO_TICKS(timeWaitms) + 1;
  return xSemaphoreTake(s, ticks) == pdFALSE? pdFAIL: pdTRUE;
}

// Used as the log_set_lock() callback. take==true acquires, false releases.
// Returns whether the requested operation succeeded; the logging code uses a
// false return on acquire to drop a message rather than block. Acquire is
// non-blocking (0 tick wait): logging must never stall a task on the lock.
bool vGiveOrTakeSemaphore(bool take, void *sem)
{
  SemaphoreHandle_t s = (SemaphoreHandle_t)sem;
  if (s == NULL) {
    return true; // no lock configured -> proceed
  }
  if (take) {
    return xSemaphoreTake(s, 0) == pdTRUE;
  }
  xSemaphoreGive(s);
  return true;
}
