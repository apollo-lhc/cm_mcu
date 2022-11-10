/*
 * Semaphore.c
 *
 *  Created on: September 06, 2022
 *      Author: pkotamnives
 *      declare and create semaphores utilized aross the cm_mcu project
 */

#include "Semaphore.h"

SemaphoreHandle_t xUARTMutex = 0;
SemaphoreHandle_t i2c1_sem = 0;
SemaphoreHandle_t i2c2_sem = 0;
SemaphoreHandle_t i2c3_sem = 0;
SemaphoreHandle_t i2c4_sem = 0;
SemaphoreHandle_t i2c5_sem = 0;
SemaphoreHandle_t i2c6_sem = 0;

void initSemaphores(void)
{
  xUARTMutex = xSemaphoreCreateMutex();
  i2c1_sem = xSemaphoreCreateMutex();
  i2c2_sem = xSemaphoreCreateMutex();
  i2c3_sem = xSemaphoreCreateMutex();
  i2c4_sem = xSemaphoreCreateMutex();
  i2c5_sem = xSemaphoreCreateMutex();
  i2c6_sem = xSemaphoreCreateMutex();
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

#define MAX_TRIES 50
int acquireI2CSemaphore(SemaphoreHandle_t s)
{
  int retval = pdTRUE;
  if (s == NULL) {
    return pdFAIL;
  }
  int tries = 0;
  while (xSemaphoreTake(s, (TickType_t)10) == pdFALSE) {
    ++tries;
    if (tries > MAX_TRIES) {
      retval = pdFAIL;
      break;
    }
  }
  return retval;
}

