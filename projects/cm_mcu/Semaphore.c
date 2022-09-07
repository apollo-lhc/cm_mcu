/*
 * Semaphore.c
 *
 *  Created on: September 06, 2022
 *      Author: pkotamnives
 *      declare and create semaphores utilized aross the cm_mcu project
 */

#include "FreeRTOS.h"
#include "semphr.h"
#include "Semaphore.h"


SemaphoreHandle_t i2c1_sem = 0;
SemaphoreHandle_t i2c2_sem = 0;
SemaphoreHandle_t i2c3_sem = 0;
SemaphoreHandle_t i2c4_sem = 0;
SemaphoreHandle_t i2c5_sem = 0;
SemaphoreHandle_t i2c6_sem = 0;

void initSemaphores()
{
  i2c1_sem = xSemaphoreCreateMutex();
  i2c2_sem = xSemaphoreCreateMutex();
  i2c3_sem = xSemaphoreCreateMutex();
  i2c4_sem = xSemaphoreCreateMutex();
  i2c5_sem = xSemaphoreCreateMutex();
  i2c6_sem = xSemaphoreCreateMutex();
}

