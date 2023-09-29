/*
 * Semaphore.h
 *
 *  Created on: Semptember 06, 2022
 *      Author: pkotamnives
 */

#ifndef PROJECTS_CM_MCU_SEMAPHORE_H_
#define PROJECTS_CM_MCU_SEMAPHORE_H_

#include "FreeRTOS.h"
#include "semphr.h"

// Mutex for UART
extern SemaphoreHandle_t xUARTMutex;

extern SemaphoreHandle_t i2c1_sem;
extern SemaphoreHandle_t i2c2_sem;
extern SemaphoreHandle_t i2c3_sem;
extern SemaphoreHandle_t i2c4_sem;
extern SemaphoreHandle_t i2c5_sem;
extern SemaphoreHandle_t i2c6_sem;

void initSemaphores(void);

SemaphoreHandle_t getSemaphore(int number);

int acquireI2CSemaphoreTime(SemaphoreHandle_t s, TickType_t tickWaits);

#define acquireI2CSemaphore(s) \
  acquireI2CSemaphoreTime(s, 10)

#define acquireI2CSemaphoreBlock(s) \
  acquireI2CSemaphoreTime(s, 0)

#endif /* PROJECTS_CM_MCU_SEMAPHORE_H_ */
