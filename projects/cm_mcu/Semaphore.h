/*
 * Semaphore.h
 *
 *  Created on: Semptember 06, 2022
 *      Author: pkotamnives
 */

#ifndef PROJECTS_CM_MCU_SEMAPHORE_H_
#define PROJECTS_CM_MCU_SEMAPHORE_H_

#include "FreeRTOS.h" // IWYU pragma: keep
#include "semphr.h"

// Mutex for UART
extern SemaphoreHandle_t xUARTMutex;

extern SemaphoreHandle_t i2c1_sem;
extern SemaphoreHandle_t i2c2_sem;
extern SemaphoreHandle_t i2c3_sem;
extern SemaphoreHandle_t i2c4_sem;
extern SemaphoreHandle_t i2c5_sem;
extern SemaphoreHandle_t i2c6_sem;

// semaphore for the log_XXX functions
extern SemaphoreHandle_t log_sem;

bool vGiveOrTakeSemaphore(bool take, void *sem);

void initSemaphores(void);

SemaphoreHandle_t getSemaphore(int number);

#define SEM_ACCESS_ERROR (-5)

// wait timeWaitms miliseconds to acquire the semaphore. If wait time is portMAX_DELAY,
// block. Return pdTRUE if acquired, pdFAIL if not.
int acquireI2CSemaphoreTime(SemaphoreHandle_t s, uint32_t timeWaitms);

#define acquireI2CSemaphore(s) \
  acquireI2CSemaphoreTime(s, 50000) // 50 s wait time -- arbitrary/too long, but preserves existing behavior

// block until semaphore is available
#define acquireI2CSemaphoreBlock(s) acquireI2CSemaphoreTime(s, portMAX_DELAY)

#endif /* PROJECTS_CM_MCU_SEMAPHORE_H_ */
