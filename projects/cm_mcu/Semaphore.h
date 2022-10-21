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

// Mutex for UART -- should really have one for each UART
extern SemaphoreHandle_t xUARTMutex;

extern SemaphoreHandle_t i2c1_sem;
extern SemaphoreHandle_t i2c2_sem;
extern SemaphoreHandle_t i2c3_sem;
extern SemaphoreHandle_t i2c4_sem;
extern SemaphoreHandle_t i2c5_sem;
extern SemaphoreHandle_t i2c6_sem;

void initSemaphores(void);

#endif /* PROJECTS_CM_MCU_SEMAPHORE_H_ */
