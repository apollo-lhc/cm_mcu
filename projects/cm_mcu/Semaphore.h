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

/*
#define i2c1_sem xSemaphoreCreateMutex()
#define i2c2_sem xSemaphoreCreateMutex()
#define i2c3_sem xSemaphoreCreateMutex()
#define i2c4_sem xSemaphoreCreateMutex()
#define i2c5_sem xSemaphoreCreateMutex()
#define i2c6_sem xSemaphoreCreateMutex()
*/

extern SemaphoreHandle_t i2c1_sem;
extern SemaphoreHandle_t i2c2_sem;
extern SemaphoreHandle_t i2c3_sem;
extern SemaphoreHandle_t i2c4_sem;
extern SemaphoreHandle_t i2c5_sem;
extern SemaphoreHandle_t i2c6_sem;

/*
#define i2c1_sem
#define i2c2_sem
#define i2c3_sem
#define i2c4_sem
#define i2c5_sem
#define i2c6_sem
*/
void initSemaphores();

#endif /* PROJECTS_CM_MCU_SEMAPHORE_H_ */
