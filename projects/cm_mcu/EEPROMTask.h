/*
 * EEPROMTask.h
 *
 *  Created on: Dec 3, 2019
 *      Author: glg62
 */

#ifndef PROJECTS_CM_MCU_EEPROMTASK_H_
#define PROJECTS_CM_MCU_EEPROMTASK_H_

#include "FreeRTOS.h"
#include "FreeRTOSConfig.h"

// define any relevant registers...

// EEPROM Queue
extern QueueHandle_t xEPRMQueue;
// messages (1st character)
#define EPRM_WRITE_SINGLE 1
#define EPRM_READ_SINGLE 2
#define EPRM_READ_DOUBLE 3
#define EPRM_SET_ID 4
#define EPRM_BUFF_IN 5
#define EPRM_BUFF_OUT 6 // ...

//	Functions
void EEPROMTask(void *parameters);
uint64_t EEPROM_read();


#endif /* PROJECTS_CM_MCU_EEPROMTASK_H_ */
