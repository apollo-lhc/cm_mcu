/*
 * CommandLineTask.h
 *
 *  Created on: Aug 22, 2019
 *      Author: pw94
 */

#ifndef PROJECTS_CM_MCU_COMMANDLINETASK_H_
#define PROJECTS_CM_MCU_COMMANDLINETASK_H_
// includes for types
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>

#include "FreeRTOS.h"
#include "FreeRTOSConfig.h"
#include "stream_buffer.h"

// Command line interface
void vCommandLineTask(void *pvParameters);

typedef struct {
  StreamBufferHandle_t UartStreamBuffer;
  uint32_t uart_base;
  UBaseType_t stack_size;
} CommandLineTaskArgs_t;

#endif /* PROJECTS_CM_MCU_COMMANDLINETASK_H_ */
