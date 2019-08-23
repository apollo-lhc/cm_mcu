/*
 * CommandLineTask.h
 *
 *  Created on: Aug 22, 2019
 *      Author: pw94
 */

#ifndef PROJECTS_PROJECT2_COMMANDLINETASK_H_
#define PROJECTS_PROJECT2_COMMANDLINETASK_H_
// includes for types
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>

#include "FreeRTOS.h"
#include "FreeRTOSConfig.h"
#include "stream_buffer.h"

// Command line interface
void vCommandLineTask(void *parameters);


typedef struct {
  StreamBufferHandle_t UartStreamBuffer;
  uint32_t uart_base;
} CommandLineArgs_t;

#endif /* PROJECTS_PROJECT2_COMMANDLINETASK_H_ */
