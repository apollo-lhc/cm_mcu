/*
 * ProgComTask.h
 *
 * Programmatic (non-CLI) UART interface for the Zynq to read and write
 * registers on devices only the MCU can reach. See ProgComTask.c for the
 * command grammar.
 */

#ifndef PROJECTS_CM_MCU_PROGCOMTASK_H_
#define PROJECTS_CM_MCU_PROGCOMTASK_H_

#include <stdint.h>

#include "FreeRTOS.h" // IWYU pragma: keep
#include "stream_buffer.h"

// Programmatic UART interface for Zynq to send commands to the CM. Not a CLI interface.
void ProgComTask(void *pvParameters);

typedef struct {
  StreamBufferHandle_t UartStreamBuffer;
  uint32_t uart_base;
  UBaseType_t stack_size;
} ProgComTaskArgs_t;

#endif /* PROJECTS_CM_MCU_PROGCOMTASK_H_ */
