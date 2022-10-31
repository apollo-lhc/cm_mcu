// *
// * FreeRTOS Kernel V10.2.0
// *

#ifndef FREERTOS_CONFIG_H
#define FREERTOS_CONFIG_H

#include "common/utils.h"
#include "common/printf.h"
#include "driverlib/sysctl.h"
#include "driverlib/rom.h"

#ifdef __cplusplus
extern "C" {
#endif

//  -----------------------------------------------------------
//  * Application specific definitions.
//  *
//  * These definitions should be adjusted for your particular hardware and
//  * application requirements.
//  *
//  * THESE PARAMETERS ARE DESCRIBED WITHIN THE 'CONFIGURATION' SECTION OF THE
//  * FreeRTOS API DOCUMENTATION AVAILABLE ON THE FreeRTOS.org WEB SITE.
//  *
//  * See http://www.freertos.org/a00110.html
//  *----------------------------------------------------------

uint32_t stopwatch_getticks(void);
void stopwatch_reset(void);

#define configUSE_PREEMPTION                    1
#define configUSE_TIME_SLICING                  1
#define configTICK_RATE_HZ                      (100)
#define configUSE_PORT_OPTIMISED_TASK_SELECTION 1
#define configUSE_QUEUE_SETS                    0
#define configCPU_CLOCK_HZ                      40000000
#define configMAX_PRIORITIES                    (5)
#define configMINIMAL_STACK_SIZE                ((unsigned short)256)
#define configTOTAL_HEAP_SIZE                   ((size_t)(36 * 1024))
#define configMAX_TASK_NAME_LEN                 (6)
#define configUSE_TRACE_FACILITY                1
#define configUSE_16_BIT_TICKS                  0
#define configIDLE_SHOULD_YIELD                 1
#define configUSE_MUTEXES                       1
#define configQUEUE_REGISTRY_SIZE               10
#define configCHECK_FOR_STACK_OVERFLOW          2
#define configUSE_RECURSIVE_MUTEXES             0
#define configUSE_APPLICATION_TASK_TAG          0
#define configUSE_COUNTING_SEMAPHORES           0
#define configUSE_TICKLESS_IDLE                 1
#define configNUM_THREAD_LOCAL_STORAGE_POINTERS 0

#define configUSE_MALLOC_FAILED_HOOK 1
#define configUSE_IDLE_HOOK          1
#define configUSE_TICK_HOOK          0

#define configSUPPORT_STATIC_ALLOCATION  0
#define configSUPPORT_DYNAMIC_ALLOCATION 1

/* Run time stats gathering definitions. */
#define configGENERATE_RUN_TIME_STATS            1
#define portCONFIGURE_TIMER_FOR_RUN_TIME_STATS() stopwatch_reset()
#define portGET_RUN_TIME_COUNTER_VALUE()         stopwatch_getticks()
#define configRECORD_STACK_HIGH_ADDRESS          1
#define configUSE_HEAP_SCHEME                                                                     \
  4 /* either 1 (only alloc), 2 (alloc/free), 3 (malloc), 4 (coalesc blocks), 5 (multiple blocks) \
     */
/* This demo makes use of one or more example stats formatting functions.  These
format the raw data provided by the uxTaskGetSystemState() function in to human
readable ASCII form.  See the notes in the implementation of vTaskList() within
FreeRTOS/Source/tasks.c for limitations. */
#define configUSE_STATS_FORMATTING_FUNCTIONS 1

/* Co-routine definitions. */
#define configUSE_CO_ROUTINES           0
#define configMAX_CO_ROUTINE_PRIORITIES (2)

/* Software timer definitions. */
#define configUSE_TIMERS             0
#define configTIMER_TASK_PRIORITY    (2)
#define configTIMER_QUEUE_LENGTH     5
#define configTIMER_TASK_STACK_DEPTH (configMINIMAL_STACK_SIZE)

/* Set the following definitions to 1 to include the API function, or zero
to exclude the API function. */
#define INCLUDE_vTaskPrioritySet            0
#define INCLUDE_uxTaskPriorityGet           0
#define INCLUDE_vTaskDelete                 1
#define INCLUDE_vTaskCleanUpResources       0
#define INCLUDE_vTaskSuspend                1
#define INCLUDE_vTaskDelayUntil             1
#define INCLUDE_vTaskDelay                  1
#define INCLUDE_eTaskGetState               0
#define INCLUDE_xTimerPendFunctionCall      0
#define INCLUDE_xSemaphoreGetMutexHolder    1
#define INCLUDE_xTaskGetHandle              0
#define INCLUDE_xTaskGetCurrentTaskHandle   1
#define INCLUDE_xTaskGetIdleTaskHandle      0
#define INCLUDE_xTaskAbortDelay             0
#define INCLUDE_xTaskGetSchedulerState      0
#define INCLUDE_xTaskGetIdleTaskHandle      0
#define INCLUDE_uxTaskGetStackHighWaterMark 1

/* Cortex-M specific definitions. */
#ifdef __NVIC_PRIO_BITS
/* __BVIC_PRIO_BITS will be specified when CMSIS is being used. */
#define configPRIO_BITS __NVIC_PRIO_BITS
#else
#define configPRIO_BITS 3 /* 7 priority levels */
#endif

/* The lowest interrupt priority that can be used in a call to a "set priority"
function. */
#define configLIBRARY_LOWEST_INTERRUPT_PRIORITY 0x7

/* The highest interrupt priority that can be used by any interrupt service
routine that makes calls to interrupt safe FreeRTOS API functions.  DO NOT CALL
INTERRUPT SAFE FREERTOS API FUNCTIONS FROM ANY INTERRUPT THAT HAS A HIGHER
PRIORITY THAN THIS! (higher priorities are lower numeric values. */
#define configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY 5

/* Interrupt priorities used by the kernel port layer itself.  These are generic
to all Cortex-M ports, and do not rely on any particular library functions. */
#define configKERNEL_INTERRUPT_PRIORITY \
  (configLIBRARY_LOWEST_INTERRUPT_PRIORITY << (8 - configPRIO_BITS))
/* !!!! configMAX_SYSCALL_INTERRUPT_PRIORITY must not be set to zero !!!!
See http://www.FreeRTOS.org/RTOS-Cortex-M3-M4.html. */
#define configMAX_SYSCALL_INTERRUPT_PRIORITY \
  (configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY << (8 - configPRIO_BITS))

/* Normal assert() semantics without relying on the provision of an assert.h
header file. */

// Various utilities for accessing pointers, etc
// This returns the link register; it's a GCC builtin
#define GET_LR() __builtin_return_address(0)
// This is ARM and GCC specific syntax and returns the program counter
#define GET_PC(_a) __asm volatile("mov %0, pc" \
                                  : "=r"(_a))
// void apollo_log_assert(void *pc, void * lr);
// const void *lr = GET_LR();

// assert handling
#define APOLLO_ASSERT_RECORD()                                        \
  volatile void *pc;                                                  \
  GET_PC(pc);                                                         \
  errbuffer_put_raw(EBUF_ASSERT, ((uint32_t)pc >> 24) & 0xFFU);       \
  errbuffer_put_raw(EBUF_CONTINUATION, ((uint32_t)pc >> 16) & 0xFFU); \
  errbuffer_put_raw(EBUF_CONTINUATION, ((uint32_t)pc >> 8) & 0xFFU);  \
  errbuffer_put_raw(EBUF_CONTINUATION, (uint32_t)pc & 0xFFU)

#ifdef DEBUG
#define APOLLO_ASSERT(exp)    \
  if (!(exp)) {               \
    taskDISABLE_INTERRUPTS(); \
    APOLLO_ASSERT_RECORD();   \
    for (;;)                  \
      ;                       \
  }
#else
#define APOLLO_ASSERT(exp)    \
  if (!(exp)) {               \
    taskDISABLE_INTERRUPTS(); \
    APOLLO_ASSERT_RECORD();   \
    ROM_SysCtlReset();        \
  }
#endif

#define configASSERT(x) APOLLO_ASSERT((x))

// non-standard, park this here for now
#ifdef REV1
#define FP_UART UART4_BASE // Front panel UART
#define ZQ_UART UART1_BASE // Zynq-facing UART
#elif defined(REV2)
#define ZQ_UART UART0_BASE // single UART in Rev 2
#endif

#define SYSTEM_STACK_SIZE 128
// these need thought re Rev1/Rev2
#ifndef NO_ECN001
#define ECN001
#endif // NO_ECN001

#define pdTICKS_TO_MS(xTicks) ((TickType_t)(xTicks) * (1000u / configTICK_RATE_HZ))
#define pdTICKS_TO_S(xTicks)  ((TickType_t)(xTicks) / configTICK_RATE_HZ)

#ifdef __cplusplus
}
#endif

#endif /* FREERTOS_CONFIG_H */
