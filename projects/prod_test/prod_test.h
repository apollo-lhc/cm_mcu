#ifndef PROD_TEST_HH
#define PROD_TEST_HH
#include <stdint.h>
#include <stdbool.h>

#include <sys/_intsup.h>
#define SYSTEM_STACK_SIZE 128
#define configCPU_CLOCK_HZ                      40000000
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
    __builtin_unreachable();  \
  }
#else
#define APOLLO_ASSERT(exp)    \
  if (!(exp)) {               \
    ROM_SysCtlReset();        \
    __builtin_unreachable();  \
  }
#endif

#define configASSERT(x) APOLLO_ASSERT((x))

extern uint32_t g_ui32SysClock;

#endif // PROD_TEST_HH
