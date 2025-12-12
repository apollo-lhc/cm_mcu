#ifndef PROD_TEST_HH
#define PROD_TEST_HH
#include <stdint.h>
#include <stdbool.h>

#define SYSTEM_STACK_SIZE  128
#define configCPU_CLOCK_HZ 40000000

extern uint32_t g_ui32SysClock;

const char* buildTime(void);
const char* gitVersion(void);

#endif // PROD_TEST_HH
