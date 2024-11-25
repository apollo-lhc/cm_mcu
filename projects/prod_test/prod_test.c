//*****************************************************************************
// production testing firmware 
//*****************************************************************************

#include <stdint.h>
#include <stdbool.h>

// TI includes
#include "inc/hw_memmap.h"
#include "driverlib/gpio.h"
#include "driverlib/sysctl.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"

// local includes
#include "common/pinout.h"

//*****************************************************************************
//
// The error routine that is called if the driver library encounters an error.
//
//*****************************************************************************
#ifdef DEBUG
void
__error__(char *pcFilename, uint32_t ui32Line)
{
  while(1);
}
#endif

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
    taskDISABLE_INTERRUPTS(); \
    APOLLO_ASSERT_RECORD();   \
    ROM_SysCtlReset();        \
    __builtin_unreachable();  \
  }
#endif

#define configASSERT(x) APOLLO_ASSERT((x))

uint32_t g_ui32SysClock;

void SystemInit(void)
{
  //
  // Run from the PLL, internal oscillator, at the defined clock speed configCPU_CLOCK_HZ
  // This function does not exist in the ROM
  g_ui32SysClock = MAP_SysCtlClockFreqSet((SYSCTL_OSC_INT | SYSCTL_USE_PLL | SYSCTL_CFG_VCO_320),
                                          configCPU_CLOCK_HZ);

  // initialize all pins, using file setup by TI PINMUX tool
  PinoutSet();
}


//*****************************************************************************
//
// main entry point
//
//*****************************************************************************
__attribute__((noreturn)) int main(void)
{
    SystemInit();


  //
  // Loop forever.
  //
  while(1) {
    // do something.     
    // Delay for a bit.
    //
    for(uint32_t ui32Loop = 0; ui32Loop < 200000; ui32Loop++) {
    }

    
  }
}
