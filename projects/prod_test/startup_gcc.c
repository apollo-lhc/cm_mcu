//*****************************************************************************
//
// startup_gcc.c - Startup code for use with GNU tools.
//
// Copyright (c) 2013-2017 Texas Instruments Incorporated.  All rights reserved.
// Software License Agreement
//
// Texas Instruments (TI) is supplying this software for use solely and
// exclusively on TI's microcontroller products. The software is owned by
// TI and/or its suppliers, and is protected under applicable copyright
// laws. You may not combine this software with "viral" open-source
// software in order to form a larger program.
//
// THIS SOFTWARE IS PROVIDED "AS IS" AND WITH ALL FAULTS.
// NO WARRANTIES, WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT
// NOT LIMITED TO, IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE. TI SHALL NOT, UNDER ANY
// CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL, OR CONSEQUENTIAL
// DAMAGES, FOR ANY REASON WHATSOEVER.
//
// This is part of revision 2.1.4.178 of the EK-TM4C1294XL Firmware Package.
//
//*****************************************************************************

#include <stdint.h>
#include "inc/hw_nvic.h"
#include "inc/hw_types.h"
#include "driverlib/rom.h"
#include "InterruptHandlers.h"
#include "FreeRTOS.h" // IWYU pragma: keep
#include "FreeRTOSConfig.h"
#include "task.h" // IWYU pragma: keep
//*****************************************************************************
//
//  Forward declaration of the default fault handlers.
//
//*****************************************************************************
void ResetISR(void);
static void NmiSR(void);
static void IntDefaultHandler(void);

/* The prototype shows it is a naked function - in effect this is just an
assembly function. */
static void HardFault_Handler(void) __attribute__((naked, aligned(8)));
//*****************************************************************************
//
// The entry point for the application.
//
//*****************************************************************************
extern int main(void);

//*****************************************************************************
//
// Reserve space for the system stack.
//
//*****************************************************************************
static uint32_t pui32Stack[SYSTEM_STACK_SIZE];

const uint32_t *getSystemStack(void)
{
  return pui32Stack;
}

//*****************************************************************************
//
// The vector table.  Note that the proper constructs must be placed on this to
// ensure that it ends up at physical address 0x0000.0000.
//
//*****************************************************************************
__attribute__((section(".isr_vector"))) void (*const g_pfnVectors[])(void) = {
    (void (*)(void))((uint32_t)pui32Stack + sizeof(pui32Stack)),
    // The initial stack pointer
    ResetISR,               // The reset handler
    NmiSR,                  // The NMI handler
    HardFault_Handler,      // The hard fault handler -- was FaultISR
    IntDefaultHandler,      // The MPU fault handler
    IntDefaultHandler,      // The bus fault handler
    IntDefaultHandler,      // The usage fault handler
    0,                      // Reserved
    0,                      // Reserved
    0,                      // Reserved
    0,                      // Reserved
    vPortSVCHandler,        // SVCall handler
    IntDefaultHandler,      // Debug monitor handler
    0,                      // Reserved
    xPortPendSVHandler,     // The PendSV handler
    xPortSysTickHandler,    // The SysTick handler
    IntDefaultHandler,      // GPIO Port A
    IntDefaultHandler,      // GPIO Port B
    IntDefaultHandler,      // GPIO Port C
    IntDefaultHandler,      // GPIO Port D
    IntDefaultHandler,      // GPIO Port E
    UART0IntHandler,        // UART0 Rx and Tx
    IntDefaultHandler,      // UART1 Rx and Tx
    IntDefaultHandler,      // SSI0 Rx and Tx
    IntDefaultHandler,      // I2C0 Master and Slave
    IntDefaultHandler,      // PWM Fault
    IntDefaultHandler,      // PWM Generator 0
    IntDefaultHandler,      // PWM Generator 1
    IntDefaultHandler,      // PWM Generator 2
    IntDefaultHandler,      // Quadrature Encoder 0
    IntDefaultHandler,      // ADC Sequence 0
    ADCSeq1Interrupt,       // ADC Sequence 1
    IntDefaultHandler,      // ADC Sequence 2
    IntDefaultHandler,      // ADC Sequence 3
    IntDefaultHandler,      // Watchdog timer
    IntDefaultHandler,      // Timer 0 subtimer A
    IntDefaultHandler,      // Timer 0 subtimer B
    IntDefaultHandler,      // Timer 1 subtimer A
    IntDefaultHandler,      // Timer 1 subtimer B
    IntDefaultHandler,      // Timer 2 subtimer A
    IntDefaultHandler,      // Timer 2 subtimer B
    IntDefaultHandler,      // Analog Comparator 0
    IntDefaultHandler,      // Analog Comparator 1
    IntDefaultHandler,      // Analog Comparator 2
    IntDefaultHandler,      // System Control (PLL, OSC, BO)
    IntDefaultHandler,      // FLASH Control
    IntDefaultHandler,      // GPIO Port F
    IntDefaultHandler,      // GPIO Port G
    IntDefaultHandler,      // GPIO Port H
    IntDefaultHandler,      // UART2 Rx and Tx
    IntDefaultHandler,      // SSI1 Rx and Tx
    IntDefaultHandler,      // Timer 3 subtimer A
    IntDefaultHandler,      // Timer 3 subtimer B
    SMBusMasterIntHandler1, // I2C1 Master and Slave
    IntDefaultHandler,      // CAN0
    IntDefaultHandler,      // CAN1
    IntDefaultHandler,      // Ethernet
    IntDefaultHandler,      // Hibernate
    IntDefaultHandler,      // USB0
    IntDefaultHandler,      // PWM Generator 3
    IntDefaultHandler,      // uDMA Software Transfer
    IntDefaultHandler,      // uDMA Error
    ADCSeq0Interrupt,       // ADC1 Sequence 0
    IntDefaultHandler,      // ADC1 Sequence 1
    IntDefaultHandler,      // ADC1 Sequence 2
    IntDefaultHandler,      // ADC1 Sequence 3
    IntDefaultHandler,      // External Bus Interface 0
    IntDefaultHandler,      // GPIO Port J
    IntDefaultHandler,      // GPIO Port K
    IntDefaultHandler,      // GPIO Port L
    IntDefaultHandler,      // SSI2 Rx and Tx
    IntDefaultHandler,      // SSI3 Rx and Tx
    IntDefaultHandler,      // UART3 Rx and Tx
    IntDefaultHandler,      // UART4 Rx and Tx
    IntDefaultHandler,      // UART5 Rx and Tx
    IntDefaultHandler,      // UART6 Rx and Tx
    IntDefaultHandler,      // UART7 Rx and Tx
    SMBusMasterIntHandler2, // I2C2 Master and Slave
    SMBusMasterIntHandler3, // I2C3 Master and Slave
    IntDefaultHandler,      // Timer 4 subtimer A
    IntDefaultHandler,      // Timer 4 subtimer B
    IntDefaultHandler,      // Timer 5 subtimer A
    IntDefaultHandler,      // Timer 5 subtimer B
    IntDefaultHandler,      // FPU
    0,                      // Reserved
    0,                      // Reserved
    SMBusMasterIntHandler4, // I2C4 Master and Slave
    SMBusMasterIntHandler5, // I2C5 Master and Slave
    IntDefaultHandler,      // GPIO Port M
    IntDefaultHandler,      // GPIO Port N
    0,                      // Reserved
    IntDefaultHandler,      // Tamper
    IntDefaultHandler,      // GPIO Port P (Summary or P0)
    IntDefaultHandler,      // GPIO Port P1
    IntDefaultHandler,      // GPIO Port P2
    IntDefaultHandler,      // GPIO Port P3
    IntDefaultHandler,      // GPIO Port P4
    IntDefaultHandler,      // GPIO Port P5
    IntDefaultHandler,      // GPIO Port P6
    IntDefaultHandler,      // GPIO Port P7
    IntDefaultHandler,      // GPIO Port Q (Summary or Q0)
    IntDefaultHandler,      // GPIO Port Q1
    IntDefaultHandler,      // GPIO Port Q2
    IntDefaultHandler,      // GPIO Port Q3
    IntDefaultHandler,      // GPIO Port Q4
    IntDefaultHandler,      // GPIO Port Q5
    IntDefaultHandler,      // GPIO Port Q6
    IntDefaultHandler,      // GPIO Port Q7
    IntDefaultHandler,      // GPIO Port R
    IntDefaultHandler,      // GPIO Port S
    IntDefaultHandler,      // SHA/MD5 0
    IntDefaultHandler,      // AES 0
    IntDefaultHandler,      // DES3DES 0
    IntDefaultHandler,      // LCD Controller 0
    IntDefaultHandler,      // Timer 6 subtimer A
    IntDefaultHandler,      // Timer 6 subtimer B
    IntDefaultHandler,      // Timer 7 subtimer A
    IntDefaultHandler,      // Timer 7 subtimer B
    IntDefaultHandler,      // I2C6 Master and Slave
    IntDefaultHandler,      // I2C7 Master and Slave
    IntDefaultHandler,      // HIM Scan Matrix Keyboard 0
    IntDefaultHandler,      // One Wire 0
    IntDefaultHandler,      // HIM PS/2 0
    IntDefaultHandler,      // HIM LED Sequencer 0
    IntDefaultHandler,      // HIM Consumer IR 0
    IntDefaultHandler,      // I2C8 Master and Slave
    IntDefaultHandler,      // I2C9 Master and Slave
    IntDefaultHandler       // GPIO Port T
};

//*****************************************************************************
//
// The following are constructs created by the linker, indicating where the
// the "data" and "bss" segments reside in memory.  The initializers for the
// for the "data" segment resides immediately following the "text" segment.
//
//*****************************************************************************
extern uint32_t _ldata;
extern uint32_t _data;
extern uint32_t _edata;
extern uint32_t _bss;
extern uint32_t _ebss;

//*****************************************************************************
//
// This is the code that gets called when the processor first starts execution
// following a reset event.  Only the absolutely necessary set is performed,
// after which the application supplied entry() routine is called.  Any fancy
// actions (such as making decisions based on the reset cause register, and
// resetting the bits in that register) are left solely in the hands of the
// application.
//
//*****************************************************************************
void ResetISR(void)
{
  uint32_t *pui32Src;
  uint32_t *pui32Dest;

  //
  // Copy the data segment initializers from flash to SRAM.
  //
  pui32Src = &_ldata;
  for (pui32Dest = &_data; pui32Dest < &_edata;) {
    *pui32Dest++ = *pui32Src++;
  }
  //
  // Zero fill the bss segment.
  //
  __asm("    ldr     r0, =_bss\n"
        "    ldr     r1, =_ebss\n"
        "    mov     r2, #0\n"
        "    .thumb_func\n"
        "zero_loop:\n"
        "        cmp     r0, r1\n"
        "        it      lt\n"
        "        strlt   r2, [r0], #4\n"
        "        blt     zero_loop");
  // Put a canary on the system stack
  pui32Dest = pui32Stack;
  for (; pui32Dest < (pui32Stack + sizeof(pui32Stack) / sizeof(uint32_t));) {
    *pui32Dest++ = 0xDEADBEEFUL;
  }

  //
  // Enable the floating-point unit.  This must be done here to handle the
  // case where main() uses floating-point and the function prologue saves
  // floating-point registers (which will fault if floating-point is not
  // enabled).  Any configuration of the floating-point unit using DriverLib
  // APIs must be done here prior to the floating-point unit being enabled.
  //
  ROM_FPULazyStackingEnable();
  // Note that this does not use DriverLib since it might not be included in
  // this project.
  //
  HWREG(NVIC_CPAC) = ((HWREG(NVIC_CPAC) & ~(NVIC_CPAC_CP10_M | NVIC_CPAC_CP11_M)) |
                      NVIC_CPAC_CP10_FULL | NVIC_CPAC_CP11_FULL);

  //
  // Call the application's entry point.
  //
  main();
}

//*****************************************************************************
//
// This is the code that gets called when the processor receives a NMI.  This
// simply enters an infinite loop, preserving the system state for examination
// by a debugger.
//
//*****************************************************************************
static void NmiSR(void)
{
  //
  // Enter an infinite loop.
  //
  configASSERT(1 == 0);
}
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-but-set-variable"
#pragma GCC diagnostic ignored "-Wunused-function"

//*****************************************************************************
//
// This is the code that gets called when the processor receives a fault
// interrupt.  This simply enters an infinite loop, preserving the system state
// for examination by a debugger, except in production code, where it just
// invokes a restart.
//
//*****************************************************************************
void GetRegistersFromStack(const uint32_t *pulFaultStackAddress) //
{
  /* These are volatile to try and prevent the compiler/linker optimizing them
away as the variables never actually get used.  If the debugger won't show the
values of the variables, make them global by moving their declaration outside
of this function. */
  volatile uint32_t r0;
  volatile uint32_t r1;
  volatile uint32_t r2;
  volatile uint32_t r3;
  volatile uint32_t r12;
  volatile uint32_t lr;  /* Link register. */
  volatile uint32_t pc;  /* Program counter. */
  volatile uint32_t psr; /* Program status register. */

  r0 = pulFaultStackAddress[0];
  r1 = pulFaultStackAddress[1];
  r2 = pulFaultStackAddress[2];
  r3 = pulFaultStackAddress[3];

  r12 = pulFaultStackAddress[4];
  lr = pulFaultStackAddress[5];
  pc = pulFaultStackAddress[6];
  psr = pulFaultStackAddress[7];
  // save the originating interrupt
  // errbuffer_put_raw(EBUF_HARDFAULT, (uint16_t)psr & 0xfUL);

#ifdef DEBUG
  /* When the following line is hit, the variables contain the register values. */
  for (;;)
    ;
#else
  ROM_SysCtlReset();
#endif // DEBUG
}

/*
 * The fault handler implementation calls a function called
 * GetRegistersFromStack().
 */
static void HardFault_Handler(void)
{
  __asm volatile(" tst lr, #4                                                \n"
                 " ite eq                                                    \n"
                 " mrseq r0, msp                                             \n"
                 " mrsne r0, psp                                             \n"
                 " ldr r1, [r0, #24]                                         \n"
                 " ldr r2, handler2_address_const                            \n"
                 " bx r2                                                     \n"
                 " handler2_address_const: .word GetRegistersFromStack    \n");
}
#pragma GCC diagnostic pop

//*****************************************************************************
//
// This is the code that gets called when the processor receives an unexpected
// interrupt.  This simply enters an infinite loop, preserving the system state
// for examination by a debugger.
//
//*****************************************************************************
static void IntDefaultHandler(void)
{
  // we should never get here
  APOLLO_ASSERT(0 == 1);
}
