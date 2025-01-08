/**
 * startup.c : TM4C startup code for use with the GNU Build System
 * (Will likely work with other Tiva/Stellaris boards as well)
 *
 * With credit to:
 *     Lukasz Janyst (bit.ly/2pxKw8x)
 *     TI's TivaWare
 *     The uctools Project (bit.ly/2oIRO9y)
 *
 * Author:   Rahul Butani
 * Modified: March 4th, 2019
 */
// from here:
// https://github.com/rrbutani/tm4c-llvm-toolchain

// Dependencies:
#include <stdint.h>

#include "driverlib/rom.h"
#include "inc/hw_types.h"
#include "inc/hw_nvic.h"

#include "FreeRTOSConfig.h"
#include "InterruptHandlers.h"

// Prototypes/Declarations:
static void IntDefaultHandler(void);

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

//
// Macros:

// Macro to create a weakly aliased placeholder interrupt that points to the
// default interrupt handler.
// This allows us to define proper strongly defined interrupt handlers
// anywhere in the project and have them override the default interrupt
// handler (taken care of by the linker) without us having to edit this file.
#define DEFINE_HANDLER(NAME) \
  void NAME##_handler(void) __attribute__((used, weak, alias("__default_int_handler")))

// Macro to generate function name of an aliased placeholder interrupt.
// (Generating these allows us to avoid hardcoding the function names)
#define HANDLER(NAME) NAME##_handler

// Define weakly aliased interrupt handlers:

// Reset is a special case:
void ResetISR(void) __attribute__((used, weak));

DEFINE_HANDLER(NmiSR);
DEFINE_HANDLER(HardFault);
DEFINE_HANDLER(mman);
DEFINE_HANDLER(bus_fault);
DEFINE_HANDLER(usage_fault);
DEFINE_HANDLER(vPortSVCHandler);
DEFINE_HANDLER(debug_monitor);
DEFINE_HANDLER(xPortPendSVHandler);
DEFINE_HANDLER(xPortSysTickHandler);

DEFINE_HANDLER(gpio_porta);
DEFINE_HANDLER(gpio_portb);
DEFINE_HANDLER(gpio_portc);
DEFINE_HANDLER(gpio_portd);
DEFINE_HANDLER(gpio_porte);
DEFINE_HANDLER(uart0);
DEFINE_HANDLER(UART1IntHandler);
DEFINE_HANDLER(ssi0);
DEFINE_HANDLER(I2CSlave0Interrupt);
DEFINE_HANDLER(pwm0_fault);
DEFINE_HANDLER(pwm0_gen0);
DEFINE_HANDLER(pwm0_gen1);
DEFINE_HANDLER(pwm0_gen2);
DEFINE_HANDLER(qei0);
DEFINE_HANDLER(adc0_seq0);
DEFINE_HANDLER(ADCSeq1Interrupt);
DEFINE_HANDLER(adc0_seq2);
DEFINE_HANDLER(adc0_seq3);
DEFINE_HANDLER(watchdog);
DEFINE_HANDLER(timer0a_32);
DEFINE_HANDLER(timer0b_32);
DEFINE_HANDLER(timer1a_32);
DEFINE_HANDLER(timer1b_32);
DEFINE_HANDLER(timer2a_32);
DEFINE_HANDLER(timer2b_32);
DEFINE_HANDLER(analog_comp0);
DEFINE_HANDLER(analog_comp1);
DEFINE_HANDLER(sysctl);
DEFINE_HANDLER(flashctl);
DEFINE_HANDLER(gpio_portf);
DEFINE_HANDLER(uart2);
DEFINE_HANDLER(ssi1);
DEFINE_HANDLER(timer3a_32);
DEFINE_HANDLER(timer3b_32);
DEFINE_HANDLER(i2c1);
DEFINE_HANDLER(qei1);
DEFINE_HANDLER(can0);
DEFINE_HANDLER(can1);
DEFINE_HANDLER(hibernation);
DEFINE_HANDLER(usb);
DEFINE_HANDLER(pwm0_gen3);
DEFINE_HANDLER(udma_soft);
DEFINE_HANDLER(udma_error);
DEFINE_HANDLER(adc1_seq0);
DEFINE_HANDLER(adc1_seq1);
DEFINE_HANDLER(adc1_seq2);
DEFINE_HANDLER(adc1_seq3);
DEFINE_HANDLER(ssi2);
DEFINE_HANDLER(ssi3);
DEFINE_HANDLER(uart3);
DEFINE_HANDLER(uart4);
DEFINE_HANDLER(uart5);
DEFINE_HANDLER(uart6);
DEFINE_HANDLER(uart7);
DEFINE_HANDLER(i2c2);
DEFINE_HANDLER(i2c3);
DEFINE_HANDLER(timer4a_32);
DEFINE_HANDLER(timer4b_32);
DEFINE_HANDLER(timer5a_32);
DEFINE_HANDLER(timer5b_32);
DEFINE_HANDLER(timer0a_64);
DEFINE_HANDLER(timer0b_64);
DEFINE_HANDLER(timer1a_64);
DEFINE_HANDLER(timer1b_64);
DEFINE_HANDLER(timer2a_64);
DEFINE_HANDLER(timer2b_64);
DEFINE_HANDLER(timer3a_64);
DEFINE_HANDLER(timer3b_64);
DEFINE_HANDLER(timer4a_64);
DEFINE_HANDLER(timer4b_64);
DEFINE_HANDLER(timer5a_64);
DEFINE_HANDLER(timer5b_64);
DEFINE_HANDLER(sysexcept);
DEFINE_HANDLER(pwm1_gen0);
DEFINE_HANDLER(pwm1_gen1);
DEFINE_HANDLER(pwm1_gen2);
DEFINE_HANDLER(pwm1_gen3);
DEFINE_HANDLER(pwm1_fault);

// The Nested Vectored Interrupt Controller (NVIC) Table:
// Mark with .nvic_table (as in the linker script) so it'll be placed correctly
void (*nvic_table[])(void) __attribute__((used, section(".isr_vector"))) = {
    ResetISR,               // The reset handler
    HANDLER(NmiSR),         // The NMI handler
    HardFault_handler,      // The hard fault handler -- was FaultISR
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
    IntDefaultHandler,      // ADC Sequence 1
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
    IntDefaultHandler,      // ADC1 Sequence 0
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
    IntDefaultHandler,      // I2C2 Master and Slave
    IntDefaultHandler,      // I2C3 Master and Slave
    IntDefaultHandler,      // Timer 4 subtimer A
    IntDefaultHandler,      // Timer 4 subtimer B
    IntDefaultHandler,      // Timer 5 subtimer A
    IntDefaultHandler,      // Timer 5 subtimer B
    IntDefaultHandler,      // FPU
    0,                      // Reserved
    0,                      // Reserved
    IntDefaultHandler,      // I2C4 Master and Slave
    IntDefaultHandler,      // I2C5 Master and Slave
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

// External Links:

// // Link to linker symbols (memory boundaries)
// // text : __text_start_vma :: __text_end_vma
// // data : __data_start_vma :: __data_end_vma
// // bss  : __bss_start_vma  ::   _bss_end_vma
// // (see tm4c.ld for more details)
// extern unsigned long __text_start_vma;
// extern unsigned long __text_end_vma;
// extern unsigned long __data_start_vma;
// extern unsigned long __data_end_vma;
// extern unsigned long __bss_start_vma;
// extern unsigned long __bss_end_vma;

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

// Link to project's entry point
extern int main(void);

// Interrupt Handlers:

// Declare a dummy interrupt handler that does nothing (essentially gets
// trapped in a loop). This works as the default interrupt handler as it
// retains the system state and stops execution when it is called; this
// interrupt handler will only ever be called if an unexpected interrupt
// (i.e. one that does not have a strongly defined interrupt handler) is
// triggered.
void __default_int_handler(void)
{
  while (1)
    ;
}

// A default reset handler. Configured in the same manner as the default
// handler above (ResetISR() is weakly aliased to this function) so
// that (if needed) this function can be overriden in a project using this
// file (shouldn't be necessary though).
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
// This is the code that gets called when the processor receives an unexpected
// interrupt.  This simply enters an infinite loop, preserving the system state
// for examination by a debugger.
//
//*****************************************************************************
static void IntDefaultHandler(void)
{
  //
  // Go into an infinite loop.
  //
  while (1) {
  }
}
