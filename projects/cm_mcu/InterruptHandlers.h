/*
 * InterruptHanders.h
 *
 *  Created on: Aug 23, 2019
 *      Author: wittich
 *
 */

#ifndef PROJECTS_CM_MCU_INTERRUPTHANDLERS_H_
#define PROJECTS_CM_MCU_INTERRUPTHANDLERS_H_

// local include
#include "common/smbus.h"

// FreeRTOS includes
#include "FreeRTOS.h"       // IWYU pragma: keep
#include "FreeRTOSConfig.h" // IWYU pragma: keep
#include "stream_buffer.h"
#include "task.h"

// UART
#ifdef REV1
extern StreamBufferHandle_t xUART1StreamBuffer;
extern StreamBufferHandle_t xUART4StreamBuffer;
void UART1IntHandler(void);
void UART4IntHandler(void);
#else  // REV2 or REV3
extern StreamBufferHandle_t xUART0StreamBuffer;
extern StreamBufferHandle_t xUART7StreamBuffer;
void UART0IntHandler(void);
void UART7IntHandler(void);
// Set true by UART7IntHandler when xStreamBufferSendFromISR() could not
// accept every byte just read from the FIFO (ProgComTask stalled, e.g. in a
// slow/timing-out I2C transaction, so nobody is draining the buffer).
// Cleared by the consumer (add_progcom_char() in ProgComTask.c) once it has
// resynchronized to the next line boundary.
extern volatile bool progcom_rx_overflow;
#endif // REV1

// SMBUs specific handler for I2C
// extern tSMBus g_sSlave0;  // for I2C #0

extern tSMBus g_sMaster1; // for I2C #1
extern tSMBus g_sMaster2; // for I2C #2
extern tSMBus g_sMaster3; // for I2C #3
extern tSMBus g_sMaster4; // for I2C #4
extern tSMBus g_sMaster5; // for I2C #5
extern tSMBus g_sMaster6; // for I2C #6

extern volatile tSMBusStatus eStatus1;
extern volatile tSMBusStatus eStatus2;
extern volatile tSMBusStatus eStatus3;
extern volatile tSMBusStatus eStatus4;
extern volatile tSMBusStatus eStatus5;
extern volatile tSMBusStatus eStatus6;

extern TaskHandle_t TaskNotifySMBus[10];
// Set true by SMBusMasterIntHandlerCore on the first error interrupt of a
// transfer, cleared by i2c_arm_notify_slot() when a new transfer is armed.
// Prevents a NACK's trailing STOP-completion interrupt (which reports
// SMBUS_OK) from clobbering the real error status. See i2c_lockup_notes.md #3.
extern volatile bool I2CMasterErrorLatched[10];

void SMBusMasterIntHandler1(void);
void SMBusMasterIntHandler2(void);
void SMBusMasterIntHandler3(void);
void SMBusMasterIntHandler4(void);
void SMBusMasterIntHandler5(void);
void SMBusMasterIntHandler6(void);

// I2C Slave
extern TaskHandle_t TaskNotifyI2CSlave;
void I2CSlave0Interrupt(void);

// I2C slave diagnostics for the ZynqMon diagnostics block; counters saturate. The ISR writes
// dup_irq and, on the first duplicate, snap_int/snap_task and sets snap_pending;
// I2CSlaveTask writes the rest and fills snap_scsr/snap_state at its next wake.
struct i2cs_diag_t {
  uint16_t dup_irq;        // DATA interrupts while the task's previous wake was still pending
  uint16_t timeouts;       // waits in a transaction that timed out (also a lone 1-byte write)
  uint16_t unhandled;      // DATA interrupts with an I2CSCSR value the task does not handle (incl. 0)
  uint16_t last_unhandled; // I2CSCSR of the latest of those
  uint16_t snap_int;       // first duplicate: interrupt status (I2CSMIS) read by the ISR
  uint16_t snap_scsr;      // first duplicate: I2CSCSR at the task's next wake
  uint16_t snap_state;     // first duplicate: the task's state at that wake; 0xFF = none yet
  TaskHandle_t snap_task;  // first duplicate: the task the ISR interrupted
  bool snap_pending;       // snap_scsr/snap_state still to be filled by the task
};
extern volatile struct i2cs_diag_t i2cs_diag;

// ADC interrupts
extern TaskHandle_t TaskNotifyADC;

void ADCSeq0Interrupt(void);
void ADCSeq1Interrupt(void);

#ifdef REV1
// soft UART
void Timer0AIntHandler(void);
#endif // REV1

// these are from the FreeRTOS code base.
void xPortPendSVHandler(void);
void vPortSVCHandler(void);
void xPortSysTickHandler(void);

#endif /* PROJECTS_CM_MCU_INTERRUPTHANDLERS_H_ */
