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
#include "FreeRTOS.h"
#include "FreeRTOSConfig.h"
#include "stream_buffer.h"
#include "task.h"


extern StreamBufferHandle_t xUART4StreamBuffer, xUART1StreamBuffer;

// UART
void UART1IntHandler( void );
void UART4IntHandler( void );

// SMBUs specific handler for I2C
//extern tSMBus g_sSlave0;  // for I2C #0

extern tSMBus g_sMaster1; // for I2C #1
extern tSMBus g_sMaster2; // for I2C #2
extern tSMBus g_sMaster3; // for I2C #3
extern tSMBus g_sMaster4; // for I2C #4
extern tSMBus g_sMaster6; // for I2C #6

extern volatile tSMBusStatus eStatus1;
extern volatile tSMBusStatus eStatus2;
extern volatile tSMBusStatus eStatus3;
extern volatile tSMBusStatus eStatus4;
extern volatile tSMBusStatus eStatus6;

void SMBusMasterIntHandler1(void);
void SMBusMasterIntHandler2(void);
void SMBusMasterIntHandler3(void);
void SMBusMasterIntHandler4(void);
void SMBusMasterIntHandler6(void);

// I2C Slave
extern TaskHandle_t TaskNotifyI2CSlave;
void I2CSlave0Interrupt();

// ADC interrupts
extern TaskHandle_t TaskNotifyADC;

void ADCSeq0Interrupt();
void ADCSeq1Interrupt();

// soft UART
void Timer0AIntHandler(void);


// these are from the FreeRTOS code base.
void xPortPendSVHandler(void);
void vPortSVCHandler(void);
void xPortSysTickHandler(void);



#endif /* PROJECTS_CM_MCU_INTERRUPTHANDLERS_H_ */
