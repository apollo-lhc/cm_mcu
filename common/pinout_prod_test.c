//*****************************************************************************
//
// Configure the device pins for different signals
//
// Copyright (C) 2014 Texas Instruments Incorporated - http://www.ti.com/ 
// 
// 
//  Redistribution and use in source and binary forms, with or without 
//  modification, are permitted provided that the following conditions 
//  are met:
//
//    Redistributions of source code must retain the above copyright 
//    notice, this list of conditions and the following disclaimer.
//
//    Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in the 
//    documentation and/or other materials provided with the   
//    distribution.
//
//    Neither the name of Texas Instruments Incorporated nor the names of
//    its contributors may be used to endorse or promote products derived
//    from this software without specific prior written permission.
//
//  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
//  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT 
//  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
//  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT 
//  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
//  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT 
//  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
//  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
//  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT 
//  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
//  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
//*****************************************************************************

// This file was automatically generated on 1/14/2025 at 3:31:23 PM
// by TI PinMux version 1.22.0+3940
//
//*****************************************************************************

#include <stdbool.h>
#include <stdint.h>
#include "inc/hw_gpio.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "driverlib/sysctl.h"
#include "pinout.h"

//*****************************************************************************
//
//! \addtogroup pinout_api
//! @{
//
//*****************************************************************************

//*****************************************************************************
//
//! Configures the device pins for the customer specific usage.
//!
//! \return None.
//
//*****************************************************************************
void
PinoutSet(void)
{
    //
    // Enable Peripheral Clocks 
    //
	MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
	MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
	MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);
	MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
	MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
	MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
	MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOG);
	MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOH);
	MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOJ);
	MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOK);
	MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOL);
	MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOM);
	MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPION);
	MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOP);
	MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOQ);

    //
    // Configure the GPIO Pin Mux for PD1
	// for AIN14_1
    //
	MAP_GPIOPinTypeADC(GPIO_PORTD_BASE, GPIO_PIN_1);

    //
    // Configure the GPIO Pin Mux for PE3
	// for AIN0_1
    //
	MAP_GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_3);

    //
    // Configure the GPIO Pin Mux for PK1
	// for AIN17_1
    //
	MAP_GPIOPinTypeADC(GPIO_PORTK_BASE, GPIO_PIN_1);

    //
    // Configure the GPIO Pin Mux for PD0
	// for AIN15_1
    //
	MAP_GPIOPinTypeADC(GPIO_PORTD_BASE, GPIO_PIN_0);

    //
    // Configure the GPIO Pin Mux for PD3
	// for AIN12_1
    //
	MAP_GPIOPinTypeADC(GPIO_PORTD_BASE, GPIO_PIN_3);

    //
    // Configure the GPIO Pin Mux for PE1
	// for AIN2_1
    //
	MAP_GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_1);

    //
    // Configure the GPIO Pin Mux for PK3
	// for AIN19_1
    //
	MAP_GPIOPinTypeADC(GPIO_PORTK_BASE, GPIO_PIN_3);

    //
    // Configure the GPIO Pin Mux for PK0
	// for AIN16_1
    //
	MAP_GPIOPinTypeADC(GPIO_PORTK_BASE, GPIO_PIN_0);

    //
    // Configure the GPIO Pin Mux for PD2
	// for AIN13_1
    //
	MAP_GPIOPinTypeADC(GPIO_PORTD_BASE, GPIO_PIN_2);

    //
    // Configure the GPIO Pin Mux for PE2
	// for AIN1_1
    //
	MAP_GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_2);

    //
    // Configure the GPIO Pin Mux for PE0
	// for AIN3_1
    //
	MAP_GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_0);

    //
    // Configure the GPIO Pin Mux for PK2
	// for AIN18_1
    //
	MAP_GPIOPinTypeADC(GPIO_PORTK_BASE, GPIO_PIN_2);

	//
	// Unlock the Port Pin and Set the Commit Bit
	//
	HWREG(GPIO_PORTD_BASE+GPIO_O_LOCK) = GPIO_LOCK_KEY;
	HWREG(GPIO_PORTD_BASE+GPIO_O_CR)   |= GPIO_PIN_7;
	HWREG(GPIO_PORTD_BASE+GPIO_O_LOCK) = 0x0;
	
    //
    // Configure the GPIO Pin Mux for PD7
	// for AIN4
    //
	MAP_GPIOPinTypeADC(GPIO_PORTD_BASE, GPIO_PIN_7);

    //
    // Configure the GPIO Pin Mux for PD5
	// for AIN6
    //
	MAP_GPIOPinTypeADC(GPIO_PORTD_BASE, GPIO_PIN_5);

    //
    // Configure the GPIO Pin Mux for PB5
	// for AIN11
    //
	MAP_GPIOPinTypeADC(GPIO_PORTB_BASE, GPIO_PIN_5);

    //
    // Configure the GPIO Pin Mux for PE4
	// for AIN9
    //
	MAP_GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_4);

    //
    // Configure the GPIO Pin Mux for PE5
	// for AIN8
    //
	MAP_GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_5);

    //
    // Configure the GPIO Pin Mux for PB4
	// for AIN10
    //
	MAP_GPIOPinTypeADC(GPIO_PORTB_BASE, GPIO_PIN_4);

    //
    // Configure the GPIO Pin Mux for PD4
	// for AIN7
    //
	MAP_GPIOPinTypeADC(GPIO_PORTD_BASE, GPIO_PIN_4);

    //
    // Configure the GPIO Pin Mux for PD6
	// for AIN5
    //
	MAP_GPIOPinTypeADC(GPIO_PORTD_BASE, GPIO_PIN_6);

    //
    // Configure the GPIO Pin Mux for PQ0
	// for GPIO_PQ0
    //
	MAP_GPIOPinTypeGPIOOutputOD(GPIO_PORTQ_BASE, GPIO_PIN_0);

    //
    // Configure the GPIO Pin Mux for PQ1
	// for GPIO_PQ1
    //
	MAP_GPIOPinTypeGPIOOutputOD(GPIO_PORTQ_BASE, GPIO_PIN_1);

    //
    // Configure the GPIO Pin Mux for PQ2
	// for GPIO_PQ2
    //
	MAP_GPIOPinTypeGPIOOutputOD(GPIO_PORTQ_BASE, GPIO_PIN_2);

    //
    // Configure the GPIO Pin Mux for PC7
	// for GPIO_PC7
    //
	MAP_GPIOPinTypeGPIOInput(GPIO_PORTC_BASE, GPIO_PIN_7);

    //
    // Configure the GPIO Pin Mux for PC6
	// for GPIO_PC6
    //
	MAP_GPIOPinTypeGPIOOutput(GPIO_PORTC_BASE, GPIO_PIN_6);

    //
    // Configure the GPIO Pin Mux for PQ3
	// for GPIO_PQ3
    //
	MAP_GPIOPinTypeGPIOOutputOD(GPIO_PORTQ_BASE, GPIO_PIN_3);

    //
    // Configure the GPIO Pin Mux for PH0
	// for GPIO_PH0
    //
	MAP_GPIOPinTypeGPIOOutputOD(GPIO_PORTH_BASE, GPIO_PIN_0);

    //
    // Configure the GPIO Pin Mux for PH1
	// for GPIO_PH1
    //
	MAP_GPIOPinTypeGPIOInput(GPIO_PORTH_BASE, GPIO_PIN_1);

    //
    // Configure the GPIO Pin Mux for PH2
	// for GPIO_PH2
    //
	MAP_GPIOPinTypeGPIOInput(GPIO_PORTH_BASE, GPIO_PIN_2);

    //
    // Configure the GPIO Pin Mux for PH3
	// for GPIO_PH3
    //
	MAP_GPIOPinTypeGPIOInput(GPIO_PORTH_BASE, GPIO_PIN_3);

    //
    // Configure the GPIO Pin Mux for PF0
	// for GPIO_PF0
    //
	MAP_GPIOPinTypeGPIOInput(GPIO_PORTF_BASE, GPIO_PIN_0);

    //
    // Configure the GPIO Pin Mux for PF1
	// for GPIO_PF1
    //
	MAP_GPIOPinTypeGPIOInput(GPIO_PORTF_BASE, GPIO_PIN_1);

    //
    // Configure the GPIO Pin Mux for PF2
	// for GPIO_PF2
    //
	MAP_GPIOPinTypeGPIOInput(GPIO_PORTF_BASE, GPIO_PIN_2);

    //
    // Configure the GPIO Pin Mux for PF3
	// for GPIO_PF3
    //
	MAP_GPIOPinTypeGPIOInput(GPIO_PORTF_BASE, GPIO_PIN_3);

    //
    // Configure the GPIO Pin Mux for PF4
	// for GPIO_PF4
    //
	MAP_GPIOPinTypeGPIOInput(GPIO_PORTF_BASE, GPIO_PIN_4);

    //
    // Configure the GPIO Pin Mux for PQ5
	// for GPIO_PQ5
    //
	MAP_GPIOPinTypeGPIOInput(GPIO_PORTQ_BASE, GPIO_PIN_5);

    //
    // Configure the GPIO Pin Mux for PQ6
	// for GPIO_PQ6
    //
	MAP_GPIOPinTypeGPIOInput(GPIO_PORTQ_BASE, GPIO_PIN_6);

    //
    // Configure the GPIO Pin Mux for PK7
	// for GPIO_PK7
    //
	MAP_GPIOPinTypeGPIOInput(GPIO_PORTK_BASE, GPIO_PIN_7);

    //
    // Configure the GPIO Pin Mux for PK6
	// for GPIO_PK6
    //
	MAP_GPIOPinTypeGPIOInput(GPIO_PORTK_BASE, GPIO_PIN_6);

    //
    // Configure the GPIO Pin Mux for PK5
	// for GPIO_PK5
    //
	MAP_GPIOPinTypeGPIOInput(GPIO_PORTK_BASE, GPIO_PIN_5);

    //
    // Configure the GPIO Pin Mux for PK4
	// for GPIO_PK4
    //
	MAP_GPIOPinTypeGPIOInput(GPIO_PORTK_BASE, GPIO_PIN_4);

    //
    // Configure the GPIO Pin Mux for PM7
	// for GPIO_PM7
    //
	MAP_GPIOPinTypeGPIOInput(GPIO_PORTM_BASE, GPIO_PIN_7);

    //
    // Configure the GPIO Pin Mux for PM6
	// for GPIO_PM6
    //
	MAP_GPIOPinTypeGPIOInput(GPIO_PORTM_BASE, GPIO_PIN_6);

    //
    // Configure the GPIO Pin Mux for PM5
	// for GPIO_PM5
    //
	MAP_GPIOPinTypeGPIOInput(GPIO_PORTM_BASE, GPIO_PIN_5);

    //
    // Configure the GPIO Pin Mux for PM4
	// for GPIO_PM4
    //
	MAP_GPIOPinTypeGPIOOutput(GPIO_PORTM_BASE, GPIO_PIN_4);

    //
    // Configure the GPIO Pin Mux for PM3
	// for GPIO_PM3
    //
	MAP_GPIOPinTypeGPIOInput(GPIO_PORTM_BASE, GPIO_PIN_3);

    //
    // Configure the GPIO Pin Mux for PM2
	// for GPIO_PM2
    //
	MAP_GPIOPinTypeGPIOOutput(GPIO_PORTM_BASE, GPIO_PIN_2);

    //
    // Configure the GPIO Pin Mux for PM1
	// for GPIO_PM1
    //
	MAP_GPIOPinTypeGPIOOutput(GPIO_PORTM_BASE, GPIO_PIN_1);

    //
    // Configure the GPIO Pin Mux for PM0
	// for GPIO_PM0
    //
	MAP_GPIOPinTypeGPIOOutput(GPIO_PORTM_BASE, GPIO_PIN_0);

    //
    // Configure the GPIO Pin Mux for PL0
	// for GPIO_PL0
    //
	MAP_GPIOPinTypeGPIOOutput(GPIO_PORTL_BASE, GPIO_PIN_0);

    //
    // Configure the GPIO Pin Mux for PL1
	// for GPIO_PL1
    //
	MAP_GPIOPinTypeGPIOOutput(GPIO_PORTL_BASE, GPIO_PIN_1);

    //
    // Configure the GPIO Pin Mux for PL2
	// for GPIO_PL2
    //
	MAP_GPIOPinTypeGPIOOutput(GPIO_PORTL_BASE, GPIO_PIN_2);

    //
    // Configure the GPIO Pin Mux for PL3
	// for GPIO_PL3
    //
	MAP_GPIOPinTypeGPIOOutput(GPIO_PORTL_BASE, GPIO_PIN_3);

    //
    // Configure the GPIO Pin Mux for PL4
	// for GPIO_PL4
    //
	MAP_GPIOPinTypeGPIOOutput(GPIO_PORTL_BASE, GPIO_PIN_4);

    //
    // Configure the GPIO Pin Mux for PL5
	// for GPIO_PL5
    //
	MAP_GPIOPinTypeGPIOOutput(GPIO_PORTL_BASE, GPIO_PIN_5);

    //
    // Configure the GPIO Pin Mux for PL7
	// for GPIO_PL7
    //
	MAP_GPIOPinTypeGPIOOutput(GPIO_PORTL_BASE, GPIO_PIN_7);

    //
    // Configure the GPIO Pin Mux for PL6
	// for GPIO_PL6
    //
	MAP_GPIOPinTypeGPIOOutput(GPIO_PORTL_BASE, GPIO_PIN_6);

    //
    // Configure the GPIO Pin Mux for PQ4
	// for GPIO_PQ4
    //
	MAP_GPIOPinTypeGPIOOutput(GPIO_PORTQ_BASE, GPIO_PIN_4);

    //
    // Configure the GPIO Pin Mux for PP2
	// for GPIO_PP2
    //
	MAP_GPIOPinTypeGPIOOutput(GPIO_PORTP_BASE, GPIO_PIN_2);

    //
    // Configure the GPIO Pin Mux for PP3
	// for GPIO_PP3
    //
	MAP_GPIOPinTypeGPIOOutput(GPIO_PORTP_BASE, GPIO_PIN_3);

    //
    // Configure the GPIO Pin Mux for PP4
	// for GPIO_PP4
    //
	MAP_GPIOPinTypeGPIOOutput(GPIO_PORTP_BASE, GPIO_PIN_4);

    //
    // Configure the GPIO Pin Mux for PP5
	// for GPIO_PP5
    //
	MAP_GPIOPinTypeGPIOOutput(GPIO_PORTP_BASE, GPIO_PIN_5);

    //
    // Configure the GPIO Pin Mux for PN0
	// for GPIO_PN0
    //
	MAP_GPIOPinTypeGPIOOutput(GPIO_PORTN_BASE, GPIO_PIN_0);

    //
    // Configure the GPIO Pin Mux for PN1
	// for GPIO_PN1
    //
	MAP_GPIOPinTypeGPIOOutput(GPIO_PORTN_BASE, GPIO_PIN_1);

    //
    // Configure the GPIO Pin Mux for PN2
	// for GPIO_PN2
    //
	MAP_GPIOPinTypeGPIOOutput(GPIO_PORTN_BASE, GPIO_PIN_2);

    //
    // Configure the GPIO Pin Mux for PN3
	// for GPIO_PN3
    //
	MAP_GPIOPinTypeGPIOOutput(GPIO_PORTN_BASE, GPIO_PIN_3);

    //
    // Configure the GPIO Pin Mux for PJ0
	// for GPIO_PJ0
    //
	MAP_GPIOPinTypeGPIOInput(GPIO_PORTJ_BASE, GPIO_PIN_0);

    //
    // Configure the GPIO Pin Mux for PJ1
	// for GPIO_PJ1
    //
	MAP_GPIOPinTypeGPIOInput(GPIO_PORTJ_BASE, GPIO_PIN_1);

    //
    // Configure the GPIO Pin Mux for PP0
	// for GPIO_PP0
    //
	MAP_GPIOPinTypeGPIOInput(GPIO_PORTP_BASE, GPIO_PIN_0);

    //
    // Configure the GPIO Pin Mux for PP1
	// for GPIO_PP1
    //
	MAP_GPIOPinTypeGPIOInput(GPIO_PORTP_BASE, GPIO_PIN_1);

    //
    // Configure the GPIO Pin Mux for PB2
	// for GPIO_PB2
    //
	MAP_GPIOPinTypeGPIOOutput(GPIO_PORTB_BASE, GPIO_PIN_2);

    //
    // Configure the GPIO Pin Mux for PB3
	// for GPIO_PB3
    //
	MAP_GPIOPinTypeGPIOInput(GPIO_PORTB_BASE, GPIO_PIN_3);

    //
    // Configure the GPIO Pin Mux for PC4
	// for GPIO_PC4
    //
	MAP_GPIOPinTypeGPIOOutput(GPIO_PORTC_BASE, GPIO_PIN_4);

    //
    // Configure the GPIO Pin Mux for PC5
	// for GPIO_PC5
    //
	MAP_GPIOPinTypeGPIOInput(GPIO_PORTC_BASE, GPIO_PIN_5);

    //
    // Configure the GPIO Pin Mux for PA2
	// for GPIO_PA2
    //
	MAP_GPIOPinTypeGPIOOutput(GPIO_PORTA_BASE, GPIO_PIN_2);

    //
    // Configure the GPIO Pin Mux for PA3
	// for GPIO_PA3
    //
	MAP_GPIOPinTypeGPIOInput(GPIO_PORTA_BASE, GPIO_PIN_3);

    //
    // Configure the GPIO Pin Mux for PA6
	// for GPIO_PA6
    //
	MAP_GPIOPinTypeGPIOOutput(GPIO_PORTA_BASE, GPIO_PIN_6);

    //
    // Configure the GPIO Pin Mux for PA7
	// for GPIO_PA7
    //
	MAP_GPIOPinTypeGPIOInput(GPIO_PORTA_BASE, GPIO_PIN_7);

    //
    // Configure the GPIO Pin Mux for PA4
	// for GPIO_PA4
    //
	MAP_GPIOPinTypeGPIOOutput(GPIO_PORTA_BASE, GPIO_PIN_4);

    //
    // Configure the GPIO Pin Mux for PA5
	// for GPIO_PA5
    //
	MAP_GPIOPinTypeGPIOInput(GPIO_PORTA_BASE, GPIO_PIN_5);

    //
    // Configure the GPIO Pin Mux for PG0
	// for I2C1SCL
    //
	MAP_GPIOPinConfigure(GPIO_PG0_I2C1SCL);
	MAP_GPIOPinTypeI2CSCL(GPIO_PORTG_BASE, GPIO_PIN_0);

    //
    // Configure the GPIO Pin Mux for PG1
	// for I2C1SDA
    //
	MAP_GPIOPinConfigure(GPIO_PG1_I2C1SDA);
	MAP_GPIOPinTypeI2C(GPIO_PORTG_BASE, GPIO_PIN_1);

    //
    // Configure the GPIO Pin Mux for PG2
	// for I2C2SCL
    //
	MAP_GPIOPinConfigure(GPIO_PG2_I2C2SCL);
	MAP_GPIOPinTypeI2CSCL(GPIO_PORTG_BASE, GPIO_PIN_2);

    //
    // Configure the GPIO Pin Mux for PG3
	// for I2C2SDA
    //
	MAP_GPIOPinConfigure(GPIO_PG3_I2C2SDA);
	MAP_GPIOPinTypeI2C(GPIO_PORTG_BASE, GPIO_PIN_3);

    //
    // Configure the GPIO Pin Mux for PG4
	// for I2C3SCL
    //
	MAP_GPIOPinConfigure(GPIO_PG4_I2C3SCL);
	MAP_GPIOPinTypeI2CSCL(GPIO_PORTG_BASE, GPIO_PIN_4);

    //
    // Configure the GPIO Pin Mux for PG5
	// for I2C3SDA
    //
	MAP_GPIOPinConfigure(GPIO_PG5_I2C3SDA);
	MAP_GPIOPinTypeI2C(GPIO_PORTG_BASE, GPIO_PIN_5);

    //
    // Configure the GPIO Pin Mux for PG6
	// for I2C4SCL
    //
	MAP_GPIOPinConfigure(GPIO_PG6_I2C4SCL);
	MAP_GPIOPinTypeI2CSCL(GPIO_PORTG_BASE, GPIO_PIN_6);

    //
    // Configure the GPIO Pin Mux for PG7
	// for I2C4SDA
    //
	MAP_GPIOPinConfigure(GPIO_PG7_I2C4SDA);
	MAP_GPIOPinTypeI2C(GPIO_PORTG_BASE, GPIO_PIN_7);

    //
    // Configure the GPIO Pin Mux for PB0
	// for I2C5SCL
    //
	MAP_GPIOPinConfigure(GPIO_PB0_I2C5SCL);
	MAP_GPIOPinTypeI2CSCL(GPIO_PORTB_BASE, GPIO_PIN_0);

    //
    // Configure the GPIO Pin Mux for PB1
	// for I2C5SDA
    //
	MAP_GPIOPinConfigure(GPIO_PB1_I2C5SDA);
	MAP_GPIOPinTypeI2C(GPIO_PORTB_BASE, GPIO_PIN_1);

    //
    // Configure the GPIO Pin Mux for PA0
	// for U0RX
    //
	MAP_GPIOPinConfigure(GPIO_PA0_U0RX);
	MAP_GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0);

    //
    // Configure the GPIO Pin Mux for PA1
	// for U0TX
    //
	MAP_GPIOPinConfigure(GPIO_PA1_U0TX);
	MAP_GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_1);

}

//*****************************************************************************
//
// Close the Doxygen group.
//! @}
//
//*****************************************************************************

