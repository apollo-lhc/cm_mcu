//*****************************************************************************
//
// bl_config.h - The configurable parameters of the boot loader.
//
// Copyright (c) 2010-2017 Texas Instruments Incorporated.  All rights reserved.
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
// This is part of revision 2.1.4.178 of the Tiva Firmware Development Package.
//
//*****************************************************************************

#ifndef __BL_CONFIG_H__
#define __BL_CONFIG_H__

// #define APOLLO_BL_UART_FP // if you want front panel, otherwise zynq
#ifdef REV1
#ifdef APOLLO_BL_UART_FP
#define SYSCTL_PERIPH_UARTx SYSCTL_PERIPH_UART4
#else
#define SYSCTL_PERIPH_UARTx SYSCTL_PERIPH_UART1
#endif
#elif defined(REV2) || defined(REV3)
#define SYSCTL_PERIPH_UARTx SYSCTL_PERIPH_UART0
#endif
//*****************************************************************************
//
// The following defines are used to configure the operation of the boot
// loader.  For each define, its interactions with other defines are described.
// First is the dependencies (i.e. the defines that must also be defined if it
// is defined), next are the exclusives (i.e. the defines that can not be
// defined if it is defined), and finally are the requirements (i.e. the
// defines that must be defined if it is defined).
//
// The following defines must be defined in order for the boot loader to
// operate:
//
//     One of CAN_ENABLE_UPDATE, ENET_ENABLE_UPDATE, I2C_ENABLE_UPDATE,
//            SSI_ENABLE_UPDATE, UART_ENABLE_UPDATE, or USB_ENABLE_UPDATE
//     APP_START_ADDRESS
//     STACK_SIZE
//     BUFFER_SIZE
//
//*****************************************************************************

//*****************************************************************************
//
// The frequency of the crystal used to clock the microcontroller.
//
// This defines the crystal frequency used by the microcontroller running the
// boot loader.  If this is unknown at the time of production, then use the
// UART_AUTOBAUD feature to properly configure the UART.
//
// Depends on: None
// Exclusive of: None
// Requires: None
//
//*****************************************************************************
#define CRYSTAL_FREQ 25000000

//*****************************************************************************
//
// This enables the boosting of the LDO voltage to 2.75V.  For boot loader
// configurations that enable the PLL (for example, using the Ethernet port)
// on a part that has the PLL errata, this should be enabled.  This applies to
// revision A2 of Fury-class devices.
//
// Depends on: None
// Exclusive of: None
// Requires: None
//
//*****************************************************************************
// #define BOOST_LDO_VOLTAGE

//*****************************************************************************
//
// The starting address of the application.  This must be a multiple of 1024
// bytes (making it aligned to a page boundary).  A vector table is expected at
// this location, and the perceived validity of the vector table (stack located
// in SRAM, reset vector located in flash) is used as an indication of the
// validity of the application image.
//
// The flash image of the boot loader must not be larger than this value.
//
// Depends on: None
// Exclusive of: None
// Requires: None
//
//*****************************************************************************
#define APP_START_ADDRESS 0x00004000

//*****************************************************************************
//
// The address at which the application locates its exception vector table.
// This must be a multiple of 1KB (making it aligned to a page boundary).
// Typically, an application will start with its vector table and this value
// will default to APP_START_ADDRESS.  This option is provided to cater for
// applications which run from external memory which may not be accessible by
// the NVIC (the vector table offset register is only 30 bits long).
//
// Depends on: None
// Exclusive of: None
// Requires: None
//
//*****************************************************************************
#define VTABLE_START_ADDRESS APP_START_ADDRESS

//*****************************************************************************
//
// The size of a single, erasable page in the flash.  This must be a power
// of 2.  The default value of 1KB represents the page size for the internal
// flash on all Tiva MCUs and this value should only be overridden if
// configuring a boot loader to access external flash devices with a page size
// different from this.
//
// Depends on: None
// Exclusive of: None
// Requires: None
//
//*****************************************************************************
#define FLASH_PAGE_SIZE 0x00000400

//*****************************************************************************
//
// The amount of space at the end of flash to reserved.  This must be a
// multiple of 1024 bytes (making it aligned to a page boundary).  This
// reserved space is not erased when the application is updated, providing
// non-volatile storage that can be used for parameters.
//
// Depends on: None
// Exclusive of: None
// Requires: None
//
//*****************************************************************************
// #define FLASH_RSVD_SPACE        0x00000800

//*****************************************************************************
//
// The number of words of stack space to reserve for the boot loader.
//
// Depends on: None
// Exclusive of: None
// Requires: None
//
//*****************************************************************************
#define STACK_SIZE 64

//*****************************************************************************
//
// The number of words in the data buffer used for receiving packets.  This
// value must be at least 3.  If using autobauding on the UART, this must be at
// least 20.  The maximum usable value is 65 (larger values will result in
// unused space in the buffer).
//
// Depends on: None
// Exclusive of: None
// Requires: None
//
//*****************************************************************************
#define BUFFER_SIZE 65

//*****************************************************************************
//
// Enables updates to the boot loader.  Updating the boot loader is an unsafe
// operation since it is not fully fault tolerant (losing power to the device
// part way though could result in the boot loader no longer being present in
// flash).
//
// Depends on: None
// Exclusive of: None
// Requires: None
//
//*****************************************************************************
// #define ENABLE_BL_UPDATE

//*****************************************************************************
//
// This definition will cause the the boot loader to erase the entire flash on
// updates to the boot loader or to erase the entire application area when the
// application is updated.  This erases any unused sections in the flash before
// the firmware is updated.
//
// Depends on: None
// Exclusive of: None
// Requires: None
//
//*****************************************************************************
// #define FLASH_CODE_PROTECTION

//*****************************************************************************
//
// Enables the call to decrypt the downloaded data before writing it into
// flash.  The decryption routine is empty in the reference boot loader source,
// which simply provides a placeholder for adding an actual decrypter.
//
// Depends on: None
// Exclusive of: None
// Requires: None
//
//*****************************************************************************
// #define ENABLE_DECRYPTION

//*****************************************************************************
//
// Enables support for the MOSCFAIL handler in the NMI interrupt.
// Note:  Sandstorm or Fury devices do not provide the MOSCFAIL reset, so this
// feature should not be enabled for these devices.
//
// Depends on: None
// Exclusive of: None
// Requires: None
//
//*****************************************************************************
// #define ENABLE_MOSCFAIL_HANDLER

//*****************************************************************************
//
// Enables the pin-based forced update check.  When enabled, the boot loader
// will go into update mode instead of calling the application if a pin is read
// at a particular polarity, forcing an update operation.  In either case, the
// application is still able to return control to the boot loader in order to
// start an update.
//
// Depends on: None
// Exclusive of: None
// Requires: FORCED_UPDATE_PERIPH, FORCED_UPDATE_PORT, FORCED_UPDATE_PIN,
//           FORCED_UPDATE_POLARITY
//
//*****************************************************************************
// #define ENABLE_UPDATE_CHECK

//*****************************************************************************
//
// The GPIO module to enable in order to check for a forced update.  This will
// be one of the SYSCTL_RCGCGPIO_Rx values, where Rx represnts the required
// GPIO port. This applies to Blizzard class and later devices for FORCED_UPDATE_PORT.
//
// Depends on: ENABLE_UPDATE_CHECK
// Exclusive of: None
// Requries: None
//
//*****************************************************************************
// #define FORCED_UPDATE_PERIPH    SYSCTL_RCGCGPIO_R1

//*****************************************************************************
//
// The GPIO port to check for a forced update.  This will be one of the
// GPIO_PORTx_BASE values, where "x" is replaced with the port name (such as
// B).  The value of "x" should match the value of "x" for
// FORCED_UPDATE_PERIPH.
//
// Depends on: ENABLE_UPDATE_CHECK
// Exclusive of: None
// Requries: None
//
//*****************************************************************************
// #define FORCED_UPDATE_PORT      GPIO_PORTB_BASE

//*****************************************************************************
//
// The pin to check for a forced update.  This is a value between 0 and 7.
//
// Depends on: ENABLE_UPDATE_CHECK
// Exclusive of: None
// Requries: None
//
//*****************************************************************************
// #define FORCED_UPDATE_PIN       4

//*****************************************************************************
//
// The polarity of the GPIO pin that results in a forced update.  This value
// should be 0 if the pin should be low and 1 if the pin should be high.
//
// Depends on: ENABLE_UPDATE_CHECK
// Exclusive of: None
// Requries: None
//
//*****************************************************************************
// #define FORCED_UPDATE_POLARITY  0

//*****************************************************************************
//
// This enables a weak pull up for the GPIO pin used in a forced update.  This
// value should be 0 if the pin should be have an internal weak pull down and
// 1 if the pin should have an interal weak pull up.
// Only FORCED_UPDATE_WPU or FORCED_UPDATE_WPD or neither should be defined.
//
// Depends on: ENABLE_UPDATE_CHECK
// Exclusive of: None
// Requries: None
//
//*****************************************************************************
// #define FORCED_UPDATE_WPU
// #define FORCED_UPDATE_WPD

//*****************************************************************************
//
// This enables the use of the GPIO_LOCK mechanism for configuration of
// protected GPIO pins (for example JTAG pins).  If this value is not defined,
// the locking mechanism will not be used.  The only legal values for this
// feature are GPIO_LOCK_KEY for Fury devices and GPIO_LOCK_KEY_DD for all
// other devices except Sandstorm devices, which do not support this feature.
//
// Depends on: ENABLE_UPDATE_CHECK
// Exclusive of: None
// Requries: None
//
//*****************************************************************************
// #define FORCED_UPDATE_KEY       GPIO_LOCK_KEY
// #define FORCED_UPDATE_KEY       GPIO_LOCK_KEY_DD

//*****************************************************************************
//
// Selects the UART as the port for communicating with the boot loader.
//
// Depends on: None
// Exclusive of: CAN_ENABLE_UPDATE, ENET_ENABLE_UPDATE, I2C_ENABLE_UPDATE,
//               SSI_ENABLE_UPDATE, USB_ENABLE_UPDATE
// Requires: UART_AUTOBAUD or UART_FIXED_BAUDRATE, UART_CLOCK_ENABLE,
//           UARTx_BASE, UART_RXPIN_CLOCK_ENABLE, UART_RXPIN_BASE,
//           UART_RXPIN_PCTL, UART_RXPIN_POS, UART_TXPIN_CLOCK_ENABLE,
//           UART_TXPIN_BASE, UART_TXPIN_PCTL and UART_TXPIN_POS
//
//*****************************************************************************
#define UART_ENABLE_UPDATE

//*****************************************************************************
//
// Enables automatic baud rate detection.  This can be used if the crystal
// frequency is unknown, or if operation at different baud rates is desired.
//
// Depends on: UART_ENABLE_UPDATE
// Exclusive of: UART_FIXED_BAUDRATE
// Requires: None
//
//*****************************************************************************
// #define UART_AUTOBAUD

//*****************************************************************************
//
// Selects the baud rate to be used for the UART.
//
// Depends on: UART_ENABLE_UPDATE, CRYSTAL_FREQ
// Exclusive of: UART_AUTOBAUD
// Requires: None
//
//*****************************************************************************
#define UART_FIXED_BAUDRATE 115200

//*****************************************************************************
//
// Selects the clock enable for the UART peripheral module
//
// Depends on: UART_ENABLE_UPDATE
// Exclusive of: None
// Requires: UARTx_BASE
//
//*****************************************************************************
#ifdef REV1
#ifdef APOLLO_BL_UART_FP
#define UART_CLOCK_ENABLE SYSCTL_RCGCUART_R4
#else
#define UART_CLOCK_ENABLE SYSCTL_RCGCUART_R1
#endif
#elif defined(REV2) || defined(REV3)
#define UART_CLOCK_ENABLE SYSCTL_RCGCUART_R0
#endif
//*****************************************************************************
//
// Selects the base address of the UART peripheral module
//
// Depends on: UART_ENABLE_UPDATE
// Exclusive of: None
// Requires: UART_CLOCK_ENABLE
//
//*****************************************************************************
#ifdef REV1
#ifdef APOLLO_BL_UART_FP
#define UARTx_BASE UART4_BASE
#else
#define UARTx_BASE UART1_BASE
#endif
#elif defined(REV2) || defined(REV3)
#define UARTx_BASE UART0_BASE
#endif
//*****************************************************************************
//
// Selects the clock enable for the GPIO corresponding to UART RX pin
//
// Depends on: UART_ENABLE_UPDATE
// Exclusive of: None
// Requires: UART_RXPIN_BASE, UART_RXPIN_PCTL and UART_RXPIN_POS
//
//*****************************************************************************
#ifdef APOLLO_BL_UART_FP
#define UART_RXPIN_CLOCK_ENABLE SYSCTL_RCGCGPIO_R4
#else
#define UART_RXPIN_CLOCK_ENABLE SYSCTL_RCGCGPIO_R0
#endif
//*****************************************************************************
//
// Selects the base address for the GPIO corresponding to UART RX pin
//
// Depends on: UART_ENABLE_UPDATE
// Exclusive of: None
// Requires: UART_RXPIN_CLOCK_ENABLE, UART_RXPIN_PCTL and UART_RXPIN_POS
//
//*****************************************************************************
// correct for REV1, REV2 and REV3
#define UART_RXPIN_BASE GPIO_PORTA_BASE

//*****************************************************************************
//
// Selects the port control value for the GPIO corresponding to UART RX pin
//
// Depends on: UART_ENABLE_UPDATE
// Exclusive of: None
// Requires: UART_RXPIN_CLOCK_ENABLE, UART_RXPIN_BASE and UART_RXPIN_POS
//
//*****************************************************************************
#define UART_RXPIN_PCTL 0x1

//*****************************************************************************
//
// Selects the pin number for the GPIO corresponding to UART RX pin
//
// Depends on: UART_ENABLE_UPDATE
// Exclusive of: None
// Requires: UART_RXPIN_CLOCK_ENABLE, UART_RXPIN_BASE and UART_RXPIN_PCTL
//
//*****************************************************************************
#ifdef APOLLO_BL_UART_FP
#define UART_RXPIN_POS 2
#else
#define UART_RXPIN_POS 0
#endif

//*****************************************************************************
//
// Selects the clock enable for the GPIO corresponding to UART TX pin
//
// Depends on: UART_ENABLE_UPDATE
// Exclusive of: None
// Requires: UART_TXPIN_BASE, UART_TXPIN_PCTL and UART_TXPIN_POS
//
//*****************************************************************************
#ifdef APOLLO_BL_UART_FP
#define UART_TXPIN_CLOCK_ENABLE SYSCTL_RCGCGPIO_R4
#else
#define UART_TXPIN_CLOCK_ENABLE SYSCTL_RCGCGPIO_R0
#endif

//*****************************************************************************
//
// Selects the base address for the GPIO corresponding to UART TX pin
//
// Depends on: UART_ENABLE_UPDATE
// Exclusive of: None
// Requires: UART_TXPIN_CLOCK_ENABLE, UART_TXPIN_PCTL and UART_TXPIN_POS
//
//*****************************************************************************
#define UART_TXPIN_BASE GPIO_PORTA_BASE

//*****************************************************************************
//
// Selects the port control value for the GPIO corresponding to UART TX pin
//
// Depends on: UART_ENABLE_UPDATE
// Exclusive of: None
// Requires: UART_TXPIN_CLOCK_ENABLE, UART_TXPIN_BASE and UART_TXPIN_POS
//
//*****************************************************************************
#define UART_TXPIN_PCTL 0x1

//*****************************************************************************
//
// Selects the pin number for the GPIO corresponding to UART TX pin
//
// Depends on: UART_ENABLE_UPDATE
// Exclusive of: None
// Requires: UART_TXPIN_CLOCK_ENABLE, UART_TXPIN_BASE and UART_TXPIN_PCTL
//
//*****************************************************************************
#ifdef APOLLO_BL_UART_FP
#define UART_TXPIN_POS 3
#else
#define UART_TXPIN_POS 1
#endif

//*****************************************************************************
//
// Selects the SSI port as the port for communicating with the boot loader.
//
// Depends on: None
// Exclusive of: CAN_ENABLE_UPDATE, ENET_ENABLE_UPDATE, I2C_ENABLE_UPDATE,
//               UART_ENABLE_UPDATE, USB_ENABLE_UPDATE
// Requires: SSI_CLOCK_ENABLE, SSIx_BASE, SSI_CLKPIN_CLOCK_ENABLE,
//           SSI_CLKPIN_BASE, SSI_CLKPIN_PCTL, SSI_CLKPIN_POS,
//           SSI_FSSPIN_CLOCK_ENABLE, SSI_FSSPIN_BASE, SSI_FSSPIN_PCTL,
//           SSI_FSSPIN_POS, SSI_MISOPIN_CLOCK_ENABLE, SSI_MISOPIN_BASE,
//           SSI_MISOPIN_PCTL, SSI_MISOPIN_POS, SSI_MOSIPIN_CLOCK_ENABLE,
//           SSI_MOSIPIN_BASE, SSI_MOSIPIN_PCTL and SSI_MOSIPIN_POS
//
//*****************************************************************************
// #define SSI_ENABLE_UPDATE

//*****************************************************************************
//
// Selects the clock enable for the SSI peripheral module
//
// Depends on: SSI_ENABLE_UPDATE
// Exclusive of: None
// Requires: SSIx_BASE
//
//*****************************************************************************
// #define SSI_CLOCK_ENABLE        SYSCTL_RCGCSSI_R0

//*****************************************************************************
//
// Selects the base address of the SSI peripheral module
//
// Depends on: SSI_ENABLE_UPDATE
// Exclusive of: None
// Requires: SSI_CLOCK_ENABLE
//
//*****************************************************************************
// #define SSIx_BASE               SSI0_BASE

//*****************************************************************************
//
// Selects the clock enable for the GPIO corresponding to SSI CLK pin
//
// Depends on: SSI_ENABLE_UPDATE
// Exclusive of: None
// Requires: SSI_CLKPIN_BASE, SSI_CLKPIN_PCTL and SSI_CLKPIN_POS
//
//*****************************************************************************
// #define SSI_CLKPIN_CLOCK_ENABLE  SYSCTL_RCGCGPIO_R0

//*****************************************************************************
//
// Selects the base address for the GPIO corresponding to SSI CLK pin
//
// Depends on: SSI_ENABLE_UPDATE
// Exclusive of: None
// Requires: SSI_CLKPIN_CLOCK_ENABLE, SSI_CLKPIN_PCTL and SSI_CLKPIN_POS
//
//*****************************************************************************
// #define SSI_CLKPIN_BASE          GPIO_PORTA_BASE

//*****************************************************************************
//
// Selects the port control value for the GPIO corresponding to SSI CLK pin
//
// Depends on: SSI_ENABLE_UPDATE
// Exclusive of: None
// Requires: SSI_CLKPIN_CLOCK_ENABLE, SSI_CLKPIN_BASE and SSI_CLKPIN_POS
//
//*****************************************************************************
// #define SSI_CLKPIN_PCTL          0x2

//*****************************************************************************
//
// Selects the pin number for the GPIO corresponding to SSI CLK pin
//
// Depends on: SSI_ENABLE_UPDATE
// Exclusive of: None
// Requires: SSI_CLKPIN_CLOCK_ENABLE, SSI_CLKPIN_BASE and SSI_CLKPIN_PCTL
//
//*****************************************************************************
// #define SSI_CLKPIN_POS            2

//*****************************************************************************
//
// Selects the clock enable for the GPIO corresponding to SSI FSS pin
//
// Depends on: SSI_ENABLE_UPDATE
// Exclusive of: None
// Requires: SSI_FSSPIN_BASE, SSI_FSSPIN_PCTL and SSI_FSSPIN_POS
//
//*****************************************************************************
// #define SSI_FSSPIN_CLOCK_ENABLE  SYSCTL_RCGCGPIO_R0

//*****************************************************************************
//
// Selects the base address for the GPIO corresponding to SSI FSS pin
//
// Depends on: SSI_ENABLE_UPDATE
// Exclusive of: None
// Requires: SSI_FSSPIN_CLOCK_ENABLE, SSI_FSSPIN_PCTL and SSI_FSSPIN_POS
//
//*****************************************************************************
// #define SSI_FSSPIN_BASE          GPIO_PORTA_BASE

//*****************************************************************************
//
// Selects the port control value for the GPIO corresponding to SSI FSS pin
//
// Depends on: SSI_ENABLE_UPDATE
// Exclusive of: None
// Requires: SSI_FSSPIN_CLOCK_ENABLE, SSI_FSSPIN_BASE and SSI_FSSPIN_POS
//
//*****************************************************************************
// #define SSI_FSSPIN_PCTL          0x2

//*****************************************************************************
//
// Selects the pin number for the GPIO corresponding to SSI FSS pin
//
// Depends on: SSI_ENABLE_UPDATE
// Exclusive of: None
// Requires: SSI_FSSPIN_CLOCK_ENABLE, SSI_FSSPIN_BASE and SSI_FSSPIN_PCTL
//
//*****************************************************************************
// #define SSI_FSSPIN_POS           3

//*****************************************************************************
//
// Selects the clock enable for the GPIO corresponding to SSI MISO pin
//
// Depends on: SSI_ENABLE_UPDATE
// Exclusive of: None
// Requires: SSI_MISOPIN_BASE, SSI_MISOPIN_PCTL and SSI_MISOPIN_POS
//
//*****************************************************************************
// #define SSI_MISOPIN_CLOCK_ENABLE SYSCTL_RCGCGPIO_R0

//*****************************************************************************
//
// Selects the base address for the GPIO corresponding to SSI MISO pin
//
// Depends on: SSI_ENABLE_UPDATE
// Exclusive of: None
// Requires: SSI_MISOPIN_CLOCK_ENABLE, SSI_MISOPIN_PCTL and SSI_MISOPIN_POS
//
//*****************************************************************************
// #define SSI_MISOPIN_BASE         GPIO_PORTA_BASE

//*****************************************************************************
//
// Selects the port control value for the GPIO corresponding to SSI MISO pin
//
// Depends on: SSI_ENABLE_UPDATE
// Exclusive of: None
// Requires: SSI_MISOPIN_CLOCK_ENABLE, SSI_MISOPIN_BASE and SSI_MISOPIN_POS
//
//*****************************************************************************
// #define SSI_MISOPIN_PCTL         0x2

//*****************************************************************************
//
// Selects the pin number for the GPIO corresponding to SSI MISO pin
//
// Depends on: SSI_ENABLE_UPDATE
// Exclusive of: None
// Requires: SSI_MISOPIN_CLOCK_ENABLE, SSI_MISOPIN_BASE and SSI_MISOPIN_PCTL
//
//*****************************************************************************
// #define SSI_MISOPIN_POS          5

//*****************************************************************************
//
// Selects the clock enable for the GPIO corresponding to SSI MOSI pin
//
// Depends on: SSI_ENABLE_UPDATE
// Exclusive of: None
// Requires: SSI_MOSIPIN_BASE, SSI_MOSIPIN_PCTL and SSI_MOSIPIN_POS
//
//*****************************************************************************
// #define SSI_MOSIPIN_CLOCK_ENABLE SYSCTL_RCGCGPIO_R0

//*****************************************************************************
//
// Selects the base address for the GPIO corresponding to SSI MOSI pin
//
// Depends on: SSI_ENABLE_UPDATE
// Exclusive of: None
// Requires: SSI_MOSIPIN_CLOCK_ENABLE, SSI_MOSIPIN_PCTL and SSI_MOSIPIN_POS
//
//*****************************************************************************
// #define SSI_MOSIPIN_BASE         GPIO_PORTA_BASE

//*****************************************************************************
//
// Selects the port control value for the GPIO corresponding to SSI MOSI pin
//
// Depends on: SSI_ENABLE_UPDATE
// Exclusive of: None
// Requires: SSI_MOSIPIN_CLOCK_ENABLE, SSI_MOSIPIN_BASE and SSI_MOSIPIN_POS
//
//*****************************************************************************
// #define SSI_MOSIPIN_PCTL         0x2

//*****************************************************************************
//
// Selects the pin number for the GPIO corresponding to SSI MOSI pin
//
// Depends on: SSI_ENABLE_UPDATE
// Exclusive of: None
// Requires: SSI_MOSIPIN_CLOCK_ENABLE, SSI_MOSIPIN_BASE and SSI_MOSIPIN_PCTL
//
//*****************************************************************************
// #define SSI_MOSIPIN_POS          4

//*****************************************************************************
//
// Selects the I2C port as the port for communicating with the boot loader.
//
// Depends on: None
// Exclusive of: CAN_ENABLE_UPDATE, ENET_ENABLE_UPDATE, SSI_ENABLE_UPDATE,
//               UART_ENABLE_UPDATE, USB_ENABLE_UPDATE
// Requires: I2C_SLAVE_ADDR, I2C_CLOCK_ENABLE, I2Cx_BASE,
//           I2C_SCLPIN_CLOCK_ENABLE, I2C_SCLPIN_BASE, I2C_SCLPIN_PCTL,
//           I2C_SCLPIN_POS, I2C_SDAPIN_CLOCK_ENABLE, I2C_SDAPIN_BASE,
//           I2C_SDAPIN_PCTL and I2C_SDAPIN_POS
//
//*****************************************************************************
// #define I2C_ENABLE_UPDATE

//*****************************************************************************
//
// Specifies the I2C address of the boot loader.
//
// Depends on: I2C_ENABLE_UPDATE
// Exclusive of: None
// Requires: None
//
//*****************************************************************************
// #define I2C_SLAVE_ADDR          0x42

//*****************************************************************************
//
// Selects the clock enable for the I2C peripheral module
//
// Depends on: I2C_ENABLE_UPDATE
// Exclusive of: None
// Requires: I2Cx_BASE
//
//*****************************************************************************
// #define I2C_CLOCK_ENABLE        SYSCTL_RCGCI2C_R0

//*****************************************************************************
//
// Selects the base address of the I2C peripheral module
//
// Depends on: I2C_ENABLE_UPDATE
// Exclusive of: None
// Requires: I2C_CLOCK_ENABLE
//
//*****************************************************************************
// #define I2Cx_BASE      I2C0_BASE

//*****************************************************************************
//
// Selects the clock enable for the GPIO corresponding to I2C SCL pin
//
// Depends on: I2C_ENABLE_UPDATE
// Exclusive of: None
// Requires: I2C_SCLPIN_BASE, I2C_SCLPIN_PCTL and I2C_SCLPIN_POS
//
//*****************************************************************************
// #define I2C_SCLPIN_CLOCK_ENABLE SYSCTL_RCGCGPIO_R1

//*****************************************************************************
//
// Selects the base address for the GPIO corresponding to I2C SCL pin
//
// Depends on: I2C_ENABLE_UPDATE
// Exclusive of: None
// Requires: I2C_SCLPIN_CLOCK_ENABLE, I2C_SCLPIN_PCTL and I2C_SCLPIN_POS
//
//*****************************************************************************
// #define I2C_SCLPIN_BASE         GPIO_PORTB_BASE

//*****************************************************************************
//
// Selects the port control value for the GPIO corresponding to I2C SCL pin
//
// Depends on: I2C_ENABLE_UPDATE
// Exclusive of: None
// Requires: I2C_SCLPIN_CLOCK_ENABLE, I2C_SCLPIN_BASE and I2C_SCLPIN_POS
//
//*****************************************************************************
// #define I2C_SCLPIN_PCTL         0x3

//*****************************************************************************
//
// Selects the pin number for the GPIO corresponding to I2C SCL pin
//
// Depends on: I2C_ENABLE_UPDATE
// Exclusive of: None
// Requires: I2C_SCLPIN_CLOCK_ENABLE, I2C_SCLPIN_BASE and I2C_SCLPIN_PCTL
//
//*****************************************************************************
// #define I2C_SCLPIN_POS          2

//*****************************************************************************
//
// Selects the clock enable for the GPIO corresponding to I2C SDA pin
//
// Depends on: I2C_ENABLE_UPDATE
// Exclusive of: None
// Requires: I2C_SDAPIN_BASE, I2C_SDAPIN_PCTL and I2C_SDAPIN_POS
//
//*****************************************************************************
// #define I2C_SDAPIN_CLOCK_ENABLE SYSCTL_RCGCGPIO_R1

//*****************************************************************************
//
// Selects the base address for the GPIO corresponding to I2C SDA pin
//
// Depends on: I2C_ENABLE_UPDATE
// Exclusive of: None
// Requires: I2C_SDAPIN_CLOCK_ENABLE, I2C_SDAPIN_PCTL and I2C_SDAPIN_POS
//
//*****************************************************************************
// #define I2C_SDAPIN_BASE         GPIO_PORTB_BASE

//*****************************************************************************
//
// Selects the port control value for the GPIO corresponding to I2C SDA pin
//
// Depends on: I2C_ENABLE_UPDATE
// Exclusive of: None
// Requires: I2C_SDAPIN_CLOCK_ENABLE, I2C_SDAPIN_BASE and I2C_SDAPIN_POS
//
//*****************************************************************************
// #define I2C_SDAPIN_PCTL         0x3

//*****************************************************************************
//
// Selects the pin number for the GPIO corresponding to I2C SDA pin
//
// Depends on: I2C_ENABLE_UPDATE
// Exclusive of: None
// Requires: I2C_SDAPIN_CLOCK_ENABLE, I2C_SDAPIN_BASE and I2C_SDAPIN_PCTL
//
//*****************************************************************************
// #define I2C_SDAPIN_POS          3

//*****************************************************************************
//
// Selects Ethernet update via the BOOTP/TFTP protocol.
//
// Depends on: None
// Exclusive of: CAN_ENABLE_UPDATE, I2C_ENABLE_UPDATE, SSI_ENABLE_UPDATE,
//               UART_ENABLE_UPDATE, USB_ENABLE_UPDATE
// Requires: CRYSTAL_FREQ
//
//*****************************************************************************
// #define ENET_ENABLE_UPDATE

//*****************************************************************************
//
// Selects if the Ethernet LEDs should be enabled.
//
// Depends on: ENET_ENABLE_UPDATE
// Exclusive of: None
// Requires: None
//
//*****************************************************************************
// #define ENET_ENABLE_LEDS

//*****************************************************************************
//
// Selects the Ethernet MAC address.  If not specified, the MAC address is
// taken from the user registers.
//
// Depends on: ENET_ENABLE_UPDATE
// Exclusive of: None
// Requires: None
//
//*****************************************************************************
// #define ENET_MAC_ADDR0          0x00
// #define ENET_MAC_ADDR1          0x00
// #define ENET_MAC_ADDR2          0x00
// #define ENET_MAC_ADDR3          0x00
// #define ENET_MAC_ADDR4          0x00
// #define ENET_MAC_ADDR5          0x00

//*****************************************************************************
//
// Sets the name of the BOOTP server to use.  This can be used to request that
// a particular BOOTP server respond to our request; the value will be either
// the server's name, or a nickname used by that server.  If not defined then
// any BOOTP server is allowed to respond.
//
// Depends on: ENET_ENABLE_UPDATE
// Exclusive of: None
// Requires: None
//
//*****************************************************************************
// #define ENET_BOOTP_SERVER       "tiva"

//*****************************************************************************
//
// Selects USB update via Device Firmware Update class.
//
// Depends on: None
// Exclusive of: CAN_ENABLE_UPDATE, ENET_ENABLE_UPDATE, I2C_ENABLE_UPDATE,
//               SSI_ENABLE_UPDATE, UART_ENABLE_UPDATE,
// Requires: CRYSTAL_FREQ, USB_VENDOR_ID, USB_PRODUCT_ID
//
//*****************************************************************************
// #define USB_ENABLE_UPDATE

//*****************************************************************************
//
// The USB vendor ID published by the DFU device.  This value is the TI
// Tiva vendor ID.  Change this to the vendor ID you have been assigned by
// USB-IF.
//
// Depends on: USB_ENABLE_UPDATE
// Exclusive of: None
// Requires: None
//
//*****************************************************************************
// #define USB_VENDOR_ID           0x1cbe

//*****************************************************************************
//
// The USB device ID published by the DFU device.  If you are using your own
// vendor ID, chose a device ID that is different from the ID you use in
// non-update operation.  If you have sublicensed TI's vendor ID, you must
// use an assigned product ID here.
//
// Depends on: USB_ENABLE_UPDATE
// Exclusive of: None
// Requires: None
//
//*****************************************************************************
// #define USB_PRODUCT_ID          0x00ff

//*****************************************************************************
//
// Selects the BCD USB device release number published in the device
// descriptor.
//
// Depends on: USB_ENABLE_UPDATE
// Exclusive of: None
// Requires: None
//
//*****************************************************************************
// #define USB_DEVICE_ID           0x0001

//*****************************************************************************
//
// Sets the maximum power consumption that the DFU device will report to the
// USB host in the configuration descriptor.  Units are milliamps.
//
// Depends on: USB_ENABLE_UPDATE
// Exclusive of: None
// Requires: None
//
//*****************************************************************************
// #define USB_MAX_POWER           150

//*****************************************************************************
//
// Determines whether the DFU device reports to the host that it is self
// powered (defined as 0) or bus powered (defined as 1).
//
// Depends on: USB_ENABLE_UPDATE
// Exclusive of: None
// Requires: None
//
//*****************************************************************************
// #define USB_BUS_POWERED         1

//*****************************************************************************
//
// Specifies the GPIO peripheral associated with the USB host/device mux.
//
// Depends on: USB_ENABLE_UPDATE
// Exclusive of: None
// Requires: USB_MUX_PERIPH, USB_MUX_PORT, USB_MUX_PIN, USB_MUX_DEVICE
//
//*****************************************************************************
// #define USB_HAS_MUX

//*****************************************************************************
//
// Specifies the GPIO peripheral associated with the USB host/device mux.
//
// Depends on: USB_ENABLE_UPDATE, USB_HAS_MUX
// Exclusive of: None
// Requires: None
//
//*****************************************************************************
// #define USB_MUX_PERIPH          SYSCTL_RCGC2_GPIOH

//*****************************************************************************
//
// Specifies the GPIO port associated with the USB host/device mux.
//
// Depends on: USB_ENABLE_UPDATE, USB_HAS_MUX
// Exclusive of: None
// Requires: None
//
//*****************************************************************************
// #define USB_MUX_PORT            GPIO_PORTH_BASE

//*****************************************************************************
//
// Specifies the GPIO pin number used to switch the USB host/device mux.  Valid
// values are 0 through 7.
//
// Depends on: USB_ENABLE_UPDATE, USB_HAS_MUX
// Exclusive of: None
// Requires: None
//
//*****************************************************************************
// #define USB_MUX_PIN             2

//*****************************************************************************
//
// Specifies the state to set the GPIO pin to to select USB device mode via
// the USB host/device mux.  Valid values are 1 (high) or 0 (low).
//
// Depends on: USB_ENABLE_UPDATE, USB_HAS_MUX
// Exclusive of: None
// Requires: None
//
//*****************************************************************************
// #define USB_MUX_DEVICE          1

//*****************************************************************************
//
// Specifies whether the target board requires configuration of the pin used
// for VBUS.  This applies to Blizzard class and later devices.
//
// Depends on: USB_ENABLE_UPDATE
// Exclusive of: None
// Requires: USB_VBUS_PERIPH, USB_VBUS_PORT, USB_VBUS_PIN
//
//*****************************************************************************
// #define USB_VBUS_CONFIG

//*****************************************************************************
//
// Specifies the GPIO peripheral containing the pin which is used for VBUS.
// The value is of the form SYSCTL_RCGCGPIO_Rx, where the Rx represents
// the required GPIO port.  This applies to Blizzard class and later
// devices.
//
// Depends on: USB_ENABLE_UPDATE, USB_VBUS_CONFIG
// Exclusive of: None
// Requires: None
//
//*****************************************************************************
// #define USB_VBUS_PERIPH          SYSCTL_RCGCGPIO_R1

//*****************************************************************************
//
// Specifies the GPIO port containing the pin which is used for VBUS.  The value
// is of the form GPIO_PORTx_BASE, where PORTx represents the required GPIO
// port.
//
// Depends on: USB_ENABLE_UPDATE, USB_VBUS_CONFIG
// Exclusive of: None
// Requires: None
//
//*****************************************************************************
// #define USB_VBUS_PORT            GPIO_PORTB_BASE

//*****************************************************************************
//
// Specifies the GPIO pin number used for VBUS.  Valid values are 0 through 7.
//
// Depends on: USB_ENABLE_UPDATE, USB_VBUS_CONFIG
// Exclusive of: None
// Requires: None
//
//*****************************************************************************
// #define USB_VBUS_PIN             1

//*****************************************************************************
//
// Specifies whether the target board requires configuration of the pin used
// for ID.  This applies to Blizzard class and later devices.
//
// Depends on: USB_ENABLE_UPDATE
// Exclusive of: None
// Requires: USB_ID_PERIPH, USB_ID_PORT, USB_ID_PIN
//
//*****************************************************************************
// #define USB_ID_CONFIG

//*****************************************************************************
//
// Specifies the GPIO peripheral containing the pin which is used for ID.
// The value is of the form SYSCTL_RCGCGPIO_Rx, where the Rx represents
// the required GPIO port.  This applies to Blizzard class and later
// devices.
//
// Depends on: USB_ENABLE_UPDATE, USB_ID_CONFIG
// Exclusive of: None
// Requires: None
//
//*****************************************************************************
// #define USB_ID_PERIPH          SYSCTL_RCGCGPIO_R1

//*****************************************************************************
//
// Specifies the GPIO port containing the pin which is used for ID.  The value
// is of the form GPIO_PORTx_BASE, where PORTx represents the required GPIO
// port.
//
// Depends on: USB_ENABLE_UPDATE, USB_ID_CONFIG
// Exclusive of: None
// Requires: None
//
//*****************************************************************************
// #define USB_ID_PORT            GPIO_PORTB_BASE

//*****************************************************************************
//
// Specifies the GPIO pin number used for ID.  Valid values are 0 through 7.
//
// Depends on: USB_ENABLE_UPDATE, USB_ID_CONFIG
// Exclusive of: None
// Requires: None
//
//*****************************************************************************
// #define USB_ID_PIN             0

//*****************************************************************************
//
// Specifies whether the target board requires configuration of the pin used
// for DP.  This applies to Blizzard class and later devices.
//
// Depends on: USB_ENABLE_UPDATE
// Exclusive of: None
// Requires: USB_DP_PERIPH, USB_DP_PORT, USB_DP_PIN
//
//*****************************************************************************
// #define USB_DP_CONFIG

//*****************************************************************************
//
// Specifies the GPIO peripheral containing the pin which is used for DP.
// The value is of the form SYSCTL_RCGCGPIO_Rx, where the Rx represents
// the required GPIO port.  This applies to Blizzard class and later
// devices.
//
// Depends on: USB_ENABLE_UPDATE, USB_DP_CONFIG
// Exclusive of: None
// Requires: None
//
//*****************************************************************************
// #define USB_DP_PERIPH          SYSCTL_RCGCGPIO_R10

//*****************************************************************************
//
// Specifies the GPIO port containing the pin which is used for DP.  The value
// is of the form GPIO_PORTx_BASE, where PORTx represents the required GPIO
// port.
//
// Depends on: USB_ENABLE_UPDATE, USB_DP_CONFIG
// Exclusive of: None
// Requires: None
//
//*****************************************************************************
// #define USB_DP_PORT            GPIO_PORTL_BASE

//*****************************************************************************
//
// Specifies the GPIO pin number used for DP.  Valid values are 0 through 7.
//
// Depends on: USB_ENABLE_UPDATE, USB_DP_CONFIG
// Exclusive of: None
// Requires: None
//
//*****************************************************************************
// #define USB_DP_PIN             6

//*****************************************************************************
//
// Specifies whether the target board requires configuration of the pin used
// for DM.  This applies to Blizzard class and later devices.
//
// Depends on: USB_ENABLE_UPDATE
// Exclusive of: None
// Requires: USB_DM_PERIPH, USB_DM_PORT, USB_DM_PIN
//
//*****************************************************************************
// #define USB_DM_CONFIG

//*****************************************************************************
//
// Specifies the GPIO peripheral containing the pin which is used for DM.
// The value is of the form SYSCTL_RCGCGPIO_Rx, where the Rx represents
// the required GPIO port.  This applies to Blizzard class and later
// devices.
//
// Depends on: USB_ENABLE_UPDATE, USB_DM_CONFIG
// Exclusive of: None
// Requires: None
//
//*****************************************************************************
// #define USB_DM_PERIPH          SYSCTL_RCGCGPIO_R10

//*****************************************************************************
//
// Specifies the GPIO port containing the pin which is used for DM.  The value
// is of the form GPIO_PORTx_BASE, where PORTx represents the required GPIO
// port.
//
// Depends on: USB_ENABLE_UPDATE, USB_DM_CONFIG
// Exclusive of: None
// Requires: None
//
//*****************************************************************************
// #define USB_DM_PORT            GPIO_PORTL_BASE

//*****************************************************************************
//
// Specifies the GPIO pin number used for DM.  Valid values are 0 through 7.
//
// Depends on: USB_ENABLE_UPDATE, USB_DM_CONFIG
// Exclusive of: None
// Requires: None
//
//*****************************************************************************
// #define USB_DM_PIN             7

//*****************************************************************************
//
// Selects the CAN port as the port for communicating with the boot loader.
//
// Depends on: None
// Exclusive of: ENET_ENABLE_UPDATE, I2C_ENABLE_UPDATE, SSI_ENABLE_UPDATE,
//               UART_ENABLE_UPDATE, USB_ENABLE_UPDATE
// Requires: CAN_RX_PERIPH, CAN_RX_PORT, CAN_RX_PIN, CAN_TX_PERIPH,
//           CAN_TX_PORT, CAN_TX_PIN, CAN_BIT_RATE, CRYSTAL_FREQ.
//
//*****************************************************************************
// #define CAN_ENABLE_UPDATE

//*****************************************************************************
//
// Enables the UART to CAN bridging for use when the CAN port is selected for
// communicating with the boot loader.
//
// Depends on: CAN_ENABLE_UPDATE
// Exclusive of: None
// Requires: None
//
//*****************************************************************************
// #define CAN_UART_BRIDGE

//*****************************************************************************
//
// Specifies the GPIO peripheral associated with CAN0 RX pin used by the boot
// loader.
//
// Depends on: CAN_ENABLE_UPDATE
// Exclusive of: None
// Requires: None
//
//*****************************************************************************
// #define CAN_RX_PERIPH           SYSCTL_RCGC2_GPIOA

//*****************************************************************************
//
// Specifies the GPIO port associated with CAN0 RX pin used by the boot loader.
//
// Depends on: CAN_ENABLE_UPDATE
// Exclusive of: None
// Requires: None
//
//*****************************************************************************
// #define CAN_RX_PORT             GPIO_PORTA_BASE

//*****************************************************************************
//
// Specifies the GPIO pin number associated with CAN0 RX pin used by the boot
// loader.
//
// Depends on: CAN_ENABLE_UPDATE
// Exclusive of: None
// Requires: None
//
//*****************************************************************************
// #define CAN_RX_PIN              4

//*****************************************************************************
//
// Specifies the GPIO peripheral associated with CAN0 TX pin used by the boot
// loader.
//
// Depends on: CAN_ENABLE_UPDATE
// Exclusive of: None
// Requires: None
//
//*****************************************************************************
// #define CAN_TX_PERIPH           SYSCTL_RCGC2_GPIOA

//*****************************************************************************
//
// Specifies the GPIO port associated with CAN0 TX pin used by the boot loader.
//
// Depends on: CAN_ENABLE_UPDATE
// Exclusive of: None
// Requires: None
//
//*****************************************************************************
// #define CAN_TX_PORT             GPIO_PORTA_BASE

//*****************************************************************************
//
// Specifies the GPIO pin number associated with CAN0 TX pin used by the boot
// loader.
//
// Depends on: CAN_ENABLE_UPDATE
// Exclusive of: None
// Requires: None
//
//*****************************************************************************
// #define CAN_TX_PIN              5

//*****************************************************************************
//
// Specifies the bit rate for CAN0 used by the boot loader.
//
// Depends on: CAN_ENABLE_UPDATE
// Exclusive of: None
// Requires: None
//
//*****************************************************************************
// #define CAN_BIT_RATE            1000000

//*****************************************************************************
//
// Boot loader hook functions.
//
// The following defines allow you to add application-specific function which
// are called at various points during boot loader execution.
//
//*****************************************************************************

//*****************************************************************************
//
// Performs application-specific low level hardware initialization on system
// reset.
//
// If hooked, this function will be called immediately after the boot loader
// code relocation completes.  An application may perform any required low
// hardware initialization during this function.  Note that the system clock
// has not been set when this function is called.  Initialization that assumes
// the system clock is set may be performed in the BL_INIT_FN_HOOK function
// instead.
//
// void MyHwInitFunc(void);
//
//*****************************************************************************
#define BL_HW_INIT_FN_HOOK bl_user_init_hw_fn

//*****************************************************************************
//
// Performs application-specific initialization on system reset.
//
// If hooked, this function will be called immediately after the boot loader
// sets the system clock.  An application may perform any additional
// initialization during this function.
//
// void MyInitFunc(void);
//
//*****************************************************************************
// #define BL_INIT_FN_HOOK         bl_user_init_fn

//*****************************************************************************
//
// Performs application-specific reinitialization on boot loader entry via SVC.
//
// If hooked, this function will be called immediately after the boot loader
// reinitializes the system clock when it is entered from an application
// via the SVC mechanism rather than as a result of a system reset.  An
// application may perform any additional reinitialization in this function.
//
// void MyReinitFunc(void);
//
//*****************************************************************************
// #define BL_REINIT_FN_HOOK       MyReinitFunc

//*****************************************************************************
//
// Informs an application that a download is starting.
//
// If hooked, this function will be called when a new firmware download is
// about to start.  The application may use this signal to initialize any
// progress display.
//
// void MyStartFunc(void);
//
//*****************************************************************************
// #define BL_START_FN_HOOK        MyStartFunc

//*****************************************************************************
//
// Informs an application of download progress.
//
// If hooked, this function will be called periodically during firmware
// download.  The application may use this to update its user interface.
// When using a protocol which does not inform the client of the final size of
// the download in advance (e.g. TFTP), the ulTotal parameter will be 0,
// otherwise it indicates the expected size of the complete download.
//
// void MyProgressFunc(unsigned long ulCompleted, unsigned long ulTotal);
//
// where:
//
// - ulCompleted indicates the number of bytes already downloaded.
// - ulTotal indicates the number of bytes expected or 0 if this is not known.
//
//*****************************************************************************
#define BL_PROGRESS_FN_HOOK bl_user_progress_hook

//*****************************************************************************
//
// Informs an application that a download has completed.
//
// If hooked, this function will be called when a firmware download ends.
// The application may use this signal to update its user interface.  Typically
// a system reset will occur shortly after this function returns as the boot
// loader attempts to boot the new image.
//
// void MyEndFunc(void);
//
//*****************************************************************************
#define BL_END_FN_HOOK bl_user_end_hook

//*****************************************************************************
//
// Allows an application to perform in-place data decryption during download.
//
// If hooked, this function will be called on receipt of any new block of
// downloaded firmware image data.  The application must decrypt this data
// in place then return at which point the boot loader will write the data to
// flash.
//
// void MyDecryptionFunc(unsigned char *pucBuffer, unsigned long ulSize);
//
// where:
//
// - pucBuffer points to the first byte of data to be decrypted.
// - ulSize indicates the number of bytes of data at pucBuffer.
//
//*****************************************************************************
// #define BL_DECRYPT_FN_HOOK      MyDecryptionFunc

//*****************************************************************************
//
// Allows an application to force a new firmware download.
//
// If hooked, this function will be called after a system reset (following
// basic initialization and the initialization hook function) to give the
// application an opportunity to force a new firmware download.  Depending upon
// the return code, the boot loader will either boot the existing firmware
// image or wait for a new download to be started.
//
// Note that this hook takes precedence over ENABLE_UPDATE_CHECK settings.  If
// the hook function is defined, the basic GPIO check offered by
// ENABLE_UPDATE_CHECK does not take place.
//
// unsigned long MyCheckUpdateFunc(void);
//
// where the return code is 0 if the boot loader should boot the existing
// image (if found) or non-zero to indicate that the boot loader should retain
// control and wait for a new firmware image to be downloaded.
//
//*****************************************************************************
#define BL_CHECK_UPDATE_FN_HOOK bl_user_checkupdate_hook

//*****************************************************************************
//
// Allows an application to replace the flash block erase function.
//
// If hooked, this function will be called whenever a block of flash is to
// be erased.  The function must erase the block and block until the operation
// has completed.  The size of the block which will be erased is defined by
// FLASH_BLOCK_SIZE.
//
// void MyFlashEraseFunc(unsigned long ulBlockAddr);
//
// where:
//
// - ulBlockAddr is the address of the flash block to be erased.
//
//*****************************************************************************
// #define BL_FLASH_ERASE_FN_HOOK  MyFlashEraseFunc

//*****************************************************************************
//
// Allows an application to replace the flash programming function.
//
// If hooked, this function will be called whenever a block of data is to be
// be written to flash.  The function must program the supplied data and block
// until the operation has has completed.
//
// void MyFlashProgramFunc(unsigned long ulDstAddr,
//                         unsigned char *pucSrcData,
//                         unsigned long ulLength);
//
// where:
//
// - ulDstAddr is the address in flash at which the data is to be programmed.
//   This must be a multiple of 4.
// - pucSrcData points to the first byte of the data to program.
// - ulLength is the number of bytes of data to program. This must be a
//   multiple of 4.
//
//*****************************************************************************
// #define BL_FLASH_PROGRAM_FN_HOOK MyFlashProgramFunc

//*****************************************************************************
//
// Allows an application to replace the flash error clear function.
//
// If hooked, this function will be called before each flash erase or program
// operation.  The function must clear any flash error indicators and prepare
// to detect access violations that may occur in a future erase or program
// operation.
//
// void MyFlashClearErrorFunc(void);
//
//*****************************************************************************
// #define BL_FLASH_CL_ERR_FN_HOOK MyFlashClearErrorFunc

//*****************************************************************************
//
// Reports whether or not a flash access violation error has occurred.
//
// If hooked, this function will be called after flash erase or program
// operations.  The return code indicates to the caller whether or not
// an access violation error has occurred since the last call to the function
// defined by BL_FLASH_CL_ERR_FN_HOOK.
//
// unsigned long MyFlashErrorFunc(void);
//
// where the return code is 0 if no error has occurred or non-zero if an
// error was detected.
//
//*****************************************************************************
// #define BL_FLASH_ERROR_FN_HOOK  bl_user_flash_error

//*****************************************************************************
//
// Reports the total size of the device flash.
//
// If hooked, this function will be called to determine the size of the flash
// device.
//
// unsigned long MyFlashSizeFunc(void);
//
// where the return code is the total number of bytes of flash supported by the
// device.  Note that this does not take into account any reserved space
// defined via the FLASH_RSVD_SPACE value in this header file.
//
//*****************************************************************************
// #define BL_FLASH_SIZE_FN_HOOK   MyFlashSizeFunc

//*****************************************************************************
//
// Reports the address of the first byte after the end of the device flash.
//
// If hooked, this function will be called to determine the address of the end
// of valid flash.
//
// unsigned long MyFlashEndFunc(void);
//
// where the return code is the address of the first byte after the end of flash.
// Note that this does not take into account any reserved space defined via
// the FLASH_RSVD_SPACE value in this header file.
//
//*****************************************************************************
// #define BL_FLASH_END_FN_HOOK    MyFlashEndFunc

//*****************************************************************************
//
// Checks whether the start address and size of an image are valid.
//
// If hooked, this function will be called whenever a new download is to be
// started.  It determines whether or not an image of a particular size may be
// flashed at a given address.  Valid addresses are:
//
//   1. APP_START_ADDRESS in all cases.
//   2. 0x00000000 if ENABLE_BL_UPDATE is defined.
//   3. The start of the reserved space if FLASH_RSVD_SPACE is defined.
//
// unsigned long MyFlashAddrCheckFunc(unsigned long ulAddr,
//                                    unsigned long ulSize);
//
// where:
//
// - ulAddr is the address in flash at which the image is to be programmed.
// - ulSize is the total size of the image if known or 0 otherwise.
//
// The return code will be 0 if the address or size is invalid or a non-zero
// value if valid.
//
//*****************************************************************************
// #define BL_FLASH_AD_CHECK_FN_HOOK MyFlashAddrCheckFunc

#endif // __BL_CONFIG_H__
