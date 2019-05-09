// higher level utilities for I2C
#include <stdarg.h>
#include <stdbool.h>
#include <stdint.h>
#include "inc/hw_i2c.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_gpio.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "driverlib/i2c.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"

#include "common/i2c_reg.h"

//initialize I2C module 1
// Slightly modified version of TI's example code
// TODO: add for I2C modules 0, 2, 3, 4 and 6
void initI2C1(const uint32_t sysclockfreq)
{
    //enable I2C module 1
    SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C1);
    //
    // Wait for the I2C1 module to be ready.
    //
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_I2C1))
    {
    }

    //reset module
    SysCtlPeripheralReset(SYSCTL_PERIPH_I2C1);

    //enable GPIO peripheral that contains I2C 6
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

    // Configure the pin muxing for I2C1 functions on port A6 and A7.
    GPIOPinConfigure(GPIO_PA6_I2C6SCL);
    GPIOPinConfigure(GPIO_PA7_I2C6SDA);

    // Select the I2C function for these pins.
    GPIOPinTypeI2CSCL(GPIO_PORTA_BASE, GPIO_PIN_6);
    GPIOPinTypeI2C(GPIO_PORTA_BASE, GPIO_PIN_7);

    // Enable and initialize the I2C1 master module.  Use the system clock for
    // the I2C1 module.  The last parameter sets the I2C data transfer rate.
    // If false the data rate is set to 100kbps and if true the data rate will
    // be set to 400kbps.
    I2CMasterInitExpClk(I2C1_BASE, sysclockfreq, false);

//    //clear I2C FIFOs
//    HWREG(I2C1_BASE + I2C_0_FIFOCTL) = 80008000;
}

// added by PW
bool writeI2Creg(uint32_t i2cbase, uint8_t ui8Addr, uint8_t ui8Reg, uint8_t *Data,
		const uint8_t ui8ByteCount)
{

    //specify that we are writing (a register address) to the
    //slave device. last argument is 'receive' boolean. false for write.
    I2CMasterSlaveAddrSet(i2cbase, ui8Addr, false);

    //specify register to be written to
    I2CMasterDataPut(i2cbase, ui8Reg);

    // Initiate send of character from Master to Slave
    //
    I2CMasterControl(i2cbase, I2C_MASTER_CMD_BURST_SEND_START);
    //
    // Delay until transmission completes
    //
    while(I2CMasterBusy(i2cbase))
    {
    }
    //handle single
    if ( ui8ByteCount == 1 ) {
        //
        // Place the character to be sent in the data register
        //
        I2CMasterDataPut(i2cbase, *Data);
        //
        // Initiate send of character from Master to Slave
        //
        I2CMasterControl(i2cbase, I2C_MASTER_CMD_BURST_SEND_FINISH);
        //
        // Delay until transmission completes
        //
        while(I2CMasterBusy(i2cbase))
        {
        }

    }
    else { // more than one byte to send
        for ( int i = 0; i < ui8ByteCount; ++i ) {
            uint32_t cmd;
            if ( i == (ui8ByteCount-1) ) { // end burst
                cmd = I2C_MASTER_CMD_BURST_SEND_FINISH;
            }
            else { // during burst
                cmd = I2C_MASTER_CMD_BURST_SEND_CONT;
            }
            //UARTprintf("write: i=%d, command=%04x\n", i, cmd);
            //
            // Place the character to be sent in the data register
            //
            I2CMasterDataPut(i2cbase, Data[i]);
            //
            // Initiate send of character from Master to Slave
            //
            I2CMasterControl(i2cbase, cmd);
            //
            // Delay until transmission completes
            //
            while(I2CMasterBusy(i2cbase))
            {
            }
        }
    }
    return true;
}

bool readI2Creg(uint32_t i2cbase, uint8_t ui8Addr, uint8_t ui8Reg, uint8_t *Data,
		const uint8_t ui8ByteCount)
{
    //specify that we are writing (a register address) to the
    //slave device
    I2CMasterSlaveAddrSet(i2cbase, ui8Addr, false);

    //specify register to be read
    I2CMasterDataPut(i2cbase, ui8Reg);

    //send control byte and register address byte to slave device
    I2CMasterControl(i2cbase, I2C_MASTER_CMD_BURST_SEND_START);

    //wait for MCU to finish transaction
    while(I2CMasterBusy(i2cbase));

    //specify that we are going to read from slave device
    I2CMasterSlaveAddrSet(i2cbase, ui8Addr, true);
    if ( ui8ByteCount == 1 ) {
        //send control byte and read from the register we
        //specified
        I2CMasterControl(i2cbase, I2C_MASTER_CMD_SINGLE_RECEIVE);

        //wait for MCU to finish transaction
        while(I2CMasterBusy(i2cbase));

        //return data pulled from the specified register
        *Data = I2CMasterDataGet(i2cbase);
    }
    else {

        for ( uint8_t i = 0; i < ui8ByteCount; ++i ) {
            //send control byte and read from the register we
            //specified
            uint32_t cmd;
            if ( i == 0 ) { // initiate burst
                cmd = I2C_MASTER_CMD_BURST_RECEIVE_START;
            }
            else if ( i == ui8ByteCount-1 ) { // end burst
                cmd = I2C_MASTER_CMD_BURST_RECEIVE_FINISH;
            }
            else { // during burst
                cmd = I2C_MASTER_CMD_BURST_RECEIVE_CONT;
            }
            I2CMasterControl(i2cbase, cmd);

            //wait for MCU to finish transaction
            while(I2CMasterBusy(i2cbase));

            //return data pulled from the specified register
            Data[i] = I2CMasterDataGet(i2cbase);
        }
    }
    return true;

}


