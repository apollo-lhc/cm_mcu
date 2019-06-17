// higher level utilities for I2C
#include <stdarg.h>
#include <stdbool.h>
#include <stdint.h>
#include "inc/hw_i2c.h"
#include "inc/hw_types.h"
#include "inc/hw_gpio.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "driverlib/i2c.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"

#include "common/i2c_reg.h"
#include "common/pinsel.h"
#include "common/utils.h"


#ifdef USE_FREERTOS
#include "FreeRTOS.h"
#endif // USE_FREERTOS

// This array helps us simplify the use of different I2C devices in the board.
uint32_t I2C_BASE[] = {
  I2C0_BASE,
  I2C1_BASE,
  I2C2_BASE,
  I2C3_BASE,
  I2C4_BASE,
  I2C5_BASE,
  I2C6_BASE,
  I2C7_BASE,
  I2C8_BASE,
  I2C9_BASE
};


//initialize I2C module 1
// Slightly modified version of TI's example code
// TODO: add for I2C modules 0, 2, 4 and 6
void initI2C1(const uint32_t sysclockfreq)
{
    //enable I2C module 1
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C1);
    //
    // Wait for the I2C1 module to be ready.
    //
    while(!MAP_SysCtlPeripheralReady(SYSCTL_PERIPH_I2C1))
    {
    }

    // Stop the Clock, Reset and Enable I2C Module
    // in Master Function
    //
    MAP_SysCtlPeripheralDisable(SYSCTL_PERIPH_I2C1);
    MAP_SysCtlPeripheralReset(SYSCTL_PERIPH_I2C1);
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C1);

    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_I2C1));

    // Enable and initialize the I2C master module.  Use the system clock for
    // the I2C1 module.  The last parameter sets the I2C data transfer rate.
    // If false the data rate is set to 100kbps and if true the data rate will
    // be set to 400kbps.
    MAP_I2CMasterInitExpClk(I2C1_BASE, sysclockfreq, false);
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_I2C1));

    //clear I2C FIFOs
    //HWREG(I2C1_BASE + I2C_0_FIFOCTL) = 80008000;
    I2CRxFIFOFlush(I2C1_BASE);
    I2CTxFIFOFlush(I2C1_BASE);

    // toggle relevant reset
    write_gpio_pin(_PWR_I2C_RESET, 0x0); // active low
    SysCtlDelay(sysclockfreq/10);;
    write_gpio_pin(_PWR_I2C_RESET, 0x1); // active low


}

// I2C controller 3 is for V_OPTICS on the CM
void initI2C3(const uint32_t sysclockfreq)
{
    //enable I2C module 3
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C3);
    //
    // Wait for the I2C3 module to be ready.
    //
    while(!MAP_SysCtlPeripheralReady(SYSCTL_PERIPH_I2C3))
    {
    }

    // Stop the Clock, Reset and Enable I2C Module
    // in Master Function
    //
    MAP_SysCtlPeripheralDisable(SYSCTL_PERIPH_I2C3);
    MAP_SysCtlPeripheralReset(SYSCTL_PERIPH_I2C3);
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C3);

    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_I2C3));

    // Enable and initialize the I2C master module.  Use the system clock for
    // the I2C3 module.  The last parameter sets the I2C data transfer rate.
    // If false the data rate is set to 100kbps and if true the data rate will
    // be set to 400kbps.
    MAP_I2CMasterInitExpClk(I2C3_BASE, sysclockfreq, false);
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_I2C3));

    //clear I2C FIFOs
    //HWREG(I2C3_BASE + I2C_0_FIFOCTL) = 80008000;
    I2CRxFIFOFlush(I2C3_BASE);
    I2CTxFIFOFlush(I2C3_BASE);

    // toggle relevant reset
    write_gpio_pin(_V_OPTICS_I2C_RESET, 0x0); // active low
    SysCtlDelay(sysclockfreq/10);;
    write_gpio_pin(_V_OPTICS_I2C_RESET, 0x1); // active low


}


// I2C controller 4 is for K_OPTICS on the CM
void initI2C4(const uint32_t sysclockfreq)
{
    //enable I2C module 3
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C4);
    //
    // Wait for the I2C4 module to be ready.
    //
    while(!MAP_SysCtlPeripheralReady(SYSCTL_PERIPH_I2C4))
    {
    }

    // Stop the Clock, Reset and Enable I2C Module
    // in Master Function
    //
    MAP_SysCtlPeripheralDisable(SYSCTL_PERIPH_I2C4);
    MAP_SysCtlPeripheralReset(SYSCTL_PERIPH_I2C4);
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C4);

    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_I2C4));

    // Enable and initialize the I2C master module.  Use the system clock for
    // the I2C4 module.  The last parameter sets the I2C data transfer rate.
    // If false the data rate is set to 100kbps and if true the data rate will
    // be set to 400kbps.
    MAP_I2CMasterInitExpClk(I2C4_BASE, sysclockfreq, false);
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_I2C4));

    //clear I2C FIFOs
    //HWREG(I2C4_BASE + I2C_0_FIFOCTL) = 80008000;
    I2CRxFIFOFlush(I2C4_BASE);
    I2CTxFIFOFlush(I2C4_BASE);

    // toggle relevant reset
    write_gpio_pin(_K_OPTICS_I2C_RESET, 0x0); // active low
    SysCtlDelay(sysclockfreq/10);;
    write_gpio_pin(_K_OPTICS_I2C_RESET, 0x1); // active low


}

// I2C controller 6 is for FPGAs on the CM
void initI2C6(const uint32_t sysclockfreq)
{
    //enable I2C module 3
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C6);
    //
    // Wait for the I2C6 module to be ready.
    //
    while(!MAP_SysCtlPeripheralReady(SYSCTL_PERIPH_I2C6))
    {
    }

    // Stop the Clock, Reset and Enable I2C Module
    // in Master Function
    //
    MAP_SysCtlPeripheralDisable(SYSCTL_PERIPH_I2C6);
    MAP_SysCtlPeripheralReset(SYSCTL_PERIPH_I2C6);
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C6);

    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_I2C6));

    // Enable and initialize the I2C master module.  Use the system clock for
    // the I2C6 module.  The last parameter sets the I2C data transfer rate.
    // If false the data rate is set to 100kbps and if true the data rate will
    // be set to 400kbps.
    MAP_I2CMasterInitExpClk(I2C6_BASE, sysclockfreq, false);
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_I2C6));

    //clear I2C FIFOs
    //HWREG(I2C6_BASE + I2C_0_FIFOCTL) = 80008000;
    I2CRxFIFOFlush(I2C6_BASE);
    I2CTxFIFOFlush(I2C6_BASE);

    // toggle relevant reset
    write_gpio_pin(_FPGA_I2C_RESET, 0x0); // active low
    SysCtlDelay(sysclockfreq/10);;
    write_gpio_pin(_FPGA_I2C_RESET, 0x1); // active low


}

// ---------------------------------------------------------------------
// There are two types of I2C devices: those with and those without internal registers.
// Subsequently there are two different types of reads and writes.


// Write to I2C register
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
    // check the error register
    uint32_t err = I2CMasterErr(i2cbase);
    if ( err != I2C_MASTER_ERR_NONE ) {
      return false;
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
        // check the error register
        err = I2CMasterErr(i2cbase);
        if ( err != I2C_MASTER_ERR_NONE ) {
          return false;
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
            // check the error register
            err = I2CMasterErr(i2cbase);
            if ( err != I2C_MASTER_ERR_NONE ) {
              return false;
            }


        }
    }
    return true;
}
// Read from an I2C device with an internal register
// this intentionally sends two starts without a finish, as per
// recommendation from various docs.
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

    // check the error register
    uint32_t err = I2CMasterErr(i2cbase);
    if ( err != I2C_MASTER_ERR_NONE ) {
      return false;
    }

    //specify that we are going to read from slave device
    I2CMasterSlaveAddrSet(i2cbase, ui8Addr, true);
    if ( ui8ByteCount == 1 ) {
        //send control byte and read from the register we
        //specified
        I2CMasterControl(i2cbase, I2C_MASTER_CMD_SINGLE_RECEIVE);

        //wait for MCU to finish transaction
        while(I2CMasterBusy(i2cbase));
        // check the error register
        err = I2CMasterErr(i2cbase);
        if ( err != I2C_MASTER_ERR_NONE ) {
          return false;
        }


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

            // check the error register
            err = I2CMasterErr(i2cbase);
            if ( err != I2C_MASTER_ERR_NONE ) {
              return false;
            }

           //return data pulled from the specified register
            Data[i] = I2CMasterDataGet(i2cbase);
        }
    }
    return true;

}

// non-register write
bool writeI2C(const uint32_t i2cbase, const uint8_t ui8Addr, uint8_t *Data,
    const uint8_t ui8ByteCount)
{

  // specify that we are writing to the
  // slave device. last argument is 'receive' boolean. false for write.
  I2CMasterSlaveAddrSet(i2cbase, ui8Addr, false);

  if ( ui8ByteCount == 1 ) {

    // Put the outgoing data into the control register
    I2CMasterDataPut(i2cbase, *Data);

    //send control byte and write the data
    I2CMasterControl(i2cbase, I2C_MASTER_CMD_SINGLE_SEND);

    //wait for MCU to finish transaction
    while(I2CMasterBusy(i2cbase));
    // see if transaction succeeded
    if ( I2CMasterErr(i2cbase) != I2C_MASTER_ERR_NONE )
      return false; // transaction failed

  }
  else {

    for ( uint8_t i = 0; i < ui8ByteCount; ++i ) {
      // Put the outgoing data into the control register
      I2CMasterDataPut(i2cbase, Data[i]);

      //send control byte and read from the register we
      //specified
      uint32_t cmd;
      if ( i == 0 ) { // initiate burst
        cmd = I2C_MASTER_CMD_BURST_SEND_START;
      }
      else if ( i == ui8ByteCount-1 ) { // end burst
        cmd = I2C_MASTER_CMD_BURST_SEND_FINISH;
      }
      else { // during burst
        cmd = I2C_MASTER_CMD_BURST_SEND_CONT;
      }
      I2CMasterControl(i2cbase, cmd);

      //wait for MCU to finish transaction
      while(I2CMasterBusy(i2cbase));

      // see if transaction succeeded
      if ( I2CMasterErr(i2cbase) != I2C_MASTER_ERR_NONE )
        return false; // transaction failed

    }
  }
  return true;
}


// non-register read
bool readI2C(const uint32_t i2cbase, const uint8_t ui8Addr, uint8_t *Data,
             const uint8_t ui8ByteCount)
{
  //specify that we are going to read from slave device
  I2CMasterSlaveAddrSet(i2cbase, ui8Addr, true);
  if ( ui8ByteCount == 1 ) {
    //send control byte and read from the register we
    //specified
    I2CMasterControl(i2cbase, I2C_MASTER_CMD_SINGLE_RECEIVE);

    //wait for MCU to finish transaction
    while(I2CMasterBusy(i2cbase));

    // check the error register
    uint32_t err = I2CMasterErr(i2cbase);
    if ( err != I2C_MASTER_ERR_NONE ) {
      return false;
    }


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
      // check the error register
      uint32_t err = I2CMasterErr(i2cbase);
      if ( err != I2C_MASTER_ERR_NONE ) {
        return false;
      }


      //return data pulled from the specified register
      Data[i] = I2CMasterDataGet(i2cbase);
    }
  }
  return true;

}

