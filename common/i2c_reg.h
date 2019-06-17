/*
 * i2c_reg.h
 *
 *  Created on: Apr 10, 2019
 *      Author: wittich
 */

#ifndef COMMON_I2C_REG_H_
#define COMMON_I2C_REG_H_

#include <stdint.h>
#include <stdbool.h>

#include "inc/hw_memmap.h"


// This array helps us simplify the use of different I2C devices in the board.
extern uint32_t I2C_BASE[];

void initI2C1(const uint32_t sysclockfreq); // power supply I2C bus
void initI2C3(const uint32_t sysclockfreq); // V optics I2C bus
void initI2C4(const uint32_t sysclockfreq); // K optics I2C bus
void initI2C6(const uint32_t sysclockfreq); // FPGA     I2C bus

// write to an i2c register
bool writeI2Creg(uint32_t i2cbase, uint8_t ui8Addr, uint8_t ui8Reg, uint8_t *Data,
                 const uint8_t ui8ByteCount);
// read from an i2c register
bool readI2Creg(uint32_t i2cbase, uint8_t ui8Addr, uint8_t ui8Reg, uint8_t *Data,
                const uint8_t ui8ByteCount);

// non-register write
bool writeI2C(const uint32_t i2cbase, const uint8_t ui8Addr, uint8_t *Data,
              const uint8_t ui8ByteCount);

// non-register read
bool readI2C(const uint32_t i2cbase, const uint8_t ui8Addr, uint8_t *Data,
             const uint8_t ui8ByteCount);

#endif /* COMMON_I2C_REG_H_ */
