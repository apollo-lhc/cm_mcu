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

void initI2C1(const uint32_t sysclockfreq);

bool writeI2Creg(uint32_t i2cbase, uint8_t ui8Addr, uint8_t ui8Reg, uint8_t *Data,
    const uint8_t ui8ByteCount);
bool readI2Creg(uint32_t i2cbase, uint8_t ui8Addr, uint8_t ui8Reg, uint8_t *Data, const uint8_t ui8ByteCount);




#endif /* COMMON_I2C_REG_H_ */
