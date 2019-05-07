/*
 * i2c_reg.h
 *
 *  Created on: Apr 10, 2019
 *      Author: wittich
 */

#ifndef PROJECTS_PROJECT2_I2C_REG_H_
#define PROJECTS_PROJECT2_I2C_REG_H_

#include <stdint.h>
#include <stdbool.h>

void initI2C1(void);

bool writeI2C(uint8_t ui8Addr, uint8_t ui8Reg, uint8_t *Data, const uint8_t ui8ByteCount);

bool readI2Creg(uint8_t ui8Addr, uint8_t ui8Reg, uint8_t *Data, const uint8_t ui8ByteCount);




#endif /* PROJECTS_PROJECT2_I2C_REG_H_ */
