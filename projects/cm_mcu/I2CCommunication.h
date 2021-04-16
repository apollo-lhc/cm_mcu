/*
 * I2CCommunication.h
 *
 *  Created on: Sep 3, 2020
 *      Author: rzou
 */

#ifndef PROJECTS_CM_MCU_I2CCOMMUNICATION_H_
#define PROJECTS_CM_MCU_I2CCOMMUNICATION_H_
// includes for types
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>

#include "FreeRTOS.h"
#include "FreeRTOSConfig.h"
#include "stream_buffer.h"
#include "common/smbus.h"

// Command line interface
int apollo_i2c_ctl_set_dev(uint8_t base);
int apollo_i2c_ctl_r(uint8_t address, uint8_t nbytes, uint8_t data[4]);
int apollo_i2c_ctl_reg_r(tSMBus **smbus, uint8_t address, uint8_t reg_address, uint8_t nbytes, uint8_t data[4]);
int apollo_i2c_ctl_w(uint8_t address, uint8_t nbytes, int value);
int apollo_i2c_ctl_reg_w(uint8_t address, uint8_t reg_address, uint8_t nbytes, int packed_data);

#endif /* PROJECTS_CM_MCU_I2CCOMMUNICATION_H_ */
