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

#include "common/smbus.h"

#define MAX_BYTES_ADDR 2
#define MAX_BYTES      4
#define I2C_MAX_TRIES 25

// Command line interface
// read an I2C device, without restart
int apollo_i2c_ctl_r(uint8_t device, uint8_t address, uint8_t nbytes, 
                     uint8_t data[4]);
// read a register from an I2C device, with restart
int apollo_i2c_ctl_reg_r(uint8_t device, uint8_t address, uint8_t nbytes_addr,
                         uint16_t packed_reg_address, uint8_t nbytes,
                         uint32_t *packed_data);
// write an I2C device, without restart
int apollo_i2c_ctl_w(uint8_t device, uint8_t address, uint8_t nbytes, 
                     unsigned int value);
// write a register to an I2C device, with restart
int apollo_i2c_ctl_reg_w(uint8_t device, uint8_t address, uint8_t nbytes_addr,
                         uint16_t packed_reg_address, uint8_t nbytes,
                         uint32_t packed_data);
// read/write a register using the PMBUS protocol
tSMBusStatus apollo_pmbus_rw(uint8_t device, bool read, uint8_t add, 
                             uint8_t cmd, uint8_t *value, uint8_t size);

#endif /* PROJECTS_CM_MCU_I2CCOMMUNICATION_H_ */
