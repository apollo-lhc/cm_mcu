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
extern const uint32_t I2C_BASE[];

void initI2C0(const uint32_t sysclockfreq); // EEPROM(Rev1) + I2C slave for master
void initI2C1(const uint32_t sysclockfreq); // power supply I2C bus
void initI2C2(const uint32_t sysclockfreq); // EEPROM(Rev2) + clocks   I2C bus
void initI2C3(const uint32_t sysclockfreq); // F2 optics I2C bus
void initI2C4(const uint32_t sysclockfreq); // F1 optics I2C bus
void initI2C5(const uint32_t sysclockfreq); // FPGA     I2C bus (Rev2)
void initI2C6(const uint32_t sysclockfreq); // FPGA     I2C bus (Rev1)

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

// I2C bus recovery: bit-bang 9 SCL clocks (up to 16 while SDA is low), then a
// STOP, then hand the pins back to the I2C controller. Buses 1-6 only (bus 0 is
// our slave port). Does not reset the controller or touch interrupts; the caller
// does that, and must own the bus.
enum i2c_recover_status {
  I2C_RECOVER_OK = 0,
  I2C_RECOVER_BAD_BUS,   // bus not valid for this revision
  I2C_RECOVER_SCL_STUCK, // SCL held low; clocking cannot help
  I2C_RECOVER_SDA_STUCK, // SDA still low after 16 clocks + STOP
  I2C_RECOVER_MUX_FAIL,  // mux select failed (apollo_i2c_bus_reset_mux)
};
struct i2c_recover_info {
  uint8_t pre;     // I2C_O_MBMON before: bit 0 SCL, bit 1 SDA
  uint8_t post;    // I2C_O_MBMON after, pins back on the controller
  uint8_t nclocks; // SCL clocks issued
  uint8_t status;  // this pass's i2c_recover_status (also the return value)
};
int i2c_bus_recover(uint8_t bus, struct i2c_recover_info *info);
const char *i2c_recover_status_str(int status);

#endif /* COMMON_I2C_REG_H_ */
