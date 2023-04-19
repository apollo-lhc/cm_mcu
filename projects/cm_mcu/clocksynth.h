/*
 * clocksynth.h
 *
 *  Created on: Jul 30, 2020
 *      Author: rzou
 */
#include "common/utils.h"

#ifndef PROJECTS_CM_MCU_CLOCKSYNTH_H_
#define PROJECTS_CM_MCU_CLOCKSYNTH_H_

int initialize_clock(void);
int load_clock(void);

#define CLOCK_I2C_BASE                                2
#define CLOCK_SYNTH5341_I2C_ADDRESS                   0x77
#define CLOCK_SYNTH5395_I2C_ADDRESS                   0x6b
#define CLOCK_SYNTH_STICKY_FLAG_REGISTER              0x11 // sticky flags for internal status register
#define CLOCK_SWITCH_I2C_ADDRESS                      0x70 // TCA9548A
#define CLOCK_SWITCH_ENABLEMAP                        0xc1
#define CLOCK_WRITE_EXPANDER_I2C_ADDRESS              0x20 // TCA9555, I2C expander to write to clock synth chip
#define CLOCK_READ_EXPANDER_I2C_ADDREASS              0x21 // TCA9555, I2C expander to read from clcok synth chip
#define CLOCK_EXPANDER_CONFIGURATION_PORT_1           0x07
#define CLOCK_EXPANDER_CONFIGURATION_PORT_SETASINPUT  1
#define CLOCK_EXPANDER_CONFIGURATION_PORT_SETASOUTPUT 0
#define CLOCK_EXPANDER_INPUT_PORT_0                   0x00
#define CLOCK_EXPANDER_INPUT_PORT_1                   0x01
#define CLOCK_EXPANDER_OUTPUT_PORT_0                  0x02
#define CLOCK_EXPANDER_OUTPUT_PORT_1                  0x03
#define CLOCK_EXPANDER_CHOOSE_CLOCKSYNTH_4XCVR        0x0f
#define CLOCK_EXPANDER_ENABLE_CLOCKSYNTH              0x13
#define CLOCK_EXPANDER_RESET_CLOCKSYNTH               0x1b
#define CLOCK_EXPANDER_CONFIGURATION_PORT_0           0x06 // configuration port to set I/O port as input or output
#define CLOCK_CHANGEPAGE_REG_ADDR                     0x01

#define CLOCK_PROGNAME_REG_ADDR_START   0x26B
#define CLOCK_PROGNAME_REG_COUNT        8
#define CLOCK_PROGNAME_REG_NAME         (CLOCK_PROGNAME_REG_COUNT + 1)
#define CLOCK_EEPROM_PROGNAME_REG_COUNT 8
#define CLOCK_EEPROM_PROGNAME_REG_NAME  (CLOCK_EEPROM_PROGNAME_REG_COUNT + 1)

void getClockProgram(int device, char progname_clkdesgid[CLOCK_PROGNAME_REG_NAME], char progname_eeprom[CLOCK_EEPROM_PROGNAME_REG_NAME]);

#endif /* PROJECTS_CM_MCU_CLOCKSYNTH_H_ */
