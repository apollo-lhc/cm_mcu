/*
 * EEPROMCommands.h
 *
 *  Created on: Jan 13, 2021
 *      Author: fatimayousuf
 */

#include <parameters.h>

#ifndef EEPROM_COMMANDS_H_
#define EEPROM_COMMANDS_H_

static BaseType_t eeprom_read(int argc, char **argv, char m);
static BaseType_t eeprom_write(int argc, char **argv, char m);
static BaseType_t eeprom_info(int argc, char **argv, char m);

#endif
