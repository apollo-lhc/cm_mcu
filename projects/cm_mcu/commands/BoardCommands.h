/*
 * BoardCommands.h
 *
 *  Created on: Jan 18, 2021
 *      Author: fatimayousuf
 */

<<<<<<< HEAD
#include "parameters.h"
=======
#include <parameters.h>
>>>>>>> continue divding commands

#ifndef BOARD_COMMANDS_H_
#define BOARD_COMMANDS_H_

<<<<<<< HEAD
BaseType_t restart_mcu(int argc, char **argv, char* m);
BaseType_t set_board_id(int argc, char **argv, char* m);
BaseType_t set_board_id_password(int argc, char **argv, char* m);
BaseType_t board_id_info(int argc, char **argv, char* m);
=======
static BaseType_t restart_mcu(int argc, char **argv, char m);
static BaseType_t set_board_id(int argc, char **argv, char m);
static BaseType_t set_board_id_password(int argc, char **argv, char m);
static BaseType_t board_id_info(int argc, char **argv, char m);
>>>>>>> continue divding commands

#endif
