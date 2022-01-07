/*
 * BoardCommands.h
 *
 *  Created on: Jan 18, 2021
 *      Author: fatimayousuf
 */

#include "parameters.h"

#ifndef BOARD_COMMANDS_H_
#define BOARD_COMMANDS_H_

BaseType_t restart_mcu(int argc, char **argv, char* m);
BaseType_t set_board_id(int argc, char **argv, char* m);
BaseType_t set_board_id_password(int argc, char **argv, char* m);
BaseType_t board_id_info(int argc, char **argv, char* m);
BaseType_t jtag_sm_ctl(int argc, char **argv, char *m);
BaseType_t time_ctl(int argc, char **argv, char *m);
BaseType_t gpio_ctl(int argc, char **argv, char *m);

#endif
