/*
 * BufferCommands.h
 *
 *  Created on: Jan 13, 2021
 *      Author: fatimayousuf
 */

#include <parameters.h>

#ifndef BUFFER_COMMANDS_H_
#define BUFFER_COMMANDS_H_

#define EBUFFOUT_MAX_ENTRIES 64

static BaseType_t errbuff_in(int argc, char **argv, char m);
static BaseType_t errbuff_out(int argc, char **argv, char m);
static BaseType_t errbuff_info(int argc, char **argv, char m);
static BaseType_t errbuff_reset(int argc, char **argv, char m);

#endif
