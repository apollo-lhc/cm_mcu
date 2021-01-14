/*
 * I2CCommands.h
 *
 *  Created on: Jan 13, 2021
 *      Author: fatimayousuf
 */

#include <parameters.h>

static BaseType_t i2c_ctl_set_dev(int argc, char **argv);
static BaseType_t i2c_ctl_r(int argc, char **argv);
static BaseType_t i2c_ctl_reg_r(int argc, char **argv);
static BaseType_t i2c_ctl_reg_w(int argc, char **argv);
static BaseType_t i2c_ctl_w(int argc, char **argv);
static BaseType_t i2c_scan(int argc, char **argv);
