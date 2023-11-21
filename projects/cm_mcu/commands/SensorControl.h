/*
 * SensorControl.h
 *
 *  Created on: Jan 13, 2021
 *      Author: fatimayousuf
 */

#ifndef SENSOR_CONTROL_H_
#define SENSOR_CONTROL_H_
#include "parameters.h"

BaseType_t sensor_summary(int argc, char **argv, char *m);

// Power
BaseType_t psmon_ctl(int argc, char **argv, char *m);
BaseType_t power_ctl(int argc, char **argv, char *m);

// Power registers
BaseType_t psmon_reg(int argc, char **argv, char *m);

// Alarms
BaseType_t alarm_ctl(int argc, char **argv, char *m);

// LED
BaseType_t led_ctl(int argc, char **argv, char *m);

// ADC
BaseType_t adc_ctl(int argc, char **argv, char *m);

// Fireflies
BaseType_t ff_ctl(int argc, char **argv, char *m);
BaseType_t ff_optpow(int argc, char **argv, char *m);
BaseType_t ff_temp(int argc, char **argv, char *m);
BaseType_t ff_status(int argc, char **argv, char *m);
BaseType_t ff_los_alarm(int argc, char **argv, char *m);
BaseType_t ff_cdr_lol_alarm(int argc, char **argv, char *m);
BaseType_t ff_laser_fault(int argc, char **argv, char *m);

// Clocks
BaseType_t clkmon_ctl(int argc, char **argv, char *m);
BaseType_t clkr0amon_ctl(int argc, char **argv, char *m);

// FPGA
BaseType_t fpga_ctl(int argc, char **argv, char *m);
BaseType_t fpga_reset(int argc, char **argv, char *m);
BaseType_t fpga_flash(int argc, char **argv, char *m);

#endif
