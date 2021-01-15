/*
 * SensorControl.h
 *
 *  Created on: Jan 13, 2021
 *      Author: fatimayousuf
 */

#include <parameters.h>

#ifndef SENSOR_CONTROL_H_
#define SENSOR_CONTROL_H_

static BaseType_t sensor_summary(int argc, char **argv, char m);

// Power
static BaseType_t power_ctl(int argc, char **argv, char m);

// Alarms
static BaseType_t alarm_ctl(int argc, char **argv, char m);

// LED
static BaseType_t led_ctl(int argc, char **argv, char m);

// ADC
static BaseType_t adc_ctl(int argc, char **argv, char m);

// Fireflies
static BaseType_t ff_ctl(int argc, char **argv, char m);
static BaseType_t ff_status(int argc, char **argv);

// FPGA
static BaseType_t fpga_ctl(int argc, char **argv, char m);
static BaseType_t fpga_reset(int argc, char **argv, char m);

#endif
