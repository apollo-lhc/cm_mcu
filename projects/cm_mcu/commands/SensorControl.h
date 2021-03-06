/*
 * SensorControl.h
 *
 *  Created on: Jan 13, 2021
 *      Author: fatimayousuf
 */

#include "parameters.h"

#ifndef SENSOR_CONTROL_H_
#define SENSOR_CONTROL_H_

BaseType_t sensor_summary(int argc, char **argv, char* m);

// Power
BaseType_t psmon_ctl(int argc, char **argv, char* m);
BaseType_t power_ctl(int argc, char **argv, char* m);

// Alarms
BaseType_t alarm_ctl(int argc, char **argv, char* m);

// LED
BaseType_t led_ctl(int argc, char **argv, char* m);

// ADC
BaseType_t adc_ctl(int argc, char **argv, char* m);

// Fireflies
BaseType_t ff_ctl(int argc, char **argv, char* m);
BaseType_t ff_status(int argc, char **argv, char* m);

// FPGA
BaseType_t fpga_ctl(int argc, char **argv, char* m);
BaseType_t fpga_reset(int argc, char **argv, char* m);

#endif
