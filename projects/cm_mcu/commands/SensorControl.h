/*
 * SensorControl.h
 *
 *  Created on: Jan 13, 2021
 *      Author: fatimayousuf
 */

#include <parameters.h>

static BaseType_t sensor_summary(int argc, char **argv);

// Power
static BaseType_t power_ctl(int argc, char **argv);

// Alarms
static BaseType_t alarm_ctl(int argc, char **argv);

// LED
static BaseType_t led_ctl(int argc, char **argv);

// ADC
static BaseType_t adc_ctl(int argc, char **argv);

// Bootloader
static BaseType_t bl_ctl(int argc, char **argv);

// Clock
static BaseType_t clock_ctl(int argc, char **argv);

// Fireflies
static BaseType_t ff_ctl(int argc, char **argv);
static BaseType_t ff_status(int argc, char **argv);

// FPGA
static BaseType_t fpga_ctl(int argc, char **argv);
static BaseType_t fpga_reset(int argc, char **argv);
