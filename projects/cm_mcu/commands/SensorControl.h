/*
 * SensorControl.h
 *
 *  Created on: Jan 13, 2021
 *      Author: fatimayousuf
 */

#ifndef SENSOR_CONTROL_H_
#define SENSOR_CONTROL_H_
#include "FreeRTOS.h" // IWYU pragma: keep
#include "portmacro.h"

// LED
BaseType_t led_ctl(int argc, char **argv, char *m);

// ADC
BaseType_t adc_ctl(int argc, char **argv, char *m);

#endif
