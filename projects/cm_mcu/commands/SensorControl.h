/*
 * SensorControl.h
 *
 *  Created on: Jan 13, 2021
 *      Author: fatimayousuf
 */

#ifndef SENSOR_CONTROL_H_
#define SENSOR_CONTROL_H_
#include "FreeRTOS.h" // IWYU pragma: keep

// Register definitions
// -------------------------------------------------
// 8 bit 2's complement signed int, valid from 0-80 C, LSB is 1 deg C
// Same address for 4 XCVR and 12 Tx/Rx devices

// two bytes, 12 FF to be disabled
#define ECU0_14G_TX_DISABLE_REG 0x34U
// one byte, 4 FF to be enabled/disabled (only 4 LSB are used)
#define ECU0_25G_XVCR_TX_DISABLE_REG 0x56U
// two bytes, 12 FF to be disabled
#define ECU0_14G_RX_DISABLE_REG 0x34U
// one byte, 4 FF to be enabled/disabled (only 4 LSB are used)
#define ECU0_25G_XVCR_RX_DISABLE_REG 0x35U
// one byte, 4 FF to be enabled/disabled (4 LSB are Rx, 4 LSB are Tx)
#define ECU0_25G_XVCR_CDR_REG 0x62U
// two bytes, 12 FF to be enabled/disabled. The byte layout
// is a bit weird -- 0-3 on byte 4a, 4-11 on byte 4b
#define ECU0_25G_TXRX_CDR_REG 0x4AU

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
BaseType_t ff_reset(int argc, char **argv, char *m);
BaseType_t ff_status(int argc, char **argv, char *m);
BaseType_t ff_los_alarm(int argc, char **argv, char *m);
BaseType_t ff_cdr_lol_alarm(int argc, char **argv, char *m);
BaseType_t ff_mux_reset(int argc, char **argv, char *m);
BaseType_t ff_dump_names(int argc, char **argv, char *m);

// Clocks
BaseType_t clkmon_ctl(int argc, char **argv, char *m);
BaseType_t clkr0amon_ctl(int argc, char **argv, char *m);

// FPGA
BaseType_t fpga_ctl(int argc, char **argv, char *m);
BaseType_t fpga_reset(int argc, char **argv, char *m);
BaseType_t fpga_flash(int argc, char **argv, char *m);

int read_ff_register(const char *name, uint16_t packed_reg_addr, uint8_t *value, size_t size, int i2c_device);

#endif
