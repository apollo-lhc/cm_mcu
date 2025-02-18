#pragma once

#include "FreeRTOS.h" // IWYU pragma: keep

#define FPGA_I2C_BASE                     5
#define N_FPGAS                           2
#define FPGA_I2C_MUX_ADDR                 0x70
#define FPGA_I2C_SYSMON_ADDR              0x32
#define FPGA_I2C_F2_GENERAL_MUXBIT        0
#define FPGA_I2C_F2_SYSMON_MUXBIT         1
#define FPGA_I2C_F1_GENERAL_MUXBIT        2
#define FPGA_I2C_F1_SYSMON_MUXBIT         3
#define SYSMON_DRP_WRITE                  0x2
#define SYSMON_DRP_READ                   0x1
#define VU13P_TEMPERATURE_ADDR            0x00
#define VU13P_TUPPER_ALARM_THRESHOLD_ADDR 0x50
#define FPGA1_TEST_DATA                   0xbf1e // ~100 C temperature
#define FPGA2_TEST_DATA                   0xbf1f // ~100 C temperature

/**
 * @brief Generates I2C data for Ultrascale sysmon DRP interactions
 *
 * @param [in] addr  DRP address
 * @param [in] data  DRP data
 * @param [in] read  true if read, false if write
 * @returns word that one can use with I2C interaction
 */
uint32_t gen_sysmon_i2cword(uint32_t addr, uint32_t data, bool read);

/**
 * @brief CLI function that tests I2C communication to FPGAs
 *
 * @param [in] argc  number of CLI arguments
 * @param [in] argv  CLI arguments
 * @param [out] m  output string
 * @return pdFALSE
 */
BaseType_t fpga_i2ctest_ctl(int argc, char **argv, char *m);
