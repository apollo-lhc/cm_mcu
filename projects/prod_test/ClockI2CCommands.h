#pragma once

#include "FreeRTOS.h" // IWYU pragma: keep

#define CLOCK_I2C_BASE             2
#define NDEVICES_CLK               5
#define U84_ADDR                   0x70
#define SI5395_I2C_ADDR            0x6b
#define R0A_MUX_BIT                0
#define R0B_MUX_BIT                1
#define R1A_MUX_BIT                2
#define R1B_MUX_BIT                3
#define R1C_MUX_BIT                4
#define SI5395_ADDR_PAGESEL        0x01U
#define SI5395_ADDR_OPN_UPPER      0x00U
#define SI5395_ADDR_OPN_LOWER0     0x02U
#define SI5395_ADDR_SCRATCH_UPPER  0x02U
#define SI5395_ADDR_SCRATCH_LOWER0 0x6BU
#define SI5395_OPN0                0x95U
#define SI5395_MAX_ATTEMPTS        500
#define TCA9555_ADDR_INPORT0       0x00
#define TCA9555_ADDR_OUTPORT0      0x02
#define NDEVICES_CLK_IOEXPANDER    2
#define U88_MUX_BIT                6
#define U88_ADDR                   0x20
#define U83_MUX_BIT                7
#define U83_ADDR                   0x21
#define U88_REG0_RESET_R0A         0x00
#define U88_REG0_DEFAULT           0x80
#define U83_REG0_RESET_R1A         0x00
#define U83_REG0_DEFAULT           0x80
#define IOEXPANDER_TEST_MASK       0x80
#define IOEXPANDER_TEST_RESULT0    0x00
#define IOEXPANDER_TEST_RESULT1    0x80
#define CLOCK_MUX_ADDR             0x70

struct dev_moni2c_addr_t {
  char *name;
  uint8_t mux_addr; // I2C address of the Mux
  uint8_t mux_bit;  // port of the mux; write value 0x1U<<mux_bit to the mux register
  uint8_t dev_addr; // I2C address of device.
};

struct dev_ioexpander_addr_t {
  uint8_t mux_addr; // I2C address of the Mux
  uint8_t mux_bit;  // port of the mux; write value 0x1U<<mux_bit to the mux register
  uint8_t dev_addr; // I2C address of device
};

/**
 * @brief Tests I2C communication to clock synths
 *
 * @param [in] argc  number of CLI arguments
 * @param [in] argv  CLI arguments
 * @param [out] m  output string
 * @return pdFALSE
 */
BaseType_t run_clock_i2ctest(int argc, char **argv, char *m);

/**
 * @brief Initializes clock MUX IO expanders
 *
 * @param [in] argc  number of CLI arguments
 * @param [in] argv  CLI arguments
 * @param [out] m  output string
 * @return pdFALSE
 */
BaseType_t init_clock_ioexpanders(int argc, char **argv, char *m);

/**
 * @brief Initializes IO expanders that are on the clock I2C MUX
 *
 * @returns 0 if no errors, otherwise sum of error codes
 */
int init_registers_clk(void);
