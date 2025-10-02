#pragma once

#include "FreeRTOS.h" // IWYU pragma: keep
#include <stdbool.h>

#define F2FF_I2C_BASE            3
#define F1FF_I2C_BASE            4
#define NDEVICES_FF              20
#define NDEVICES_FF_IOEXPANDER   4
#define N_IOEXP_CHECKS           6
#define FF_I2C_MUX1_ADDR         0x70
#define FF_I2C_MUX2_ADDR         0x71
#define FF_I2C_F1_FF1_T_MUX_BIT  0
#define FF_I2C_F1_FF1_R_MUX_BIT  1
#define FF_I2C_F1_FF5_B_MUX_BIT  2
#define FF_I2C_F1_FF2_T_MUX_BIT  3
#define FF_I2C_F1_FF2_R_MUX_BIT  4
#define FF_I2C_F1_IOEXP1_MUX_BIT 7
#define FF_I2C_F1_FF3_T_MUX_BIT  0
#define FF_I2C_F1_FF3_R_MUX_BIT  1
#define FF_I2C_F1_FF6_B_MUX_BIT  2
#define FF_I2C_F1_FF4_T_MUX_BIT  3
#define FF_I2C_F1_FF4_R_MUX_BIT  4
#define FF_I2C_F1_IOEXP2_MUX_BIT 6
#define FF_I2C_F2_FF1_T_MUX_BIT  0
#define FF_I2C_F2_FF1_R_MUX_BIT  1
#define FF_I2C_F2_FF5_B_MUX_BIT  2
#define FF_I2C_F2_FF2_T_MUX_BIT  3
#define FF_I2C_F2_FF2_R_MUX_BIT  4
#define FF_I2C_F2_IOEXP1_MUX_BIT 7
#define FF_I2C_F2_FF3_T_MUX_BIT  0
#define FF_I2C_F2_FF3_R_MUX_BIT  1
#define FF_I2C_F2_FF6_B_MUX_BIT  2
#define FF_I2C_F2_FF4_T_MUX_BIT  3
#define FF_I2C_F2_FF4_R_MUX_BIT  4
#define FF_I2C_F2_IOEXP2_MUX_BIT 6
#define IOEXP1_I2C_ADDR          0x20
#define IOEXP2_I2C_ADDR          0x21
#define FF_12X_TX_I2C_ADDR       0x50
#define FF_12X_RX_I2C_ADDR       0x54
#define FF_12X_DISABLE_PAGE      0x00
#define FF_12X_DISABLE_ADDR      0x35
#define FF_4X_I2C_ADDR           0x50
#define FF_4X_DISABLE_PAGE       0x00
#define FF_4X_DISABLE_ADDR       0x56
#define FF_PAGESEL_ADDR          0x7F
#define TCA9555_ADDR_INPORT0     0x00
#define TCA9555_ADDR_INPORT1     0x01
#define TCA9555_ADDR_OUTPORT0    0x02
#define TCA9555_ADDR_OUTPORT1    0x03
#define IOEXP1_PRESENT_MASK1     0xFF
#define IOEXP2_PRESENT_MASK0     0x0C
#define IOEXP2_PRESENT_MASK1     0xF0
#define IOEXP1_PRESENT_EXPECT1   0xF3 // FF2TX and FF2RX installed
#define IOEXP2_PRESENT_EXPECT0   0x08 // FF5 installed
#define IOEXP2_PRESENT_EXPECT1   0x00 // All switches 3v3
#define IOEXP2_RESET_MASK        0x80
#define IOEXP2_DEASSERT_RESET    0x80
#define IOEXP2_ASSERT_RESET      0x00

enum device_class {
  DEV_FF_TX,
  DEV_FF_RX,
  DEV_FF_B04,
  DEV_IOEXP,
  DEV_NONE
};

struct dev_ff_i2c_addr_t {
  char *name;                  // Name
  uint8_t i2c_ctrl;            // I2C controller index
  uint8_t mux_addr;            // I2C address of the Mux
  uint8_t mux_bit;             // port of the mux; write 0x1U<<mux_bit to select
  uint8_t dev_addr;            // I2C address of device.
  enum device_class dev_class; // Device class
};

struct ff_ioexp_param_t {
  uint8_t dev_index;    // index to IOexpander dev_ff_i2c_addr_t
  uint8_t present_addr; // I2C register address
  int reset_pin;        // Associated MUX reset
};

/**
 * @brief Tests I2C communication to fireflies/IO expanders
 *
 * @param [out] m  output string
 * @param [inout] copied  output already used in buffer
 * @param [in] ff_mask  bitmap of firefly presence
 * @return true if test succeeds, false otherwise
 */
bool firefly_i2ctest(char *m, int32_t *copied, int32_t ff_mask);

/**
 * @brief CLI wrapper around firefly_i2ctest
 *
 * @param [in] argc  number of CLI arguments
 * @param [in] argv  CLI arguments
 * @param [out] m  output string
 * @return pdFALSE
 */
BaseType_t firefly_i2ctest_ctl(int argc, char **argv, char *m);

/**
 * @brief generates bitmap from binary string
 *
 * @param [in] argc  number of CLI arguments
 * @param [in] argv  CLI arguments
 * @return firefly bit mask, ordered by site
 */
int32_t firefly_string_to_mask(int argc, char **argv);

/**
 * @brief CLI function that initializes optics IO expanders
 *
 * @param [in] argc  number of CLI arguments
 * @param [in] argv  CLI arguments
 * @param [out] m  output string
 * @return pdFALSE
 */
BaseType_t firefly_ioexpanders_init_ctl(int argc, char **argv, char *m);

/**
 * @brief Initializes IO expanders that are on the optics MUXes
 *
 * @returns 0 if no errors, otherwise sum of error codes
 */
int init_registers_firefly(void);
