#include "FreeRTOS.h" // IWYU pragma: keep

#define NDEVICES_CLK               5
#define SI5395_ADDR_PAGESEL        0x01U
#define SI5395_ADDR_OPN_UPPER      0x00U
#define SI5395_ADDR_OPN_LOWER0     0x02U
#define SI5395_ADDR_SCRATCH_UPPER  0x02U
#define SI5395_ADDR_SCRATCH_LOWER0 0x6BU
#define SI5395_OPN0                0x95U
#define SI5395_MAX_ATTEMPTS        500

struct dev_moni2c_addr_t {
  char *name;
  uint8_t mux_addr;             // I2C address of the Mux
  uint8_t mux_bit;              // port of the mux; write value 0x1U<<mux_bit to the mux register
  uint8_t dev_addr;             // I2C address of device.
  uint16_t eeprom_progname_reg; // register on eeprom for reading program version
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
