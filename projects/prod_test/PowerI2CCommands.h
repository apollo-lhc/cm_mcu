// TODO clean up
#include "FreeRTOS.h" // IWYU pragma: keep

#define PAGE_COMMAND    0x0
#define N_PM_ADDRS_DCDC 7
#define NPAGES_PS       2

// how to find an I2C device, with a mux infront of it.
struct dev_i2c_addr_t {
  char *name;
  uint8_t mux_addr; // I2C address of the Mux
  uint8_t mux_bit;  // port of the mux; write value 0x1U<<mux_bit to the mux register
  uint8_t dev_addr; // I2C address of device.
};

BaseType_t run_dcdc_i2ctest(int argc, char **argv, char *m);
