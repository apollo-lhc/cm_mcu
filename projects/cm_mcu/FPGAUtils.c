/*
 * FPGAUtils.c
 *
 * Generic register accessors for the FPGA's I2C diagnostic-register block.
 */
#include "FPGAUtils.h"
#include "I2CCommunication.h"

#if defined(REV2) || defined(REV3)

#define FPGA_I2C_BUS      5
#define FPGA_I2C_MUX_ADDR 0x70
#define FPGA_I2C_DEV_ADDR 0x2b

// channel 2 selects F1, channel 0 selects F2 -- schematic page 4.04
// defaults to FPGA F2, beware
static inline uint8_t fpga_i2c_mux_val(int fpga)
{
  return (fpga == 0) ? (0x1U << 2) : (0x1U << 0);
}

int fpga_i2c_reg_r(int fpga, uint16_t reg, uint8_t nbytes, uint32_t *data)
{
  // mux select
  int r = apollo_i2c_ctl_w(FPGA_I2C_BUS, FPGA_I2C_MUX_ADDR, 1, fpga_i2c_mux_val(fpga));
  if (r == 0) {
    // read fpga register on mux success
    r = apollo_i2c_ctl_reg_r(FPGA_I2C_BUS, FPGA_I2C_DEV_ADDR, 1, reg, nbytes, data);
  }
  // clear mux
  int rc = apollo_i2c_ctl_w(FPGA_I2C_BUS, FPGA_I2C_MUX_ADDR, 1, 0); // always clear the mux
  return (r != 0) ? r : rc;
}

int fpga_i2c_reg_w(int fpga, uint16_t reg, uint8_t nbytes, uint32_t data)
{
  // mux select
  int r = apollo_i2c_ctl_w(FPGA_I2C_BUS, FPGA_I2C_MUX_ADDR, 1, fpga_i2c_mux_val(fpga));
  if (r == 0) {
    // write to fpga register on mux success
    r = apollo_i2c_ctl_reg_w(FPGA_I2C_BUS, FPGA_I2C_DEV_ADDR, 1, reg, nbytes, data);
  }
  // clear mux
  int rc = apollo_i2c_ctl_w(FPGA_I2C_BUS, FPGA_I2C_MUX_ADDR, 1, 0); // always clear the mux
  return (r != 0) ? r : rc;
}

#endif // defined(REV2) || defined(REV3)
