/*
 * FPGAUtils.h
 *
 * Generic register accessors for the FPGA's I2C generic-register block.
 */
#ifndef PROJECTS_CM_MCU_FPGAUTILS_H_
#define PROJECTS_CM_MCU_FPGAUTILS_H_
#include <stdint.h>

#if defined(REV2) || defined(REV3)

// Read/write a register in the FPGA's generic I2C diagnostic-register block
// (behind the TCA9548 mux at 0x70 on I2C bus 5, device 0x2b).
// fpga: 0 = F1, 1 = F2. nbytes: 1-4. Caller must hold i2c5_sem before calling.
// Returns 0 on success, or an SMBus error code from I2CCommunication.c on failure
// (see SMBUS_get_error() in common/smbus_helper.h).
int fpga_i2c_reg_r(int fpga, uint16_t reg, uint8_t nbytes, uint32_t *data);
int fpga_i2c_reg_w(int fpga, uint16_t reg, uint8_t nbytes, uint32_t data);

#endif // defined(REV2) || defined(REV3)

#endif /* PROJECTS_CM_MCU_FPGAUTILS_H_ */
