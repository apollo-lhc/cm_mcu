/*
 * I2CMasterTask.h
 *
 *  Created on: Feb 7, 2020
 *      Author: wittich
 */

#ifndef PROJECTS_CM_MCU_I2CMASTERTASK_H_
#define PROJECTS_CM_MCU_I2CMASTERTASK_H_

// includes for types
#include <stdint.h>
#include <stdbool.h>

void I2CMasterTask(void *parameters);

struct I2CMasterTaskArgs_t {
  uint32_t i2c_master;

};

#endif /* PROJECTS_CM_MCU_I2CMASTERTASK_H_ */
