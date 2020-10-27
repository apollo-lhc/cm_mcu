/*
 * clocksynth.c
 *
 *  Created on: Jul 30, 2020
 *      Author: rzou
 */
#include "clocksynth.h"
#include "common/utils.h"
#include "I2CCommunication.h"
#include "Tasks.h"
#include "common/uart.h"
#include "inc/hw_memmap.h"
#include <string.h>

#define SCRATCH_SIZE 512
static char m[SCRATCH_SIZE];
int snprintf(char *buf, unsigned int count, const char *format, ...);
// clang-format off
int PreambleList[][2] = {{ 0x0B24 , 0xC0 },
                         { 0x0B25 , 0x00 },
                         { 0x0502 , 0x01 },
                         { 0x0505 , 0x03 },
                         { 0x0957 , 0x17 },
                         { 0x0B4E , 0x1A }};

int PostambleList[][2] = {{ 0x001C , 0x01 },
                          { 0x0B24 , 0xC3 },
                          { 0x0B25 , 0x02 }};

// hack: hardcoded register list for 156.25 MHz
int RegisterList[][2] = {{ 0x000B , 0x74 },
                         { 0x0017 , 0xD0 },
                         { 0x0018 , 0xFF },
                         { 0x0021 , 0x0F },
                         { 0x002B , 0x02 },
                         { 0x002C , 0x20 },
                         { 0x0102 , 0x01 },
                         { 0x0112 , 0x06 },
                         { 0x0113 , 0x09 },
                         { 0x0114 , 0x33 },
                         { 0x0115 , 0x08 },
                         { 0x0117 , 0x06 },
                         { 0x0118 , 0x09 },
                         { 0x0119 , 0x33 },
                         { 0x011A , 0x08 },
                         { 0x0126 , 0x06 },
                         { 0x0127 , 0x09 },
                         { 0x0128 , 0x33 },
                         { 0x0129 , 0x08 },
                         { 0x012B , 0x06 },
                         { 0x012C , 0x09 },
                         { 0x012D , 0x33 },
                         { 0x012E , 0x08 },
                         { 0x0141 , 0x40 },
                         { 0x0238 , 0xD8 },
                         { 0x0239 , 0xD6 },
                         { 0x023E , 0xC0 },
                         { 0x0306 , 0x16 },
                         { 0x030B , 0x80 },
                         { 0x0339 , 0x1F },
                         { 0x090E , 0x02 },
                         { 0x091C , 0x04 },
                         { 0x0943 , 0x01 },
                         { 0x094E , 0x49 },
                         { 0x094F , 0x02 },
                         { 0x0A03 , 0x01 },
                         { 0x0A04 , 0x01 },
                         { 0x0A05 , 0x01 },
                         { 0x0B44 , 0x0F },
                         { 0x0B4A , 0x0E },
                         { 0x0B57 , 0x0E },
                         { 0x0B58 , 0x01 },
                         };
// clang-format on

int initialize_clock()
{
  int status = -10;
  status = apollo_i2c_ctl_set_dev(CLOCK_I2C_BASE);
  if (status != 0)
    return status;
  // Enable clocksynth, two i/o expanders via switch
  status = apollo_i2c_ctl_w(CLOCK_SWITCH_I2C_ADDRESS, 1, CLOCK_SWITCH_ENABLEMAP);
  if (status != 0)
    return status;
  // Setting clock write expander to have all I/O ports (P0-7,P10-17) set as outputs
  status =
      apollo_i2c_ctl_reg_w(CLOCK_WRITE_EXPANDER_I2C_ADDRESS, CLOCK_EXPANDER_CONFIGURATION_PORT_0, 1,
                           CLOCK_EXPANDER_CONFIGURATION_PORT_SETASOUTPUT);
  if (status != 0)
    return status;
  status =
      apollo_i2c_ctl_reg_w(CLOCK_WRITE_EXPANDER_I2C_ADDRESS, CLOCK_EXPANDER_CONFIGURATION_PORT_1, 1,
                           CLOCK_EXPANDER_CONFIGURATION_PORT_SETASOUTPUT);
  if (status != 0)
    return status;
  // Make clock buffer for xcvrs pick synthesized clock
  status = apollo_i2c_ctl_reg_w(CLOCK_WRITE_EXPANDER_I2C_ADDRESS, CLOCK_EXPANDER_OUTPUT_PORT_0, 1,
                                CLOCK_EXPANDER_CHOOSE_CLOCKSYNTH_4XCVR);
  if (status != 0)
    return status;
  // Configuring Clock Synthesizer chip (enable and reset) via expander
  status = apollo_i2c_ctl_reg_w(CLOCK_WRITE_EXPANDER_I2C_ADDRESS, CLOCK_EXPANDER_OUTPUT_PORT_1, 1,
                                CLOCK_EXPANDER_ENABLE_CLOCKSYNTH);
  if (status != 0)
    return status;
  status = apollo_i2c_ctl_reg_w(CLOCK_WRITE_EXPANDER_I2C_ADDRESS, CLOCK_EXPANDER_OUTPUT_PORT_1, 1,
                                CLOCK_EXPANDER_RESET_CLOCKSYNTH);
  if (status != 0)
    return status;
  // Clear sticky flags of clock synth status monitor (raised high after reset)
  status = apollo_i2c_ctl_reg_w(CLOCK_SYNTH_I2C_ADDRESS, CLOCK_SYNTH_STICKY_FLAG_REGISTER, 1, 0);
  return status;
}

static int write_register(int RegList[][2], int n_row)
{
  bool ChangePage = true;
  int HighByte = -1;
  int status = -10;
  for (int i = 0; i < n_row; i++) {
    uint8_t NewHighByte = RegList[i][0] >> 8; // most significant 8 bits, 16 bits in total
    if (NewHighByte != HighByte) {
      ChangePage = true;
    }
    else {
      ChangePage = false;
    }
    HighByte = NewHighByte;
    uint8_t LowByte = RegList[i][0] - (NewHighByte << 8);
    
    if (ChangePage) {
      snprintf(m, SCRATCH_SIZE, "ChangePage: 0x%02x 0x01 1 0x%02x\r\n", CLOCK_SYNTH_I2C_ADDRESS, NewHighByte);
      UARTPrint(UART1_BASE,  m);
      status = apollo_i2c_ctl_reg_w(CLOCK_SYNTH_I2C_ADDRESS, 0x01, 1, NewHighByte);
      if (status != 0)
        return status;
    }
    //    snprintf(m, SCRATCH_SIZE, "About to write to: 0x%02x 0x%02x 1 0x%02x\r\n", CLOCK_SYNTH_I2C_ADDRESS,LowByte,RegList[i][1]);
    //    UARTPrint(UART1_BASE, m);
    status = apollo_i2c_ctl_reg_w(CLOCK_SYNTH_I2C_ADDRESS, LowByte, 1, RegList[i][1]);
  }
  return status;
}

int load_clock()
{
  initialize_clock();
  int row = sizeof(PreambleList) / sizeof(PreambleList[0]);
  int status = -10;
  status = write_register(PreambleList, row);
  if (status != 0)
    return status;
  vTaskDelay(pdMS_TO_TICKS(330)); //300 ms minimum
  row = sizeof(RegisterList) / sizeof(RegisterList[0]);
  status = write_register(RegisterList, row);
  if (status != 0)
    return status;
  vTaskDelay(pdMS_TO_TICKS(330));
  row = sizeof(PostambleList) / sizeof(PostambleList[0]);
  status = write_register(PostambleList, row);
  return status;
}
