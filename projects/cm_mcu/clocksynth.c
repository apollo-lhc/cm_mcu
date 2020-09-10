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

int PreambleList[][2] = {{2852 , 192},
			 {2853 , 0},
			 {1282 , 1},
			 {1285 , 3},
			 {2391 , 23},
			 {2894 , 26}};

int PostambleList[][2] = {{ 28 , 1 },
			  { 2852 , 195 },
			  { 2853 , 2 }};

// hack: hardcoded register list for 156.25 MHz
int RegisterList[][2] = {{ 11 , 116 },
			 { 23 , 208 },
			 { 24 , 255 },
			 { 33 , 15 },
			 { 43 , 2 },
			 { 44 , 32 },
			 { 258 , 1 },
			 { 274 , 6 },
			 { 275 , 9 },
			 { 276 , 51 },
			 { 277 , 8 },
			 { 279 , 6 },
			 { 280 , 9 },
			 { 281 , 51 },
			 { 282 , 8 },
			 { 294 , 6 },
			 { 295 , 9 },
			 { 296 , 51 },
			 { 297 , 8 },
			 { 299 , 6 },
			 { 300 , 9 },
			 { 301 , 51 },
			 { 302 , 8 },
			 { 321 , 64 },
			 { 568 , 216 },
			 { 569 , 214 },
			 { 574 , 192 },
			 { 774 , 22 },
			 { 779 , 128 },
			 { 825 , 31 },
			 { 2318 , 2 },
			 { 2332 , 4 },
			 { 2371 , 1 },
			 { 2382 , 73 },
			 { 2383 , 2 },
			 { 2563 , 1 },
			 { 2564 , 1 },
			 { 2565 , 1 },
			 { 2884 , 15 },
			 { 2890 , 14 },
			 { 2903 , 14 },
			 { 2904 , 1 }};

int initialize_clock()
{
  int status = -10;
  status = apollo_i2c_ctl_set_dev(CLOCK_I2C_BASE);
  if (status != 0)
    return status;
  //Enable clocksynth, two i/o expanders via switch
  status = apollo_i2c_ctl_w(CLOCK_SWITCH_I2C_ADDRESS, 1, CLOCK_SWITCH_ENABLEMAP);
  if (status != 0)
    return status;
  //Setting clock write expander to have all I/O ports (P0-7,P10-17) set as outputs
  status = apollo_i2c_ctl_reg_w(CLOCK_WRITE_EXPANDER_I2C_ADDRESS, CLOCK_EXPANDER_CONFIGURATION_PORT_0, 1, CLOCK_EXPANDER_CONFIGURATION_PORT_SETASOUTPUT);
  if (status != 0)
    return status;
  status = apollo_i2c_ctl_reg_w(CLOCK_WRITE_EXPANDER_I2C_ADDRESS, CLOCK_EXPANDER_CONFIGURATION_PORT_1, 1, CLOCK_EXPANDER_CONFIGURATION_PORT_SETASOUTPUT);
  if (status != 0)
    return status;
  //Make clock buffer for xcvrs pick synthesized clock
  status = apollo_i2c_ctl_reg_w(CLOCK_WRITE_EXPANDER_I2C_ADDRESS, CLOCK_EXPANDER_OUTPUT_PORT_0, 1, CLOCK_EXPANDER_CHOOSE_CLOCKSYNTH_4XCVR);
  if (status != 0)
    return status;
  //Configuring Clock Synthesizer chip (enable and reset) via expander
  status = apollo_i2c_ctl_reg_w(CLOCK_WRITE_EXPANDER_I2C_ADDRESS, CLOCK_EXPANDER_OUTPUT_PORT_1, 1, CLOCK_EXPANDER_ENABLE_CLOCKSYNTH);
  if (status != 0)
    return status;
  status = apollo_i2c_ctl_reg_w(CLOCK_WRITE_EXPANDER_I2C_ADDRESS, CLOCK_EXPANDER_OUTPUT_PORT_1, 1, CLOCK_EXPANDER_RESET_CLOCKSYNTH);
  if (status != 0)
    return status;
  //Clear sticky flags of clock synth status monitor (raised high after reset)
  status = apollo_i2c_ctl_reg_w(CLOCK_SYNTH_I2C_ADDRESS, CLOCK_SYNTH_STICKY_FLAG_REGISTER, 1, 0);
  return status;
}


static int write_register(int RegList[][2], int n_row){
  bool ChangePage = true;
  int HighByte = -1;
  int status = -10;
  for (int i=0;i<n_row;i++) {
    int NewHighByte = RegList[i][0]>>8; //most significant 8 bits, 16 bits in total
    if (NewHighByte!=HighByte) {
      ChangePage = true;
    } else {
      ChangePage = false;
    }
    HighByte = NewHighByte;
    int LowByte = RegList[i][0]-(NewHighByte<<8);
    if (ChangePage) {
      status = apollo_i2c_ctl_reg_w(CLOCK_SWITCH_I2C_ADDRESS, 0x01, 1, NewHighByte);
      if (status != 0)
	return status;
    }
    status = apollo_i2c_ctl_reg_w(CLOCK_SWITCH_I2C_ADDRESS, LowByte, 1, RegList[i][1]);
  }
  return status;
}
  
int load_clock()
{
  initialize_clock();
  int row = sizeof(PreambleList)/sizeof(PreambleList[0]);
  int status = -10;
  status = write_register(PreambleList,row);
  if (status != 0) 
    return status;
  vTaskDelay(pdMS_TO_TICKS(1));
  row = sizeof(RegisterList)/sizeof(RegisterList[0]);
  status = write_register(RegisterList,row);
  if (status != 0)
    return status;
  vTaskDelay(pdMS_TO_TICKS(1));
  row = sizeof(PostambleList)/sizeof(PostambleList[0]);
  status = write_register(PostambleList,row);
  return status;
}
