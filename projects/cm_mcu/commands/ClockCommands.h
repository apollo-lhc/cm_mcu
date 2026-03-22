#ifndef CLOCK_COMMANDS_H_
#define CLOCK_COMMANDS_H_
#include "FreeRTOS.h" // IWYU pragma: keep
#include "portmacro.h"

BaseType_t clkmon_ctl(int argc, char **argv, char *m);
BaseType_t clk_freq_fpga_cmd(int argc, char **argv, char *m);
BaseType_t clk_prog_name(int argc, char **argv, char *m);
BaseType_t clk_reset(int argc, char **argv, char *m);
BaseType_t clearclk_ctl(int argc, char **argv, char *m);
BaseType_t init_load_clock_ctl(int argc, char **argv, char *m);

#endif
