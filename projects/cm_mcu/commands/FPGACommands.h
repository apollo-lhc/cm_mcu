#ifndef FPGA_COMMANDS_H_
#define FPGA_COMMANDS_H_
#include "FreeRTOS.h" // IWYU pragma: keep
#include "portmacro.h"

BaseType_t fpga_ctl(int argc, char **argv, char *m);
BaseType_t fpga_reset(int argc, char **argv, char *m);
BaseType_t fpga_flash(int argc, char **argv, char *m);

#endif
