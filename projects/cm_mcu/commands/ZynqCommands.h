#ifndef ZYNQ_COMMANDS_H_
#define ZYNQ_COMMANDS_H_
#include "FreeRTOS.h" // IWYU pragma: keep
#include "portmacro.h"

BaseType_t zmon_ctl(int argc, char **argv, char *m);

#endif
