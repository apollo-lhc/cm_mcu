#ifndef POWER_COMMANDS_H_
#define POWER_COMMANDS_H_
#include "FreeRTOS.h" // IWYU pragma: keep
#include "portmacro.h"

BaseType_t psmon_ctl(int argc, char **argv, char *m);
BaseType_t power_ctl(int argc, char **argv, char *m);
BaseType_t psmon_reg(int argc, char **argv, char *m);
BaseType_t snapshot(int argc, char **argv, char *m);
BaseType_t sn_all(int argc, char **argv, char *m);

#endif
