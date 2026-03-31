#ifndef ALARM_COMMANDS_H_
#define ALARM_COMMANDS_H_
#include "FreeRTOS.h" // IWYU pragma: keep
#include "portmacro.h"

BaseType_t alarm_ctl(int argc, char **argv, char *m);

#endif
