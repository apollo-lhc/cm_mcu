#ifndef SOFTWARECOMMANDS_H
#define SOFTWARECOMMANDS_H
// Commands for the software system/OS interaction
// Wittich 10/2022
#include "parameters.h"

BaseType_t stack_ctl(int argc, char **argv, char *m);
BaseType_t mem_ctl(int argc, char **argv, char *m);
BaseType_t uptime(int argc, char **argv, char *m);
BaseType_t TaskStatsCommand(int argc, char **argv, char *m);
BaseType_t watchdog_ctl(int argc, char **argv, char *m);
BaseType_t zmon_ctl(int argc, char **argv, char *m);
BaseType_t log_ctl(int argc, char **argv, char *m);
BaseType_t ver_ctl(int argc, char **argv, char *m);
BaseType_t sem_ctl(int argc, char **argv, char *m);
portBASE_TYPE taskInfo(int argc, char *argv[], char *m);

#endif // SOFTWARECOMMANDS_H
