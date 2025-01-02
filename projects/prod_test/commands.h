#pragma once
#include "FreeRTOS.h" // IWYU pragma: keep

struct command_t {
  const char *commandstr;
  BaseType_t (*interpreter)(int argc, char **, char *m);
  const char *helpstr;
  const int num_args;
};

struct microrl_user_data_t {
  uint32_t uart_base;
};

#define SCRATCH_SIZE 512

BaseType_t help_command_fcn(int argc, char **argv, char *m);
BaseType_t bl_ctl(int argc, char **argv, char *m);
BaseType_t power_ctl(int argc, char **argv, char *m);
BaseType_t power_off_ctl(int argc, char **argv, char *m);
BaseType_t restart_mcu(int argc, char **argv, char *m);
BaseType_t hello_world_fcn(int argc, char **argv, char *m);

