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

typedef enum {
  CLI_OK = 0,      // success, no more output pending
  CLI_MORE = 1,    // success, more output chunks to fetch
  CLI_ERROR = -1,  // command failed
} cli_status_t;


#define SCRATCH_SIZE 512

BaseType_t help_command_fcn(int argc, char **argv, char *m);
BaseType_t adc_ctl(int argc, char **argv, char *m);
BaseType_t bl_ctl(int argc, char **argv, char *m);
BaseType_t power_ctl(int argc, char **argv, char *m);
BaseType_t power_off_ctl(int argc, char **argv, char *m);
BaseType_t restart_mcu(int argc, char **argv, char *m);
BaseType_t prodtest_firststep_ctl(int argc, char **argv, char *m);
BaseType_t ver_ctl(int argc, char **argv, char *m);

// ADC utility functions
char *const getADCname(int i);
float getADCvalue(int i);
