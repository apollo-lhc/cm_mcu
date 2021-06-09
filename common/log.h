/**
 * Copyright (c) 2020 rxi
 *
 * This library is free software; you can redistribute it and/or modify it
 * under the terms of the MIT license. See `log.c` for details.
 */

#ifndef LOG_H
#define LOG_H

//#include <stdio.h>
#include <stdarg.h>
#include <stdbool.h>
#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "printf.h"
//#include <time.h>

#define LOG_VERSION "0.1.0_pw"

enum log_level_t { LOG_FATAL, LOG_ERROR, LOG_WARN, LOG_INFO, LOG_DEBUG, LOG_TRACE, NUM_LOG_LEVELS };

enum log_facility_t {
  LOG_DEFAULT, // incorrect and/or unset facility
  LOG_SERVICE, // ISR and various tasks like that
  LOG_MON,
  LOG_FFLY,
  LOG_PWRCTL,
  LOG_I2C,
  LOG_ALM,
  LOG_CLI,
  NUM_LOG_FACILITIES
};

typedef struct {
  va_list ap;
  const char *fmt;
  const char *file;
  TickType_t time;
  void *udata;
  int line;
  int level;
  enum log_facility_t fac;
} log_Event;

typedef void (*log_LogFn)(log_Event *ev);
typedef void (*log_LockFn)(bool lock, void *udata);


#define log_trace(LOG_FACILITY, ...) log_log(LOG_TRACE, __FILE__, __LINE__, LOG_FACILITY, __VA_ARGS__)
#define log_debug(LOG_FACILITY, ...) log_log(LOG_DEBUG, __FILE__, __LINE__, LOG_FACILITY, __VA_ARGS__)
#define log_info(LOG_FACILITY, ...)  log_log(LOG_INFO, __FILE__, __LINE__, LOG_FACILITY, __VA_ARGS__)
#define log_warn(LOG_FACILITY, ...)  log_log(LOG_WARN, __FILE__, __LINE__, LOG_FACILITY, __VA_ARGS__)
#define log_error(LOG_FACILITY, ...) log_log(LOG_ERROR, __FILE__, __LINE__, LOG_FACILITY, __VA_ARGS__)
#define log_fatal(LOG_FACILITY, ...) log_log(LOG_FATAL, __FILE__, __LINE__, LOG_FACILITY, __VA_ARGS__)

const char* log_level_string(int level);
const char *log_facility_string(int level);
void log_set_lock(log_LockFn fn, void *udata);
void log_set_level(int level, int facility);
void log_set_quiet(bool enable);
bool log_get_quiet();
int log_get_current_level(int facility);
int log_add_callback(log_LogFn fn, void *udata, int level);
//int log_add_fp(FILE *fp, int level);

void log_log(int level, const char *file, int line, enum log_facility_t facility,
             const char *fmt, ...);

#endif
