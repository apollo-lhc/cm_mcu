/*
 * Copyright (c) 2020 rxi
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to
 * deal in the Software without restriction, including without limitation the
 * rights to use, copy, modify, merge, publish, distribute, sublicense, and/or
 * sell copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
 * IN THE SOFTWARE.
 */

#include "log.h"
#include "task.h"
#include "printf.h"

#define MAX_CALLBACKS 8
#define stdout_callback ApolloLog

typedef struct {
  log_LogFn fn;
  void *udata;
  int level;
} log_Callback;

static struct {
  void *udata;
  log_LockFn lock;
  int level[NUM_LOG_FACILITIES];
  bool quiet;
  log_Callback callbacks[MAX_CALLBACKS];
} L;

static const char *level_strings[] = {"FATAL", "ERROR", "WARN", "INFO", "DEBUG", "TRACE",};

static const char *facility_strings[] = { "UNK", "SERVICE", "MON", "FFLY", "PWRCTL", "I2C", "ALM", "CLI", 
};

#ifdef LOG_USE_COLOR
static const char *level_colors[] = { //
    "\x1b[35m", "\x1b[31m", "\x1b[33m", "\x1b[32m", "\x1b[36m", "\x1b[94m" //
};
#endif

#ifdef NOTDEF
static void stdout_callback(log_Event *ev) {
  char buf[16];
  buf[strftime(buf, sizeof(buf), "%H:%M:%S", ev->time)] = '\0';
#ifdef LOG_USE_COLOR
  fprintf(
    ev->udata, "%s %s%-5s\x1b[0m \x1b[90m%s:%d:\x1b[0m ",
    buf, level_colors[ev->level], level_strings[ev->level],
    ev->file, ev->line);
#else
  fprintf(
    ev->udata, "%s %-5s %s:%d: ",
    buf, level_strings[ev->level], ev->file, ev->line);
#endif
  vfprintf(ev->udata, ev->fmt, ev->ap);
  fprintf(ev->udata, "\n");
  fflush(ev->udata);
}


static void file_callback(log_Event *ev) {
  char buf[64];
  buf[strftime(buf, sizeof(buf), "%Y-%m-%d %H:%M:%S", ev->time)] = '\0';
  fprintf(
    ev->udata, "%s %-5s %s:%d: ",
    buf, level_strings[ev->level], ev->file, ev->line);
  vfprintf(ev->udata, ev->fmt, ev->ap);
  fprintf(ev->udata, "\n");
  fflush(ev->udata);
}
#endif

void Print(const char* str);

void ApolloLog(log_Event *ev)
{
  char tmp[256];
  int r = 0;
#ifdef LOG_USE_COLOR
  r = snprintf(tmp, 256, "%s", level_colors[ev->level]);
#endif // LOG_USE_COLOR
  r += snprintf(tmp + r, 256 - r, "%d %-5s %-5s %s:%u:", ev->time, facility_strings[ev->fac], level_strings[ev->level],
                ev->file, ev->line);
  r += vsnprintf(tmp+r, 256-r, ev->fmt, ev->ap);
#ifdef LOG_USE_COLOR
  snprintf(tmp + r, 256 - r, "%s", "\033[0m");
#endif
  Print(tmp);
}

static void lock(void)   {
  if (L.lock) {
    L.lock(true, L.udata);
  }
}


static void unlock(void) {
  if (L.lock) {
    L.lock(false, L.udata);
  }
}


const char* log_level_string(int level) {
  if (level >= NUM_LOG_LEVELS ) {
    return "UNKN";
  }
  return level_strings[level];
}

const char *log_facility_string(int facility)
{
  if (facility >= NUM_LOG_FACILITIES) {
    facility = 0;
  }
  return facility_strings[facility];
}

void log_set_lock(log_LockFn fn, void *udata) {
  L.lock = fn;
  L.udata = udata;
}


void log_set_level(int level, int facility) {
  if ( facility >= NUM_LOG_FACILITIES) {
    return ;
  }
  L.level[facility] = level;
}


void log_set_quiet(bool enable) {
  L.quiet = enable;
}

bool log_get_quiet()
{
  return L.quiet;
}

int log_get_current_level(int facility)
{
  if ( facility >= NUM_LOG_FACILITIES ) {
    return -1;
  }
  return L.level[facility];
}


int log_add_callback(log_LogFn fn, void *udata, int level) {
  for (int i = 0; i < MAX_CALLBACKS; i++) {
    if (!L.callbacks[i].fn) {
      L.callbacks[i] = (log_Callback) { fn, udata, level };
      return 0;
    }
  }
  return -1;
}

#ifdef NOTDEF
int log_add_fp(FILE *fp, int level) {
  return log_add_callback(file_callback, fp, level);
}
#endif // NOTDEF


static void init_event(log_Event *ev, void *udata) {
  ev->time = xTaskGetTickCount();
  ev->udata = udata;
}


void log_log(int level, const char *file, int line, enum log_facility_t facility,
             const char *fmt, ...)
{
  log_Event ev = {
    .fmt   = fmt,
    .file  = file,
    .line  = line,
    .level = level,
    .fac = facility,
  };

  lock();
  if (facility >= NUM_LOG_FACILITIES ) {
    facility = LOG_DEFAULT; //
  }

  if (!L.quiet && level <= L.level[facility]) {
    init_event(&ev, NULL);
    va_start(ev.ap, fmt);
    stdout_callback(&ev);
    va_end(ev.ap);
  }


  if (!L.quiet) {
    for (int i = 0; i < MAX_CALLBACKS && L.callbacks[i].fn; i++) {
      log_Callback *cb = &L.callbacks[i];
      if (level <= L.level[facility] ) {
        init_event(&ev, cb->udata);
        va_start(ev.ap, fmt);
        cb->fn(&ev);
        va_end(ev.ap);
      }
    }
  }

  unlock();
}
