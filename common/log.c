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
#include "time.h"
#include <string.h>

#define MAX_CALLBACKS   8
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

static const char *const level_strings[] = {
    "FTL",
    "ERR",
    "WRN",
    "INF",
    "DBG",
    "TRC",
};

static const char *const facility_strings[] = {
    "UNK",
    "SRV",
    "MON",
    "MONI2C",
    "PWR",
    "I2C",
    "ALM",
    "CLI",
};

#ifdef LOG_USE_COLOR
static const char *const level_colors[] = {
    //
    "\x1b[35m", "\x1b[31m", "\x1b[33m", "\x1b[32m", "\x1b[36m", "\x1b[94m" //
};
#endif

#ifdef NOTDEF
static void stdout_callback(log_Event *ev)
{
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

static void file_callback(log_Event *ev)
{
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

// apollo log specfic functions start
// prototype
void Print(const char *str);

struct buff_t {
  size_t size;
  char *data;
  size_t last;
};
#define LOG_BUFFER_SIZE 1024
char log_buffer[LOG_BUFFER_SIZE];
struct buff_t b = {
    .size = LOG_BUFFER_SIZE,
    .data = log_buffer,
    .last = 0,
};

static size_t lenx(const char *s, size_t maxlen)
{
  // I need the cast here to tell the ptrdiff the size of the pointer.
  ptrdiff_t plen = (char *)__builtin_memchr(s, '\0', maxlen) - s;
  size_t len = plen;
  return len > maxlen ? maxlen : len;
}

static void log_add_string(const char *s, struct buff_t *b)
{
  // add a string to the circular buffer. the "last" element should
  // always point to the '\0' of the last string added.
  // two cases: either the string fits in one copy, or in two
  long left = b->size - b->last;
  size_t len = lenx(s, left) + 1; // +1 for string terminator
  if (len <= left) {
    // single copy is going to work
    memcpy(b->data + b->last, s, len);
    b->last += len - 1; // string terminator
  }
  else { // need two copies, len > left
    memcpy(b->data + b->last, s, left);
    long len2 = len - left;
    memcpy(b->data, s + left, len2);
    b->last = len - left - 1;
  }
}
#if 0
void log_add_string2(const char *s, struct buff_t *bu)
{
  // add a string to the circular buffer. the "last" element should
  // always point to the '\0' of the last string added.
  // two cases: either the string fits into the current buffer from 
  // the current "last" position, or it does not fit. If it does not fit, 
  // we copy it to two locations: the end of the buffer, and the beginning
  // of the wrap-around area.
  size_t len = strlen(s);
  long left = bu->size - bu->last; // how much space is left in the buffer
  strlcpy(bu->data + bu->last, s, left); // copy the string to the end of the buffer
  if (len <= left) {
    // single copy is going to work
    bu->last += len;
  }
  else {                                     // need two copies, len < left
    strlcpy(bu->data, s + left - 1, bu->size); // minus 1 for \0 string terminator
    bu->last = len - left + 1; // +1 for \0 string terminator
  }
}
#endif // NOTDEF
// This does not handle the case if sz is too small (i.e., smaller than bu->size)
int log_dump_buffer_to_string(char *s, size_t sz)
{
  int copied = snprintf(s, sz, "%s", b.data + b.last + 1);
  copied += snprintf(s + copied, sz - copied, "%s", b.data);
  return copied;
}

// print to callback
void log_dump(void (*f)(const char *s))
{
  f(b.data + b.last + 1);
  f(b.data);
}

void ApolloLog(log_Event *ev)
{
  // note to self: before you change the size of this buffer
  // willy-nilly remember that this has to be accomodated on the stack of every
  // FreeRTOS task that calls any of the log_ functions.
#define SZ 128
  char tmp[SZ];
  int r = 0;
#ifdef LOG_USE_COLOR
  r = snprintf(tmp, SZ, "%s", level_colors[ev->level]);
#endif // LOG_USE_COLOR
  r += snprintf(tmp + r, SZ - r, "20%u %-3s %-3s %s:%u:", ev->time,
                facility_strings[ev->fac], level_strings[ev->level], ev->file, ev->line);
  r += vsnprintf(tmp + r, SZ - r, ev->fmt, ev->ap);
#ifdef LOG_USE_COLOR
  r += snprintf(tmp + r, SZ - r, "%s", "\033[0m");
#endif
  configASSERT(r < SZ); // not the best way to go but ....
  log_add_string(tmp, &b);
  Print(tmp);
}

// apollo log specfic functions end

static void lock(void)
{
  if (L.lock) {
    L.lock(true, L.udata);
  }
}

static void unlock(void)
{
  if (L.lock) {
    L.lock(false, L.udata);
  }
}

const char *log_level_string(int level)
{
  if (level >= NUM_LOG_LEVELS) {
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

void log_set_lock(log_LockFn fn, void *udata)
{
  L.lock = fn;
  L.udata = udata;
}

void log_set_level(int level, int facility)
{
  if (facility >= NUM_LOG_FACILITIES) {
    return;
  }
  L.level[facility] = level;
}

void log_set_quiet(bool enable)
{
  L.quiet = enable;
}

bool log_get_quiet(void)
{
  return L.quiet;
}

int log_get_current_level(int facility)
{
  if (facility >= NUM_LOG_FACILITIES) {
    return -1;
  }
  return L.level[facility];
}

int log_add_callback(log_LogFn fn, void *udata, int level)
{
  for (int i = 0; i < MAX_CALLBACKS; i++) {
    if (!L.callbacks[i].fn) {
      L.callbacks[i] = (log_Callback){fn, udata, level};
      return 0;
    }
  }
  return -1;
}

#ifdef NOTDEF
int log_add_fp(FILE *fp, int level)
{
  return log_add_callback(file_callback, fp, level);
}
#endif // NOTDEF

static void init_event(log_Event *ev, void *udata)
{
#ifdef REV1 // no RTC in Rev1
  ev->time = xTaskGetTickCount();
#else  // REV2 and later
  struct tm now;
  ROM_HibernateCalendarGet(&now);
  if (now.tm_year < 120) { // RTC not yet set
    ev->time = xTaskGetTickCount();
  }
  else { // RTC is set
    ev->time = now.tm_min + 100 * now.tm_hour + 10000 * (now.tm_mday) +
               1000000 * (now.tm_mon + 1) + 100000000 * (now.tm_year - 100);
  }
#endif // REV2 and later
  ev->udata = udata;
}

void log_log(int level, const char *file, int line, enum log_facility_t facility,
             const char *fmt, ...)
{
  log_Event ev = {
      .fmt = fmt,
      .file = file,
      .line = line,
      .level = level,
      .fac = facility,
  };

  lock();
  if (facility >= NUM_LOG_FACILITIES) {
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
      if (level <= L.level[facility]) {
        init_event(&ev, cb->udata);
        va_start(ev.ap, fmt);
        cb->fn(&ev);
        va_end(ev.ap);
      }
    }
  }

  unlock();
}
