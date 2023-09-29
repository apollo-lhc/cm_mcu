/*
 * utils.c
 *
 *  Created on: May 9, 2019
 *      Author: wittich
 */

#include "common/utils.h"
#include "common/pinsel.h"
#include "common/printf.h"
#include "driverlib/gpio.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "driverlib/debug.h"
#include "driverlib/eeprom.h"
#include "FreeRTOS.h"
#include "Tasks.h"

typedef struct error_buffer_t error_buffer_t;
typedef error_buffer_t *errbuf_handle_t;

uint64_t EPRMMessage(uint64_t action, uint64_t addr, uint64_t data)
{
  return ((action << 48) | (addr << 32) | data);
}

// write single word to eeprom
void write_eeprom(uint32_t data, uint32_t addr)
{
  uint64_t message;
  message = EPRMMessage((uint64_t)EPRM_WRITE_SINGLE, addr, data);
  xQueueSendToBack(xEPRMQueue_in, &message, portMAX_DELAY);
  return;
}

// write single word to eeprom, bypassing the gatekeeper task
// this should only be used in ISR-like routines
void write_eeprom_raw(uint32_t data, uint32_t addr)
{
  ROM_EEPROMProgram(&data, addr, 4);
  return;
}

// read single word from eeprom
uint32_t read_eeprom_single(uint32_t addr)
{
  uint64_t message = EPRMMessage((uint64_t)EPRM_READ_SINGLE, addr, 0);
  xQueueSendToBack(xEPRMQueue_in, &message, portMAX_DELAY);
  xQueueReceive(xEPRMQueue_out, &message, portMAX_DELAY);
  return (uint32_t)message;
}
// reads single word from eeprom, bypassing the gatekeeper task
uint32_t read_eeprom_raw(uint32_t addr)
{
  uint32_t data, *dataptr;
  dataptr = &data;
  MAP_EEPROMRead(dataptr, addr, 4);
  return data;
}

// read 2 words from eeprom
uint64_t read_eeprom_multi(uint32_t addr)
{
  uint64_t data, message;
  message = EPRMMessage((uint64_t)EPRM_READ_DOUBLE, addr, 0);
  xQueueSendToBack(xEPRMQueue_in, &message, portMAX_DELAY);
  xQueueReceive(xEPRMQueue_out, &data, portMAX_DELAY);
  return data;
}

// write by pin number or name
void write_gpio_pin(int pin, uint8_t value)
{
  ASSERT(value == 1 || value == 0);
  uint32_t gport;
  uint8_t gpin;
  pinsel(pin, &gport, &gpin);
  uint8_t pinval;
  if (value == 1)
    pinval = gpin;
  else
    pinval = 0;
  MAP_GPIOPinWrite(gport, gpin, pinval);
  return;
}

// write by pin number or name
uint8_t read_gpio_pin(int pin)
{
  uint32_t gport;
  uint8_t gpin;
  uint8_t value;
  pinsel(pin, &gport, &gpin);
  value = MAP_GPIOPinRead(gport, gpin);
  if (value)
    value = 1;
  return value;
}

// write by pin number or name
uint8_t toggle_gpio_pin(int pin)
{
  uint8_t val = read_gpio_pin(pin);
  if (val)
    val = 0;
  else
    val = 1;
  write_gpio_pin(pin, val);
  return val;
}

// Set up all the active low pins to be off (i.e., high)
static const int pins[] = {
    _FPGA_I2C_RESET,      //
    _PWR_I2C_RESET,       //
    _CLOCKS_I2C_RESET,    //
    _F2_OPTICS_I2C_RESET, //
    _F1_OPTICS_I2C_RESET, //
#ifdef REV2
    _F1_JTAG_BYPASS, //
    _F1_JTAG_BYPASS, //
#endif               // REV2
};
#define NPINS (sizeof(pins) / pins[0])

void setupActiveLowPins(void)
{
  for (int i = 0; i < NPINS; ++i) {
    write_gpio_pin(pins[i], 0x1);
  }
}

// EEPROM Buffer

// error codes. these should correspond to the names in utils.h
static const char *ebuf_errstrings[] = {
    "",
    "Restart",
    "Buffer Reset",
    "Power Off - Manual",
    "Power Off - Temp High",
    "Power On",
    "Temp Normal",
    "Hard fault",
    "Assertion failed",
    "Stack Overflow",
    "Volt Normal",
    "(continue)", // item 10
    "Power Failure",
    "Temp High (TM4C FPGA FF DCDC)",
    "MARK",
    "I2C error",
    "Power Failure CLEAR",
    "Volt High (GEN FPGA1 FPGA2)",
    "Clock Init failed",
};
#define EBUF_N_ERRSTRINGS (sizeof(ebuf_errstrings) / sizeof(ebuf_errstrings[0]))

// put the error string into the provided buffer and return
// the number of chars copied into the buffer.
int errbuffer_get_messagestr(const uint32_t word, char *m, size_t s)
{
  uint16_t errcode = EBUF_ERRCODE(word);
  uint16_t errdata = EBUF_DATA(word);
  uint16_t counter = EBUF_COUNTER(word);
  uint16_t realcount = counter * EBUF_COUNTER_UPDATE + 1;

  uint16_t days = EBUF_ENTRY_TIMESTAMP_DAYS(word);
  uint16_t hours = EBUF_ENTRY_TIMESTAMP_HOURS(word);
  uint16_t minutes = EBUF_ENTRY_TIMESTAMP_MINS(word);

  if (errcode > (EBUF_N_ERRSTRINGS - 1)) {
    return snprintf(m, s, "\r\n\t%s %d (word %d)", " Invalid error code:", errcode, (int)word);
  }
  int copied = snprintf(m, s, "\r\n %02u %02u:%02u \t %x %s ", days, hours, minutes, realcount,
                        ebuf_errstrings[errcode]);
  // below handle those cases where additional data is available
  switch (errcode) {
    case EBUF_RESTART:
      if (errdata & EBUF_RESTART_SW)
        copied += snprintf(m + copied, s - copied, "(SW)");
      if (errdata & EBUF_RESTART_EXT)
        copied += snprintf(m + copied, s - copied, "(EXT)");
      if (errdata & EBUF_RESTART_WDOG)
        copied += snprintf(m + copied, s - copied, "(WDOG)");
      if (errdata & EBUF_RESTART_POR)
        copied += snprintf(m + copied, s - copied, "(POR)");
      break;
    case EBUF_HARDFAULT:
      copied += snprintf(m + copied, s - copied, "(ISRNUM= 0x%02x)", errdata);
      break;
    case EBUF_PWR_FAILURE:
      copied += snprintf(m + copied, s - copied, "(supply mask) 0x%02x", errdata);
      break;
    case EBUF_ASSERT:
      copied += snprintf(m + copied, s - copied, "(pc) 0x%02x", errdata);
      break;
    case EBUF_I2C:
      copied += snprintf(m + copied, s - copied, " (dev = %c)", (char)errdata);
      break;
    case EBUF_CLKINIT_FAILURE:
      copied += snprintf(m + copied, s - copied, " ");
      break;
    default:
      if (errcode > EBUF_WITH_DATA) {
        copied += snprintf(m + copied, s - copied, "0x%02x", errdata);
      }
      break;
  }
  return copied;
}

struct error_buffer_t {
  uint32_t minaddr;
  uint32_t maxaddr;
  uint32_t head;
  uint32_t capacity; // in # entries
  uint16_t last;     // most recent error code
  uint16_t counter;
  uint16_t n_continue; // number of continue codes since last error code
};

static error_buffer_t errbuf = {
    .minaddr = 0, .maxaddr = 0, .head = 0, .capacity = 0, .last = 0, .counter = 0};
const errbuf_handle_t ebuf = &errbuf;

uint32_t decrease_head(void)
{
  uint32_t head = ebuf->head - 4;
  if (head < ebuf->minaddr) {
    head = ebuf->maxaddr;
  }
  return head;
}
uint32_t increase_head(void)
{
  uint32_t head = ebuf->head + 4;
  if (head > ebuf->maxaddr) {
    head = ebuf->minaddr;
  }
  return head;
}

uint32_t errbuffer_findhead(void)
{
  uint32_t ahead = ebuf->minaddr, head = ahead, cap = ebuf->capacity;
  uint32_t previous = 1, i = 0;
  while (i <= cap) {
    ahead += 4;
    if (ahead > ebuf->maxaddr) {
      ahead = ebuf->minaddr;
    }
    uint32_t entry = read_eeprom_raw(ahead);
    if (entry == 0 && previous == 0) {
      break;
    }
    previous = entry;
    head = ahead;
    i++;
  }
  return head;
}

void errbuffer_init(uint8_t minblk, uint8_t maxblk)
{
  ebuf->minaddr = EEPROMAddrFromBlock(minblk);
  ebuf->maxaddr = EEPROMAddrFromBlock(maxblk + 1) - 4;
  ebuf->capacity = (uint32_t)(maxblk - minblk + 1) * 16;
  ebuf->head = errbuffer_findhead();
  ebuf->last = 0;
  ebuf->counter = 0;
  ebuf->n_continue = 0;
}

void errbuffer_reset(void)
{
  uint32_t addr = ebuf->minaddr;
  uint32_t maddr = ebuf->maxaddr;

  write_eeprom(0, addr);
  write_eeprom(0, addr + 4);
  addr += 0x8;
  while (addr <= maddr) {
    write_eeprom(0xffffffff, addr);
    addr += 4;
  }
  ebuf->head = ebuf->minaddr;
  ebuf->counter = 0;
  ebuf->last = 0;
  errbuffer_put(EBUF_RESET_BUFFER, 0);
  return;
}

// Nota Bene
// this _put (and _put_raw below) need to be checked if
// the writing of codes only every EBUF_COUNTER_UPDATE
// works with continuation codes.
// Maybe this is an unneeded complication.
void errbuffer_put(uint16_t errcode, uint16_t errdata)
{
  const uint16_t oldcount = ebuf->counter;
  // special handling for continuation code
  if (errcode == EBUF_CONTINUATION) {
    ebuf->n_continue++;
#if 0
    if((oldcount == 0) || (oldcount - 1) % EBUF_COUNTER_UPDATE == 0) {
      write_eeprom(errbuffer_entry(errcode, errdata), ebuf->head);
      ebuf->head = increase_head();
      write_eeprom(0, increase_head());
    }
    return;
#endif
  }
  // If duplicated error code, and error code should use counter (excluding continuation)
  if ((errcode == ebuf->last) && (errcode != EBUF_CONTINUATION)) {

    // if counter is not a multiple of COUNTER_UPDATE, don't write new entry
    if (oldcount % EBUF_COUNTER_UPDATE != 0) {
      ebuf->counter = ebuf->counter + 1;
    }

    // if counter has already reached max value, increment head
    if (oldcount % (1 << COUNTER_OFFSET) == 0) {
      ebuf->counter = 0;
      ebuf->n_continue = 0;
      ebuf->head = increase_head();
      write_eeprom(0, increase_head());
    }

    // if counter is multiple of COUNTER_UPDATE, write entry and increment counter
    if (oldcount % EBUF_COUNTER_UPDATE == 0) {
      ebuf->counter = ebuf->counter + 1;

      int n = ebuf->n_continue;
      while (n > 0) {
        n--;
        ebuf->head = decrease_head();
      }
      write_eeprom(errbuffer_entry(errcode, errdata), decrease_head());
    }
    // assuming that the right # of continue codes will follow
  }
  else { // If new error code...
    ebuf->counter = 0;
    ebuf->last = errcode;
    write_eeprom(errbuffer_entry(errcode, errdata), ebuf->head);
    ebuf->head = increase_head();
    write_eeprom(0, increase_head());
  }
  ebuf->n_continue = 0;
  return;
}

void errbuffer_put_raw(uint16_t errcode, uint16_t errdata)
{
  uint16_t oldcount = ebuf->counter;
  // If duplicated error code (except continuation)...
  if (errcode == ebuf->last && errcode != EBUF_CONTINUATION) {

    // if counter is not a multiple of COUNTER_UPDATE, don't write new entry
    if (oldcount % EBUF_COUNTER_UPDATE != 0) {
      ebuf->counter = ebuf->counter + 1;
    }

    // if counter has already reached max value, increment head
    if (oldcount % (1 << COUNTER_OFFSET) == 0) { // Change this to use COUNTER_OFFSET
      ebuf->counter = 0;
      ebuf->head = increase_head();
      write_eeprom_raw(0, increase_head());
    }

    // if counter is multiple of COUNTER_UPDATE, write entry and increment counter
    if (oldcount % EBUF_COUNTER_UPDATE == 0) {
      ebuf->counter = ebuf->counter + 1;
      write_eeprom_raw(errbuffer_entry(errcode, errdata), decrease_head());
    }
  }
  else { // If new error code...
    ebuf->counter = 0;
    ebuf->last = errcode;
    write_eeprom_raw(errbuffer_entry(errcode, errdata), ebuf->head);
    ebuf->head = increase_head();
    write_eeprom_raw(0, increase_head());
  }
  return;
}

void errbuffer_get(const uint32_t num, uint32_t (*arrptr)[num])
{
  // we wind back the head by num counts, and then
  // advance it by num counts after reading the entries
  int i = 0;
  while (i < num) {
    i++;
    ebuf->head = decrease_head();
  }
  i = 0;
  while (i < num) {
    (*arrptr)[i] = read_eeprom_single(ebuf->head);
    ebuf->head = increase_head();
    i++;
  }
  return;
}

uint32_t errbuffer_capacity(void)
{
  return ebuf->capacity;
}
uint32_t errbuffer_minaddr(void)
{
  return ebuf->minaddr;
}
uint32_t errbuffer_maxaddr(void)
{
  return ebuf->maxaddr;
}
uint32_t errbuffer_head(void)
{
  return ebuf->head;
}
uint16_t errbuffer_last(void)
{
  return ebuf->last;
}
uint16_t errbuffer_counter(void)
{
  return ebuf->counter;
}
uint16_t errbuffer_continue(void)
{
  return ebuf->n_continue;
}

uint32_t errbuffer_entry(uint16_t errcode, uint16_t errdata)
{
  uint16_t eprmtime = xTaskGetTickCountFromISR() * portTICK_PERIOD_MS / 60000; // Time in minutes
  uint16_t count = ((ebuf->counter / 4) << (ERRCODE_OFFSET + ERRDATA_OFFSET)) & COUNTER_MASK;
  uint16_t code = (errcode << ERRDATA_OFFSET) & ERRCODE_MASK;
  uint16_t message = count | code | (errdata & ERRDATA_MASK);
  uint32_t entry = ((uint32_t)eprmtime << 16) | ((uint32_t)message);
  return entry;
}

// Specific error functions using continuation codes
void errbuffer_temp_high(uint8_t tm4c, uint8_t fpga, uint8_t ffly, uint8_t dcdc)
{
  if (ffly == (uint8_t)(-55)) {
    ffly = 0;
  }
  errbuffer_put(EBUF_TEMP_HIGH, tm4c);
  errbuffer_put(EBUF_CONTINUATION, fpga);
  errbuffer_put(EBUF_CONTINUATION, ffly);
  errbuffer_put(EBUF_CONTINUATION, dcdc);
  return;
}

void errbuffer_volt_high(uint8_t genfpga, uint8_t fpga1, uint8_t fpga2)
{
  errbuffer_put(EBUF_VOLT_HIGH, genfpga);
  errbuffer_put(EBUF_CONTINUATION, fpga1);
  errbuffer_put(EBUF_CONTINUATION, fpga2);
  return;
}

void errbuffer_power_fail(uint16_t failmask)
{
  errbuffer_put(EBUF_PWR_FAILURE, (failmask >> 8) & 0xFFU);
  errbuffer_put(EBUF_CONTINUATION, failmask & 0xFFU);
}

void errbuffer_power_fail_clear(void)
{
  errbuffer_put(EBUF_PWR_FAILURE_CLR, 0);
}

// These register locations are defined by the ARM Cortex-M4F
// specification and do not depend on the TM4C1290NCPDT
// ARM DWT
#define DEMCR_TRCENA 0x01000000

/* Core Debug registers */
#define DEMCR      (*(volatile uint32_t *)0xE000EDFC)
#define DWT_CTRL   (*(volatile uint32_t *)0xe0001000)
#define CYCCNTENA  (1 << 0)
#define DWT_CYCCNT ((volatile uint32_t *)0xE0001004)
#define CPU_CYCLES *DWT_CYCCNT

static uint32_t counter, prev_count;

void stopwatch_reset(void)
{
  /* Enable DWT */
  DEMCR |= DEMCR_TRCENA;
  *DWT_CYCCNT = 0;
  /* Enable CPU cycle counter */
  DWT_CTRL |= CYCCNTENA;
  counter = prev_count = 0U;
}

uint32_t stopwatch_getticks(void)
{
  uint32_t curr_count = CPU_CYCLES;
  uint32_t diff = curr_count - prev_count;
  prev_count = curr_count;
  counter += diff >> 12; // degrade counter a bit-- don't need this precision
  return counter;
}

void float_to_ints(float val, int *tens, int *fraction)
{
  *tens = (int)val;
  *fraction = (int)ABS((val - *tens) * 100.0f + 0.5f);

  return;
}

// compare two times in seconds and make sure they are within the last 60 seconds
// if not the value is 'stale' and the function returns true
bool checkStale(unsigned oldTime, unsigned newTime)
{
  return ((oldTime < newTime) && (newTime - oldTime) > 60);
}
