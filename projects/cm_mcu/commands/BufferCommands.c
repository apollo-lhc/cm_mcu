/*
 * BufferCommands.c
 *
 *  Created on: Jan 14, 2021
 *      Author: fatimayousuf
 */

#include "BufferCommands.h"

// This command takes 1 arg, the data to be written to the buffer
BaseType_t errbuff_in(int argc, char **argv, char* m)
{
  int copied = 0;

  uint32_t data;
  data = strtoul(argv[1], NULL, 16);
  errbuffer_put(data, 0);
  copied +=
      snprintf(m + copied, SCRATCH_SIZE - copied, "Data written to EEPROM buffer: %x\r\n", data);

  return pdFALSE;
}
#define EBUFFOUT_MAX_ENTRIES 64
BaseType_t errbuff_out(int argc, char **argv, char* m)
{
  int copied = 0;

  uint32_t num = strtoul(argv[1], NULL, 10);
  if (num > EBUFFOUT_MAX_ENTRIES) {
    copied += snprintf(m + copied, SCRATCH_SIZE - copied, "Please enter a number 64 or less \r\n");
    return pdFALSE;
  }
  uint32_t arr[EBUFFOUT_MAX_ENTRIES];
  uint32_t(*arrptr)[EBUFFOUT_MAX_ENTRIES] = &arr;
  errbuffer_get(num, arrptr);

  static int i = 0;
  if (i == 0) {
    copied += snprintf(m + copied, SCRATCH_SIZE - copied, "Entries in EEPROM buffer:\r\n");
  }
  for (; i < num; ++i) {
    uint32_t word = (*arrptr)[i];
    uint16_t errcode = EBUF_ERRCODE(word);
    // if this is a continuation and it's not the first entry we see
    if (errcode == EBUF_CONTINUATION && i != 0) {
      uint16_t errdata = EBUF_DATA(word);
      copied += snprintf(m + copied, SCRATCH_SIZE - copied, " 0x%02x", errdata);
    }
    else {
      copied += errbuffer_get_messagestr(word, m + copied, SCRATCH_SIZE - copied);
    }
    if ((SCRATCH_SIZE - copied) < 30 && (i < num)) { // catch when buffer is almost full
      ++i;
      return pdTRUE;
    }
  }
  copied += snprintf(m + copied, SCRATCH_SIZE - copied, "\r\n");
  i = 0;
  return pdFALSE;
}

// Takes no arguments
BaseType_t errbuff_info(int argc, char **argv, char* m)
{
  int copied = 0;
  uint32_t cap, minaddr, maxaddr, head;
  uint16_t last, counter, n_continue;

  cap = errbuffer_capacity();
  minaddr = errbuffer_minaddr();
  maxaddr = errbuffer_maxaddr();
  head = errbuffer_head();
  last = errbuffer_last();
  counter = errbuffer_counter();
  n_continue = errbuffer_continue();

  copied += snprintf(m + copied, SCRATCH_SIZE - copied, "Capacity:        %d words\r\n", cap);
  copied += snprintf(m + copied, SCRATCH_SIZE - copied, "Min address:     0x%08x\r\n", minaddr);
  copied += snprintf(m + copied, SCRATCH_SIZE - copied, "Max address:     0x%08x\r\n", maxaddr);
  copied += snprintf(m + copied, SCRATCH_SIZE - copied, "Head address:    0x%08x\r\n", head);
  copied += snprintf(m + copied, SCRATCH_SIZE - copied, "Last entry:      0x%0x\r\n", last);
  copied += snprintf(m + copied, SCRATCH_SIZE - copied, "Message counter: %d\r\n", counter);
  copied += snprintf(m + copied, SCRATCH_SIZE - copied, "Continue codes:  %d\r\n", n_continue);

  return pdFALSE;
}

// Takes no arguments
BaseType_t errbuff_reset(int argc, char **argv, char* m)
{
  errbuffer_reset();
  return pdFALSE;
}
