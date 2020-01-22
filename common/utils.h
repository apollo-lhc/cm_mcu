/*
 * util.h
 *
 *  Created on: May 9, 2019
 *      Author: wittich
 */

#ifndef COMMON_UTILS_H_
#define COMMON_UTILS_H_

#include <stdint.h>
#include <stdbool.h>

// write to and read from eeprom
void write_eeprom(uint32_t data, uint32_t addr);
uint32_t read_eeprom_single(uint32_t addr);
uint64_t read_eeprom_multi(uint32_t addr);


// write, read or toggle GPIO pin by name
void write_gpio_pin(int pin, uint8_t value);
uint8_t read_gpio_pin(int pin);
uint8_t toggle_gpio_pin(int pin);


void setupActiveLowPins(void);

// EEPROM buffer

// error codes
#define RESTART 1
#define RESET_BUFFER 2

typedef struct error_buffer_t error_buffer_t;
typedef error_buffer_t* errbuf_handle_t;

extern errbuf_handle_t ebuf;

void errbuffer_init(errbuf_handle_t ebuf, uint8_t minblk, uint8_t maxblk);
void errbuffer_reset(errbuf_handle_t ebuf);

void errbuffer_put(errbuf_handle_t ebuf, uint16_t data);
void errbuffer_getlast5(errbuf_handle_t ebuf, uint32_t (*arrptr)[5]);
uint32_t errbuffer_entry(void);

uint32_t errbuffer_capacity(errbuf_handle_t ebuf);
uint32_t errbuffer_minaddr(errbuf_handle_t ebuf);
uint32_t errbuffer_maxaddr(errbuf_handle_t ebuf);
uint32_t errbuffer_head(errbuf_handle_t ebuf);


#endif /* COMMON_UTILS_H_ */
