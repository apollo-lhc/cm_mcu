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


// write, read or toggle GPIO pin by name
void write_gpio_pin(int pin, uint8_t value);
uint8_t read_gpio_pin(int pin);
uint8_t toggle_gpio_pin(int pin);


void setupActiveLowPins(void);


// EEPROM
#define SN_ADDR 64
#define FF_ADDR (SN_ADDR+4)

void write_eeprom(uint32_t data, uint32_t addr);
uint32_t read_eeprom_single(uint32_t addr);
uint64_t read_eeprom_multi(uint32_t addr);

// EEPROM buffer
#define EBUF_MINBLK 2
#define EBUF_MAXBLK 5

#define ERRDATA_OFFSET 7	// Number of bits reserved for error data
#define ERRCODE_OFFSET 5	// Number of bits reserved for error codes
#define COUNTER_OFFSET (16-ERRDATA_OFFSET-ERRCODE_OFFSET)	// Number of bits reserved for message counter

#define ERRDATA_MASK ((1<<(ERRDATA_OFFSET))-1)
#define ERRCODE_MASK ((1<<(ERRDATA_OFFSET+ERRCODE_OFFSET))-1-ERRDATA_MASK)
#define COUNTER_MASK ((1<<(ERRDATA_OFFSET+ERRCODE_OFFSET+COUNTER_OFFSET))-1-ERRDATA_MASK-ERRCODE_MASK)

#define COUNTER_UPDATE 4	//Number of repeated entries that initiates a hardware counter update (re-write entry)

extern const char* ebuf_errstrings[];

// error codes without data
#define EBUF_RESTART			1
#define EBUF_RESET_BUFFER		2
#define EBUF_POWER_OFF			3
#define EBUF_POWER_OFF_TEMP		4
#define EBUF_POWER_ON			5
#define EBUF_TEMP_NORMAL		6
#define EBUF_HARDFAULT      7
#define EBUF_ASSERT         8
#define EBUF_STACKOVERFLOW  9

// error codes with data
#define EBUF_WITH_DATA			10		// for comparison
#define EBUF_CONTINUATION		10
#define EBUF_PWR_FAILURE		11
#define EBUF_TEMP_HIGH			12

// Restart Reasons, values of reset cause (RESC) register,
// at 0x5c offset in TM4C1290NCPDT
#define EBUF_RESTART_WDOG (SYSCTL_CAUSE_WDOG1|SYSCTL_CAUSE_WDOG0)
#define EBUF_RESTART_SW   SYSCTL_CAUSE_SW
#define EBUF_RESTART_POR  SYSCTL_CAUSE_POR
#define EBUF_RESTART_EXT  SYSCTL_CAUSE_EXT





typedef struct error_buffer_t error_buffer_t;
typedef error_buffer_t* errbuf_handle_t;

extern errbuf_handle_t ebuf;

void errbuffer_init(errbuf_handle_t ebuf, uint8_t minblk, uint8_t maxblk);
void errbuffer_reset(errbuf_handle_t ebuf);
void errbuffer_put(errbuf_handle_t ebuf, uint16_t errcode, uint16_t errdata);
// TODO: change get to not count continue codes as entries (append to prior entries?)
void errbuffer_get(errbuf_handle_t ebuf, uint32_t num, uint32_t (*arrptr)[num]);

// this version bypasses the gatekeeper task and should only be used in ISR only
void errbuffer_put_raw(errbuf_handle_t ebuf, uint16_t errcode, uint16_t errdata);

uint32_t errbuffer_capacity(errbuf_handle_t ebuf);
uint32_t errbuffer_minaddr(errbuf_handle_t ebuf);
uint32_t errbuffer_maxaddr(errbuf_handle_t ebuf);
uint32_t errbuffer_head(errbuf_handle_t ebuf);
uint16_t errbuffer_last(errbuf_handle_t ebuf);
uint16_t errbuffer_counter(errbuf_handle_t ebuf);
uint16_t errbuffer_continue(errbuf_handle_t ebuf);

uint32_t errbuffer_entry(uint16_t errcode, uint16_t errdata);

// specific error functions
void errbuffer_temp_high(uint8_t tm4c, uint8_t fpga, uint8_t ffly, uint8_t dcdc);
// timers used for FreeRTOS accounting
void stopwatch_reset(void);
uint32_t stopwatch_getticks();

#endif /* COMMON_UTILS_H_ */
