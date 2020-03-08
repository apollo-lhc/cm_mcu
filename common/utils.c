/*
 * utils.c
 *
 *  Created on: May 9, 2019
 *      Author: wittich
 */

#include <stddef.h>
#include <stdlib.h>
#include "common/utils.h"
#include "common/pinsel.h"
#include "driverlib/gpio.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "driverlib/debug.h"
#include "driverlib/eeprom.h"
#include "FreeRTOS.h"
#include "Tasks.h"

// write single word to eeprom
void write_eeprom(uint32_t data, uint32_t addr)
{
	uint64_t message;
	message = EPRMMessage((uint64_t)EPRM_WRITE_SINGLE,addr,data);
	xQueueSendToBack(xEPRMQueue_in, &message, portMAX_DELAY);
	return;
}

// read single word from eeprom
uint32_t read_eeprom_single(uint32_t addr)
{
	uint32_t data;
	uint64_t message = EPRMMessage((uint64_t)EPRM_READ_SINGLE,addr,0);
	xQueueSendToBack(xEPRMQueue_in, &message, portMAX_DELAY);
	xQueueReceive(xEPRMQueue_out, &data, portMAX_DELAY);
	return data;
}

// read 2 words from eeprom
uint64_t read_eeprom_multi(uint32_t addr)
{
	uint64_t data, message;
	message = EPRMMessage((uint64_t)EPRM_READ_DOUBLE,addr,0);
	xQueueSendToBack(xEPRMQueue_in, &message, portMAX_DELAY);
	xQueueReceive(xEPRMQueue_out, &data, portMAX_DELAY);
	return data;
}

// write by pin number or name
void write_gpio_pin(int pin, uint8_t value)
{
  ASSERT(value==1||value==0);
  uint32_t gport;
  uint8_t  gpin;
  pinsel(pin, &gport, &gpin);
  uint8_t pinval;
  if ( value == 1 )
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
  uint8_t  gpin;
  uint8_t value;
  pinsel(pin, &gport, &gpin);
  value = MAP_GPIOPinRead(gport, gpin);
  if ( value ) value = 1;
  return value;
}

// write by pin number or name
uint8_t toggle_gpio_pin(int pin)
{
  uint8_t val = read_gpio_pin(pin);
  if ( val ) val = 0;
  else
	  val = 1;
  write_gpio_pin(pin, val);
  return val;
}

// Set up all the active low pins to be off (i.e., high)
void setupActiveLowPins(void)
{
  const int pins[] = {
      _FPGA_I2C_RESET,
      _PWR_I2C_RESET,
      _CLOCKS_I2C_RESET,
      _V_OPTICS_I2C_RESET,
      _K_OPTICS_I2C_RESET
  };
  const int8_t npins = sizeof(pins)/pins[0];
  for (int8_t i = 0; i < npins; ++i ) {
    write_gpio_pin(pins[i], 0x1);
  }
}

// EEPROM Buffer

struct error_buffer_t {
	uint32_t minaddr;
	uint32_t maxaddr;
	uint32_t head;
	uint32_t capacity;	// in # entries
	uint16_t last;		// most recent error code
	uint16_t counter;
};

error_buffer_t errbuf = {.minaddr=0,.maxaddr=0,.head=0,.capacity=0,.last=0,.counter=0};
errbuf_handle_t ebuf = &errbuf;


uint32_t decrease_head(errbuf_handle_t ebuf){
	uint32_t head = ebuf->head - 4;
	if(head<ebuf->minaddr){head=ebuf->maxaddr;}
	return head;
}
uint32_t increase_head(errbuf_handle_t ebuf){
	uint32_t head = ebuf->head + 4;
	if (head>ebuf->maxaddr){head=ebuf->minaddr;}
	return head;
}

uint32_t errbuffer_findhead(errbuf_handle_t ebuf){
	uint32_t ahead=ebuf->minaddr, head=ahead, cap=ebuf->capacity;
	uint32_t entry, previous=1, i=0;
	while(i<=cap){
		ahead+=4;
		if (ahead>ebuf->maxaddr){ahead=ebuf->minaddr;}
		entry = read_eeprom_single(ahead);
		if(entry==0&&previous==0){	break;	}
		previous = entry;
		head = ahead;
		i++;
	}
	return head;
}

void errbuffer_init(errbuf_handle_t ebuf, uint8_t minblk, uint8_t maxblk){
	ebuf->minaddr=EEPROMAddrFromBlock(minblk);
	ebuf->maxaddr=EEPROMAddrFromBlock(maxblk+1)-4;
	ebuf->capacity= (uint32_t)(maxblk-minblk+1)*16;
	ebuf->head=errbuffer_findhead(ebuf);
	ebuf->last=0;
	ebuf->counter=0;
}

void errbuffer_reset(errbuf_handle_t ebuf){
	uint32_t addr=ebuf->minaddr;
	uint32_t maddr=ebuf->maxaddr;

	write_eeprom(0,addr);
	write_eeprom(0,addr+4);
	addr+=0x8;
	while(addr<=maddr){
			write_eeprom(0xffffffff,addr);
			addr+=4;
		}
	ebuf->head = ebuf->minaddr;
	ebuf->counter=0;
	ebuf->last=0;
	errbuffer_put(ebuf, EBUF_RESET_BUFFER,0);
	return;
}

void errbuffer_put(errbuf_handle_t ebuf, uint16_t errcode, uint16_t errdata){
	uint16_t oldcount=ebuf->counter;
	// If duplicated error code...
	if(errcode == ebuf->last){

		// if counter is not a multiple of COUNTER_UPDATE, don't write new entry
		if(oldcount%COUNTER_UPDATE!=0){ ebuf->counter=ebuf->counter+1; }

		// if counter has already reached max value, increment head
		if(oldcount%(1<<COUNTER_OFFSET)==0){	//Change this to use COUNTER_OFFSET
			ebuf->counter=0;
			ebuf->head = increase_head(ebuf);
			write_eeprom(0,increase_head(ebuf)); }

		// if counter is multiple of COUNTER_UPDATE, write entry and increment counter
		if(oldcount%COUNTER_UPDATE==0){
			ebuf->counter=ebuf->counter+1;
			write_eeprom(errbuffer_entry(errcode,errdata),decrease_head(ebuf)); }
	}
	else { // If new error code...
		ebuf->counter=0;
		ebuf->last = errcode;
		write_eeprom(errbuffer_entry(errcode,errdata),ebuf->head);
		ebuf->head=increase_head(ebuf);
		write_eeprom(0,increase_head(ebuf));
	}
	return;
}

void errbuffer_get(errbuf_handle_t ebuf, uint32_t num, uint32_t (*arrptr)[num]){
	int i=0,j=0,max=num;
	while(i<max){i++; ebuf->head = decrease_head(ebuf);}
	while(j<max){
		(*arrptr)[j] = read_eeprom_single(ebuf->head);
		ebuf->head = increase_head(ebuf);
		j++;
	}
	return;
}

uint32_t errbuffer_capacity(errbuf_handle_t ebuf){
	return ebuf->capacity; }
uint32_t errbuffer_minaddr(errbuf_handle_t ebuf){
	return ebuf->minaddr; }
uint32_t errbuffer_maxaddr(errbuf_handle_t ebuf){
	return ebuf->maxaddr; }
uint32_t errbuffer_head(errbuf_handle_t ebuf){
	return ebuf->head; }
uint16_t errbuffer_last(errbuf_handle_t ebuf){
	return ebuf->last; }
uint16_t errbuffer_counter(errbuf_handle_t ebuf){
	return ebuf->counter; }

uint32_t errbuffer_entry(uint16_t errcode, uint16_t errdata){
	uint16_t eprmtime = xTaskGetTickCountFromISR()*portTICK_PERIOD_MS/60000;	// Time in minutes
	uint16_t count = ((ebuf->counter/4)<<(ERRCODE_OFFSET+ERRDATA_OFFSET))&COUNTER_MASK;
	uint16_t code = (errcode<<ERRDATA_OFFSET)&ERRCODE_MASK;
	uint16_t message = count|code|errdata;
	uint32_t entry = ((uint32_t)eprmtime<<16)|((uint32_t)message);
	return entry;
}

