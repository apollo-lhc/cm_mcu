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
	xQueueSendToBack(xEPRMQueue_in, &message, portMAX_DELAY);	// is there a good time to set this to?
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
	uint32_t capacity;
};

void decrease_head(errbuf_handle_t ebuf){
	(ebuf->head)-=4;
	if(ebuf->head<ebuf->minaddr){ebuf->head=ebuf->maxaddr;}
	return;
}
void increase_head(errbuf_handle_t ebuf){
	(ebuf->head)+=4;	// Increment by 1 word
	if (ebuf->head>ebuf->maxaddr){ebuf->head=ebuf->minaddr;}
	return;
}

uint32_t errbuffer_findhead(errbuf_handle_t ebuf){
	uint32_t b1=ebuf->minaddr,b2=ebuf->minaddr+4, maddr=ebuf->maxaddr;
	uint32_t addr = b1, entry, previous=1;
	while(addr<=maddr){
		entry = read_eeprom_single(addr);
		if(entry==0&&previous==0){
			b2 = addr;
			b1 = addr-4;
			break;
		}
		previous = entry;
		addr+=4;
	}
	write_eeprom(0,b1);
	write_eeprom(0,b2);
	return b1;
}

errbuf_handle_t errbuffer_init(uint8_t minblk, uint8_t maxblk){
	errbuf_handle_t ebuf = pvPortMalloc(sizeof(error_buffer_t));
	ebuf->minaddr=EEPROMAddrFromBlock(minblk);
	ebuf->maxaddr=EEPROMAddrFromBlock(maxblk);
	ebuf->capacity= (uint32_t)((ebuf->maxaddr - ebuf->minaddr)>>2);
	ebuf->head=errbuffer_findhead(ebuf);

	errbuffer_put(ebuf,RESTART);
	return ebuf;
}

void errbuffer_reset(errbuf_handle_t ebuf){
	// TODO: set all words to FFFFFFFF except 1st and 2nd which are 00000000
	// TODO: set head to 1st entry
	return;
}

void errbuffer_put(errbuf_handle_t ebuf, uint32_t entry){
	write_eeprom(entry,ebuf->head);
	increase_head(ebuf);
	write_eeprom(0,ebuf->head+4);
	return;
}

void errbuffer_getlast5(errbuf_handle_t ebuf, uint32_t (*arrptr)[5]){
	int i=0,j=0,max=5;
	while(i<max){i++; decrease_head(ebuf);}
	while(j<max){
		(*arrptr)[j] = read_eeprom_single(ebuf->head);
		increase_head(ebuf);
		j++;
	}
	return;
}

uint32_t errbuffer_entry(void){
	return 0x12345678;	// just for testing
}

uint32_t errbuffer_capacity(errbuf_handle_t ebuf){
	return ebuf->capacity; }
uint32_t errbuffer_minaddr(errbuf_handle_t ebuf){
	return ebuf->minaddr; }
uint32_t errbuffer_maxaddr(errbuf_handle_t ebuf){
	return ebuf->maxaddr; }

