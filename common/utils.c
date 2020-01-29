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
	uint32_t init;
};

error_buffer_t errbuf = {.minaddr=0,.maxaddr=0,.head=0,.capacity=0,.init=0};
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
	ebuf->init=1;
	ebuf->minaddr=EEPROMAddrFromBlock(minblk);
	ebuf->maxaddr=EEPROMAddrFromBlock(maxblk+1)-4;
	ebuf->capacity= (uint32_t)(maxblk-minblk+1)*16;
	ebuf->head=errbuffer_findhead(ebuf);

	errbuffer_put(ebuf,RESTART);
}

void errbuffer_reset(errbuf_handle_t ebuf){
	if(ebuf->init==0){ errbuffer_init(ebuf,EBUFMINBLK,EBUFMAXBLK); }

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
	errbuffer_put(ebuf, RESET_BUFFER);
	return;
}

void errbuffer_put(errbuf_handle_t ebuf, uint16_t errcode){
	if(ebuf->init==0){ errbuffer_init(ebuf,EBUFMINBLK,EBUFMAXBLK); }

	uint16_t eprmtime = xTaskGetTickCountFromISR()*portTICK_PERIOD_MS/60000;	// Time in minutes
	uint32_t entry = ((uint32_t)eprmtime<<16)+(uint32_t)errcode;

	write_eeprom(entry,ebuf->head);
	ebuf->head = increase_head(ebuf);
	write_eeprom(0,increase_head(ebuf));
	return;
}

void errbuffer_getlast5(errbuf_handle_t ebuf, uint32_t (*arrptr)[5]){
	if(ebuf->init==0){ errbuffer_init(ebuf,EBUFMINBLK,EBUFMAXBLK); }

	int i=0,j=0,max=5;
	while(i<max){i++; ebuf->head = decrease_head(ebuf);}
	while(j<max){
		(*arrptr)[j] = read_eeprom_single(ebuf->head);
		ebuf->head = increase_head(ebuf);
		j++;
	}
	return;
}

uint32_t errbuffer_capacity(errbuf_handle_t ebuf){
	if(ebuf->init==0){ errbuffer_init(ebuf,EBUFMINBLK,EBUFMAXBLK); }
	return ebuf->capacity; }
uint32_t errbuffer_minaddr(errbuf_handle_t ebuf){
	if(ebuf->init==0){ errbuffer_init(ebuf,EBUFMINBLK,EBUFMAXBLK); }
	return ebuf->minaddr; }
uint32_t errbuffer_maxaddr(errbuf_handle_t ebuf){
	if(ebuf->init==0){ errbuffer_init(ebuf,EBUFMINBLK,EBUFMAXBLK); }
	return ebuf->maxaddr; }
uint32_t errbuffer_head(errbuf_handle_t ebuf){
	if(ebuf->init==0){ errbuffer_init(ebuf,EBUFMINBLK,EBUFMAXBLK); }
	return ebuf->head; }

