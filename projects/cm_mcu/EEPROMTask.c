/*
 * EEPROMTask.c
 *
 *  Created on: Dec 3, 2019
 *      Author: glg62
 */

// includes for types
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>

#include "driverlib/rom.h"
#include "driverlib/eeprom.h"
#include "common/utils.h"

#include "Tasks.h"

QueueHandle_t xEPRMQueue_in;
QueueHandle_t xEPRMQueue_out;


uint64_t EPRMMessage(uint64_t action,uint64_t addr,uint64_t data){
	return ((action<<48)|(addr<<32)|data);
}

// write single word to eeprom
void write_single(uint32_t data, uint32_t addr)
{
	uint32_t dlen,*dataptr;
	dataptr = &data;
	dlen = 4;
	ROM_EEPROMProgram(dataptr,addr,dlen);
}
// read single word from eeprom
uint64_t read_single(uint32_t addr)
{
	uint32_t data,*dataptr;
	uint32_t dlen = 4;
	data = 0x0;
	dataptr = &data;
	ROM_EEPROMRead(dataptr,addr,dlen);
	return (uint64_t)data;
}
// read 2 words from eeprom
uint64_t read_double(uint32_t addr)
{
	static uint32_t dataptr[2] = {0x0, 0x0};
	uint32_t dlen = 8;
	uint32_t *data0 = &dataptr[0];
	ROM_EEPROMRead(data0,addr,dlen);
	uint64_t data = ((uint64_t)dataptr[0])|((uint64_t)dataptr[1]<<32);
	return data;
}

void EEPROMTask(void *parameters){
	uint64_t message_in, message_out;

	// At the moment, let's do 1 byte key , 1 byte optional addr, 2 bytes data
	// Example message:
	// 0x 0001 0022 ffffffff
	// Writes ffffffff to register 0x22

	for(;;){
		if(xQueueReceive(xEPRMQueue_in, &message_in, portMAX_DELAY)){

			uint16_t message_type = (uint16_t)(message_in>>48);
			uint16_t addr = (uint16_t)(message_in>>32);
			uint32_t data = (uint32_t)(message_in);

			switch(message_type){
			case EPRM_WRITE_SINGLE:
				write_single(data,addr);
				break;
			case EPRM_READ_SINGLE:
				message_out = read_single(addr);
				xQueueSendToBack(xEPRMQueue_out, &message_out, portMAX_DELAY);
				break;
			case EPRM_READ_DOUBLE:
				message_out = read_double(addr);
				xQueueSendToBack(xEPRMQueue_out, &message_out, portMAX_DELAY);
				break;
			case EPRM_UNLOCK_BLOCK: ;
				uint32_t *dataptr = &data;
				ROM_EEPROMBlockUnlock(addr, dataptr, 1);
				break;
			case EPRM_LOCK_BLOCK:
				ROM_EEPROMBlockLock(addr);
				break;
			case EPRM_PASS_SET: ;
				uint32_t *passptr = &data;
				EEPROMBlockProtectSet(1, EEPROM_PROT_RW_LRO_URW);
				EEPROMBlockPasswordSet(1, passptr, 1);
				EEPROMBlockLock(1);
				break;
			default:
				break;
			}
			continue;
		}
	}
}
