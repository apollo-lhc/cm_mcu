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

#include "driverlib/eeprom.h"
#include "common/utils.h"

#include "Tasks.h"

QueueHandle_t xEPRMQueue_in;
QueueHandle_t xEPRMQueue_out;


uint64_t EPRMMessage(uint64_t action,uint64_t addr,uint64_t data){
	return ((action<<48)|(addr<<32)|data);
}

void EEPROMTask(void *parameters){
	uint64_t message_in, message_out;

	// At the moment, let's do 1 byte key , 1 byte optional addr, 2 bytes data
	// write data should be at most 1 word (32 bits)

	for(;;){
		if(xQueueReceive(xEPRMQueue_in, &message_in, portMAX_DELAY)){
			// Example message:
			// 0x 0001 0022 ffffffff
			// Corresponds to writing ffffffff to register 0x22
			uint16_t message_type = (uint16_t)(message_in>>48);
			uint16_t addr = (uint16_t)(message_in>>32);
			uint32_t data = (uint32_t)(message_in);

			switch(message_type){
			case EPRM_WRITE_SINGLE:
				write_eeprom_single(data,addr);
				break;
			case EPRM_READ_SINGLE:
				message_out = (uint64_t)read_eeprom_single(addr);
				xQueueSendToBack(xEPRMQueue_out, &message_out, portMAX_DELAY);
				break;
			case EPRM_READ_DOUBLE:
				message_out = read_eeprom_multi(addr);
				xQueueSendToBack(xEPRMQueue_out, &message_out, portMAX_DELAY);
				break;
			case EPRM_UNLOCK_BLOCK: ;
				uint32_t *dataptr = &data;
				EEPROMBlockUnlock(addr, dataptr, 1);
				break;
			case EPRM_LOCK_BLOCK:
				EEPROMBlockLock(addr);
				break;
			case EPRM_BUFF_IN:
				// need buffer method
				break;
			case EPRM_BUFF_OUT:
				// need buffer method and return message
				break;
			default:
				break;
			}
			continue;
		}
	}
}
