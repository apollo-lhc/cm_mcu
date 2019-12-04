/*
 * EEPROMTask.c
 *
 *  Created on: Dec 3, 2019
 *      Author: glg62
 */


#include "driverlib/eeprom.h"
#include "common/utils.h"

#include "EEPROMTask.h"

// This queue is used to handle EEPROM commands
QueueHandle_t xEPRMQueue;

// Define outgoing message out here, then make simple function to retrieve it
static uint64_t message_out;

uint64_t EEPROM_read(){
	return message_out;
}

void EEPROMTask(void *parameters){
	uint64_t message_in;
	// At the moment, let's do 1 byte key , 1 byte optional addr, 2 bytes data
	// write data should be at most 1 word (32 bits)

	// outgoing messages should be going exclusively to cli task, I think?

	for(;;){
		xQueueReceive(xEPRMQueue, &message_in, portMAX_DELAY);
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
			break;
		case EPRM_READ_DOUBLE:
			message_out = read_eeprom_multi(addr);
			break;
		case EPRM_SET_ID:
			// TODO
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
