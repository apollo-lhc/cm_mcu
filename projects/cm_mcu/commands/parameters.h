/*
 * parameters.h
 *
 *  Created on: Jan 13, 2021
 *      Author: fatimayousuf
 */

#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_types.h"
#include "inc/hw_memmap.h"
#include "inc/hw_nvic.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "driverlib/systick.h"
#include "driverlib/sysctl.h"
#include "driverlib/eeprom.h"

// local includes
#include "common/i2c_reg.h"
#include "common/uart.h"
#include "common/power_ctl.h"
#include "common/pinsel.h"
#include "common/smbus.h"
#include "common/utils.h"
#include "common/microrl.h"

// FreeRTOS includes
#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "stream_buffer.h"
#include "queue.h"

// strlen, strtol, and strncpy
#include <string.h>
#include <stdlib.h>

// TivaWare includes
#include "driverlib/uart.h"

#include "MonitorTask.h"
#include "CommandLineTask.h"
#include "I2CCommunication.h"
#include "Tasks.h"
#include "AlarmUtilities.h"

#include "clocksynth.h"

#ifdef DEBUG_CON
// prototype of mutex'd print
#define DPRINT(x) Print(x)
#else // DEBUG_CON
#define DPRINT(x)
#endif // DEBUG_CON

void Print(const char *str);

// local sprintf prototype
int snprintf(char *buf, unsigned int count, const char *format, ...);

#define MAX_INPUT_LENGTH  50
#define MAX_OUTPUT_LENGTH 512
#define SCRATCH_SIZE 512

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wformat" // because of our mini-sprintf

extern tSMBus g_sMaster1;
extern tSMBusStatus eStatus1;
extern tSMBus g_sMaster2;
extern tSMBusStatus eStatus2;
extern tSMBus g_sMaster3;
extern tSMBusStatus eStatus3;
extern tSMBus g_sMaster4;
extern tSMBusStatus eStatus4;
extern tSMBus g_sMaster6;
extern tSMBusStatus eStatus6;
