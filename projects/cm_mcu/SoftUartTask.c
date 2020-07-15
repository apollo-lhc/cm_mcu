/*
 * SoftUartTask.c
 *
 *  Created on: Jan 3, 2020
 *      Author: wittich
 */
// includes for types
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

// to be moved
#include "driverlib/sysctl.h"
#include "driverlib/timer.h"
#include "driverlib/gpio.h"
#include "inc/hw_memmap.h"
#include "inc/hw_ints.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"

#include "common/softuart.h"

#include "Tasks.h"
#include "MonitorTask.h"

#define SZ 20

// Data Format
// https://github.com/apollo-lhc/SM_ZYNQ_FW/blob/develop/src/CM_interface/CM_Monitoring_data_format.txt

#define SENSOR_MESSAGE_START_OF_FRAME_NIB 2
#define SENSOR_MESSAGE_DATA_FRAME_NIB     0
#define SENSOR_MESSAGE_HEADER_OFFSET 6
#define SENSOR_SIX_BITS 0x3F
#define SENSOR_MESSAGE_START_OF_FRAME (SENSOR_MESSAGE_START_OF_FRAME_NIB << SENSOR_MESSAGE_HEADER_OFFSET)
#define SENSOR_MESSAGE_DATA_FRAME (SENSOR_MESSAGE_DATA_FRAME_NIB << SENSOR_MESSAGE_HEADER_OFFSET)

static
void format_data(const uint8_t sensor, const uint16_t data, uint8_t message[4])
{
  // header and start of sensor (6 bits, sensor[7:2]
  message[0] = SENSOR_MESSAGE_START_OF_FRAME  | (( sensor >> 2 ) & SENSOR_SIX_BITS) ;
  // data frame 1, rest of sensor[1:0] (2 bits) and start of data[15:12] (4 bits)
  message[1]  = SENSOR_MESSAGE_DATA_FRAME ;
  message[1] |= ((sensor & 0x3 )<< 4) | ((data>>12)&0xF);
  // data frame 2, data[11:6] (6 bits)
  message[2]  = SENSOR_MESSAGE_DATA_FRAME;
  message[2] |= (data >> 6) & 0x3F;
  // // data frame 3, data[5:0] ( 6 bits )
  message[3]  = SENSOR_MESSAGE_DATA_FRAME;
  message[3] |= data & 0x3F;
}

#ifdef SUART_TEST_MODE
static uint8_t  testaddress = 0x0;
static uint16_t testdata    = 0xaaff;
static bool inTestMode = true;
static uint8_t testmode = 2;
void setSUARTTestData(uint8_t sensor, uint16_t value)
{
  testaddress = sensor;
  testdata = value;  
}

uint8_t getSUARTTestMode()
{
  return testmode;
}

uint8_t getSUARTTestSensor()
{
  return testaddress;
}

uint16_t getSUARTTestData()
{
  return testdata;
}
#else //
const static bool inTestMode = false;
#endif // SUART_TEST_MODE
//
// The buffer used to hold the transmit data.
//
unsigned char g_pucTxBuffer[16];
//
// The buffer used to hold the receive data.
//
//unsigned short g_pusRxBuffer[16];

// The number of processor clocks in the time period of a single bit on the
// software UART interface.
//
unsigned long g_ulBitTime;

#define TARGET_BAUD_RATE 115200

tSoftUART g_sUART;

extern uint32_t g_ui32SysClock;

QueueHandle_t xSoftUartQueue;

void SoftUartTask(void *parameters)
{
  // Setup
  // initialize to the current tick time
  TickType_t xLastWakeTime = xTaskGetTickCount();

  // Initialize the software UART instance data.
  //
  SoftUARTInit(&g_sUART);
  //
  // Configure the pins used for the software UART.
  //
  SoftUARTTxGPIOSet(&g_sUART, GPIO_PORTM_BASE, GPIO_PIN_6);
  // SoftUARTRxGPIOSet(&g_sUART, GPIO_PORTE_BASE, GPIO_PIN_1);
  //
  // Configure the data buffers used as the transmit and receive buffers.
  //
  SoftUARTTxBufferSet(&g_sUART, g_pucTxBuffer, 16);
  // SoftUARTRxBufferSet(&g_sUART, g_pusRxBuffer, 16);
  //
  // Enable the GPIO modules that contains the GPIO pins to be used by
  // the software UART.
  //
  // MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
  // MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
  //
  // Configure the software UART module: 8 data bits, no parity, and one
  // stop bit.
  //
  // this also sets the pins.
  SoftUARTConfigSet(&g_sUART,
                    (SOFTUART_CONFIG_WLEN_8 | SOFTUART_CONFIG_PAR_NONE |
                     SOFTUART_CONFIG_STOP_ONE));
  //
  // Compute the bit time for the chosen baud rate
  //
  g_ulBitTime = (g_ui32SysClock / TARGET_BAUD_RATE) - 1;
  //
  // Configure the timers used to generate the timing for the software
  // UART.  The interface in this example is run at 38,400 baud,
  // requiring a timer tick at 38,400 Hz.
  //
  MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
  MAP_TimerConfigure(TIMER0_BASE, (TIMER_CFG_SPLIT_PAIR | TIMER_CFG_A_PERIODIC |
                                   TIMER_CFG_B_PERIODIC));
  MAP_TimerLoadSet(TIMER0_BASE, TIMER_A, g_ulBitTime);
  MAP_TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT | TIMER_TIMB_TIMEOUT);
  MAP_TimerEnable(TIMER0_BASE, TIMER_A);
  //
  // Set the priorities of the interrupts associated with the software
  // UART.  The receiver is higher priority than the transmitter, and the
  // receiver edge interrupt is higher priority than the receiver timer
  // interrupt.
  //
  // MAP_IntPrioritySet(INT_GPIOE, 0x00);
  // MAP_IntPrioritySet(INT_TIMER0B, 0x40);
  // 0x80 corresponds to 4 << (8-5)
  // Remember that on the TM4C only 3 highest bits
  // are used for setting interrupt priority, and numerically lower
  // values are logically higher.
  // this is lower that configMAX_SYSCALL_INTERRUPT_PRIORITY but the ISR
  // does not call any FreeRTOS functions so this will work.
  MAP_IntPrioritySet(INT_TIMER0A, 0x80); // THIS NEEDS TO BE at a HIGH PRIORITY!!!! DONOT CHANGE
  //
  // Enable the interrupts associated with the software UART.
  //
  // MAP_IntEnable(INT_GPIOE);
  // MAP_IntEnable(INT_TIMER0A);
  // MAP_IntEnable(INT_TIMER0B);

  //
  // Enable the transmit FIFO half full interrupt in the software UART.
  //
  SoftUARTIntEnable(&g_sUART, SOFTUART_INT_TX);

  uint8_t message[4] = {0x9c, 0x2c, 0x2b, 0x3e};

  bool enable = true;

  // Loop forever
  for (;;) {
    uint32_t qmessage;
    // check for a new item in the queue but don't wait
    if (xQueueReceive(xSoftUartQueue, &qmessage, 0)) {
      switch (qmessage) {
      case SOFTUART_ENABLE_TRANSMIT:
        enable = true;
        break;
      case SOFTUART_DISABLE_TRANSMIT:
        enable = false;
        break;
#ifdef SUART_TEST_MODE
      case SOFTUART_TEST_SINGLE:
        inTestMode = true;
        enable = true;
        testmode = 0;
        break;
      case SOFTUART_TEST_INCREMENT:
        inTestMode = true;
        enable = true;
        testmode = 1;
        break;
      case SOFTUART_TEST_OFF:
        inTestMode = false;
        break;
      case SOFTUART_TEST_SEND_ONE:
        inTestMode = true;
        enable = true;
        testmode = 0;
        break;
      case SOFTUART_TEST_RAW:
        message[0] = 0x55;
        message[1] = 0xaa;
        message[2] = 0x55;
        message[3] = 0xaa;
        inTestMode = true;
        enable = true;
        testmode = 2;
        break;
#endif // SUART_TEST_MODE
      }
    }
    if (enable) {
      // Enable the interrupts during transmission
      MAP_IntEnable(INT_TIMER0A);

      if (inTestMode) {
#ifdef SUART_TEST_MODE
        // non-incrementing, single word
        if (testmode == 0) {
          format_data(testaddress, testdata, message);
          for (int i = 0; i < 4; ++i) {
            SoftUARTCharPut(&g_sUART, message[i]);
          }
          // one shot mode -- disable 
          if (qmessage == SOFTUART_TEST_SEND_ONE)
            enable = false;
        }
        // increment test mode, send
        else if (testmode == 1) {
          for (int jj = 0; jj < 10; ++jj) {
            testdata++;
            testaddress++;
            format_data(testaddress, testdata, message);
            for (int i = 0; i < 4; ++i) {
              SoftUARTCharPut(&g_sUART, message[i]);
            }
          }
        }
        else if ( testmode == 2 ) { // test mode, no formatting
          for (int i = 0; i < 4; ++i) {
            SoftUARTCharPut(&g_sUART, message[i]);
          }
        }
#endif // SUART_TEST_MODE
      }      // end test mode
      else { // normal mode 

        // Fireflies
        for (int j = 0; j < NFIREFLIES; ++j) {
          int16_t temperature = getFFvalue(j);
          format_data(j, temperature, message);
          // send data buffer
          for (int i = 0; i < 4; ++i) {
            SoftUARTCharPut(&g_sUART, message[i]);
          }
        }
        typedef union {
          uint16_t us;
          __fp16 f;
        } convert_16_t;
        // MonitorTask values -- power supply
        const int offsetMonitor = 32;
        const int num_uart_psmon_registers = 8; // ugh
        int offset = offsetMonitor;
        for (int j = 0; j < dcdc_args.n_devices; ++j) { // loop over supplies
          for (int l = 0; l < dcdc_args.n_pages;
               ++l) { // loop over register pages
            for (int k = 0; k < dcdc_args.n_commands;
                 ++k) { // loop over commands
              // HACK -- skip status command
              if (k == 5)
                continue;
              int index = j * (dcdc_args.n_commands * dcdc_args.n_pages) +
                          l * dcdc_args.n_commands + k;
              convert_16_t u;
              u.f = dcdc_args.pm_values[index];
              int reg = offset + k;
              format_data(reg, u.us, message);
              for (int i = 0; i < 4; ++i) {
                SoftUARTCharPut(&g_sUART, message[i]);
              }
            }
            offset += num_uart_psmon_registers;
          }
        }
        // ADC values
        const int offsetADC = 96;
        for (int j = 0; j < ADC_CHANNEL_COUNT; ++j) {
          convert_16_t u;
          u.f = getADCvalue(j);
          format_data(j + offsetADC, u.us, message);
          for (int i = 0; i < 4; ++i) {
            SoftUARTCharPut(&g_sUART, message[i]);
          }
        }
        // MonitorTask -- FPGA values
        // THIS WILL BREAK IF WE HAVE MORE THAN ONE PAGE OR IF WE HAVE
        // MORE THAN A SIMPLE SET OF COMMANDS -- NOTA BENE
        const int offsetFPGA = 128;
        for (int j = 0; j < fpga_args.n_commands * fpga_args.n_devices; ++j) {
          convert_16_t u;
          u.f = fpga_args.pm_values[j];
          format_data(j + offsetFPGA, u.us, message);
          for (int i = 0; i < 4; ++i) {
            SoftUARTCharPut(&g_sUART, message[i]);
          }
        }
        // git version
        const int offsetGIT = 118;
        char buff[SZ];
        memset(buff, 0, SZ); // technically not needed
        strncpy(buff, gitVersion(), SZ);
        uint16_t *p = (uint16_t *)buff;
        // each message sends two chars
        for (int j = 0; j < SZ / 2; j++) {
          format_data(j + offsetGIT, *p, message);
          ++p;
          for (int i = 0; i < 4; ++i) {
            SoftUARTCharPut(&g_sUART, message[i]);
          }
        }
        // uptime
        const int offsetUPTIME = 192;
        TickType_t now =
            xTaskGetTickCount() / (configTICK_RATE_HZ * 60); // time in minutes
        uint16_t now_16 = now & 0xFFFFU;                     // lower 16 bits
        format_data(offsetUPTIME, now_16, message);
        for (int i = 0; i < 4; ++i) {
          SoftUARTCharPut(&g_sUART, message[i]);
        }
        now_16 = (now >> 16) & 0xFFFFU; // upper 16 bits
        format_data(offsetUPTIME + 1, now_16, message);
        for (int i = 0; i < 4; ++i) {
          SoftUARTCharPut(&g_sUART, message[i]);
        }
      } // if not test mode 

      // wait for the transmission to finish
      vTaskDelay(pdMS_TO_TICKS(10));

      // disable the interrupt after the transmission has completed
      while (g_sUART.ui16TxBufferRead != g_sUART.ui16TxBufferWrite)
        vTaskDelay(pdMS_TO_TICKS(10));

      MAP_IntDisable(INT_TIMER0A);
    } // if ( enabled)
    // wait here for the x msec, where x is 2nd argument below.
    vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(5000));
  }

  return;
}
