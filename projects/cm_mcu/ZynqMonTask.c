/*
 * ZynqMonTask.c
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

#ifdef REV1
#include "common/softuart.h"
#elif defined(REV2)
#include "driverlib/uart.h"
#endif

#include "Tasks.h"
#include "MonitorTask.h"
#include "MonitorI2CTask.h"
#include "clocksynth.h"
#include "common/log.h"

#include "ZynqMon_addresses.h"

// Rev 2
// this needs to be split into a SoftUART version (Rev1) and a hard UART version (Rev2)

#define SZ 20

// Data Format
// https://github.com/apollo-lhc/SM_ZYNQ_FW/blob/develop/src/CM_interface/CM_Monitoring_data_format.txt
//
// Register List
// See Google Docs, 'CM uC Sensor register map'

#define SENSOR_MESSAGE_START_OF_FRAME_NIB 2
#define SENSOR_MESSAGE_DATA_FRAME_NIB     0
#define SENSOR_MESSAGE_HEADER_OFFSET      6
#define SENSOR_SIX_BITS                   0x3F
#define SENSOR_MESSAGE_START_OF_FRAME \
  (SENSOR_MESSAGE_START_OF_FRAME_NIB << SENSOR_MESSAGE_HEADER_OFFSET)
#define SENSOR_MESSAGE_DATA_FRAME (SENSOR_MESSAGE_DATA_FRAME_NIB << SENSOR_MESSAGE_HEADER_OFFSET)

static void format_data(const uint8_t sensor, const uint16_t data, uint8_t message[4])
{
  // header and start of sensor (6 bits, sensor[7:2]
  message[0] = SENSOR_MESSAGE_START_OF_FRAME | ((sensor >> 2) & SENSOR_SIX_BITS);
  // data frame 1, rest of sensor[1:0] (2 bits) and start of data[15:12] (4 bits)
  message[1] = SENSOR_MESSAGE_DATA_FRAME;
  message[1] |= ((sensor & 0x3) << 4) | ((data >> 12) & 0xF);
  // data frame 2, data[11:6] (6 bits)
  message[2] = SENSOR_MESSAGE_DATA_FRAME;
  message[2] |= (data >> 6) & 0x3F;
  // data frame 3, data[5:0] ( 6 bits )
  message[3] = SENSOR_MESSAGE_DATA_FRAME;
  message[3] |= data & 0x3F;
}

#ifdef ZYNQMON_TEST_MODE
static uint8_t testaddress = 0x0;
static uint16_t testdata = 0xaaff;
static bool inTestMode = true;
static uint8_t testmode = 2;
void setZYNQMonTestData(uint8_t sensor, uint16_t value)
{
  testaddress = sensor;
  testdata = value;
}

uint8_t getZYNQMonTestMode(void)
{
  return testmode;
}

uint8_t getZYNQMonTestSensor(void)
{
  return testaddress;
}

uint16_t getZYNQMonTestData(void)
{
  return testdata;
}
#else  //
const static bool inTestMode = false;
#endif // ZYNQMON_TEST_MODE
//
// The buffer used to hold the transmit data.
//
unsigned char g_pucTxBuffer[16];
//
// The buffer used to hold the receive data.
//
// unsigned short g_pusRxBuffer[16];

// The number of processor clocks in the time period of a single bit on the
// software UART interface.
//
unsigned long g_ulBitTime;

#define TARGET_BAUD_RATE 115200

QueueHandle_t xZynqMonQueue;

// category | count
// -------- | -----
// FPGA     | 4*N_FPGA
// GIT      | 20 chars
// ADC      | N_ADC+1 (21)
// PSMON    | 2*10*N_Devices (2 is for number of pages)
// uptime   | 1 uint32, transmitted as 2 uint16's

#ifdef REV1
extern uint32_t g_ui32SysClock;
tSoftUART g_sUART;

// For REV 1
void InitSUART(void)
{

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
                    (SOFTUART_CONFIG_WLEN_8 | SOFTUART_CONFIG_PAR_NONE | SOFTUART_CONFIG_STOP_ONE));
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
  MAP_TimerConfigure(TIMER0_BASE,
                     (TIMER_CFG_SPLIT_PAIR | TIMER_CFG_A_PERIODIC | TIMER_CFG_B_PERIODIC));
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
}

void ZMUartCharPut(unsigned char c)
{
  SoftUARTCharPut(&g_sUART, c);
}

#elif defined(REV2)
void ZMUartCharPut(unsigned char c)
{
  ROM_UARTCharPut(UART4_BASE, c);
}
#else
#error "Unknown board revision"
#endif

struct zynqmon_data_t zynqmon_data[ZM_NUM_ENTRIES];

// in all these functions, the "start" argument is used to set the
// address that is sent with the data to the Zynq and indicates
// a memory location in the Zynq memory.
// the order that the data is stored in on the zynqmon_data array
// does not have to correspond to the order in which the data is
// stored in, or sent to, the Zynq.

// this only needs to be called once
#define ZM_GIT_VERSION_LENGTH 20
void zm_set_gitversion(struct zynqmon_data_t data[], int start)
{
  // git version
  char buff[ZM_GIT_VERSION_LENGTH];
  // clear the buffer
  memset(buff, 0, ZM_GIT_VERSION_LENGTH); // technically not needed

  const char *v = gitVersion();
  // find v[0-9] in this string and assume this is where the actual git tag starts
  // version is a semver version string like v0.99.1
  while (*v != '\0') { // assumes this c string is well-terminated
    char n = *(v + 1); // char after 'v' should be a number 0-9
    if (*v == 'v' && (n >= '0' && n <= '9'))
      break; // success, I found the string
    ++v;
  }
  // on failure I send an empty string

  // get the git version and copy it into the buffer
  strncpy(buff, v, ZM_GIT_VERSION_LENGTH);
  // make sure our string is well-terminated
  buff[ZM_GIT_VERSION_LENGTH - 1] = '\0';
  // loop over the buffer and copy it into the data struct
  // each data word consists of two chars.
  for (int j = 0; j < ZM_GIT_VERSION_LENGTH; j += 2) {
    data[j / 2].sensor = start + (j / 2);
    data[j / 2].data.c[0] = buff[j];
    data[j / 2].data.c[1] = buff[j + 1];
  }
}

// store the uptime. This is a 32 bit unsigned int. Updates once per loop
void zm_set_uptime(struct zynqmon_data_t data[], int start)
{
  TickType_t now = xTaskGetTickCount() / (configTICK_RATE_HZ * 60); // time in minutes

  uint16_t now_16 = now & 0xFFFFU; // lower 16 bits
  data[0].sensor = start;
  data[0].data.us = now_16;
  now_16 = (now >> 16) & 0xFFFFU; // upper 16 bits
  data[1].sensor = start + 1;
  data[1].data.us = now_16;
}

// wasting half the data packet here; could repack it
// updated once per loop. Store the firefly temperature data
void zm_set_firefly_temps(struct zynqmon_data_t data[], int start)
{
  // Fireflies
  // update the data for ZMON
  for (uint8_t i = 0; i < NFIREFLIES; i++) {
    data[i].sensor = i + start; // sensor id
    if (!isFFStale()) {
      data[i].data.i = getFFtemp(i); // sensor value and type
    }
    else {
      data[i].data.i = -56; // special stale value
    }
    log_debug(LOG_SERVICE, " ff %d opt pow : 0x%02x\r\n", i, data[i].data.us);
  }
}

#ifdef REV2
// updated once per loop. Store the ffpart-bit (1 for 25GBs FFL, else 0) and present-bit data
void zm_set_firefly_bits(struct zynqmon_data_t data[], int start)
{

  int ll = 0;

  for (int j = 0; j < NFIREFLY_ARG; ++j) { // loop over ff structs

    uint16_t val;
    if (isFFStale()) {
      data[ll].data.us = 0xFF; // special stale value
    }
    else {
      if (j == 0)
        // 6 bits for ffpart-bits of FFL12 on FPGA1
        // ffpart_bit_mask = P3_RX | P2_RX | P1_RX
        // P3_RX | P3_TX | P2_RX | P2_TX | P1_RX | P1_TX
        // e.g. if ffpart_bit_mask  = 101, val = 110011
        val = ((ff_bitmask_args[0].ffpart_bit_mask & 0x1) << 1) | (ff_bitmask_args[0].ffpart_bit_mask & 0x1) | ((ff_bitmask_args[0].ffpart_bit_mask & 0x2) << 2) | ((ff_bitmask_args[0].ffpart_bit_mask & 0x2) << 1) | ((ff_bitmask_args[0].ffpart_bit_mask & 0x4) << 3) | ((ff_bitmask_args[0].ffpart_bit_mask & 0x4) << 2);
      else if (j == 2)
        // 6 bits for ffpart_bits of FFL12 on FPGA2
        // ffpart_bit_mask = P3_RX | P2_RX | P1_RX
        // P3_RX | P3_TX | P2_RX | P2_TX | P1_RX | P1_TX
        // e.g. if ffpart_bit_mask  = 101, val = 110011
        val = ((ff_bitmask_args[2].ffpart_bit_mask & 0x1) << 1) | (ff_bitmask_args[2].ffpart_bit_mask & 0x1) | ((ff_bitmask_args[2].ffpart_bit_mask & 0x2) << 2) | ((ff_bitmask_args[2].ffpart_bit_mask & 0x2) << 1) | ((ff_bitmask_args[2].ffpart_bit_mask & 0x4) << 3) | ((ff_bitmask_args[2].ffpart_bit_mask & 0x4) << 2);
      else
        val = 0xF; // default 0b1111 FFLDAG is always 25Gbs
      log_debug(LOG_SERVICE, "25GB bits? for ff argv %d: 0x%02x\r\n", j, val);
      data[ll].data.us = val; // present-bit
    }
    data[ll].sensor = ll + start;
    ++ll;
    if (isFFStale()) {
      data[ll].data.us = 0xFF; // special stale value
    }
    else {
      val = getFFpresentbit(j);
      log_debug(LOG_SERVICE, "present bits? for ff argv %d: 0x%02x\r\n", j, val);
      data[ll].data.us = val; // present-bit
    }
    data[ll].sensor = ll + start;
    ++ll;
  }
}

#define ZM_CLK_VERSION_LENGTH 20
void zm_set_clkconfigversion(struct zynqmon_data_t data[], int start, int n)
{
  char buff[ZM_CLK_VERSION_LENGTH];
  // clear the buffer
  memset(buff, 0, ZM_CLK_VERSION_LENGTH); // technically not needed
  const char *v = clkprog_args[n].progname_clkdesgid;
  // find v[0-9] in this string after R0A for instance.
  // version is a string like v0004
  while (*v != '\0') { // assumes this c string is well-terminated
    char n = *(v + 1); // char after 'v' should be a number 0-9
    if (*v == 'v' && (n >= '0' && n <= '9'))
      break; // success, I found the string
    ++v;
  }
  // get the clock config version and copy it into the buffer
  strncpy(buff, v, ZM_CLK_VERSION_LENGTH);
  // make sure our string is well-terminated
  buff[ZM_CLK_VERSION_LENGTH - 1] = '\0';
  // loop over the buffer and copy it into the data struct
  // each data word consists of two chars.
  for (int j = 0; j < ZM_CLK_VERSION_LENGTH; j += 2) {
    data[j / 2].sensor = start + (j / 2);
    data[j / 2].data.c[0] = buff[j];
    data[j / 2].data.c[1] = buff[j + 1];
  }
}

void zm_set_firefly_info(struct zynqmon_data_t data[], int start)
{
  // Fireflies
  // update the data for ZMON
  int ll = 0;

  for (int j = 0; j < NFIREFLIES; ++j) { // loop over ff structs

    if (isFFStale()) {
      data[ll].data.us = 0xff; // special stale value
    }
    else {
      data[ll].data.us = getFFtemp(j); // sensor value and type
      log_debug(LOG_SERVICE, "temp ? for ff %d: 0x%02x\r\n", j, getFFtemp(j));
    }
    data[ll].sensor = ll + start;
    ++ll;
    if (isFFStale()) {
      data[ll].data.us = 0xff; // special stale value
    }
    else {
      data[ll].data.us = getFFoptpow(j); // sensor value and type
      log_debug(LOG_SERVICE, "opt power ? for ff %d: 0x%02x\r\n", j, getFFoptpow(j));
    }
    data[ll].sensor = ll + start;
    ++ll;
  }
}

#endif // REV2

// store the zynqmon ADCMon data
void zm_set_adcmon(struct zynqmon_data_t data[], int start)
{
  // update the data for ZMON
  for (int i = 0; i < ADC_CHANNEL_COUNT; i++) {
    data[i].sensor = i + start;              // sensor id
    data[i].data.f = (__fp16)getADCvalue(i); // sensor value and type
  }
}

// store the PSMON data. In Rev1 legacy version the start argument is not used.
void zm_set_psmon_legacy(struct zynqmon_data_t data[], int start)
{
  // MonitorTask values -- power supply
  // this indirection list is required because of a mistake
  // made when initially putting together the register list.
  const size_t offsets[] = {32, 40, 48, 56, 160, 168, 64, 72, 80, 88};
  // update times, in seconds. If the data is stale, send NaN
  TickType_t last = pdTICKS_TO_S(dcdc_args.updateTick);
  TickType_t now = pdTICKS_TO_S(xTaskGetTickCount());

  bool stale = checkStale(last, now);

  for (int j = 0; j < 5; ++j) {                   // loop over supplies FIXME hardcoded value
    for (int l = 0; l < dcdc_args.n_pages; ++l) { // loop over register pages
      for (int k = 0; k < 5; ++k) {               // loop over FIRST FIVE commands
        int index =
            j * (dcdc_args.n_commands * dcdc_args.n_pages) + l * dcdc_args.n_commands + k;

        if (stale) {
          data[index].data.f = (__fp16)__builtin_nanf("");
        }
        else {
          data[index].data.f = (__fp16)dcdc_args.pm_values[index];
          if (data[index].data.f < -900.f)
            data[index].data.f = (__fp16)__builtin_nanf("");
        }
        uint8_t reg = offsets[j * 2 + l] + k;
        data[index].sensor = reg;
      }
    }
  }
}
void zm_set_psmon(struct zynqmon_data_t data[], int start)
{
  // MonitorTask values -- power supply
  // update times, in seconds. If the data is stale, send NaN
  TickType_t last = pdTICKS_TO_S(dcdc_args.updateTick);
  TickType_t now = pdTICKS_TO_S(xTaskGetTickCount());

  bool stale = checkStale(last, now);

  int ll = 0;

  for (int j = 0; j < dcdc_args.n_devices; ++j) {      // loop over supplies
    for (int l = 0; l < dcdc_args.n_pages; ++l) {      // loop over register pages
      for (int k = 0; k < dcdc_args.n_commands; ++k) { // loop over FIRST SIX commands
        if (k > 5) {
          break; // only consider first six commands
        }
        int index =
            j * (dcdc_args.n_commands * dcdc_args.n_pages) + l * dcdc_args.n_commands + k;

        if (stale) {
          data[ll].data.f = (__fp16)__builtin_nanf("");
        }
        else {
          data[ll].data.f = (__fp16)dcdc_args.pm_values[index];
          if (data[l].data.f < -900.f)
            data[ll].data.f = (__fp16)__builtin_nanf("");
        }
        data[ll].sensor = ll + start;
        ++ll;
      }
    }
  }
}

void zm_set_clock(struct zynqmon_data_t data[], int start, int n)
{
  // MonitorI2CTask values -- clock chips
  // update times, in seconds. If the data is stale, send NaN
  // n=1 is r0a and n=0 is else
  struct MonitorI2CTaskArgs_t args_st[2] = {clockr0a_args, clock_args};

  TickType_t last = pdTICKS_TO_S(args_st[n].updateTick);
  TickType_t now = pdTICKS_TO_S(xTaskGetTickCount());
  bool stale = checkStale(last, now);

  int ll = 0;

  for (int j = 0; j < args_st[n].n_devices; ++j) {      // loop over supplies
    for (int l = 0; l < args_st[n].n_pages; ++l) {      // loop over register pages
      for (int k = 0; k < args_st[n].n_commands; ++k) { // loop over clock commands FIXME : don't send sticky-bit ones
        int index =
            (j * args_st[n].n_commands * args_st[n].n_pages) + k;

        if (stale) {
          data[ll].data.us = 56; // special stale value
        }
        else {
          data[ll].data.us = args_st[n].sm_values[index];
        }
        data[ll].sensor = ll + start;
        ++ll;
      }
    }
  }
}

void zm_set_fpga(struct zynqmon_data_t data[], int start)
{
  // FPGA values
  TickType_t last = pdTICKS_TO_S(fpga_args.updateTick);
  TickType_t now = pdTICKS_TO_S(xTaskGetTickCount());

  bool stale = checkStale(last, now);

  for (int j = 0; j < fpga_args.n_commands * fpga_args.n_devices; ++j) {

    if (stale) {
      data[j].data.f = (__fp16)__builtin_nanf("");
    }
    else {
      data[j].data.f = (__fp16)fpga_args.pm_values[j];
      if (data[j].data.f < -900.f)
        data[j].data.f = (__fp16)__builtin_nanf("");
    }
    data[j].sensor = j + start;
  }
}

void zm_send_data(struct zynqmon_data_t data[])
{
  // https://docs.google.com/spreadsheets/d/1E-JD7sRUnkbXNqfgCUgriTZWfCXark6IN9ir_9b362M/edit#gid=0
  for (int i = 0; i < ZMON_VALID_ENTRIES; ++i) {
    uint8_t message[4];
    format_data(data[i].sensor, data[i].data.us, message);
    for (int j = 0; j < 4; ++j) {
      ZMUartCharPut(message[j]);
    }
  }
}

void ZynqMonTask(void *parameters)
{
  // Setup
#ifdef REV1
  InitSUART(); // Rev1
#endif
  // will be done centrally in Rev 2

#ifdef ZYNQMON_TEST_MODE
  uint8_t message[4] = {0x9c, 0x2c, 0x2b, 0x3e};
#endif // ZYNQMON_TEST_MODE

  // reset the data we will send
  memset(zynqmon_data, 0, ZM_NUM_ENTRIES * sizeof(struct zynqmon_data_t));

  bool enable = true;
  // initialize to the current tick time
  TickType_t xLastWakeTime = xTaskGetTickCount();

  // Loop forever
  for (;;) {
    uint32_t qmessage;
    // check for a new item in the queue but don't wait
    if (xQueueReceive(xZynqMonQueue, &qmessage, 0)) {
      switch (qmessage) {
        case ZYNQMON_ENABLE_TRANSMIT:
          enable = true;
          break;
        case ZYNQMON_DISABLE_TRANSMIT:
          enable = false;
          break;
#ifdef ZYNQMON_TEST_MODE
        case ZYNQMON_TEST_SINGLE:
          inTestMode = true;
          enable = true;
          testmode = 0;
          break;
        case ZYNQMON_TEST_INCREMENT:
          inTestMode = true;
          enable = true;
          testmode = 1;
          break;
        case ZYNQMON_TEST_OFF:
          inTestMode = false;
          break;
        case ZYNQMON_TEST_SEND_ONE:
          inTestMode = true;
          enable = true;
          testmode = 0;
          break;
        case ZYNQMON_TEST_RAW:
          message[0] = 0x55;
          message[1] = 0xaa;
          message[2] = 0x55;
          message[3] = 0xaa;
          inTestMode = true;
          enable = true;
          testmode = 2;
          break;
#endif // ZYNQMON_TEST_MODE
      }
    }
    if (enable) {
#ifdef REV1
      // Enable the interrupts during transmission
      MAP_IntEnable(INT_TIMER0A);
#endif // REV1

      if (inTestMode) {
#ifdef ZYNQMON_TEST_MODE
        // non-incrementing, single word
        if (testmode == 0) {
          format_data(testaddress, testdata, message);
          for (int i = 0; i < 4; ++i) {
            ZMUartCharPut(message[i]);
          }
          // one shot mode -- disable
          if (qmessage == ZYNQMON_TEST_SEND_ONE)
            enable = false;
        }
        // increment test mode, send
        else if (testmode == 1) {
          for (int jj = 0; jj < 10; ++jj) {
            testdata++;
            testaddress++;
            format_data(testaddress, testdata, message);
            for (int i = 0; i < 4; ++i) {
              ZMUartCharPut(message[i]);
            }
          }
        }
        else if (testmode == 2) { // test mode, no formatting
          for (int i = 0; i < 4; ++i) {
            ZMUartCharPut(message[i]);
          }
        }
#endif       // ZYNQMON_TEST_MODE
      }      // end test mode
      else { // normal mode
        zm_fill_structs();
        zm_send_data(zynqmon_data);
      } // if not test mode

#ifdef REV1
      // wait for the transmission to finish
      vTaskDelay(pdMS_TO_TICKS(10));

      // disable the interrupt after the transmission has completed
      while (g_sUART.ui16TxBufferRead != g_sUART.ui16TxBufferWrite)
        vTaskDelay(pdMS_TO_TICKS(10));

      MAP_IntDisable(INT_TIMER0A);
#endif // REV1
    }  // if ( enabled)

    // monitor stack usage for this task
    static UBaseType_t vv = 4096;
    CHECK_TASK_STACK_USAGE(vv);

    // wait here for the x msec, where x is 2nd argument below.
    vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(5000));
  }

  return;
}
