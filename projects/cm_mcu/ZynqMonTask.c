/*
 * ZynqMonTask.c
 *
 *  Created on: Jan 3, 2020
 *      Author: wittich
 *
 *  This task sends data to the Zynq for monitoring purposes.
 *  It should not read out any data, but only send data that is
 *  already available in the system.
 */
// includes for types
#include <assert.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

// to be moved
#include "FireflyUtils.h"
#include "MonI2C_addresses.h"
#include "MonUtils.h"
#include "inc/hw_memmap.h"
#include "driverlib/rom.h"

#ifdef REV1
#include "common/softuart.h"
#include "driverlib/timer.h"
#include "driverlib/gpio.h"
#include "inc/hw_ints.h"
#endif

#include "Tasks.h"
#include "MonitorTask.h"
#include "common/log.h"

#include "ZynqMon_addresses.h"
#include "ZynqMonTask.h"
#if defined(REV2) || defined(REV3)
#include "MCU_Reg.h" // not built for REV1
#include "I2CCommunication.h"
#include "InterruptHandlers.h"
#endif

// Rev 2
// this was split into a SoftUART version (Rev1) and a hard UART version (Rev2+)

#define SZ 20

// Data Format
// https://github.com/apollo-lhc/SM_ZYNQ_FW/blob/develop/src/CM_interface/CM_Monitoring_data_format.txt
//
// Register List
// See Google Docs, 'CM uC Sensor register map'

#define RESERVED_DATA                0x9 // 0b1001
#define SENSOR_MESSAGE_HEADER_OFFSET 6
#define SENSOR_SIX_BITS              0x3F
#define SENSOR_MESSAGE_START_OF_FRAME \
  (2 << SENSOR_MESSAGE_HEADER_OFFSET)
#define SENSOR_MESSAGE_START_OF_FRAME_V2 \
  (3 << SENSOR_MESSAGE_HEADER_OFFSET)
#define SENSOR_MESSAGE_DATA_FRAME (0 << SENSOR_MESSAGE_HEADER_OFFSET)

#define MESSAGE_SIZE 5

static void format_data(const uint16_t sensor, const uint16_t data, uint8_t message[MESSAGE_SIZE])
{
  // header for v2 (0b11) and start of sensor[9:4] (6 bits)
  message[0] = SENSOR_MESSAGE_START_OF_FRAME_V2 | ((sensor >> 4) & SENSOR_SIX_BITS);

  // rest of sensor[3:0] (4 bits) and start of reserved[3:2] (2 bits)
  message[1] = SENSOR_MESSAGE_DATA_FRAME;
  message[1] |= ((sensor & 0xf) << 2) | ((RESERVED_DATA >> 2) & 0x3);

  // rest of reserved[1:0] (2 bits) and start of data[15:12] (4 bits)
  message[2] = SENSOR_MESSAGE_DATA_FRAME;
  message[2] |= ((RESERVED_DATA & 0x3) << 4) | ((data >> 12) & 0xF);

  // data[11:6] (6 bits)
  message[3] = SENSOR_MESSAGE_DATA_FRAME;
  message[3] |= (data >> 6) & 0x3F;
  // data[5:0] ( 6 bits )
  message[4] = SENSOR_MESSAGE_DATA_FRAME;
  message[4] |= data & 0x3F;
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

// Whether ZynqMonTask is transmitting; read by MCU_Reg.c for the Runtime page.
static volatile bool zm_transmit_enabled = true;

bool getZynqMonTransmitEnabled(void)
{
  return zm_transmit_enabled;
}

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
  // ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
  // ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
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
  ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
  ROM_TimerConfigure(TIMER0_BASE,
                     (TIMER_CFG_SPLIT_PAIR | TIMER_CFG_A_PERIODIC | TIMER_CFG_B_PERIODIC));
  ROM_TimerLoadSet(TIMER0_BASE, TIMER_A, g_ulBitTime);
  ROM_TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT | TIMER_TIMB_TIMEOUT);
  ROM_TimerEnable(TIMER0_BASE, TIMER_A);
  //
  // Set the priorities of the interrupts associated with the software
  // UART.  The receiver is higher priority than the transmitter, and the
  // receiver edge interrupt is higher priority than the receiver timer
  // interrupt.
  //
  // ROM_IntPrioritySet(INT_GPIOE, 0x00);
  // ROM_IntPrioritySet(INT_TIMER0B, 0x40);
  // 0x80 corresponds to 4 << (8-5)
  // Remember that on the TM4C only 3 highest bits
  // are used for setting interrupt priority, and numerically lower
  // values are logically higher.
  // this is lower that configMAX_SYSCALL_INTERRUPT_PRIORITY but the ISR
  // does not call any FreeRTOS functions so this will work.
  ROM_IntPrioritySet(INT_TIMER0A, 0x80); // THIS NEEDS TO BE at a HIGH PRIORITY!!!! DONOT CHANGE
  //
  // Enable the interrupts associated with the software UART.
  //
  // ROM_IntEnable(INT_GPIOE);
  // ROM_IntEnable(INT_TIMER0A);
  // ROM_IntEnable(INT_TIMER0B);

  //
  // Enable the transmit FIFO half full interrupt in the software UART.
  //
  SoftUARTIntEnable(&g_sUART, SOFTUART_INT_TX);
}

void ZMUartCharPut(unsigned char c)
{
  SoftUARTCharPut(&g_sUART, c);
}

#elif defined(REV2) || defined(REV3)
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

  const char *v_temp = gitVersion();
  // find v[0-9] in this string and assume this is where the actual git tag starts
  // version is a semver version string like v0.99.1
  while (*v_temp != '\0') { // assumes this c string is well-terminated
    char n = *(v_temp + 1); // char after 'v' should be a number 0-9
    if (*v_temp == 'v' && (n >= '0' && n <= '9'))
      break; // success, I found the string
    ++v_temp;
  }
  // Search from the END of the string, not the start: on a tagged build
  // GIT_VERSION carries the tag twice ("... (tag: v0.99.4) v0.99.4") and the scan
  // above stops at the first copy, inside the parentheses. See issue #194.
  const char *v = strrchr(v_temp, 'v');
  if (v == NULL) // no tag reachable (shallow clone, no tags): send an empty string
    v = "";

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
    int16_t t = isFFStale() ? FF_TEMP_INVALID : getFFtemp(i);
    if (t != FF_TEMP_INVALID) {
      data[i].data.us = (uint16_t)t; // sensor value and type
    }
    else {
      // no usable reading: stale, read error, or a negative (out of range)
      // value from the device. All reported with the existing stale marker.
      data[i].data.i = -56; // special stale value
    }
    log_debug(LOG_SERVICE, " ff %d opt pow : 0x%02x\r\n", i, data[i].data.us);
  }
}

#if defined(REV2) || defined(REV3)
uint16_t getFFtXdisablebit(const uint8_t i)
{
  return get_FF_CHANNEL_DISABLE_data(i);
}
// updated once per loop.
// For each firefly device, send
// (1) ffpart-bit (1 for 25GBs FFL, else 0)
// (2) present-bit
// (3) tx-disable-bit (e.g. via ECU0_14G_TX_DISABLE_REG)
void zm_set_firefly_bits(struct zynqmon_data_t data[], int start)
{

  int ll = 0;

  for (int j = 0; j < NFIREFLY_ARG; ++j) { // loop over ff structs

    uint16_t val;
    if (isFFStale()) {
      data[ll].data.us = 0xFF; // special stale value
    }
    else {
      // val = ff_bitmask_args[j].ffpart_bit_mask;
      val = getFFpartbit(j);
      log_debug(LOG_SERVICE, "25GB bits? for ff argv %d: 0x%02x\r\n", j, val);
      log_debug(LOG_CLI, "25G? for ff argv %d: 0x%02x\r\n", j, val);
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
      log_debug(LOG_CLI, "present? for ff argv %d: 0x%02x\r\n", j, val);
      data[ll].data.us = val; // present-bit
    }
    data[ll].sensor = ll + start;
    ++ll;
  }
}

#define ZM_CLK_VERSION_LENGTH 6
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
  // if there is a version string, it should start with a 'v'
  if (*v == 'v') {
    // get the clock config version and copy it into the buffer
    strncpy(buff, v, ZM_CLK_VERSION_LENGTH);
    // make sure our string is well-terminated
    buff[ZM_CLK_VERSION_LENGTH - 1] = '\0';
    // loop over the buffer and copy it into the data struct
    // each data word consists of two chars.
  }
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

    int16_t t = isFFStale() ? FF_TEMP_INVALID : getFFtemp(j);
    if (t == FF_TEMP_INVALID) {
      // stale, read error, or a negative (out of range) value from the device
      data[ll].data.us = 0xff; // special stale value
    }
    else {
      data[ll].data.us = (uint16_t)t; // sensor value and type
      log_debug(LOG_SERVICE, "temp ? for ff %d: 0x%02x\r\n", j, t);
    }
    data[ll].sensor = ll + start;
    ++ll;

    if (isFFStale()) {
      data[ll].data.us = 0xff; // special stale value
    }
    else {
      data[ll].data.us = (uint16_t)getFFavgoptpow(j); // sensor value and type
      log_debug(LOG_SERVICE, "opt power ? for ff %d: 0x%02x\r\n", j, data[ll].data.us);
    }
    data[ll].sensor = ll + start;
    ++ll;

    if (isFFStale()) {
      data[ll].data.us = 0xff; // special stale value
    }
    else {
      data[ll].data.us = getFFtXdisablebit(j); // sensor value and type
      log_debug(LOG_SERVICE, "TX-disabled? for ff argv %d: 0x%02x\r\n", j, data[ll].data.us);
    }
    data[ll].sensor = ll + start;
    ++ll;
    // LOS_ALARM
    if (isFFStale()) {
      data[ll].data.us = 0xff; // special stale value
    }
    else {
      data[ll].data.us = get_FF_LOS_ALARM_data(j); // sensor value and type
      log_debug(LOG_SERVICE, "LOS_ALARM? for ff argv %d: 0x%02x\r\n", j, data[ll].data.us);
    }
    data[ll].sensor = ll + start;
    ++ll;
    // CDR_LOL_ALARM
    if (isFFStale()) {
      data[ll].data.us = 0xff; // special stale value
    }
    else {
      data[ll].data.us = get_FF_CDR_LOL_ALARM_data(j); // sensor value and type
      log_debug(LOG_SERVICE, "CDR_LOL_ALARM? for ff argv %d: 0x%02x\r\n", j, data[ll].data.us);
    }
    data[ll].sensor = ll + start;
    ++ll;
    // CDR_ENABLE
    if (isFFStale()) {
      data[ll].data.us = 0xff; // special stale value
    }
    else {
      data[ll].data.us = get_FF_CDR_ENABLE_data(j); // sensor value and type
      log_debug(LOG_SERVICE, "CDR_ENABLE? for ff argv %d: 0x%02x\r\n", j, data[ll].data.us);
    }
    data[ll].sensor = ll + start;
    ++ll;
    // POWER_ALARM_0
    if (isFFStale()) {
      data[ll].data.us = 0xff; // special stale value
    }
    else {
      data[ll].data.us = get_FF_POWER_ALARM_0_data(j); // sensor value and type
      log_debug(LOG_SERVICE, "POWER_ALARM_0 for ff %d: 0x%04x\r\n", j, data[ll].data.us);
    }
    data[ll].sensor = ll + start;
    ++ll;
    // POWER_ALARM_1
    if (isFFStale()) {
      data[ll].data.us = 0xff; // special stale value
    }
    else {
      data[ll].data.us = get_FF_POWER_ALARM_1_data(j); // sensor value and type
      log_debug(LOG_SERVICE, "POWER_ALARM_1 for ff %d: 0x%04x\r\n", j, data[ll].data.us);
    }
    data[ll].sensor = ll + start;
    ++ll;
  }
}

#define ZM_N_OPTICAL_FF12_NCHANNELS 12
#define ZM_N_OPTICAL_FF4_NCHANNELS  4
#ifdef REV2
#define ZM_N_OPTICAL_FF12_PER_FPGA 3
#define ZM_N_OPTICAL_FF4_PER_FPGA  4
#define ZM_OPTICAL_FF12_START      1
#define ZM_OPTICAL_FF4_START       6
#else // REV3 due to the enclosing ifdef
#define ZM_N_OPTICAL_FF12_PER_FPGA 4
#define ZM_N_OPTICAL_FF4_PER_FPGA  2
#define ZM_OPTICAL_FF12_START      1
#define ZM_OPTICAL_FF4_START       8
#endif
#define ZM_N_OPTICAL_FF12 (2 * ZM_N_OPTICAL_FF12_PER_FPGA)
#define ZM_N_OPTICAL_FF4  (2 * ZM_N_OPTICAL_FF4_PER_FPGA)
void zm_set_firefly_optpow12(struct zynqmon_data_t data[], int start)
{
  // received optical power for the firefly 12 channel part, 25G only
  int ll = 0;
  for (int j = 0; j < ZM_N_OPTICAL_FF12; ++j) { // loop over ff structs
    bool stale = isFFStale();
    // where in the range of NFIREFLIES_F1 to NFIREFLIES_F2 is this device?
    int whichTxFF = ZM_OPTICAL_FF12_START + (j % ZM_N_OPTICAL_FF12_PER_FPGA) * 2 + NFIREFLIES_F1 * ((int)j / ZM_N_OPTICAL_FF12_PER_FPGA);
    log_debug(LOG_SERVICE, "whichTxFF12 %d\r\n", whichTxFF);
    // loop over the channels
    for (int k = 0; k < ZM_N_OPTICAL_FF12_NCHANNELS; ++k) {
      if (stale) {
        data[ll].data.us = 0xff; // special stale value
      }
      else {
        data[ll].data.us = (uint16_t)getFFoptpow(whichTxFF, k); // sensor value and type
        log_debug(LOG_SERVICE, "opt power ? for ff %d: 0x%02x\r\n", j, data[ll].data.us);
      }
      data[ll].sensor = ll + start;
      ++ll;
    } // loop over channels
  }   // loop over Tx firefly devices
}

void zm_set_firefly_optpow4(struct zynqmon_data_t data[], int start)
{
  // received optical power for the firefly 4 channel part, 25G only
  int ll = 0;
  for (int j = 0; j < ZM_N_OPTICAL_FF4; ++j) { // loop over ff structs
    bool stale = isFFStale();
    // where in the range of NFIREFLIES_F1 to NFIREFLIES_F2 is this device?
    int whichTxFF = ZM_OPTICAL_FF4_START + (j % ZM_N_OPTICAL_FF4_PER_FPGA) + NFIREFLIES_F1 * ((int)j / ZM_N_OPTICAL_FF4_PER_FPGA);
    log_debug(LOG_SERVICE, "whichTxFF4  %d\r\n", whichTxFF);

    // loop over the channels
    for (int k = 0; k < ZM_N_OPTICAL_FF4_NCHANNELS; ++k) {
      if (stale) {
        data[ll].data.us = 0xff; // special stale value
      }
      else {
        data[ll].data.us = (uint16_t)getFFoptpow(whichTxFF, k); // sensor value and type
        log_debug(LOG_SERVICE, "opt power ? for ff %d: 0x%02x\r\n", j, data[ll].data.us);
      }
      data[ll].sensor = ll + start;
      ++ll;
    } // loop over channels
  }   // loop over Tx firefly devices
}

#endif // REV2 or 3

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

        if (dcdc_args.commands[k].type == PM_STATUS) {
          // bit field (STATUS_WORD): send the raw word, as fp16 rounds values above
          // 2048 and corrupts the low bits. 0xFFFF = no data (stale, power-off
          // sentinel, read error); the LGA80D never sets bits 8 and 10.
          float v = dcdc_args.pm_values[index];
          if (stale || !(v >= 0.f && v <= 65535.f))
            data[ll].data.us = 0xFFFFU;
          else
            data[ll].data.us = (uint16_t)v;
        }
        else if (stale) {
          data[ll].data.f = (__fp16)__builtin_nanf("");
        }
        else {
          data[ll].data.f = (__fp16)dcdc_args.pm_values[index];
          if (data[ll].data.f < -900.f) // power-off sentinel from MonitorTask
            data[ll].data.f = (__fp16)__builtin_nanf("");
        }
        data[ll].sensor = ll + start;
        ++ll;
      }
    }
  }
}

void zm_set_clock(struct zynqmon_data_t data[], int start)
{
  // MonitorI2CTask values -- clock chips
  // update times, in seconds. If the data is stale, send 56

  TickType_t last = pdTICKS_TO_S(clk_args.updateTick);
  TickType_t now = pdTICKS_TO_S(xTaskGetTickCount());
  bool stale = checkStale(last, now);

  int ll = 0;

  for (int j = 0; j < clk_args.n_devices; ++j) { // loop over supplies
    for (int k = 0; k < clk_args.n_commands; ++k) {
      if (stale) {
        data[ll].data.us = 56; // special stale value
      }
      else {
        data[ll].data.us = clk_args.commands[k].retrieveData(j);
      }
      data[ll].sensor = ll + start;
      ++ll;
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

TaskHandle_t zm_stack_tasks[ZM_STACK_NTASKS];

#if defined(REV2) || defined(REV3)
// Diagnostics block, see the diag_* entries in PL_MEM_CM_rev{2,3}.yml.
// Bump the version whenever the layout of the block changes.
#define ZM_DIAG_SCHEMA_VERSION 8

static_assert(MCU_HEALTH_ADC_ERROR <= 0xFFFFU, "health bits must fit in one 16-bit slot");

static void zm_set_u16s(struct zynqmon_data_t data[], int start, const uint16_t values[], int n)
{
  for (int i = 0; i < n; ++i) {
    data[i].sensor = start + i;
    data[i].data.us = values[i];
  }
}
#define ZM_SET_U16S(data, start, values) \
  zm_set_u16s(data, start, values, (int)(sizeof(values) / sizeof(values[0])))

static void zm_set_u32(struct zynqmon_data_t data[], int start, uint32_t value)
{
  // lower 16 bits first, as in zm_set_uptime()
  const uint16_t values[] = {(uint16_t)(value & 0xFFFFU), (uint16_t)(value >> 16)};
  ZM_SET_U16S(data, start, values);
}

void zm_set_diag_hdr(struct zynqmon_data_t data[], int start)
{
  const uint16_t values[] = {ZM_DIAG_SCHEMA_VERSION, (uint16_t)mcu_reg_get_health()};
  ZM_SET_U16S(data, start, values);
}

void zm_set_diag_reset_cause(struct zynqmon_data_t data[], int start)
{
  zm_set_u32(data, start, mcu_reg_get_reset_cause());
}

// Why the previous boot ended, from the error log: its newest fatal entry (assert, hard
// fault, stack overflow, malloc failure; 0 if none) and, for an assert, the PC.
void zm_set_diag_prev_assert_pc(struct zynqmon_data_t data[], int start)
{
  zm_set_u32(data, start, errbuffer_prev_assert_pc());
}

void zm_set_diag_prev_fatal(struct zynqmon_data_t data[], int start)
{
  const uint16_t values[] = {errbuffer_prev_fatal()};
  ZM_SET_U16S(data, start, values);
}

// The same fields as ProgCom page 0x01. The Zynq does not poll ProgCom, so
// this duplication is deliberate: ZynqMon is its only path to monitoring.
void zm_set_diag_pwr(struct zynqmon_data_t data[], int start)
{
  struct power_snapshot_t snap;
  getPowerSnapshotCopy(&snap);
  const uint16_t values[] = {
      (uint16_t)snap.generation,
      (uint16_t)(snap.fsm_state | (mcu_reg_power_flags(&snap) << 8)),
      snap.live_mask,
      snap.expected_mask,
      snap.software_ignore_mask,
      snap.failed_mask,
  };
  ZM_SET_U16S(data, start, values);
}

// Power failure counters and the state POWER_FAILURE was last entered from
void zm_set_diag_pwr_fail(struct zynqmon_data_t data[], int start)
{
  const struct power_diag_t *d = getPowerDiag();
  const uint16_t values[] = {d->fail_step, d->supply_fail, d->alarm_off, d->config_fail};
  ZM_SET_U16S(data, start, values);
}

// I2C master failure counters, buses 1-5 (the FPGA bus is I2C5 on REV2/3), each
// {START_FAIL, ERROR}, in the order of the diag_i2c names and postfixes
void zm_set_diag_i2c(struct zynqmon_data_t data[], int start)
{
  uint16_t values[10];
  for (uint8_t bus = 1; bus <= 5; ++bus) {
    values[2 * (bus - 1)] = i2c_get_start_fail(bus);
    values[2 * (bus - 1) + 1] = i2c_get_error(bus);
  }
  ZM_SET_U16S(data, start, values);
}

// I2C slave diagnostics, copied with the ISR masked so the snapshot is consistent
void zm_set_diag_i2cs(struct zynqmon_data_t data[], int start)
{
  struct i2cs_diag_t d;
  taskENTER_CRITICAL();
  d = i2cs_diag;
  taskEXIT_CRITICAL();
  // interrupted task as its index in zm_stack_tasks (the CM_DIAG_STACK order);
  // 0xFE = another task (e.g. IDLE), 0xFF = no duplicate yet
  uint16_t task = 0xFFU;
  if (d.snap_task != NULL) {
    task = 0xFEU;
    for (int i = 0; i < ZM_STACK_NTASKS; ++i) {
      if (zm_stack_tasks[i] == d.snap_task)
        task = (uint16_t)i;
    }
  }
  const uint16_t values[] = {d.dup_irq, d.timeouts, d.unhandled, d.last_unhandled,
                             d.snap_int, d.snap_scsr, d.snap_state, task};
  ZM_SET_U16S(data, start, values);
}

// LGA80D bad READ_TEMPERATURE_1/_3 reads, copied in one so a dump never mixes two events
void zm_set_diag_lga80d_badt(struct zynqmon_data_t data[], int start)
{
  struct lga80d_badt_t b[2];
  getLGA80DBadTempCopy(b);
  const uint16_t values[] = {b[0].n_ffff, b[0].n_other, b[0].raw, b[0].row,
                             b[1].n_ffff, b[1].n_other, b[1].raw, b[1].row};
  ZM_SET_U16S(data, start, values);
}

// Stack high-water mark in words. 0xFFFF if the task was never created:
// uxTaskGetStackHighWaterMark(NULL) would report ZMON's own stack instead.
void zm_set_diag_stack(struct zynqmon_data_t data[], int start)
{
  uint16_t values[ZM_STACK_NTASKS];
  for (int i = 0; i < ZM_STACK_NTASKS; ++i)
    values[i] = (zm_stack_tasks[i] != NULL)
                    ? (uint16_t)uxTaskGetStackHighWaterMark(zm_stack_tasks[i])
                    : 0xFFFFU;
  ZM_SET_U16S(data, start, values);
}
#endif // REV2 or REV3

void zm_send_data(struct zynqmon_data_t data[])
{
  // https://docs.google.com/spreadsheets/d/1E-JD7sRUnkbXNqfgCUgriTZWfCXark6IN9ir_9b362M/edit#gid=0
  for (int i = 0; i < ZMON_VALID_ENTRIES; ++i) {
    uint8_t message[MESSAGE_SIZE];
    format_data(data[i].sensor, data[i].data.us, message);
    for (int j = 0; j < MESSAGE_SIZE; ++j) {
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
  uint8_t message[MESSAGE_SIZE] = {0xf7, 0x0a, 0x1c, 0x2b, 0x3e};
#endif // ZYNQMON_TEST_MODE

  // reset the data we will send
  memset(zynqmon_data, 0, ZM_NUM_ENTRIES * sizeof(struct zynqmon_data_t));

  // initialize to the current tick time
  TickType_t xLastWakeTime = xTaskGetTickCount();

  // Loop forever
  for (;;) {
    uint32_t qmessage;
    // check for a new item in the queue but don't wait
    if (xQueueReceive(xZynqMonQueue, &qmessage, 0)) {
      switch (qmessage) {
        case ZYNQMON_ENABLE_TRANSMIT:
          zm_transmit_enabled = true;
          break;
        case ZYNQMON_DISABLE_TRANSMIT:
          zm_transmit_enabled = false;
          break;
#ifdef ZYNQMON_TEST_MODE
        case ZYNQMON_TEST_SINGLE:
          inTestMode = true;
          zm_transmit_enabled = true;
          testmode = 0;
          break;
        case ZYNQMON_TEST_INCREMENT:
          inTestMode = true;
          zm_transmit_enabled = true;
          testmode = 1;
          break;
        case ZYNQMON_TEST_OFF:
          inTestMode = false;
          break;
        case ZYNQMON_TEST_SEND_ONE:
          inTestMode = true;
          zm_transmit_enabled = true;
          testmode = 0;
          break;
        case ZYNQMON_TEST_RAW:
          message[0] = 0x55;
          message[1] = 0xaa;
          message[2] = 0x55;
          message[3] = 0xaa;
          message[4] = 0x55;
          inTestMode = true;
          zm_transmit_enabled = true;
          testmode = 2;
          break;
#endif // ZYNQMON_TEST_MODE
      }
    }
    if (zm_transmit_enabled) {
#ifdef REV1
      // Enable the interrupts during transmission
      ROM_IntEnable(INT_TIMER0A);
#endif // REV1

      if (inTestMode) {
#ifdef ZYNQMON_TEST_MODE
        // non-incrementing, single word
        if (testmode == 0) {
          format_data(testaddress, testdata, message);
          for (int i = 0; i < MESSAGE_SIZE; ++i) {
            ZMUartCharPut(message[i]);
          }
          // one shot mode -- disable
          if (qmessage == ZYNQMON_TEST_SEND_ONE)
            zm_transmit_enabled = false;
        }
        // increment test mode, send
        else if (testmode == 1) {
          for (int jj = 0; jj < 10; ++jj) {
            testdata++;
            testaddress++;
            format_data(testaddress, testdata, message);
            for (int i = 0; i < MESSAGE_SIZE; ++i) {
              ZMUartCharPut(message[i]);
            }
          }
        }
        else if (testmode == 2) { // test mode, no formatting
          for (int i = 0; i < MESSAGE_SIZE; ++i) {
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

      ROM_IntDisable(INT_TIMER0A);
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
