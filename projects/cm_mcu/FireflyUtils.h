#ifndef FIREFLYUTILS_H
#define FIREFLYUTILS_H
#include <stdbool.h>
#include <stdint.h>
#include "Tasks.h"

// Function declarations
// the following are true for both Rev1 and Rev2
#define FF_I2CMUX_1_ADDR 0x70
#define FF_I2CMUX_2_ADDR 0x71
#define I2C_DEVICE_CLK   2

// REV1
#ifdef REV1
#define NFIREFLY_ARG      5
#define NFIREFLIES_F1     11
#define NFIREFLIES_F2     14
#define NFIREFLIES_IT_F1  8
#define NFIREFLIES_DAQ_F1 3
#define NFIREFLIES_IT_F2  4
#define NFIREFLIES_DAQ_F2 10
#elif defined(REV2) // REV2
// REV 2
#define NFIREFLY_ARG     4
#define NFIREFLIES_F1    10
#define NFIREFLIES_F2    10
#define NFIREFLIES_IT_F1 6
#define NFIREFLIES_IT_F2 6
#elif defined(REV3)
#define NFIREFLY_ARG     4
#define NFIREFLIES_F1    10
#define NFIREFLIES_F2    10
#define NFIREFLIES_IT_F1 8
#define NFIREFLIES_IT_F2 8
#else
#error "Define Revision!"
#endif // REV
#define CLK_PAGE_COMMAND 1
#define NFIREFLIES       (NFIREFLIES_F1 + NFIREFLIES_F2)

// register address of the first and last entry of the
// device identifier in the memory map of the FF devices
#define FF_VENDOR_START_BIT_FFDAQ 168
#define FF_VENDOR_STOP_BIT_FFDAQ  184
#define FF_VENDOR_START_BIT_FF12  171
#define FF_VENDOR_STOP_BIT_FF12   187
#define FF_VENDOR_COUNT_FFDAQ     (FF_VENDOR_STOP_BIT_FFDAQ - FF_VENDOR_START_BIT_FFDAQ)
#define FF_VENDOR_COUNT_FF12      (FF_VENDOR_STOP_BIT_FF12 - FF_VENDOR_START_BIT_FF12)

struct arg_moni2c_ff_t {
  char *ff_part;                    // ff part
  struct MonitorTaskI2CArgs_t *arg; // ff arg
  uint8_t int_idx;                  // start idx of this arg in ff_moni2c_addrs
  uint8_t dev_int_idx;              // start idx of the device in its arg
  uint8_t num_dev;                  // number of devices in this ff arg.
};

extern struct dev_moni2c_addr_t ff_moni2c_addrs[NFIREFLIES];
extern struct dev_moni2c_addr_t ff_moni2c_addrs_f1[NFIREFLIES_F1];
extern struct dev_moni2c_addr_t ff_moni2c_addrs_f2[NFIREFLIES_F2];
extern struct arg_moni2c_ff_t ff_moni2c_arg[NFIREFLY_ARG];

// Samtec firefly specific commands
bool getFFch_low(uint8_t val, int channel);
bool getFFch_high(uint8_t val, int channel);
bool isEnabledFF(int ff);
void readFFpresent(void);
uint16_t getFFtemp(const uint8_t i);
float getFFavgoptpow(const uint8_t i);
float getFFoptpow(const uint8_t i, const uint8_t ch);
uint16_t getFFpresentbit(const uint8_t i);
uint8_t getFFpartbit(const uint8_t i);
#if defined(REV2) || defined(REV3)
void getFFpart(void);
uint32_t ff_map_25gb_parts(void);
uint16_t getFF12Ch25GTxMask(int device);
#endif

uint8_t getFFstatus(const uint8_t i);
unsigned isFFStale(void);
TickType_t getFFupdateTick(int ff_t);
void init_registers_ff(void);

uint16_t read_arbitrary_ff_register(uint16_t regnumber, int num_ff, uint8_t *value, uint8_t size);

extern uint32_t ff_PRESENT_mask;
extern uint32_t ff_USER_mask;
#if defined(REV2) || defined(REV3)
extern uint32_t f1_ff12xmit_4v0_sel;
extern uint32_t f2_ff12xmit_4v0_sel;
struct ff_bit_mask_t {
  uint8_t ffpart_bit_mask;   // this mask is only used for detecting 12-ch 25Gbps on the REV2 board
  uint32_t present_bit_mask; // this mask is used for all ffs to detect if it is mounted or not
};
#endif

#ifdef REV1
extern uint32_t present_0X20_F2, present_0X21_F2, present_FFLDAQ_F1, present_FFL12_F1, present_FFLDAQ_0X20_F2, present_FFL12_0X20_F2, present_FFLDAQ_0X21_F2, present_FFL12_0X21_F2;
#elif defined(REV2) || defined(REV3)
extern uint32_t present_FFLDAQ_F1, present_FFL12_F1_bar, present_FFLDAQ_F2, present_FFL12_F2_bar;
#endif // REV2

#define FF_PAGE_SELECT_BYTE 0x7FU // page select byte

#endif // FIREFLYUTILS_H
