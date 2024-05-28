/**
 * @file FileflyUtils.c
 * @brief Utilities to access and control SamTec Firefly devices.
 *
 * This file contains utility functions and definitions for interacting with
 * SamTec Firefly devices.

 * @author Peter Wittich
 * @date September 20, 2022
 */
#include <stdbool.h>
#include <stdint.h>
#include <string.h> // memset

#include "common/log.h"

#include "Semaphore.h"
#include "I2CCommunication.h"
#include "MonitorI2CTask.h"
#include "common/smbus_helper.h"
#include "driverlib/eeprom.h"
#include "FireflyUtils.h"

uint32_t ff_PRESENT_mask = 0; // global variable from getting combined ff signals
uint32_t ff_USER_mask = 0;    // global variable of ff signals from user input
#ifdef REV2
uint32_t f1_ff12xmit_4v0_sel = 0; // global variable for FPGA1 12-ch xmit ff's power-supply physical selection
uint32_t f2_ff12xmit_4v0_sel = 0; // global variable for FPGA2 12-ch xmit ff's power-supply physical selection

struct ff_bit_mask_t ff_bitmask_args[4] = {
    {0U, 0U}, // {3, 6} bits correspond to ffl12_f1 devices
    {0U, 0U}, // {0, 4} and bits correspond to ffl4_f1 devices
    {0U, 0U}, // {3, 6} bits correspond to ffl12_f2 devices
    {0U, 0U}, // {0, 4} bits correspond to ffl4_f2 devices
};

#endif
// outputs from *_PRESENT pins for constructing ff_PRESENT_mask
#ifdef REV1
//      4.05 I2C KU15P OPTICS
uint32_t present_FFL4_F1, present_FFL12_F1,
    //      4.06 I2C VU7P OPTICS (the I/O expanders at 0x20 and 0x21 have mixed 4-ch (ffl4) and 12-ch (FFL12) pins)
    present_0X20_F2, present_0X21_F2, present_FFL4_0X20_F2, present_FFL12_0X20_F2,
    present_FFL4_0X21_F2, present_FFL12_0X21_F2 = 0;
#elif defined(REV2)
//      4.05 I2C FPGA31 OPTICS
uint32_t present_FFL4_F1, present_FFL12_F1,
    //      4.06 I2C FPGA2 OPTICS
    present_FFL4_F2, present_FFL12_F2 = 0;
#endif // REV2

void setFFmask(uint32_t ff_combined_present)
{

  log_info(LOG_SERVICE, "Setting a bit mask of enabled Fireflys to 1 \r\n");

  // int32_t data = (~ff_combined_present) & 0xFFFFFU; // the bit value for an FF mask is an inverted bit value of the PRESENT signals
#ifdef REV1
  uint32_t data = (~ff_combined_present) & 0x1FFFFFFU;
#elif defined(REV2)
  uint32_t data = (~ff_combined_present) & 0xFFFFFU;
#endif // REV1
  ff_USER_mask = read_eeprom_single(EEPROM_ID_FF_ADDR);
  ff_PRESENT_mask = data;
  uint64_t block = EEPROMBlockFromAddr(ADDR_FF);

  uint64_t unlock = EPRMMessage((uint64_t)EPRM_UNLOCK_BLOCK, block, PASS);
  xQueueSendToBack(xEPRMQueue_in, &unlock, portMAX_DELAY);

  uint64_t message = EPRMMessage((uint64_t)EPRM_WRITE_SINGLE, ADDR_FF, data);
  xQueueSendToBack(xEPRMQueue_in, &message, portMAX_DELAY);

  uint64_t lock = EPRMMessage((uint64_t)EPRM_LOCK_BLOCK, block << 32, 0);
  xQueueSendToBack(xEPRMQueue_in, &lock, portMAX_DELAY);

  return;
}

void readFFpresent(void)
{
  // grab the semaphore to ensure unique access to I2C controller
  // otherwise, block its operations indefinitely until it's available
  acquireI2CSemaphoreBlock(i2c4_sem);

#ifdef REV1
  // to port 7
  apollo_i2c_ctl_w(4, 0x70, 1, 0x80);
  apollo_i2c_ctl_reg_r(4, 0x20, 1, 0x01, 1, &present_FFL12_F1);
  // to port 6
  apollo_i2c_ctl_w(4, 0x71, 1, 0x40);
  apollo_i2c_ctl_reg_r(4, 0x21, 1, 0x00, 1, &present_FFL4_F1);
#elif defined(REV2)
  // to port 7
  apollo_i2c_ctl_w(4, 0x70, 1, 0x80);
  apollo_i2c_ctl_reg_r(4, 0x20, 1, 0x01, 1, &present_FFL12_F1);
  // to port 6
  apollo_i2c_ctl_w(4, 0x71, 1, 0x40);
  apollo_i2c_ctl_reg_r(4, 0x21, 1, 0x00, 1, &present_FFL4_F1);
  apollo_i2c_ctl_reg_r(4, 0x21, 1, 0x01, 1, &f1_ff12xmit_4v0_sel); // reading FPGA1 12-ch xmit FF's power-supply physical selection (i.e either 3.3v or 4.0v)
#endif

  // if we have a semaphore, give it
  if (xSemaphoreGetMutexHolder(i2c4_sem) == xTaskGetCurrentTaskHandle()) {
    xSemaphoreGive(i2c4_sem);
  }

  // grab the semaphore to ensure unique access to I2C controller
  // otherwise, block its operations indefinitely until it's available
  acquireI2CSemaphoreBlock(i2c3_sem);

#ifdef REV1
  // to port 0
  apollo_i2c_ctl_w(3, 0x72, 1, 0x01);
  apollo_i2c_ctl_reg_r(3, 0x20, 1, 0x01, 1, &present_0X20_F2);
  // to port 1
  apollo_i2c_ctl_w(3, 0x72, 1, 0x02);
  apollo_i2c_ctl_reg_r(3, 0x21, 1, 0x01, 1, &present_0X21_F2);
#elif defined(REV2)
  // to port 7
  apollo_i2c_ctl_w(3, 0x70, 1, 0x80);
  apollo_i2c_ctl_reg_r(3, 0x20, 1, 0x01, 1, &present_FFL12_F2);
  // to port 6
  apollo_i2c_ctl_w(3, 0x71, 1, 0x40);
  apollo_i2c_ctl_reg_r(3, 0x21, 1, 0x00, 1, &present_FFL4_F2);
  apollo_i2c_ctl_reg_r(3, 0x21, 1, 0x01, 1, &f2_ff12xmit_4v0_sel); // reading FPGA2 12-ch xmit FF's power-supply physical selection (i.e either 3.3v or 4.0v)

#endif
  // if we have a semaphore, give it
  if (xSemaphoreGetMutexHolder(i2c3_sem) == xTaskGetCurrentTaskHandle()) {
    xSemaphoreGive(i2c3_sem);
  }

#ifdef REV1
  uint32_t present_FFL12_BOTTOM_F1 = present_FFL12_F1 & 0x3FU;    // bottom 6 bits
  uint32_t present_FFL12_TOP_F1 = (present_FFL12_F1 >> 6) & 0x3U; // top 2 bits
  present_FFL4_F1 = (present_FFL4_F1 >> 5) & 0x7U;                // bits 5-7
  present_FFL12_0X20_F2 = (present_0X20_F2 >> 6) & 0x3U;          // bit 6-7
  present_FFL4_0X20_F2 = present_0X20_F2 & 0x3FU;                 // bottom 6 bits
  present_FFL12_0X21_F2 = (present_0X21_F2 >> 4) & 0x3U;          // bit 4-5
  present_FFL4_0X21_F2 = (present_0X21_F2 >> 2) & 0xFU;           // bit 4 bits

  uint32_t ff_combined_present = ((present_FFL12_0X21_F2) << 23) | // 2 bits
                                 ((present_FFL12_0X20_F2) << 21) | // 2 bits
                                 ((present_FFL4_0X21_F2) << 17) |  // 4 bits
                                 ((present_FFL4_0X20_F2) << 11) |  // 6 bits
                                 ((present_FFL12_TOP_F1) << 9) |   // 2 bits
                                 (present_FFL4_F1) << 6 |          // 3 bits
                                 ((present_FFL12_BOTTOM_F1));      // 6 bits

#elif defined(REV2)
  present_FFL12_F1 = present_FFL12_F1 & 0x3FU;     // bottom 6 bits
  present_FFL12_F2 = present_FFL12_F2 & 0x3FU;     // bottom 6 bits
  present_FFL4_F1 = (present_FFL4_F1 >> 4) & 0xFU; // bits 4-7
  present_FFL4_F2 = (present_FFL4_F2 >> 4) & 0xFU; // bits 4-7

  uint32_t ff_combined_present = ((present_FFL4_F2) << 16) |  // 4 bits
                                 ((present_FFL12_F2) << 10) | // 6 bits
                                 (present_FFL4_F1) << 6 |     // 4 bits
                                 ((present_FFL12_F1));        // 6 bits

  ff_bitmask_args[1].present_bit_mask = (~present_FFL4_F1) & 0xFU;   // 4 bits
  ff_bitmask_args[0].present_bit_mask = (~present_FFL12_F1) & 0x3FU; // 6 bits
  ff_bitmask_args[3].present_bit_mask = (~present_FFL4_F2) & 0xFU;   // 4 bits
  ff_bitmask_args[2].present_bit_mask = (~present_FFL12_F2) & 0x3FU; // 6 bits

  f1_ff12xmit_4v0_sel = (f1_ff12xmit_4v0_sel >> 4) & 0x7; // bits 4-6
  f2_ff12xmit_4v0_sel = (f2_ff12xmit_4v0_sel >> 4) & 0x7; // bits 4-6
#endif

  setFFmask(ff_combined_present);
}

bool isEnabledFF(int ff)
{
  // firefly config stored in on-board EEPROM via user input
  // and firefly config via PRESENT signals at the first boot
  // must be true for a firefly to be enabled.
  if (!((1 << ff) & ff_PRESENT_mask) || !((1 << ff) & ff_USER_mask)) {
    return false;
  }
  else {
    return true;
  }
}

unsigned isFFStale(void)
{
  TickType_t now = pdTICKS_TO_S(xTaskGetTickCount());
  TickType_t last[4];
  last[0] = pdTICKS_TO_S(ffl12_f1_args.updateTick);
  last[1] = pdTICKS_TO_S(ffl4_f1_args.updateTick);
  last[2] = pdTICKS_TO_S(ffl12_f2_args.updateTick);
  last[3] = pdTICKS_TO_S(ffl4_f2_args.updateTick);

  unsigned mask = 0U;
  for (int ff_t = 0; ff_t < 4; ++ff_t) {
    if (checkStale(last[ff_t], now)) {
      mask |= (1U << ff_t);
    }
  }

  return mask; // bits set for stale tasks. no bits set == not stale.
}

// this will return the tick of the _lowest_ set bit.
TickType_t getFFupdateTick(int mask)
{
  log_debug(LOG_SERVICE, "mask = %x\r\n", mask);
  if (__builtin_popcount(mask) == 0) {
    log_warn(LOG_SERVICE, "empty mask\r\n");
  }
  if (mask & 0x1U) {
    return ffl12_f1_args.updateTick;
  }
  else if (mask & 0x02U) {
    return ffl4_f1_args.updateTick;
  }
  else if (mask & 0x04U) {
    return ffl12_f2_args.updateTick;
  }
  else {
    return ffl4_f2_args.updateTick;
  }
}

uint16_t getFFtemp(const uint8_t i)
{
  int i1 = 1;
  uint16_t val;
  configASSERT(i < NFIREFLIES);
  if (i < NFIREFLIES_IT_F1) {
    int index = i * (ffl12_f1_args.n_commands * ffl12_f1_args.n_pages) + i1;
    val = ffl12_f1_args.sm_values[index];
  }

  else if (NFIREFLIES_IT_F1 <= i && i < NFIREFLIES_IT_F1 + NFIREFLIES_DAQ_F1) {
    int index = (i - NFIREFLIES_IT_F1) * (ffl4_f1_args.n_commands * ffl4_f1_args.n_pages) + i1;
    val = ffl4_f1_args.sm_values[index];
  }

  else if (NFIREFLIES_F1 <= i && i < NFIREFLIES_F1 + NFIREFLIES_IT_F2) {
    int index = (i - NFIREFLIES_F1) * (ffl12_f2_args.n_commands * ffl12_f2_args.n_pages) + i1;
    val = ffl12_f2_args.sm_values[index];
  }
  else {
    int index = (i - NFIREFLIES_F1 - NFIREFLIES_IT_F2) * (ffl4_f2_args.n_commands * ffl4_f2_args.n_pages) + i1;
    val = ffl4_f2_args.sm_values[index];
  }

  return val;
}

#ifdef REV2
uint16_t getFFavgoptpow(const uint8_t i)
{

  uint16_t avg_val = 0;
  uint16_t sum_val = 0;
  configASSERT(i < NFIREFLIES);

  for (int n = 0; n < 4; ++n) {
    if (ff_moni2c_arg[n].int_idx <= i && i < ff_moni2c_arg[n].int_idx + ff_moni2c_arg[n].num_dev) {
      for (int i1 = 4; i1 < ff_moni2c_arg[n].arg->n_commands; ++i1) {
        int dev = i - ff_moni2c_arg[n].int_idx + ff_moni2c_arg[n].dev_int_idx;
        int index = dev * (ff_moni2c_arg[n].arg->n_commands * ff_moni2c_arg[n].arg->n_pages) + i1;
        sum_val += ff_moni2c_arg[n].arg->sm_values[index];
      }
      avg_val = sum_val / (ff_moni2c_arg[n].arg->n_commands - 4);
    }
  }

  return avg_val;
}

uint16_t getFFpresentbit(const uint8_t i)
{
  if (i > 3) {
    log_warn(LOG_SERVICE, "caught %d > total fireflies %d\r\n", i, NFIREFLIES);
    return 56;
  }
  uint16_t val = ff_bitmask_args[i].present_bit_mask;

  return val;
}

extern struct sm_command_t sm_command_fflit_f1[];
extern struct sm_command_t sm_command_fflit_f2[];

void getFFpart(void)
{
  // Write device vendor part for identifying FF device
  uint8_t nstring = VENDOR_STOP_BIT_FF12 - VENDOR_START_BIT_FF12 + 1;
  char vendor_string[nstring];
  uint8_t data;

  SemaphoreHandle_t semaphores[2] = {i2c4_sem, i2c3_sem};
  const int ff_ndev_offset[2] = {0, NFIREFLIES_IT_F1 + NFIREFLIES_DAQ_F1};
  const uint32_t ndevices[2] = {NDEVICES_FFL12_F1 / 2, NDEVICES_FFL12_F2 / 2};
  const uint32_t dev_present_mask[2] = {present_FFL12_F1, present_FFL12_F2};
  const uint32_t dev_xmit_4v0_sel[2] = {f1_ff12xmit_4v0_sel, f2_ff12xmit_4v0_sel};

  struct MonitorI2CTaskArgs_t args_st[2] = {ffl12_f1_args, ffl12_f2_args};

  for (int f = 0; f < 2; ++f) {

    // grab the semaphore to ensure unique access to I2C controller
    // otherwise, block its operations indefinitely until it's available
    acquireI2CSemaphoreBlock(semaphores[f]);
    uint32_t tmp_ffpart_bit_mask = 0U;
    bool detect_ff = false;
    for (uint32_t n = 0; n < ndevices[f]; n++) {
      uint8_t vendor_data_rxch[4];
      int8_t vendor_part_rxch[17];

      data = 0x1U << args_st[f].devices[(2 * n) + 1].mux_bit;
      log_debug(LOG_SERVICE, "Mux set to 0x%02x\r\n", data);
      int rmux = apollo_i2c_ctl_w(args_st[f].i2c_dev, args_st[f].devices[(2 * n) + 1].mux_addr, 1, data);
      if (rmux != 0) {
        log_warn(LOG_SERVICE, "Mux write error %s\r\n", SMBUS_get_error(rmux));
      }
      for (uint8_t i = VENDOR_START_BIT_FF12; i < VENDOR_STOP_BIT_FF12; i++) {
        uint32_t vendor_char_rxch;
        int res = apollo_i2c_ctl_reg_r(args_st[f].i2c_dev, args_st[f].devices[(2 * n) + 1].dev_addr, 1, (uint16_t)i, 1, &vendor_char_rxch);
        if (res != 0) {
          log_warn(LOG_SERVICE, "GetFFpart read Error %s, break\r\n", SMBUS_get_error(res));
          vendor_part_rxch[i - VENDOR_START_BIT_FF12] = 0;
          break;
        }
        for (int j = 0; j < 4; ++j) {
          vendor_data_rxch[j] = (vendor_char_rxch >> (3 - j) * 8) & 0xFF;
        }
        convert_8_t tmp1;
        tmp1.us = vendor_data_rxch[3]; // change from uint_8 to int8_t, preserving bit pattern
        vendor_part_rxch[i - VENDOR_START_BIT_FF12] = tmp1.s;
        vendor_part_rxch[i - VENDOR_START_BIT_FF12 + 1] = '\0'; // null-terminated
      }

      char *vendor_string_rxch = (char *)vendor_part_rxch;

      if ((dev_present_mask[f] & (1 << (2 * n))) == 0) { // check that there is a FF installed in this ch
        if (!detect_ff) {
          detect_ff = true;
          if (strstr(vendor_string_rxch, "14") == NULL && strstr(vendor_string_rxch, "CRRNB") == NULL) { // the first 25Gbs 12-ch detected on FPGA1(2)
            tmp_ffpart_bit_mask = tmp_ffpart_bit_mask | (0x1U << n);                                     // bit 1 for a 25Gbs ch and assign to a Bit-mask of Firefly 12-ch part
          }
          else {
            if (f == 0)
              ffl12_f1_args.commands = sm_command_fflit_f1; // if the 14Gbsp 12-ch part is found, change the set of commands to sm_command_fflit_f1
            else
              ffl12_f2_args.commands = sm_command_fflit_f2; // if the 14Gbsp 12-ch part is found, change the set of commands to sm_command_fflit_f2
          }
          log_info(LOG_SERVICE, "Getting Firefly 12-ch part (FPGA%d): %s \r\n:", f + 1, vendor_string_rxch);
          strncpy(vendor_string, vendor_string_rxch, nstring);
        }
        else {
          if (strstr(vendor_string_rxch, "14") == NULL && strstr(vendor_string_rxch, "CRRNB") == NULL) {
            tmp_ffpart_bit_mask = tmp_ffpart_bit_mask | (0x1U << n); // bit 1 for a 25Gbs ch and assign to a Bit-mask of Firefly 12-ch part
          }
          else {
            if (strncmp(vendor_string_rxch, vendor_string, nstring) != 0) {
              log_info(LOG_SERVICE, "Different Firefly 12-ch part(FPGA%d) on %s \r\n:", f + 1, ff_moni2c_addrs[(2 * n) + 1 + ff_ndev_offset[f]].name);
              log_info(LOG_SERVICE, "with %s \r\n:", vendor_string_rxch);
            }
          }
        }
      }
      else {
        log_info(LOG_SERVICE, "No Firefly 12-ch part(FPGA%d) on %s \r\n:", f + 1, ff_moni2c_addrs[(2 * n) + 1 + ff_ndev_offset[f]].name);
      }
      memset(vendor_data_rxch, 0, sizeof(vendor_data_rxch));
      memset(vendor_part_rxch, 0, sizeof(vendor_part_rxch));
      rmux = apollo_i2c_ctl_w(args_st[f].i2c_dev, args_st[f].devices[(2 * n) + 1].mux_addr, 1, 0);
      if (rmux != 0) {
        log_warn(LOG_SERVICE, "Mux write error %s\r\n", SMBUS_get_error(rmux));
      }
      log_debug(LOG_SERVICE, "%s: reset mux\r\n", args_st[f].devices[(2 * n) + 1].name);
    }

    log_debug(LOG_SERVICE, "Bit-mask of Firefly 12-ch part (FPGA%d): 0x%02x \r\n:", f + 1, tmp_ffpart_bit_mask);
    log_debug(LOG_SERVICE, "Bit-mask of xmit_3v8_sel(FPGA%d): 0x%02x \r\n:", f + 1, dev_xmit_4v0_sel[f]);
    // Warning if 25Gbs found but is connected to 3.3V or Non-25Gbs found but is connected to 3.8V
    if ((dev_xmit_4v0_sel[f] ^ tmp_ffpart_bit_mask) != 0U) {
      log_warn(LOG_SERVICE, "FPGA%d 12-ch FFs have unmatched xmit_3v8_sel(0x%02x) and 12-ch ff-mask(0x%02x) \r\n", f + 1, dev_xmit_4v0_sel[f], tmp_ffpart_bit_mask);
    }

    if (f == 0)
      ff_bitmask_args[0].ffpart_bit_mask = tmp_ffpart_bit_mask;
    else
      ff_bitmask_args[2].ffpart_bit_mask = tmp_ffpart_bit_mask;

    // if we have a semaphore, give it
    if (xSemaphoreGetMutexHolder(semaphores[f]) == xTaskGetCurrentTaskHandle()) {
      xSemaphoreGive(semaphores[f]);
    }
  }
}
#endif
