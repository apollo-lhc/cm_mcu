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

#include "MonI2C_addresses.h"
#include "MonUtils.h"
#include "MonitorTaskI2C.h"
#include "common/log.h"

#include "Semaphore.h"
#include "I2CCommunication.h"
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

void setFFmask(uint32_t ff_combined_present)
{

  log_info(LOG_SERVICE, "%s:FF EEPROM masks\r\n", __func__);

  // int32_t data = (~ff_combined_present) & 0xFFFFFU; // the bit value for an FF mask is an inverted bit value of the PRESENT signals
#ifdef REV1
  uint32_t data = (~ff_combined_present) & 0x1FFFFFFU;
#elif defined(REV2)
  uint32_t data = (ff_combined_present) & 0xFFFFFU;
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
  // outputs from *_PRESENT pins for constructing ff_PRESENT_mask
#ifdef REV1
  //      4.05 I2C KU15P OPTICS
  uint32_t present_FFL4_F1, present_FFL12_F1,
      //      4.06 I2C VU7P OPTICS (the I/O expanders at 0x20 and 0x21 have mixed 4-ch (ffl4) and 12-ch (FFL12) pins)
      present_0X20_F2, present_0X21_F2, present_FFL4_0X20_F2, present_FFL12_0X20_F2,
      present_FFL4_0X21_F2, present_FFL12_0X21_F2 = 0;
#elif defined(REV2)
  //      4.05 I2C FPGA31 OPTICS
  uint32_t present_FFL4_F1_bar, present_FFL12_F1_bar,
      //      4.06 I2C FPGA2 OPTICS
      present_FFL4_F2_bar, present_FFL12_F2_bar = 0;
#endif // REV2

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
  apollo_i2c_ctl_reg_r(4, 0x20, 1, 0x01, 1, &present_FFL12_F1_bar); // active low
  // to port 6
  apollo_i2c_ctl_w(4, 0x71, 1, 0x40);
  apollo_i2c_ctl_reg_r(4, 0x21, 1, 0x00, 1, &present_FFL4_F1_bar); // active low
  apollo_i2c_ctl_reg_r(4, 0x21, 1, 0x01, 1, &f1_ff12xmit_4v0_sel); // reading FPGA1 12-ch xmit FF's power-supply physical selection (i.e either 3.3v or 4.0v)
  f1_ff12xmit_4v0_sel = (f1_ff12xmit_4v0_sel >> 4) & 0x7;          // bits 4-6

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
  apollo_i2c_ctl_reg_r(3, 0x20, 1, 0x01, 1, &present_FFL12_F2_bar); // active low
  // to port 6
  apollo_i2c_ctl_w(3, 0x71, 1, 0x40);
  apollo_i2c_ctl_reg_r(3, 0x21, 1, 0x00, 1, &present_FFL4_F2_bar); // active low
  apollo_i2c_ctl_reg_r(3, 0x21, 1, 0x01, 1, &f2_ff12xmit_4v0_sel); // reading FPGA2 12-ch xmit FF's power-supply physical selection (i.e either 3.3v or 4.0v)
  f2_ff12xmit_4v0_sel = (f2_ff12xmit_4v0_sel >> 4) & 0x7;          // bits 4-6

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
  present_FFL12_F1_bar = present_FFL12_F1_bar & 0x3FU;     // bottom 6 bits
  present_FFL12_F2_bar = present_FFL12_F2_bar & 0x3FU;     // bottom 6 bits
  present_FFL4_F1_bar = (present_FFL4_F1_bar >> 4) & 0xFU; // bits 4-7
  present_FFL4_F2_bar = (present_FFL4_F2_bar >> 4) & 0xFU; // bits 4-7

  // active low
  uint32_t ff_combined_present_bar = ((present_FFL4_F2_bar) << 16) |  // 4 bits
                                     ((present_FFL12_F2_bar) << 10) | // 6 bits
                                     (present_FFL4_F1_bar) << 6 |     // 4 bits
                                     ((present_FFL12_F1_bar));        // 6 bits
  uint32_t ff_combined_present = ~ff_combined_present_bar;            // make active high
  // masks are active high
  ff_bitmask_args[1].present_bit_mask = (~present_FFL4_F1_bar) & 0xFU;   // 4 bits
  ff_bitmask_args[0].present_bit_mask = (~present_FFL12_F1_bar) & 0x3FU; // 6 bits
  ff_bitmask_args[3].present_bit_mask = (~present_FFL4_F2_bar) & 0xFU;   // 4 bits
  ff_bitmask_args[2].present_bit_mask = (~present_FFL12_F2_bar) & 0x3FU; // 6 bits

  // dump all the masks using log_info
  log_info(LOG_SERVICE, "F1 4v0 switch:    0x%x\r\n", f1_ff12xmit_4v0_sel);
  log_info(LOG_SERVICE, "F2 4v0 switch:    0x%x\r\n", f2_ff12xmit_4v0_sel);
  log_info(LOG_SERVICE, "F1 12-ch FF mask: 0x%02x\r\n", ff_bitmask_args[0].present_bit_mask);
  log_info(LOG_SERVICE, "F1  4-ch FF mask: 0x%02x\r\n", ff_bitmask_args[1].present_bit_mask);
  log_info(LOG_SERVICE, "F2 12-ch FF mask: 0x%02x\r\n", ff_bitmask_args[2].present_bit_mask);
  log_info(LOG_SERVICE, "F2  4-ch FF mask: 0x%02x\r\n", ff_bitmask_args[3].present_bit_mask);
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
  TickType_t last[2];
  last[0] = pdTICKS_TO_S(ff_f1_args.updateTick);
  last[1] = pdTICKS_TO_S(ff_f2_args.updateTick);

  unsigned mask = 0U;
  for (int ff_t = 0; ff_t < 2; ++ff_t) {
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
  else if (mask & 0x1U) {
    return ff_f1_args.updateTick;
  }
  else if (mask & 0x02U) {
    return ff_f2_args.updateTick;
  }
  log_warn(LOG_SERVICE, "invalid mask\r\n");
  return 0;
}

uint16_t getFFtemp(const uint8_t i)
{
  configASSERT(i < NFIREFLIES);
  return get_FF_TEMPERATURE_data(i);
}

#ifdef REV2
// returns optical power in uW
// not that the 4 channel and 12 channel data is encoded differently
// see the relevant data sheets and comments below
#define SWAP_BYTES(x) __builtin_bswap16(x)
float getFFavgoptpow(const uint8_t i)
{

  uint16_t sum_val = 0;
  configASSERT(i < NFIREFLIES);

  if (FireflyType(i) == DEVICE_25G4) {
    sum_val += get_FF_OPT_POWER_CH1_data(i);
    sum_val += get_FF_OPT_POWER_CH2_data(i);
    sum_val += get_FF_OPT_POWER_CH3_data(i);
    sum_val += get_FF_OPT_POWER_CH4_data(i);
    float sum_valf = sum_val / 10.f; // LSB is 0.1 uW
    float nchannels = 4.f;
    return sum_valf / nchannels;
  }
  else { // this is a 12 channel part; do calculation even if CERN-B
    sum_val += SWAP_BYTES(get_FF_OPT_POWER_CH1_data(i));
    sum_val += SWAP_BYTES(get_FF_OPT_POWER_CH2_data(i));
    sum_val += SWAP_BYTES(get_FF_OPT_POWER_CH3_data(i));
    sum_val += SWAP_BYTES(get_FF_OPT_POWER_CH4_data(i));
    sum_val += SWAP_BYTES(get_FF_OPT_POWER_CH5_data(i));
    sum_val += SWAP_BYTES(get_FF_OPT_POWER_CH6_data(i));
    sum_val += SWAP_BYTES(get_FF_OPT_POWER_CH7_data(i));
    sum_val += SWAP_BYTES(get_FF_OPT_POWER_CH8_data(i));
    sum_val += SWAP_BYTES(get_FF_OPT_POWER_CH9_data(i));
    sum_val += SWAP_BYTES(get_FF_OPT_POWER_CH10_data(i));
    sum_val += SWAP_BYTES(get_FF_OPT_POWER_CH11_data(i));
    sum_val += SWAP_BYTES(get_FF_OPT_POWER_CH12_data(i));
    float sumvalf = sum_val / 10.f; // LSB is 0.1 uW
    float nchannels = 12.f;
    return sumvalf / nchannels;
  }
}

// get optical power for a single channel
float getFFoptpow(const uint8_t i, const uint8_t ch)
{
  configASSERT(i < NFIREFLIES);
  configASSERT(ch < 12);
  float val;
  if (FireflyType(i) == DEVICE_25G4) {
    switch (ch) {
      case 0:
        val = get_FF_OPT_POWER_CH1_data(i);
        break;
      case 1:
        val = get_FF_OPT_POWER_CH2_data(i);
        break;
      case 2:
        val = get_FF_OPT_POWER_CH3_data(i);
        break;
      case 3:
        val = get_FF_OPT_POWER_CH4_data(i);
        break;
      default:
        val = -1;
        break;
    }
  }
  else {
    switch (ch) {
      case 0:
        val = SWAP_BYTES(get_FF_OPT_POWER_CH1_data(i));
        break;
      case 1:
        val = SWAP_BYTES(get_FF_OPT_POWER_CH2_data(i));
        break;
      case 2:
        val = SWAP_BYTES(get_FF_OPT_POWER_CH3_data(i));
        break;
      case 3:
        val = SWAP_BYTES(get_FF_OPT_POWER_CH4_data(i));
        break;
      case 4:
        val = SWAP_BYTES(get_FF_OPT_POWER_CH5_data(i));
        break;
      case 5:
        val = SWAP_BYTES(get_FF_OPT_POWER_CH6_data(i));
        break;
      case 6:
        val = SWAP_BYTES(get_FF_OPT_POWER_CH7_data(i));
        break;
      case 7:
        val = SWAP_BYTES(get_FF_OPT_POWER_CH8_data(i));
        break;
      case 8:
        val = SWAP_BYTES(get_FF_OPT_POWER_CH9_data(i));
        break;
      case 9:
        val = SWAP_BYTES(get_FF_OPT_POWER_CH10_data(i));
        break;
      case 10:
        val = SWAP_BYTES(get_FF_OPT_POWER_CH11_data(i));
        break;
      case 11:
        val = SWAP_BYTES(get_FF_OPT_POWER_CH12_data(i));
        break;
      default:
        val = 0;
        break;
    }
  }
  return val / 10.f; // LSB is 0.1 uW
}
#undef SWAP_BYTES

uint16_t getFFpresentbit(const uint8_t i)
{
  if (i > 3) {
    log_error(LOG_SERVICE, "caught %d > total fireflies %d\r\n", i, NFIREFLIES);
    return 56;
  }
  uint16_t val = ff_bitmask_args[i].present_bit_mask;

  return val;
}

// figure out which parts are 25G and which are not, for 12 channel parts
// sets ff_bitmask_args[0].ffpart_bit_mask and ff_bitmask_args[2].ffpart_bit_mask
uint32_t ff_map_25gb_parts(void)
{
  uint32_t ff_25gb_parts = 0U;
  uint32_t ff_25gb_pairs = 0U;
  for (int i = 0; i < NFIREFLIES; ++i) {
    if (!isEnabledFF(i)) { // skip the FF if it's not enabled via the FF config
      continue;
    }

    char name[17];
    memset(name, '\0', 17);
    int startReg = VENDOR_START_BIT_FFDAQ;
    int count = VENDOR_COUNT_FFDAQ;
    int type = FireflyType(i);
    if (type == DEVICE_CERNB || type == DEVICE_25G12) {
      startReg = VENDOR_START_BIT_FF12;
      count = VENDOR_COUNT_FF12;
    }
    int ret = 0;
    // build up name of the device (vendor string)
    for (int c = 0; c < count / 4; ++c) {
      uint8_t v[4];
      ret += read_arbitrary_ff_register(startReg + 4 * c, i, v, 4);
      name[4 * c] = v[0];
      name[4 * c + 1] = v[1];
      name[4 * c + 2] = v[2];
      name[4 * c + 3] = v[3];
    }
    if (ret != 0) {
      log_error(LOG_SERVICE, "Error reading vendor string for FF %d\r\n", i);
      // what to do? FIXME: return error?
    }
    log_info(LOG_SERVICE, "F%d FF%02d: %s\r\n", i / 10 + 1, i % 10, name);
    // skip 4 channel parts
    if (type == DEVICE_25G4) {
      continue;
    }
    if (strstr(name, "14") == NULL &&
        strstr(name, "CRRNB") == NULL && strstr(name, "CERNB") == NULL) {
      ff_25gb_parts |= (0x1U << i);
      int ipair = i / 2; // counts pairs of 12 channel devices
      ff_25gb_pairs |= (0x1U << ipair);
    }
  }
  log_info(LOG_SERVICE, "ff 25G12 mask: 0x%08lx\r\n", ff_25gb_parts);
  // these masks have one bit per pair of receiver/transceiver FIXME: do they?
  // ff_bitmask_args[0].ffpart_bit_mask = ff_25gb_pairs & 0x7U;
  // ff_bitmask_args[2].ffpart_bit_mask = (ff_25gb_pairs >> 5) & 0x7U;
  ff_bitmask_args[0].ffpart_bit_mask = ff_25gb_parts & 0x3fU; // six bits
  ff_bitmask_args[2].ffpart_bit_mask = (ff_25gb_parts >> 10) & 0x3fU;
  // dump the masks
  log_info(LOG_SERVICE, "F1 25G12 mask: 0x%02x\r\n", ff_bitmask_args[0].ffpart_bit_mask);
  log_info(LOG_SERVICE, "F2 25G12 mask: 0x%02x\r\n", ff_bitmask_args[2].ffpart_bit_mask);
  log_info(LOG_SERVICE, "Fx 25G12 pair mask: 0x%02x\r\n", ff_25gb_pairs);
  // pair mask into two parts
  uint32_t pair_mask_low = ff_25gb_pairs & 0x7U;         // 3 bits
  uint32_t pair_mask_high = (ff_25gb_pairs >> 5) & 0x7U; // 3 pairs of Tx/Rx + 2 XCVRs = 5 shifts
  log_info(LOG_SERVICE, "F1 25G pair mask: 0x%02x\r\n", pair_mask_low);
  log_info(LOG_SERVICE, "F2 25G pair mask: 0x%02x\r\n", pair_mask_high);
  // check if the 4v switch settings match
  // F1
  if (pair_mask_low != f1_ff12xmit_4v0_sel) {
    log_error(LOG_SERVICE, "4v switch and part mismatch F1: 0x%x != 0x%x\r\n",
              f1_ff12xmit_4v0_sel, pair_mask_low);
  }
  // F2
  if (pair_mask_high != f2_ff12xmit_4v0_sel) {
    log_error(LOG_SERVICE, "4v switch and part mismatch F2: 0x%x != 0x%x\r\n",
              f2_ff12xmit_4v0_sel, pair_mask_high);
  }
  // // F1
  // if (ff_bitmask_args[0].ffpart_bit_mask != f1_ff12xmit_4v0_sel) {
  //   log_error(LOG_SERVICE, "4v switch and part mismatch F1: 0x%x != 0x%x\r\n",
  //             f1_ff12xmit_4v0_sel, ff_bitmask_args[0].ffpart_bit_mask);
  // }
  // // F2
  // if (ff_bitmask_args[2].ffpart_bit_mask != f2_ff12xmit_4v0_sel) {
  //   log_error(LOG_SERVICE, "4v switch and part mismatch F2: 0x%x != 0x%x\r\n",
  //             f2_ff12xmit_4v0_sel, ff_bitmask_args[2].ffpart_bit_mask);
  // }
  return ff_25gb_pairs;
}
#endif
