/*
 * power_ctl.h
 *
 *  Created on: May 9, 2019
 *      Author: wittich
 */

#ifndef COMMON_POWER_CTL_H_
#define COMMON_POWER_CTL_H_

#include <stdint.h>
#include <stdbool.h>

#include "pinsel.h"

// X-macros for Alarm messages
#define X_MACRO_QUEUE_MESSAGES                                   \
  X(TEMP_ALARM, "Temperature Alarm")                             \
  X(TEMP_ALARM_CLEAR, "Temperature Alarm Clear")                 \
  X(CURRENT_ALARM, "Current Alarm")                              \
  X(CURRENT_ALARM_CLEAR, "Current Alarm Clear")                  \
  X(PS_ANYFAIL_ALARM, "Power Supply Any Fail Alarm")             \
  X(PS_ANYFAIL_ALARM_CLEAR, "Power Supply Any Fail Alarm Clear") \
  X(PS_GOOD, "Power Supply Good")                                \
  X(PS_BAD, "Power Supply Bad")                                  \
  X(PS_ON, "Power Supply On")                                    \
  X(PS_OFF, "Power Supply Off")                                  \
  X(PS_ERROR, "Power Supply Error")                              \
  X(PS_STATUS, "Power Supply Status")                            \
  X(HUH, "Undefined Message")

// Generate enum and text arrays using X-macros
enum MsgQueue_Message {
#define X(name, text) name,
  X_MACRO_QUEUE_MESSAGES
#undef X
};

extern const char *msgqueue_message_text[];

#define X_MACRO_PS_STATES \
  X(PWR_UNKNOWN)          \
  X(PWR_ON)               \
  X(PWR_OFF)              \
  X(PWR_DISABLED)         \
  X(PWR_FAILED)

// power supply state
enum ps_state {
#define X(name) name,
  X_MACRO_PS_STATES
#undef X
};

enum ps_state getPSStatus(int i);
void setPSStatus(int i, enum ps_state theState);
#define PS_NUM_PRIORITIES 6

#ifdef REV1
// -----------------------------------------------------
//
// Rev 1
//
// -----------------------------------------------------
// Number of enable and power good/OK pins
#define N_PS_ENABLES 16
#define N_PS_OKS     14
// Masks for the ENABLE bits and the OK/PG (power good)
// bits, for the pins defined in the enables[]
// and oks[] arrays.
#define PS_OKS_MASK     ((1U << N_PS_OKS) - 1)
#define PS_OKS_F1_MASK  0x0F03U
#define PS_OKS_F2_MASK  0x30CCU
#define PS_OKS_GEN_MASK 0x0030U
#define PS_ENS_MASK     ((1U << N_PS_ENABLES) - 1)
#define PS_ENS_GEN_MASK 0x000CU
#define PS_ENS_F2_MASK  0xC332U
#define PS_ENS_F1_MASK  0x3CC1U

// OK masks for various stages of the turn-on.
// these are indices into the oks[] array
// L1-L5, note NO L3!!! no PG on the L3 supplies
#define PS_OKS_F1_MASK_L1 0x0003U
#define PS_OKS_F1_MASK_L2 0x0030U // these two pins are common to VU and KU
#define PS_OKS_F1_MASK_L4 0x0300U
#define PS_OKS_F1_MASK_L5 0x0C00U
#define PS_OKS_F2_MASK_L1 0x000CU
#define PS_OKS_F2_MASK_L2 PS_OKS_F1_MASK_L2
#define PS_OKS_F2_MASK_L4 0x00C0U
#define PS_OKS_F2_MASK_L5 0x3000U

#elif defined(REV2) || defined(REV3) // Rev 2 or 3
// -----------------------------------------------------
//
// Rev 2 and Rev 3
//
// -----------------------------------------------------
// Number of enable and power good/OK pins

#define N_PS_ENABLES      10
#define N_PS_OKS          12
#define PS_OKS_MASK       ((1U << N_PS_OKS) - 1)
#define PS_OKS_F1_MASK    0x543U
#define PS_OKS_F2_MASK    0xA8CU
#define PS_OKS_GEN_MASK   0x030U
#define PS_ENS_MASK       ((1U << N_PS_ENABLES) - 1)
#define PS_ENS_GEN_MASK   0x00CU
#define PS_ENS_F1_MASK    0x151U
#define PS_ENS_F2_MASK    0x2A2U

// OK masks for various stages of the turn-on.
// these are indices into the oks[] array
// L1-L6
#define PS_OKS_F1_MASK_L1 0x003U
#define PS_OKS_F1_MASK_L2 0x030U // these two pins are common to F1 and F2
#define PS_OKS_F1_MASK_L3 0x040U
#define PS_OKS_F1_MASK_L4 0x100U
#define PS_OKS_F1_MASK_L5 0x400U
#define PS_OKS_F2_MASK_L1 0x00CU
#define PS_OKS_F2_MASK_L2 PS_OKS_F1_MASK_L2
#define PS_OKS_F2_MASK_L3 0x080U
#define PS_OKS_F2_MASK_L4 0x200U
#define PS_OKS_F2_MASK_L5 0x800U
#endif // REV 2

bool turn_on_ps(uint16_t);
bool check_ps(void);
bool disable_ps(void);
void turn_on_ps_at_prio(bool f2_enable, bool f1_enable, int prio);
void blade_power_ok(bool isok);

extern const struct gpio_pin_t oks[N_PS_OKS];

#endif /* COMMON_POWER_CTL_H_ */
