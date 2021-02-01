/*
 * power_ctl.c
 *
 *  Created on: May 9, 2019
 *      Author: wittich
 */

#include <string.h>

#include "common/power_ctl.h"
#include "common/pinsel.h"
#include "common/utils.h"

#include "driverlib/rom_map.h"
#include "driverlib/sysctl.h"

#ifdef USE_FREERTOS
#include "FreeRTOS.h"
#include "task.h"
#endif // USE_FREERTOS

void ShortDelay(); // needs to be implemented in each project

// local sprintf prototype
int snprintf(char *buf, unsigned int count, const char *format, ...);

void Print(const char *); // needs to be implemented in each project

// clang-format off
// if you update this you need to update N_PS_ENABLES
// ------------------------------------------
//
// REV 1 
//
// ------------------------------------------
static const struct gpio_pin_t enables[] = {
    {  CTRL_K_VCCINT_PWR_EN, 1},
    {  CTRL_V_VCCINT_PWR_EN, 1},
    {  CTRL_VCC_1V8_PWR_EN,  2},
    {  CTRL_VCC_3V3_PWR_EN,  2},
    {  CTRL_V_MGTY1_VCCAUX_PWR_EN, 3},
    {  CTRL_V_MGTY2_VCCAUX_PWR_EN, 3},
    {  CTRL_K_MGTY_VCCAUX_PWR_EN,  3},
    {  CTRL_K_MGTH_VCCAUX_PWR_EN,  3},
    {  CTRL_V_MGTY1_AVCC_PWR_EN, 4},
    {  CTRL_V_MGTY2_AVCC_PWR_EN, 4},
    {  CTRL_K_MGTY_AVCC_PWR_EN,  4},
    {  CTRL_K_MGTH_AVCC_PWR_EN,  4},  
    {  CTRL_K_MGTY_AVTT_PWR_EN,  5},
    {  CTRL_K_MGTH_AVTT_PWR_EN,  5},
    {  CTRL_V_MGTY1_AVTT_PWR_EN, 5},
    {  CTRL_V_MGTY2_AVTT_PWR_EN, 5}
};

//if you update this you need to update N_PS_OKS too
// Notice that the VCCAUX is not included here; the
// TPS5218 supply does not have any such output
const
struct gpio_pin_t oks[] = {
    { K_VCCINT_PG_A, 1},
    { K_VCCINT_PG_B, 1},
    { V_VCCINT_PG_A, 1},
    { V_VCCINT_PG_B, 1},
    { VCC_1V8_PG,    2},
    { VCC_3V3_PG,    2},
    { V_MGTY1_AVCC_OK, 4},
    { V_MGTY2_AVCC_OK, 4},
    { K_MGTY_AVCC_OK,  4},
    { K_MGTH_AVCC_OK,  4},
    { K_MGTY_AVTT_OK,  5},
    { K_MGTH_AVTT_OK,  5},
    { V_MGTY1_AVTT_OK, 5},
    { V_MGTY2_AVTT_OK, 5}
};
// clang-format on
#define PS_NUM_PRIORITIES 5

// ------------------------------------------
//
// REV 2
//
// ------------------------------------------
// add here 

// these arrays hold the current and old status of these power supplies
static enum ps_state states[N_PS_OKS] = {PWR_UNKNOWN};
enum ps_state getPSStatus(int i)
{
  if (i < 0 || i >= N_PS_OKS)
    return PWR_UNKNOWN;
  return states[i];
}

void setPSStatus(int i, enum ps_state theState)
{
  if (i < 0 || i >= N_PS_OKS)
    return;
  states[i] = theState;
}

// turn off all power supplies in the proper order
// de-assert BLADE_POWER_OK on successful exit.
bool disable_ps(void)
{
  bool success = true;

  // first set the supplies to off to tell the
  // other tasks to prepare
  for (int o = 0; o < N_PS_OKS; ++o)
    if (states[o] != PWR_DISABLED)
      states[o] = PWR_OFF;
  // this long delay (probably too long) allows all I2C tasks
  // to finish their activity. For Rev2 of the CM (when the I2C
  // pullups are from management power) this delay can be reduced or
  // removed.
  vTaskDelay(pdMS_TO_TICKS(500));

  // disable in reverse order
  for (int prio = PS_NUM_PRIORITIES; prio > 0; --prio) {
    // disable the supplies at the relevant priority
    for (int e = 0; e < N_PS_ENABLES; ++e) {
      if (enables[e].priority == prio) {
        write_gpio_pin(enables[e].name, 0x0);
      }
    } // loop over enables
    bool ready_to_proceed = false;
    while (!ready_to_proceed) {
      bool all_ready = true;
      for (int o = 0; o < N_PS_OKS; ++o) {
        if (oks[o].priority >= prio) {
          int8_t val = read_gpio_pin(oks[o].name);
          if (val == 1) { // all supplies are supposed to be off now
            all_ready = false;
            states[o] = PWR_UNKNOWN;
          }
        }
      } // loop over 'ok' bits
      if (all_ready)
        ready_to_proceed = true;
    }
    // lowest_enabled_ps_prio = prio;
  } // loop over priorities

  // turn off POWER_OK when we are done
  if (success)
    write_gpio_pin(BLADE_POWER_OK, 0x0);
  else
    write_gpio_pin(BLADE_POWER_OK, 0x1);
  return success;
}

// check the power supplies and turn them on one by one
// Assert BLADE_POWER_OK if you are successful.
// Return immediately if BLADE_POWER_EN is not asserted by the SM.
// Which supplies to enable is based on the ps_en_mask (passed in),
// which references entries in the enables[] array.
bool turn_on_ps(uint16_t ps_en_mask)
{
  // if blade_power_en is false, return with failure
  bool blade_power_en = (read_gpio_pin(BLADE_POWER_EN) == 1);
  if (!blade_power_en) {
    write_gpio_pin(BLADE_POWER_OK, 0x0);
    return false;
  }

  // loop over the enables
  for (int prio = 1; prio <= PS_NUM_PRIORITIES; ++prio) {
    // enable the supplies at the relevant priority
    for (int e = 0; e < N_PS_ENABLES; ++e) {
      if (enables[e].priority == prio) {
        // check if this supply is to be enabled
        if (((1U << e) & ps_en_mask) == 0) // not in mask
          continue;
        write_gpio_pin(enables[e].name, 0x1);
      }
    }
  }

  write_gpio_pin(BLADE_POWER_OK, 0x1);
  return true;
}

// Enable supply at some priority. Also send in vu and ku enable.
void turn_on_ps_at_prio(bool f2_enable, bool f1_enable, int prio)
{
  // loop over the enables
  for (int e = 0; e < N_PS_ENABLES; ++e) {
    // if this enable matches the requested priority
    if (enables[e].priority == prio) {
      // check if this supply is to be enabled
      // current spot in mask
      uint16_t currmask = (1U << e);
      // should this supply be enabled?
      // three cases -- either it's a general supply (1.8 and 3.3V),
      // or it's a VU supply and the enable is set, or it's a KU supply
      // and the enable is set.
      bool enableSupply = (currmask & PS_ENS_GEN_MASK) ||               // general
                          ((currmask & PS_ENS_F2_MASK) && f2_enable) || // f2
                          ((currmask & PS_ENS_F1_MASK) && f1_enable);   // f1
      if (enableSupply) {
        write_gpio_pin(enables[e].name, 0x1);
      }
    }
  }
}

void blade_power_ok(bool isok)
{
  uint8_t val = (isok == true) ? 0x1U : 0x0U;
  write_gpio_pin(BLADE_POWER_OK, val);
}
