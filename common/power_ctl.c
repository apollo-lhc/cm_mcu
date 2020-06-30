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
int snprintf( char *buf, unsigned int count, const char *format, ... );



void Print(const char* ); // needs to be implemented in each project



// if you update this you need to update N_PS_ENABLES
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
    {  CTRL_K_MGTH_AVCC_PWR_EN,  4},  // this one is broken on S/N 001
    {  CTRL_K_MGTY_AVTT_PWR_EN,  5},
    {  CTRL_K_MGTH_AVTT_PWR_EN,  5},
    {  CTRL_V_MGTY1_AVTT_PWR_EN, 5},
    {  CTRL_V_MGTY2_AVTT_PWR_EN, 5}
};

//if you update this you need to update N_PS_OKS too
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
const int num_priorities = 5;

// these arrays hold the current and old status of these power supplies
static enum ps_state new_states[N_PS_OKS] = { PWR_UNKNOWN };
static enum ps_state states[N_PS_OKS] = { PWR_UNKNOWN };

// this variable holds the current lowest enabled power supply
static int lowest_enabled_ps_prio = 0;

int getLowestEnabledPSPriority()
{
  return lowest_enabled_ps_prio;
}


enum ps_state getPSStatus(int i)
{
  if ( i < 0 || i >= N_PS_OKS) return PWR_UNKNOWN;
  return states[i];
}

void setPSStatus(int i, enum ps_state theState)
{
  if ( i < 0 || i >= N_PS_OKS) return;
  states[i] = theState;
}

bool update_failed_ps(int prio){

  bool ku_enable = (read_gpio_pin(TM4C_DIP_SW_1) == 1);
  bool vu_enable = (read_gpio_pin(TM4C_DIP_SW_2) == 1);
  bool failure = false;

  for ( int o = 0; o < N_PS_OKS; ++o ) {
    if ( oks[o].priority <= prio ) {
      int8_t val = read_gpio_pin(oks[o].name);
      if ( val == 0 ) {
        // if this is a VU7P supply and dip switch says ignore it, continue
        if (!vu_enable  && (strncmp(pin_names[oks[o].name], "V_", 2) == 0) ) {
          new_states[o] = PWR_DISABLED;
          continue;
        }
        // ditto for KU15P
        if ( !ku_enable && (strncmp(pin_names[oks[o].name], "K_", 2) == 0) ) {
          new_states[o] = PWR_DISABLED;
          continue;
        }
        // remember the VCC_ supplies
        if ((states[o]==PWR_ON)||(states[o]==PWR_UNKNOWN)){
          new_states[o]=PWR_FAILED;
          errbuffer_put(EBUF_PWR_FAILURE,o);
          failure=true;
        }
        else {
          new_states[o]=PWR_OFF;
        }
      }
      else {
        new_states[o] = PWR_ON;
      }
    }
  }
  memcpy(states, new_states, sizeof(states));
  return failure;
}

//
// check the power supplies and turn them on one by one
// Assert BLADE_POWER_OK if you are successful.
// Return immediately if BLADE_POWER_EN is not asserted by the SM.
bool set_ps()
{
  bool success = true; // return value
  // read two dip switches to see if we are powering either or both FPGAs
  bool ku_enable = (read_gpio_pin(TM4C_DIP_SW_1) == 1);
  bool vu_enable = (read_gpio_pin(TM4C_DIP_SW_2) == 1);

  // if blade_power_en is false, return with failure
  bool blade_power_en = (read_gpio_pin(BLADE_POWER_EN)==1);
  if ( ! blade_power_en ) {
    write_gpio_pin(BLADE_POWER_OK, 0x0);
    success = false;
    return success;
  }

  // loop over the enables
  for ( int prio = 1; prio <= num_priorities; ++prio ) {
    // enable the supplies at the relevant priority
    lowest_enabled_ps_prio = prio;
    for ( int e = 0; e < N_PS_ENABLES; ++e ) {
      if ( enables[e].priority == prio ) {
        // if the supply is for VU7P and dip switch says ignore it, continue
        if (!vu_enable && (strncmp( pin_names[enables[e].name], "CTRL_V_", 7) == 0) )
          continue;
        // ditto for KU15P
        if (!ku_enable && (strncmp( pin_names[enables[e].name], "CTRL_K_", 7) == 0) )
          continue;
        // remember that there are some VCC_ supplies too!
        write_gpio_pin(enables[e].name, 0x1);
      }
    }

    ShortDelay();

    // check power good at this level or higher priority (lower number)
    bool ps_failure = update_failed_ps(prio);

    // loop over 'ok' bits
    if (  ps_failure ) {

      Print("set_ps: Power supply check failed. ");
      Print("Turning off all supplies at this level or lower.\r\n");
      // turn off all supplies at current priority level or lower
      // that is probably overkill since they should not all be
      //lowest_enabled_ps_prio = oks[o].priority - 1;
      for ( int e = 0; e < N_PS_ENABLES; ++e ) {
        if ( enables[e].priority >= prio )
          write_gpio_pin(enables[e].name, 0x0);

      }
      for ( int o = 0; o < N_PS_OKS; ++o ) {
        if ( states[o] == PWR_DISABLED) continue;
        if (oks[o].priority >= prio ) {
          states[o] = PWR_OFF;
        }
      }
      // turn off the state variable too
      success = false;
      break;
    }
  } // loop over priorities

  if ( success )
    write_gpio_pin(BLADE_POWER_OK, 0x1);
  else
    write_gpio_pin(BLADE_POWER_OK, 0x0);
  return success;

}
// check_ps(Void)
// in this function we check the status of the supplies. The function returns
// true if all supplies it expects to be good, are good. That means that if one
// supply is disabled then it will not check it and return 'good' even if the
// supply is not good (in fact it will not be checked.)
//
// BLADE_POWER_OK will be asserted if this function returns successfully
bool
check_ps(void)
{

  bool success = true;

  bool ku_enable = (read_gpio_pin(TM4C_DIP_SW_1) == 1);
  bool vu_enable = (read_gpio_pin(TM4C_DIP_SW_2) == 1);

  // first check all the GPIO pins for various status bits
  for ( int o = 0; o < N_PS_OKS; ++o ) {
    // if this is a VU7P supply and dip switch says ignore it, continue
    if ( !vu_enable && (strncmp(pin_names[oks[o].name], "V_", 2) == 0) ) {
      new_states[o] = PWR_DISABLED;
      continue;
    }
    // ditto for KU15P
    else if ( !ku_enable && (strncmp(pin_names[oks[o].name], "K_", 2) == 0) ) {
      new_states[o] = PWR_DISABLED;
      continue;
    }
    else {
      int8_t val = read_gpio_pin(oks[o].name);
      if ( val == 0 ) {
        new_states[o] = PWR_OFF;
        success = false;
      }
      else {
        new_states[o] = PWR_ON;
      }
    }
  } // loop over N_PS_OKS

  // find out if any of the failures are new failures or not
  for ( int o = 0; o < N_PS_OKS; ++o ) {
    if ( (new_states[o] != states[o])  &&
        (states[o] != PWR_UNKNOWN) &&
        (states[o] != PWR_DISABLED)) {
      errbuffer_put(EBUF_PWR_FAILURE,o);
      char tmp[128];
      snprintf(tmp, 128, "check_ps: New failed supply %s (level %d)\r\n",
               pin_names[oks[o].name], oks[o].priority);
      Print(tmp);
    }
    states[o] = new_states[o];
  }

  // now find the lowest priority pin that is off
  int min_good_prio = 99;
  for ( int o = 0; o < N_PS_OKS; ++o ) {
    if ( states[o] == PWR_OFF ) {
      if ( oks[o].priority < min_good_prio)
        min_good_prio = oks[o].priority;
    }
  }
  if ( ! success ) {
    // turn off all supplies at current priority level or lower
    for ( int e = 0; e < N_PS_ENABLES; ++e ) {
      if ( enables[e].priority > min_good_prio ) {
        write_gpio_pin(enables[e].name, 0x0);
      }
    }
  } // loop over priorities
  if ( success )
    write_gpio_pin(BLADE_POWER_OK, 0x1);
  else
    write_gpio_pin(BLADE_POWER_OK, 0x0);

  return success;
}

// turn off all power supplies in the proper order
// de-assert BLADE_POWER_OK on successful exit.
bool
disable_ps(void)
{
  bool success = true;

  // first set the supplies to off to tell the
  // other tasks to prepare
  for ( int o = 0; o < N_PS_OKS; ++o )
    if ( states[o] != PWR_DISABLED )
      states[o] = PWR_OFF;
  // this long delay (probably too long) allows all I2C tasks
  // to finish their activity. For Rev2 of the CM (when the I2C
  // pullups are from management power) this delay can be reduced or
  // removed.
  vTaskDelay(pdMS_TO_TICKS(500));

  // disable in reverse order
  for (int prio = num_priorities; prio > 0;  --prio) {
    // disable the supplies at the relevant priority
    for ( int e = 0; e < N_PS_ENABLES; ++e ) {
      if ( enables[e].priority == prio ) {
        write_gpio_pin(enables[e].name, 0x0);
      }
    } // loop over enables
    bool ready_to_proceed = false;
    while ( ! ready_to_proceed ) {
      bool all_ready = true;
      for ( int o = 0; o < N_PS_OKS; ++o ) {
        if ( oks[o].priority >= prio ) {
          int8_t val = read_gpio_pin(oks[o].name);
          if ( val == 1 ) { // all supplies are supposed to be off now
            all_ready = false;
            states[o] = PWR_UNKNOWN;
          }
        }
      } // loop over 'ok' bits
      if ( all_ready) ready_to_proceed = true;
    }
    lowest_enabled_ps_prio = prio;
  } // loop over priorities

  // turn off POWER_OK when we are done
  if ( success )
    write_gpio_pin(BLADE_POWER_OK, 0x0);
  else
    write_gpio_pin(BLADE_POWER_OK, 0x1);
  return success;
}

