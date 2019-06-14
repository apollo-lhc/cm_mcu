/*
 * power_ctl.c
 *
 *  Created on: May 9, 2019
 *      Author: wittich
 */


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
static struct gpio_pin_t enables[] = {
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
static const int nenables = sizeof(enables)/sizeof(enables[0]);

//if you update this you need to update N_PS_OKS too
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
  { K_MGTH_AVCC_OK,  4}, // this one is broken on S/N 001
  { K_MGTY_AVTT_OK,  5},
  { K_MGTH_AVTT_OK,  5},
  { V_MGTY1_AVTT_OK, 5},
  { V_MGTY2_AVTT_OK, 5}
};
const int num_priorities = 5;

// this array states[] holds the current status of these power supplies
static enum ps_state states[N_PS_OKS] = { UNKNOWN };

// this variable holds the current lowest enabled power supply
static int lowest_enabled_ps_prio = 0;

int getLowestEnabledPSPriority()
{
  return lowest_enabled_ps_prio;
}

enum ps_state getPSStatus(int i)
{
  if ( i < 0 || i >= N_PS_OKS) return UNKNOWN;
  return states[i];
}
void setPSStatus(int i, enum ps_state theState)
{
  if ( i < 0 || i >= N_PS_OKS) return;
  states[i] = theState;
}

//
// check the power supplies and turn them on one by one
//
bool set_ps(bool KU15P, bool VU7PMGT1, bool VU7PMGT2)
{
  bool success = true; // return value

  // data structure to turn on various power supplies. This should be ordered
  // such that the priority increases, though it's not necessary
  for ( int prio = 1; prio <= num_priorities; ++prio ) {
    // enable the supplies at the relevant priority
    lowest_enabled_ps_prio = prio;
    for ( int e = 0; e < nenables; ++e ) {
      if ( enables[e].priority == prio ) {
        write_gpio_pin(enables[e].name, 0x1);
      }
    }

    ShortDelay();

    // check power good at this level or higher priority (lower number)
    bool all_good = true;
    int o = -1;
    for ( o = 0; o < N_PS_OKS; ++o ) {
      if ( oks[o].priority <= prio ) {
        int8_t val = read_gpio_pin(oks[o].name);
        if ( val == 0 ) {
          all_good = false;
          break;
        }
      }
    } // loop over 'ok' bits
    if (  ! all_good ) {
      // o tells you which one died.
      Print("set_ps: Power supply check failed: ");
      Print(pin_names[oks[o].name]);
      Print(". Turning off all supplies at this level or lower.\n");
      // turn off all supplies at current priority level or lower
      // that is probably overkill since they should not all be
      lowest_enabled_ps_prio = oks[o].priority - 1;
      for ( int e = 0; e < nenables; ++e ) {
        if ( enables[e].priority >= prio )
          write_gpio_pin(enables[e].name, 0x0);

      }
      success = false;
      break;
    }
  } // loop over priorities

  return success;

}
// check_ps(Void)
// in this function we check the status of the supplies. The function returns
// true if all supplies it expects to be good, are good. That means that if one
// supply is disabled then it will not check it and return 'good' even if the
// supply is not good (in fact it will not be checked.)

bool
check_ps(void)
{

  bool success = true;
  enum ps_state new_states[N_PS_OKS];
  // first check all the GPIO pins for various status bits
  for ( int o = 0; o < N_PS_OKS; ++o ) {
    int8_t val = read_gpio_pin(oks[o].name);
    if ( val == 0 ) {
      new_states[o] = PWR_OFF;
      success = false;
    }
    else {
      new_states[o] = PWR_ON;
    }
  }
  // find out if any of the failures are new failures or not
  for ( int o = 0; o < N_PS_OKS; ++o ) {
    if ( new_states[o] != states[o]  && states[o] != UNKNOWN) {
      static char tmp[128];
      snprintf(tmp, 128, "check_ps: New failed supply %s (level %d)\n", pin_names[oks[o].name],
               oks[o].priority);
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
    for ( int e = 0; e < nenables; ++e ) {
      if ( enables[e].priority > min_good_prio ) {
        write_gpio_pin(enables[e].name, 0x0);
      }
    }
  } // loop over priorities

  return success;
}

bool
disable_ps(void)
{
  bool success = true;
  // disable in reverse order
  for (int prio = num_priorities; prio > 0;  --prio) {
    // disable the supplies at the relevant priority
    for ( int e = 0; e < nenables; ++e ) {
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
           if ( val == 1 ) {
             all_ready = false;
           }
         }
       } // loop over 'ok' bits
      if ( all_ready) ready_to_proceed = true;
    }
    lowest_enabled_ps_prio = prio;
  } // loop over priorities
  return success;
}

