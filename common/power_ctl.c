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


extern uint32_t g_ui32SysClock;

// data structures to hold GPIO PIN information
struct gpio_pin_t {
  int name;
  int priority;
};

struct gpio_pin_t enables[] = {
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
//  {  CTRL_K_MGTH_AVCC_PWR_EN,  4},  // this one is broken on S/N 001
  {  CTRL_K_MGTY_AVTT_PWR_EN,  5},
  {  CTRL_K_MGTH_AVTT_PWR_EN,  5},
  {  CTRL_V_MGTY1_AVTT_PWR_EN, 5},
  {  CTRL_V_MGTY2_AVTT_PWR_EN, 5}
};
const int nenables = sizeof(enables)/sizeof(enables[0]);

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
 // { K_MGTH_AVCC_OK,  4}, // this one is broken on S/N 001
  { K_MGTY_AVTT_OK,  5},
  { K_MGTH_AVTT_OK,  5},
  { V_MGTY1_AVTT_OK, 5},
  { V_MGTY2_AVTT_OK, 5}
};
const int noks = sizeof(oks)/sizeof(oks[0]);
const int num_priorities = 5;



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
	  for ( int e = 0; e < nenables; ++e ) {
		  if ( enables[e].priority == prio ) {
			  write_gpio_pin(enables[e].name, 0x1);
		  }
	  }

#ifdef USE_FREERTOS
    vTaskDelay(pdMS_TO_TICKS(100));
#else
	  //
	  // Delay for a bit
	  //
	  MAP_SysCtlDelay(g_ui32SysClock/6);
#endif // USER_FREERTOS
	  // check power good at this level or higher priority (lower number)
	  bool all_good = true;
	  int o = -1;
	  for ( o = 0; o < noks; ++o ) {
		  if ( oks[o].priority <= prio ) {
			  int8_t val = read_gpio_pin(oks[o].name);
			  if ( val == 0 ) {
				  all_good = false;
				  break;
			  }
		  }
	  } // loop over 'ok' bits
	  if (  ! all_good ) {
		  // o tells you which one died. should I print something on UART?
		  // turn off all supplies at current priority level or lower
		  // that is probably overkill since they should not all be
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

bool
check_ps(void)
{
  bool success = true;
  for ( int prio = 1; prio <= num_priorities; ++prio ) {
    // enable the supplies at the relevant priority
    bool all_good = true;
    int o = -1;
    for ( o = 0; o < noks; ++o ) {
      if ( oks[o].priority <= prio ) {
        int8_t val = read_gpio_pin(oks[o].name);
        if ( val == 0 ) {
          all_good = false;
          break;
        }
      }
    } // loop over 'ok' bits
    if (  ! all_good ) {
      // o tells you which one died. should I print something on UART?
      // turn off all supplies at current priority level or lower
      //UART4Print("Failure reading port ");
      //UART4Print(pin_names[enables[o].name]);
      for ( int e = 0; e < nenables; ++e ) {
        if ( enables[e].priority >= prio ) {
          write_gpio_pin(enables[e].name, 0x0);
        }
        success = false;

        break;
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
      for ( int o = 0; o < noks; ++o ) {
         if ( oks[o].priority >= prio ) {
           int8_t val = read_gpio_pin(oks[o].name);
           if ( val == 1 ) {
             all_ready = false;
           }
         }
       } // loop over 'ok' bits
      if ( all_ready) ready_to_proceed = true;
    }
  } // loop over priorities
  return success;
}

