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

// these are all LED messages
#define PS_GOOD    (1)
#define PS_BAD     (2)
#define PS_ON      (3)
#define PS_OFF     (4)
// this should go elsewhere
#define RED_LED_OFF     (5)
#define RED_LED_ON      (6)
#define RED_LED_TOGGLE  (7)
#define RED_LED_TOGGLE3 (8)
#define RED_LED_TOGGLE4 (9)


bool set_ps(bool KU15P, bool VU7PMGT1, bool VU7PMGT2);
bool check_ps(void);
bool disable_ps(void);


#endif /* COMMON_POWER_CTL_H_ */
