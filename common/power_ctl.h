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

bool set_ps(bool KU15P, bool VU7PMGT1, bool VU7PMGT2);
bool check_ps(void);



#endif /* COMMON_POWER_CTL_H_ */
