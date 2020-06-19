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
#define PS_ERROR   (5) // error generated by pwr_ctl
#define PS_STATUS  (6)
// alarms
#define TEMP_ALARM (7)
#define TEMP_ALARM_CLEAR (8)
#define CURRENT_ALARM (9)
#define CURRENT_ALARM_CLEAR (10)
#define PS_ANYFAIL_ALARM (11)
#define PS_ANYFAIL_ALARM_CLEAR (12)

// alarms

#define HUH             (99)

enum ps_state { PWR_UNKNOWN, PWR_ON, PWR_OFF, PWR_DISABLED, PWR_FAILED };
enum ps_state getPSStatus(int i);
void setPSStatus(int i, enum ps_state theState);
int getLowestEnabledPSPriority();

#define N_PS_ENABLES 16
#define N_PS_OKS 14

bool set_ps(void);
bool check_ps(void);
bool disable_ps(void);


#endif /* COMMON_POWER_CTL_H_ */
