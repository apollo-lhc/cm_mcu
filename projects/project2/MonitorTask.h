/*
 * MonitorTask.h
 *
 *  Created on: May 28, 2019
 *      Author: wittich
 */

#ifndef PROJECTS_PROJECT2_MONITORTASK_H_
#define PROJECTS_PROJECT2_MONITORTASK_H_


extern float pm_values[];
#define ABS(x) ((x)<0?(-(x)):(x))

// pilfered and adapted from http://billauer.co.il/blog/2018/01/c-pmbus-xilinx-fpga-kc705/
enum { PM_VOLTAGE, PM_NONVOLTAGE, PM_STATUS, PM_LINEAR11, PM_LINEAR16U, PM_LINEAR16S } pm_types ;

struct pm_list {
  unsigned char command;
  int size;
  char *name;
  char *units;
  int type;
};

#define NSUPPLIES_PS (5) // 5 devices, 2 pages each
#define NCOMMANDS_PS 7 // number of entries in above array
#define NPAGES_PS    2 // number of pages on the power supplies.

extern struct pm_list pm_command_dcdc[];


#endif /* PROJECTS_PROJECT2_MONITORTASK_H_ */
