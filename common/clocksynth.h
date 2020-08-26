/*
 * clocksynth.h
 *
 *  Created on: Jul 30, 2020
 *      Author: pw94
 */
#include "common/utils.h"
#include "projects/cm_mcu/CommandLineTask.h"

#ifndef COMMON_CLOCKSYNTH_H_
#define COMMON_CLOCKSYNTH_H_

void initialize();
char** load_register();
void write_register(int RegList[][2], int n_row);

int load_clock();


#endif /* COMMON_CLOCKSYNTH_H_ */
