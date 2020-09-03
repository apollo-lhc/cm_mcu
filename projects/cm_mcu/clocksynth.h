/*
 * clocksynth.h
 *
 *  Created on: Jul 30, 2020
 *      Author: pw94, rzou
 */
#include "common/utils.h"

#ifndef PROJECTS_CM_MCU_CLOCKSYNTH_H_
#define PROJECTS_CM_MCU_CLOCKSYNTH_H_

void initialize_clock();
void write_register(int RegList[][2], int n_row);

int load_clock();


#endif /* PROJECTS_CM_MCU_CLOCKSYNTH_H_ */
