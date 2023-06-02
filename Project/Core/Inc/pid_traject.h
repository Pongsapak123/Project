/*
 * pid_traject.h
 *
 *  Created on: Jun 1, 2023
 *      Author: naina
 */

#ifndef INC_PID_TRAJECT_H_
#define INC_PID_TRAJECT_H_
#include "stm32f4xx.h"
#include "math.h"
#include "arm_math.h"

void PID(float setposition);
void Trajectory_Gen(double x_init, double x_fi, double v_fi, double Accel);
void Trajectory_Eva();

#endif /* INC_PID_TRAJECT_H_ */
