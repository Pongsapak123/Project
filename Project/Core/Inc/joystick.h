/*
 * joystick.h
 *
 *  Created on: Jun 1, 2023
 *      Author: naina
 */

#ifndef INC_JOYSTICK_H_
#define INC_JOYSTICK_H_
#include "stm32f4xx.h"
#include "math.h"
#include "arm_math.h"

void JoyStickControl();
void Calculate_Position(float x_c1, float x_c2, float x_c3, float y_c1,
		float y_c2, float y_c3);

#endif /* INC_JOYSTICK_H_ */
