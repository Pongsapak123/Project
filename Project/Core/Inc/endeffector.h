/*
 * endeffector.h
 *
 *  Created on: Jun 1, 2023
 *      Author: naina
 */

#ifndef INC_ENDEFFECTOR_H_
#define INC_ENDEFFECTOR_H_
#include "stm32f4xx.h"
#include "math.h"
#include "arm_math.h"

#define End_Address 0x15

void EndEffector_Event(char EndEffector_State);


#endif /* INC_ENDEFFECTOR_H_ */
