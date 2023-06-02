/*
 * Function.h
 *
 *  Created on: May 21, 2023
 *      Author: naina
 */

#ifndef INC_FUNCTION_H_
#define INC_FUNCTION_H_
#include "stm32f4xx.h"
#include "math.h"
#include "arm_math.h"

#define Max_Velocity 1400 // mm/s
#define Max_Workspace 700 // mm

#define Max_Counter_PWM 65536

#define Heartbeat_Protocol 0x00
#define y_axis_Moving_Status registerFrame[0x10].U16
#define x_axis_Moving_Status 0x40
#define x_axis_Target_Position 0x41
#define x_axis_Target_Speed 0x42
#define x_axis_Target_Acceleration_Time 0x43

void Init_Homing();
void Photo_IT();
void motor(uint32_t speed, int DIR);
void read_pos();

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);
void Test_Range();

#endif /* INC_FUNCTION_H_ */
