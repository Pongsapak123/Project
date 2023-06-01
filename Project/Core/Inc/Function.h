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

#define End_Address 0x15

#define Max_Velocity 1400 // mm/s
#define Max_Workspace 700 // mm

#define Max_Counter_PWM 65536

void Init_Homing();
void Photo_IT();
void motor(uint32_t speed, int DIR);
void read_pos();
void PID(float setposition);
void EndEffector_Event(char EndEffector_State);

void JoyStickControl();
void Calculate_Position(float x_c1, float x_c2, float x_c3, float y_c1,
		float y_c2, float y_c3);

void Trajectory_Gen(double x_init, double x_fi, double v_fi, double Accel);
void Trajectory_Eva();
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);
void PID_Position(float setposition);
void PID_Velocity(int setvelocity);
void Test_Range();

#endif /* INC_FUNCTION_H_ */
