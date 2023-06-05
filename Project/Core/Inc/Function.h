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

#define Max_Velocity 945 // mm/s
#define Max_Acceleration 4161
#define Max_Workspace 700 // mm


#define Max_Counter_PWM 65536

#define Heartbeat_Protocol registerFrame[0x00].U16

#define Base_System_Status registerFrame[0x01].U16
#define End_Effector_Status registerFrame[0x02].U16

#define y_axis_Moving_Status registerFrame[0x10].U16
#define y_axis_Actual_Position registerFrame[0x11].U16
#define y_axis_Actual_Speed registerFrame[0x12].U16
#define y_axis_Actual_Acceleration registerFrame[0x13].U16

#define Pick_Tray_Origin_x registerFrame[0x20].U16
#define Pick_Tray_Origin_y registerFrame[0x21].U16
#define Pick_Tray_Origin_Orientation registerFrame[0x22].U16

#define Place_Tray_Origin_x registerFrame[0x23].U16
#define Place_Tray_Origin_y registerFrame[0x24].U16
#define Place_Tray_Origin_Orientation registerFrame[0x25].U16

#define Goal_Point_x registerFrame[0x30].U16
#define Goal_Point_y registerFrame[0x31].U16

#define x_axis_Moving_Status registerFrame[0x40].U16
#define x_axis_Target_Position registerFrame[0x41].U16
#define x_axis_Target_Speed registerFrame[0x42].U16
#define x_axis_Target_Acceleration_Time registerFrame[0x43].U16

#define x_axis_Actual_Position registerFrame[0x44].U16
#define x_axis_Actual_Speed registerFrame[0x45].U16



void Init_Homing();
void Photo_IT();
void motor(uint32_t speed, int DIR);
void read_pos();

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);

void Test_Range();

#endif /* INC_FUNCTION_H_ */
