/*
 * Function.c
 *
 *  Created on: May 21, 2023
 *      Author: naina
 */

#include "Function.h"
#include "endeffector.h"
#include "joystick.h"
#include "pid_traject.h"
#include "main.h"
#include "ModBusRTU.h"

extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim1;

extern int32_t QEIReadRaw;
extern float PosY;
extern float pos_i;
extern float pos_f;
extern int position_index;
extern float position_test[18];
extern uint8_t State_PID;
extern u16u8_t registerFrame[200];

extern enum State_Machine {
	INIT,
	INIT_HOMING,
	IDLE,
	SETPICKTRAY,
	SETPLACETRAY,
	RUNTRAYMODE,
	RUNPOINTMODE,
	EMERGENCY,
	OUT_EMBERGENCY,
	SENSOR_CHECK,
} State;

extern int homing;

void read_pos() {
	QEIReadRaw = __HAL_TIM_GET_COUNTER(&htim2);
	PosY = QEIReadRaw * (120.0 / 8192.0);
}

void motor(uint32_t speed, int DIR) {
	if (DIR == -1) {
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, SET); //1

		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, speed);

	} else if (DIR == 1) {
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, RESET); //0
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, speed);
	}
}

void Init_Homing() {
	HAL_GPIO_WritePin(Switch_Relay_1_GPIO_Port, Switch_Relay_1_Pin, SET);
	HAL_GPIO_WritePin(Switch_Relay_2_GPIO_Port, Switch_Relay_2_Pin, RESET);
	HAL_GPIO_WritePin(Switch_Relay_3_GPIO_Port, Switch_Relay_3_Pin, RESET);
	static uint16_t state_homing = 0;

	switch (state_homing) {
	case 0:

		if (HAL_GPIO_ReadPin(Photoelectric_sensor_3_GPIO_Port,
		Photoelectric_sensor_3_Pin) == 0) {
			__HAL_TIM_SET_COUNTER(&htim2, 0);
			motor(0, 1);
			state_homing = 1;
		} else {
			motor(Max_Counter_PWM * 0.25, -1);
		}
		break;

	case 1:
		if (HAL_GPIO_ReadPin(Photoelectric_sensor_1_GPIO_Port,
		Photoelectric_sensor_1_Pin) == 0) {
			motor(0, 1);
			HAL_Delay(400);
			__HAL_TIM_SET_COUNTER(&htim2, 0);
			QEIReadRaw = __HAL_TIM_GET_COUNTER(&htim2);
			PosY = QEIReadRaw * (120.0 / 8192.0);

			y_axis_Moving_Status= 0;

			state_homing = 0;
			EndEffector_Event(6);

			HAL_GPIO_WritePin(Switch_Relay_1_GPIO_Port, Switch_Relay_1_Pin,
					RESET);
			HAL_GPIO_WritePin(Switch_Relay_2_GPIO_Port, Switch_Relay_2_Pin,
					RESET);
			HAL_GPIO_WritePin(Switch_Relay_3_GPIO_Port, Switch_Relay_3_Pin,
					RESET);
			State = IDLE;
		} else {
			motor(Max_Counter_PWM * 0.18, 1);
		}
		break;
	}
}

void Joy_Homing() {
	static uint16_t state_homing = 0;
	switch (state_homing) {
	case 0:
		if (HAL_GPIO_ReadPin(Photoelectric_sensor_3_GPIO_Port,
		Photoelectric_sensor_3_Pin) == 0) {
			motor(0, 1);
			state_homing = 1;
		} else {
			motor(Max_Counter_PWM * 0.25, -1);
		}
		break;

	case 1:
		if (HAL_GPIO_ReadPin(Photoelectric_sensor_1_GPIO_Port,
		Photoelectric_sensor_1_Pin) == 0) {
			motor(0, 1);
			HAL_Delay(400);
			__HAL_TIM_SET_COUNTER(&htim2, 0);
			QEIReadRaw = __HAL_TIM_GET_COUNTER(&htim2);
			PosY = QEIReadRaw * (120.0 / 8192.0);

			homing = 0;
			state_homing = 0;

		} else {
			motor(Max_Counter_PWM * 0.18, 1);
		}
		break;
	}
}

void Test_Range() {
	read_pos();

	static uint16_t state_test_range = 0;
	switch (state_test_range) {
	case 0:
		if (HAL_GPIO_ReadPin(Photoelectric_sensor_3_GPIO_Port,
		Photoelectric_sensor_3_Pin) == 0) {
			motor(0, 1);
			HAL_Delay(500);
			__HAL_TIM_SET_COUNTER(&htim2, 0);
			HAL_Delay(500);
			state_test_range = 1;
		} else {
			motor(Max_Counter_PWM * 0.25, -1);
		}
		break;

	case 1:
		if (HAL_GPIO_ReadPin(Photoelectric_sensor_1_GPIO_Port,
		Photoelectric_sensor_1_Pin) == 0) {
			motor(0, 1);
			HAL_Delay(200);
//			__HAL_TIM_SET_COUNTER(&htim2, 23893);
		} else {
			motor(Max_Counter_PWM * 0.2, 1);
		}
		break;
	}
}
