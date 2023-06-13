/*
 * joystick.c
 *
 *  Created on: Jun 1, 2023
 *      Author: naina
 */
#include "Function.h"
#include "endeffector.h"
#include "joystick.h"
#include "pid_traject.h"
#include "main.h"
#include "ModBusRTU.h"
#include "string.h"
#include "math.h"

extern SPI_HandleTypeDef hspi3;

uint8_t TX[10] = { 0x01, 0x42 };
uint8_t RX[10];
uint8_t i = 0;
uint8_t workState = 0;

//Motor
uint16_t fast = 65536 * 0.35;
uint16_t slow = 65536 * 0.18;
uint8_t state_motor = 0; //0 is slow and 1 is fast

uint8_t RX_last = 0x00;
uint8_t button_last = 0x00;

int8_t count = 0;
float y_c[3];
float x_c[3];

extern float PosY;
extern u16u8_t registerFrame[200];

extern float a1;
extern float b;
extern float c;
//float cos_zeta1= a1/ c;
extern float cos_zeta;
extern float sin_zeta;

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

extern enum {
	PICK, PLACE
} TRAY_STATUS;

int imod3;
float x_pre_final[9];
float y_pre_final[9];
float x_final_joy[9] = { 0, 0, 0, 0, 0, 0, 0, 0, 0 };
float y_final_joy[9] = { 0, 0, 0, 0, 0, 0, 0, 0, 0 };
float q;

extern float Pick_Point_Y[9];
extern float Pick_Point_X[9];

extern float Place_Point_Y[9];
extern float Place_Point_X[9];

int homing = 0;

uint32_t time_joy_offset = 0;

void JoyStickControl() {

	read_pos();
	HAL_GPIO_WritePin(JoyStick_SS_PIN_GPIO_Port, JoyStick_SS_PIN_Pin, 0);
	HAL_SPI_TransmitReceive(&hspi3, TX, RX, 10, 30);
	HAL_GPIO_WritePin(JoyStick_SS_PIN_GPIO_Port, JoyStick_SS_PIN_Pin, 1);
	// O 0xdf   0x7f

//	static uint32_t timestamp_joy = 0;

	if (RX[4] == 0xfe && RX_last == 0xff) { //Select Speed Button
		if (state_motor == 1) {
			state_motor = 0;
		} else if (state_motor == 0) {
			state_motor = 1;
		}
	} else if (RX[4] == 0xbf && button_last == 0xFF
			&& HAL_GetTick() - time_joy_offset >= 1000) { //X Button
		time_joy_offset = HAL_GetTick();
		y_c[count] = PosY;

		if (x_axis_Actual_Position>= 0 && x_axis_Actual_Position <= 3500) {
			x_c[count] = (float)x_axis_Actual_Position/10.0;
		} else if(x_axis_Actual_Position >= 65535-3500 && x_axis_Actual_Position <= 65535) {
			x_c[count] = -((float)(65536%x_axis_Actual_Position))/10.0;
		}

		count += 1;
		if (count >= 2) {
			count = 2;
		}
	}

//	else if (RX[4] == 0x7f && button_last == 0xFF
//			&& HAL_GetTick() - time_joy_offset >= 1000) { // Delete Button
//		time_joy_offset = HAL_GetTick();
//
//		y_c[count] = 0;
//		x_c[count] = 0;
//		count -= 1;
//		if (count <= 0) {
//			count = 0;
//		}
//	}
	else if (RX[4] == 0xdf && button_last == 0xFF
			&& HAL_GetTick() - time_joy_offset >= 1000 && count >= 2) { // Delete Button
		Calculate_Position(x_c[0], x_c[1], x_c[2], y_c[0], y_c[1], y_c[2]);

		if (TRAY_STATUS == PICK) {
//			Calculate_Position(104, 114.5, 63.7, 221.5, 281.9, 281.7);
			Pick_Tray_Origin_x= (int16_t)(x_c[0] * 10);
			Pick_Tray_Origin_y = (int16_t)(y_c[0] * 10);
			Pick_Tray_Origin_Orientation = asinf(fabsf(sin_zeta))  * 100;
			memcpy(Pick_Point_X, x_final_joy, sizeof(x_final_joy) + 1);
			memcpy(Pick_Point_Y, y_final_joy, sizeof(y_final_joy) + 1);

			count = 0;
		} else if (TRAY_STATUS == PLACE) {
//			Calculate_Position(70.4, 128.2, 130.3, -267.1, -269.1, -218.4);
			Place_Tray_Origin_x = (int16_t)(x_c[0] * 10);
			Place_Tray_Origin_y = (int16_t)(y_c[0] * 10);
			Place_Tray_Origin_Orientation = asinf(fabsf(sin_zeta))  * 100;
			memcpy(Place_Point_X, x_final_joy, sizeof(x_final_joy) + 1);
			memcpy(Place_Point_Y, y_final_joy, sizeof(y_final_joy) + 1);
			count = 0;
		}

//			Pick_Tray_Origin_x= 500;
//			Pick_Tray_Origin_y= 2000;
//			Pick_Tray_Origin_Orientation= 18000;;
//
//			Place_Tray_Origin_x= -500;
//			Place_Tray_Origin_y= -2000;
//			Place_Tray_Origin_Orientation= 9000;;

		HAL_GPIO_WritePin(Switch_Relay_1_GPIO_Port, Switch_Relay_1_Pin, RESET);
		HAL_GPIO_WritePin(Switch_Relay_2_GPIO_Port, Switch_Relay_2_Pin, RESET);
		HAL_GPIO_WritePin(Switch_Relay_3_GPIO_Port, Switch_Relay_3_Pin, RESET);
		memset(x_c, 0, sizeof(x_c));
		memset(y_c, 0, sizeof(y_c));

		y_axis_Moving_Status= 0;
//		End_Effector_Status = 0;

		count = 0;

		motor(0, 0);
		State = IDLE;

	}
//	else if (RX[4] == 0xEF && button_last == 0xFF) {
//		homing = 1;
//	}

//motor speed Select
	switch (state_motor) {
	case 0:
		if (RX[3] == 0xfe) { //Not be push
			motor(0, 1);
			x_axis_Moving_Status= 0;
		}
		else if (RX[3] == 0xee) { //UP
			x_axis_Moving_Status= 0;
			motor(fast, 1);
		}
		else if (RX[3] == 0xbe) { //Down
			x_axis_Moving_Status= 0;
			motor(fast, -1);
		}
		else if (RX[3] == 0x7e) { //left
			x_axis_Moving_Status = 8;
			motor(0, 1);
		}
		else if (RX[3] == 0xde) { //right
			x_axis_Moving_Status = 4;
			motor(0, 1);
		}

		break;

		case 1:
		if (RX[3] == 0xfe) { //Not be push
			motor(0, 1);
			x_axis_Moving_Status= 0;
		}
		else if (RX[3] == 0xee) { //UP
			x_axis_Moving_Status= 0;
			motor(slow, 1);
		}
		else if (RX[3] == 0xbe) { //Down
			x_axis_Moving_Status= 0;
			motor(slow, -1);
		}
		else if (RX[3] == 0x7e) { //left
			x_axis_Moving_Status = 8;
			motor(0, 1);
		}
		else if (RX[3] == 0xde) { //right
			x_axis_Moving_Status = 4;
			motor(0, 1);
		}

	}

	switch (homing) {
	case 0:
		break;
	case 1:
		Joy_Homing();
		break;
	}

	RX_last = RX[4];
	button_last = RX[4];
}

void Calculate_Position(float x_c1, float x_c2, float x_c3, float y_c1,
		float y_c2, float y_c3) {

//Parameter use in Equation
//Trigonometry
	a1 = x_c2 - x_c1;
	b = y_c2 - y_c1;
	c = sqrt((a1 * a1) + (b * b));

//float cos_zeta1= a1/ c;
	cos_zeta = a1 / c;
	sin_zeta = b / c;
//	float tan_zeta1= b / a;

	x_final_joy[0] = (((10 * cos_zeta) - (40 * sin_zeta)) + x_c1) * 10;
	x_final_joy[1] = (((30 * cos_zeta) - (40 * sin_zeta)) + x_c1) * 10;
	x_final_joy[2] = (((50 * cos_zeta) - (40 * sin_zeta)) + x_c1) * 10;
	x_final_joy[3] = (((10 * cos_zeta) - (25 * sin_zeta)) + x_c1) * 10;
	x_final_joy[4] = (((30 * cos_zeta) - (25 * sin_zeta)) + x_c1) * 10;
	x_final_joy[5] = (((50 * cos_zeta) - (25 * sin_zeta)) + x_c1) * 10;
	x_final_joy[6] = (((10 * cos_zeta) - (10 * sin_zeta)) + x_c1) * 10;
	x_final_joy[7] = (((30 * cos_zeta) - (10 * sin_zeta)) + x_c1) * 10;
	x_final_joy[8] = (((50 * cos_zeta) - (10 * sin_zeta)) + x_c1) * 10;

	y_final_joy[0] = ((40 * cos_zeta) + (10 * sin_zeta)) + y_c1;
	y_final_joy[1] = ((40 * cos_zeta) + (30 * sin_zeta)) + y_c1;
	y_final_joy[2] = ((40 * cos_zeta) + (50 * sin_zeta)) + y_c1;
	y_final_joy[3] = ((25 * cos_zeta) + (10 * sin_zeta)) + y_c1;
	y_final_joy[4] = ((25 * cos_zeta) + (30 * sin_zeta)) + y_c1;
	y_final_joy[5] = ((25 * cos_zeta) + (50 * sin_zeta)) + y_c1;
	y_final_joy[6] = ((10 * cos_zeta) + (10 * sin_zeta)) + y_c1;
	y_final_joy[7] = ((10 * cos_zeta) + (30 * sin_zeta)) + y_c1;
	y_final_joy[8] = ((10 * cos_zeta) + (50 * sin_zeta)) + y_c1;

}

