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

uint8_t y_count = 0;
float y_c[3];

extern float PosY;
extern u16u8_t registerFrame[200];

extern enum State_Machine {
	INIT,
	INIT_HOMING,
	IDLE,
	SETPICKTRAY,
	SETPLACETRAY,
	RUNTRAYMODE,
	RUNPOINTMODE,
	EMERGENCY_LIMIT,
	SENSOR_CHECK
} State;

int imod3;
float x_pre_final[9];
float y_pre_final[9];
float x_final_joy[9] = { 0, 0, 0, 0, 0, 0, 0, 0, 0 };
float y_final_joy[9] = { 0, 0, 0, 0, 0, 0, 0, 0, 0 };
float q;

void JoyStickControl() {

	read_pos();
	HAL_GPIO_WritePin(JoyStick_SS_PIN_GPIO_Port, JoyStick_SS_PIN_Pin, 0);
	HAL_SPI_TransmitReceive(&hspi3, TX, RX, 10, 30);
	HAL_GPIO_WritePin(JoyStick_SS_PIN_GPIO_Port, JoyStick_SS_PIN_Pin, 1);

	if (RX[4] == 0xfe && RX_last == 0xff) { //Select Speed Button
		if (state_motor == 1) {
			state_motor = 0;
		} else if (state_motor == 0) {
			state_motor = 1;
		}
	} else if (RX[4] == 0xbf && button_last == 0xFF) { //X Button

		if (y_count >= 2) {

			Pick_Tray_Origin_x= 500;
			Pick_Tray_Origin_y = 2000;
			Pick_Tray_Origin_Orientation = 18000;;

			Place_Tray_Origin_x = -500;
			Place_Tray_Origin_y = -2000;
			Place_Tray_Origin_Orientation = 9000;;

			y_axis_Moving_Status= 0;
			State = IDLE;
			y_count = 0;

			motor(0, 0);
		} else {
			y_count += 1;
			y_c[y_count] = PosY;
		}


	} else if (RX[4] == 0xEF && button_last == 0xFF) {
		workState = 1;
		State = INIT_HOMING;
	}

//motor speed Select
	switch (state_motor) {
	case 0:
		if (RX[3] == 0xfe) { //Not be push
			motor(0, 1);
			x_axis_Moving_Status= 0;
		}
		else if (RX[3] == 0xee) { //UP
			x_axis_Moving_Status= 0;
			motor(fast, -1);
		}
		else if (RX[3] == 0xbe) { //Down
			x_axis_Moving_Status= 0;
			motor(fast, 1);
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
			motor(slow, -1);
		}
		else if (RX[3] == 0xbe) { //Down
			x_axis_Moving_Status= 0;
			motor(slow, 1);
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
	RX_last = RX[4];
	button_last = RX[4];
}

void Calculate_Position(float x_c1, float x_c2, float x_c3, float y_c1,
		float y_c2, float y_c3) {
	int i = 0;
//Parameter use in Equation
//Trigonometry
	float a = x_c2 - x_c1;
	float b = y_c2 - y_c1;
	float c = sqrt((a * a) + (b * b));
//float cos_zeta = a / c;
	float sin_zeta = b / c;
//Calculate
	for (i = 0; i < 3; i++) {
		q = floor((i + 1) / 3);
		imod3 = (i + 1) % 3;
		//Position of hole then not rotation
		x_pre_final[i] = x_c1 + (50 - (20 * ((2 - (2 * q)) / (imod3 + q))));
		y_pre_final[i] = y_c[0] - (40 - (15 * ((2 - (2 * q)) / (imod3 + q))));
		//Position when rotation 25
		x_final_joy[i] = x_pre_final[i] + (15 * sin_zeta);
		y_final_joy[i] = y_pre_final[i] + (20 * sin_zeta);
	}
//Second Row
	x_final_joy[3] = x_pre_final[0] + (15 * 2 * sin_zeta);
	y_final_joy[3] = y_pre_final[0] + (20 * 2 * sin_zeta);

	x_final_joy[4] = x_pre_final[1] + (15 * 2 * sin_zeta);
	y_final_joy[4] = y_pre_final[1] + (20 * 2 * sin_zeta);

	x_final_joy[5] = x_pre_final[2] + (15 * 2 * sin_zeta);
	y_final_joy[5] = y_pre_final[2] + (20 * 2 * sin_zeta);
//Third Row
	x_final_joy[6] = x_pre_final[0] + (15 * 3 * sin_zeta);
	y_final_joy[6] = y_pre_final[0] + (20 * 3 * sin_zeta);

	x_final_joy[7] = x_pre_final[1] + (15 * 3 * sin_zeta);
	y_final_joy[7] = y_pre_final[1] + (20 * 3 * sin_zeta);

	x_final_joy[8] = x_pre_final[2] + (15 * 3 * sin_zeta);
	y_final_joy[8] = y_pre_final[2] + (20 * 3 * sin_zeta);
}

