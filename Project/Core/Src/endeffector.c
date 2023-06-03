/*
 * endeffector.c
 *
 *  Created on: Jun 1, 2023
 *      Author: naina
 */
#include "Function.h"
#include "endeffector.h"
#include "joystick.h"
#include "pid_traject.h"
#include "main.h"

extern I2C_HandleTypeDef hi2c2;

static uint8_t Test_Start_data[2] = { 0x01, 1 };
static uint8_t Test_Stop_data[2] = { 0x01, 0 };
static uint8_t Reset_data[4] = { 0x00, 0xFF, 0x55, 0xAA };
static uint8_t In_Emergency_data[1] = { 0xFF };
static uint8_t Out_Emergency_data[4] = { 0xE5, 0x7A, 0xFF, 0x81 };
static uint8_t Run_Mode_data[2] = { 0x10, 0x13 };
static uint8_t Close_Run_Mode_data[2] = { 0x10, 0x8C };
static uint8_t Pick_data[2] = { 0x10, 0x5A };
static uint8_t Place_data[2] = { 0x10, 0x69 };
static uint8_t Read_data[1];

extern enum Laser {
	Init,
	Test_Start,
	Test_Stop,
	Reset,
	In_Emergency,
	Out_Emergency,
	Run_Mode,
	Close_Run_Mode,
	Pick,
	Place,
	Read,
} EndEffector_State;

void EndEffector_Event(char EndEffector_State) {
	if (hi2c2.State == HAL_I2C_STATE_READY) {
		switch (EndEffector_State) {
		case Init:
			break;

		case Test_Start:
			HAL_I2C_Master_Transmit(&hi2c2, End_Address << 1, Test_Start_data,
					2, 10000);
			EndEffector_State = Init;
			break;

		case Test_Stop:
			HAL_I2C_Master_Transmit(&hi2c2, End_Address << 1, Test_Stop_data, 2,
					10000);
			EndEffector_State = Init;
			break;

		case Reset:
			HAL_I2C_Master_Transmit(&hi2c2, End_Address << 1, Reset_data, 4,
					10000);
			EndEffector_State = Init;
			break;
		case In_Emergency:
			HAL_I2C_Master_Transmit(&hi2c2, End_Address << 1, In_Emergency_data,
					1, 10000);
			EndEffector_State = Init;
			break;
		case Out_Emergency:
			HAL_I2C_Master_Transmit(&hi2c2, End_Address << 1,
					Out_Emergency_data, 4, 10000);
			EndEffector_State = Init;
			break;
		case Run_Mode:
			HAL_I2C_Master_Transmit(&hi2c2, End_Address << 1, Run_Mode_data, 2,
					10000);
			EndEffector_State = Init;
			break;
		case Close_Run_Mode:
			HAL_I2C_Master_Transmit(&hi2c2, End_Address << 1,
					Close_Run_Mode_data, 2, 10000);
			EndEffector_State = Init;
			break;

		case Pick:
			HAL_I2C_Master_Transmit(&hi2c2, End_Address << 1, Pick_data, 2,
					10000);
			HAL_Delay(2000);
			EndEffector_State = Init;
			break;
		case Place:
			HAL_I2C_Master_Transmit(&hi2c2, End_Address << 1, Place_data, 2,
					10000);
			HAL_Delay(2000);
			EndEffector_State = Init;
			break;
		case Read:
			HAL_I2C_Master_Receive(&hi2c2, End_Address << 1, Read_data, 1,
					10000);
			break;
		}

	}
}
