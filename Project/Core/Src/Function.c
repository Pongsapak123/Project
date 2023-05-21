/*
 * Function.c
 *
 *  Created on: May 21, 2023
 *      Author: naina
 */

#include "Function.h"
#include "math.h"
#include "main.h"

#define Max_Counter_PWM 65536

I2C_HandleTypeDef hi2c2;

SPI_HandleTypeDef hspi3;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim5;

UART_HandleTypeDef huart2;
UART_HandleTypeDef huart6;

inline uint64_t micros();

void Init_Homing();
void Photo_IT();
void motor(uint32_t speed, int DIR);
void read_pos();
void PID(float setposition);
void EndEffector_Event(char EndEffector_State);
void Trajectory_Gen(double x_init, double x_fi, double v_fi, double Accel);
void Trajectory_Eva();
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);

void PID_Position(float setposition);
void PID_Velocity(int setvelocity);
void Test_Range();

void Trajectory_Gen(double x_init, double x_fi, double v_fi, double Accel) {
	x_initial = x_init;
	x_final = x_fi;
	v_final = v_fi;
	Acceleration = Accel;
	deltaX = fabs(x_final - x_initial);
	if (x_final - x_initial > 0) {
		direct = 1;
	} else if (x_final - x_initial < 0) {
		direct = -1;
	}

	t_acceleration = v_final / Acceleration;

	t_triangle = sqrt(deltaX / Acceleration);

	if (t_triangle <= t_acceleration) {
		t_final = 2 * (t_triangle);
		trajectory_type = 1;

	} else if (t_triangle > t_acceleration) {
		t_final = (2 * t_acceleration)
				+ (((deltaX) - (t_acceleration * v_final)) / v_final);
		trajectory_type = 2;
	}
}

void Trajectory_Eva() {
	switch (trajectory_type) {
	case 0:

		break;
	case 1:
		if (t_count <= t_triangle) {
			x = x_initial
					+ (1.0 / 2.0 * direct * Acceleration * (t_Acce * t_Acce));
			v = Acceleration * t_Acce * direct;
			a = Acceleration * direct;
			x_final1 = x;
			v_final1 = v;
			t_Acce = t_Acce + t_diff;
			t_count = t_count + t_diff;

		} else if (t_count <= t_final) {
			x = x_final1 + (v_final1 * t_DeAcce)
					- (1.0 / 2.0 * direct * Acceleration * t_DeAcce * t_DeAcce);
			v = v_final1 - (Acceleration * t_DeAcce * direct);
			a = -Acceleration * direct;
			t_DeAcce = t_DeAcce + t_diff;
			t_count = t_count + t_diff;
		} else {
			x = x_final;
			v = 0;
			t_Acce = traject_us / 1000000;
			t_DeAcce = traject_us / 1000000;
			t_Cons = traject_us / 1000000;
			t_count = traject_us / 1000000;

//			State = IDLE;
			trajectory_type = 0;
		}
		break;
	case 2:
		if (t_count <= t_acceleration) {
			x = x_initial + 1.0 / 2.0 * Acceleration * direct * t_Acce * t_Acce;
			v = Acceleration * t_Acce * direct;
			a = Acceleration * direct;
			x_final1 = x;
			v_final1 = v;

			t_Acce = t_Acce + t_diff;
			t_count = t_count + t_diff;

		} else if (t_count <= t_final - t_acceleration) {
			x = (v_final * t_Cons * direct) + x_final1;
			v = v_final * direct;
			a = 0;
			x_final2 = x;
			v_final2 = v;

			t_Cons = t_Cons + t_diff;
			t_count = t_count + t_diff;
		} else if (t_count <= t_final) {
			x =
					x_final2 + (v_final2 * (t_DeAcce))
							- (1.0 / 2.0 * direct * Acceleration
									* (t_DeAcce * t_DeAcce));
			v = v_final2 - (Acceleration * t_DeAcce * direct);
			a = -Acceleration * direct;

			t_DeAcce = t_DeAcce + t_diff;
			t_count = t_count + t_diff;
		} else {
			x = x_final;
			v = 0;
			t_Acce = traject_us / 1000000;
			t_DeAcce = traject_us / 1000000;
			t_Cons = traject_us / 1000000;
			t_count = traject_us / 1000000;

//			State = IDLE;
			trajectory_type = 0;
		}
		break;
	}

}

void PID(float setposition) {

	current_pos = PosY;
	current_velocity = (current_pos - previous_pos) / (pid_us / 1000000);
	previous_pos = current_pos;

	if (pos_f < 0) {
		pos_f = 0;
	} else if (pos_f > 700) {
		pos_f = 700;
	}

	Error = setposition - PosY;

	if (!((Dutyfeedback >= Max_Counter_PWM)
			&& ((Error >= 0 && Intregral >= 0) || (Error < 0 && Intregral < 0)))) {
		Intregral = Intregral + Error;
	}

	Dutyfeedback = (Kp * Error) + (Kd * ((Error - Last_Error) / deltaT))
			+ (Intregral * Ki);

	if (Dutyfeedback >= Max_Counter_PWM * 0.7) {
		Dutyfeedback = Max_Counter_PWM * 0.7;
	} else if (Dutyfeedback <= Max_Counter_PWM * -0.7) {
		Dutyfeedback = Max_Counter_PWM * -0.7;
	}

	if (Dutyfeedback < 0) {
		dir = -1;
	} else if (Dutyfeedback > 0) {
		dir = 1;
	}

	if (Error > 1.0) {
		Dutyfeedback += 1 * Kp; //230
	} else if (Error < -1.0) {
		Dutyfeedback -= 1 * Kp;
	}

	Dutyfeedback = fabs(Dutyfeedback);

//	pos_f >= PosY - 1 && pos_f <= PosY + 1

//	if(PosY != x){
//		State_PID = 1;
//		State = PID_TEST;
//	}
	motor(Dutyfeedback, dir);
	Last_Error = Error;
//	if (PosY >= pos_f * 1.01) {
//		overshoot_check = 1;
//	} pos_f >= PosY - 0.2 && pos_f <= PosY + 0.2
//	if (pos_f >= PosY - 0.2 && pos_f <= PosY + 0.2) {
////		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
//
////		HAL_Delay(500);
////		overshoot_check = 0;
//		Intregral = 0;
//		Dutyfeedback = 0;
//		pos_i = PosY;
////		State_PID = 2;
////		State = IDLE;
//	}

//	}
}

void Init_Homing() {
	static uint16_t state_homing = 0;
	switch (state_homing) {
	case 0:
		if (HAL_GPIO_ReadPin(Photoelectric_sensor_3_GPIO_Port,
		Photoelectric_sensor_3_Pin) == 0) {
			__HAL_TIM_SET_COUNTER(&htim2, 0);
			motor(0, 1);
			state_homing = 1;
		} else {
			motor(Max_Counter_PWM * 0.2, -1);
		}
		break;

	case 1:
		if (HAL_GPIO_ReadPin(Photoelectric_sensor_2_GPIO_Port,
		Photoelectric_sensor_2_Pin) == 0) {
			motor(0, 1);
			HAL_Delay(200);
			__HAL_TIM_SET_COUNTER(&htim2, 23893);
			QEIReadRaw = __HAL_TIM_GET_COUNTER(&htim2);
			PosY = QEIReadRaw * (120.0 / 8192.0);
			pos_i = PosY;
//			State_PID = 2;
			State = IDLE;
		} else {
			motor(Max_Counter_PWM * 0.2, 1);
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

void Photo_IT() {
	switch (state_IT) {
	case 0:
		if (state_IT == 0) {
//			motor(0, 1);
//			motor(Max_Counter_PWM * 0.5, -1);
		} else if (state_IT == 1) {
			Dutyfeedback = 0;
			motor(0, 1);
		}
		break;

	case 1:
		Dutyfeedback = 0;
		motor(0, 1);
		break;
	}
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

void read_pos() {
	QEIReadRaw = __HAL_TIM_GET_COUNTER(&htim2);
	PosY = QEIReadRaw * (120.0 / 8192.0);
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	if (GPIO_Pin == Photoelectric_sensor_1_Pin) {
		if (State == PID_TEST ) {
			Dutyfeedback = 0;
			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
			state_IT = 1;
			State = PHOTO_LIMIT;
		}

	}
	if (GPIO_Pin == Photoelectric_sensor_3_Pin) {
		if (State == PID_TEST) {
			Dutyfeedback = 0;
			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
			state_IT = 1;
			State = PHOTO_LIMIT;
		}
	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if (htim == &htim5) {
		_micros += UINT32_MAX;
	}
}

uint64_t micros() {
	return __HAL_TIM_GET_COUNTER(&htim5) + _micros;
}
