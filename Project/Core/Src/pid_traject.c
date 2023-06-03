/*
 * pid_traject.c
 *
 *  Created on: Jun 1, 2023
 *      Author: naina
 */
#include "Function.h"
#include "endeffector.h"
#include "joystick.h"
#include "pid_traject.h"
#include "main.h"

extern TIM_HandleTypeDef htim1;

extern uint8_t State_PID;

extern uint64_t traject_us;
extern uint64_t pid_us;

int8_t dir;

float Kp = 2400.0; //2600
float Ki = 1.3; //1
float Kd = 0.7; //0.7

float current_pos = 0.0;
float previous_pos = 0.0;
float current_velocity = 0.0;
float previous_velocity = 0.0;
float max_velocity = 0.0;
float acc = 0.0;
float max_acc = 0.0;
float rangeTarget = 0.5;

float velocityfeedback;
extern float Dutyfeedback;

float Error = 0;
float Last_Error = 0;
float Intregral = 0;
float deltaT = 0.00001;

extern float PosY;
extern float pos_i;
extern float pos_f;
extern int position_index;
extern float position_test[18];

extern double x;
extern double v;
extern double a;

int trajectory_type = 0;
int direct = 0;
double x_initial = 0;
double x_final = 0;
double v_final = 0;
double Acceleration = 0;

double deltaX = 0;
double x_final1 = 0;
double x_final2 = 0;
double v_final1 = 0;
double v_final2 = 0;
double v_initial = 0;
extern double t_Acce;
extern double t_DeAcce;
extern double t_Cons;
extern double t_count;
extern double t_diff;

double t_acceleration = 0;
double t_final = 0;
double t_triangle = 0;

extern int state_laser;

extern enum State_Machine {
	INIT, INIT_HOMING, CALIBRATE, TRAJECT_GEN, PID_STATE, EMERGENCY_LIMIT, IDLE
} State;

void PID(float setposition) {

	current_pos = PosY;
	current_velocity = (current_pos - previous_pos) / (pid_us / 1000000.0);
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

	motor(Dutyfeedback, dir);
	Last_Error = Error;

	if (pos_f - PosY <= 0.2 && pos_f - PosY >= -0.2) {
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
		Intregral = 0;
		Dutyfeedback = 0;
		v = 0;
		a = 0;

		if ((position_index + 2) % 2 == 0) {
			state_laser = 3;
		} else if ((position_index + 2) % 2 == 1) {
			state_laser = 4;
		}

		if (position_index < 17) {
			position_index++;
			pos_i = PosY;
			pos_f = position_test[position_index];
			State = TRAJECT_GEN;
			State_PID = 0;
		} else {
			State_PID = 2;
			position_index = 0;
			State = INIT_HOMING;
		}

	}

}

void Trajectory_Gen(double x_init, double x_fi, double v_fi, double Accel) {
	x_initial = x_init;
	x_final = x_fi;
	v_final = v_fi;
	Acceleration = Accel;
	t_Acce = traject_us / 1000000;
	t_DeAcce = traject_us / 1000000;
	t_Cons = traject_us / 1000000;
	t_count = traject_us / 1000000;

	deltaX = fabs(x_final - x_initial);
	if (x_final - x_initial > 0) {
		direct = 1;
	} else if (x_final - x_initial < 0) {
		direct = -1;
	}

	t_acceleration = v_final / Acceleration;

	t_triangle = sqrt(deltaX / Acceleration);

	if (t_triangle < t_acceleration) {
		t_final = 2 * (t_triangle);
		trajectory_type = 1;

	} else if (t_triangle >= t_acceleration) {
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

//			State = IDLE;
			trajectory_type = 0;
		}
		break;
	}

}

