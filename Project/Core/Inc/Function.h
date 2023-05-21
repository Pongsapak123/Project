/*
 * Function.h
 *
 *  Created on: May 21, 2023
 *      Author: naina
 */

#ifndef INC_FUNCTION_H_
#define INC_FUNCTION_H_
#include "stm32f4xx.h"
//#include "math.h"

//#include "main.h"

//#define Photoelectric_sensor_1_Pin GPIO_PIN_8
//#define Photoelectric_sensor_1_GPIO_Port GPIOD
//
//#define Photoelectric_sensor_2_Pin GPIO_PIN_2
//#define Photoelectric_sensor_2_GPIO_Port GPIOB
//
//#define Photoelectric_sensor_3_Pin GPIO_PIN_6
//#define Photoelectric_sensor_3_GPIO_Port GPIOC

// for Trajectory gen and eva///
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

double t_Acce = 0.001;
double t_DeAcce = 0.001;
double t_Cons = 0.001;
double t_count = 0.001;
double t_diff = 0.001;
double t_acceleration = 0;
double t_final = 0;
double t_triangle = 0;
double x;
double v;
double a;
///////////////////////////////

// for PID control
int32_t QEIReadRaw;
float PosY;
int8_t dir;

float Kp = 100.0; //2700   70 75 0
float Ki = 2.0; //200	0.0001 1 0.00025
float Kd = 1.0; //5	0.0 0.7 1000

float current_pos = 0.0;
float previous_pos = 0.0;
float current_velocity = 0.0;
float previous_velocity = 0.0;
float max_velocity = 0.0;
float acc = 0.0;
float max_acc = 0.0;
float rangeTarget = 0.5;

float velocityfeedback;
float Dutyfeedback;

float Error = 0;
float Last_Error = 0;
float Last_ErrordeltaT;
float Intregral = 0;
float deltaT = 0.00001;
////////////////////////////////

float pos_i = 0;
float pos_f = 0;

uint8_t direction = 0;

uint8_t State_PID = 2;
uint8_t state_IT = 0;
uint8_t go_next = 0;
uint8_t Re = 0;

uint64_t traject_us = 100;
uint64_t pid_us = 100;
uint64_t _micros = 0;
uint64_t checker = 0;
int photo1;
int photo2;
int photo3;

enum {
	INIT, INIT_HOMING, PID_TEST, PHOTO_LIMIT, IDLE
} State = INIT;

#endif /* INC_FUNCTION_H_ */
