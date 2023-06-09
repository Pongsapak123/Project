/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2023 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "math.h"
#include "arm_math.h"
#include "Function.h"
#include "endeffector.h"
#include "joystick.h"
#include "pid_traject.h"
#include "ModBusRTU.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c2;

SPI_HandleTypeDef hspi3;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim5;
TIM_HandleTypeDef htim11;

UART_HandleTypeDef huart2;
UART_HandleTypeDef huart6;
DMA_HandleTypeDef hdma_usart2_tx;

/* USER CODE BEGIN PV */
////////////Traject/////////
uint64_t traject_us = 1000;
uint64_t pid_us = 1000;

float pos_i = 0;
float pos_f = 0;

uint8_t state_IT = 0;
uint8_t go_next = 0;
uint8_t Re = 0;

double t_Acce = 0.001;
double t_DeAcce = 0.001;
double t_Cons = 0.001;
double t_count = 0.001;
double t_diff = 0.001;

double x;
double v;
double a;

float Dutyfeedback;
///////////////////////////////

// for PID control///////////
uint8_t State_PID = 2;
float Intregral = 0;

float current_pos = 0.0;
float current_velocity = 0.0;
float acc = 0.0;
////////////////////////////////

int state_laser = 0;
/////////////////////////////

uint64_t _micros = 0;

ModbusHandleTypedef hmodbus;
u16u8_t registerFrame[200];

////////////read sensor/////////
struct GPIO_Check {
	int photo1;
	int photo2;
	int photo3;
	int emer;
	int ramp;
} GPIO_test;

int PHOTO1;
int PHOTO2;
int32_t QEIReadRaw;
float PosY;
////////////read sensor/////////

////////////JoyStick/////////

float Pick_Point_Y[9];
float Pick_Point_X[9];

float Place_Point_Y[9];
float Place_Point_X[9];

int position_index = 0;
float position_test[18] = { 99.34, 544.89, 119.34, 564.89, 139.64, 584.89,
		99.34, 544.89, 119.34, 564.89, 139.64, 584.89, 99.34, 544.89, 119.34,
		564.89, 139.64, 584.89, };

enum State_Machine {
	INIT,
	INIT_HOMING,
	IDLE,
	SETPICKTRAY,
	SETPLACETRAY,
	RUNTRAYMODE,
	RUNPOINTMODE,
	EMERGENCY_LIMIT,
	SENSOR_CHECK,
} State = SENSOR_CHECK;

int LAST_STATE;

enum Laser {
	Init,
	Test_Start,
	Test_Stop,
	Reset,
	In_Emergency,
	Out_Emergency,
	Run_Mode,
	Close_Run_Mode,
	Pick,
	Pick_Check,
	Place,
	Place_Check,
	Read,
} EndEffector_State = Init;

enum State_Machine_RUNTRAYMODE {
	GOPICK, GOPLACE
} State_RUNTRAYMODE = GOPICK;

enum State_Machine_Control {
	TRAJECTGEN, TRAJECTEVA_PID, TRAJECTGENNEXT
} State_Control = TRAJECTGEN;

enum TRAY_STATUS_ENUM {
	PICK, PLACE
} TRAY_STATUS;

struct BaseSystemBit {
	int SetPickTray;
	int SetPlaceTray;
	int Home;
	int RunTrayMode;
	int RunPointMode;
};

struct EndEffectorStatusBit {
	int LaserOff;
	int LaserOn;
	int GripperPower;
	int GripperPicking;
	int GripperPlacing;
};

struct yaxisMovingStatusBit {
	int JogPick;
	int JogPlease;
	int Home;
	int GoPick;
	int GoPlace;
	int Gopoint;
};

struct xaxisMovingStatusBit {
	int Home;
	int Run;
	int JogLeft;
	int JogRight;
};

int state_laser_test = 0;

float a1 = 0;
float b = 0;
float c = 0;
//float cos_zeta = a / c;
float cos_zeta;
float sin_zeta;
//	float tan_zeta = b / a;

int last_endeffecter_status = 0;

int flag = 0;
uint64_t timestamp_wait = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART6_UART_Init(void);
static void MX_TIM5_Init(void);
static void MX_SPI3_Init(void);
static void MX_TIM11_Init(void);
static void MX_I2C2_Init(void);
/* USER CODE BEGIN PFP */
inline uint64_t micros();
void ramp_set(int number);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// for Trajectory gen and eva///
/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {
	/* USER CODE BEGIN 1 */

	/* USER CODE END 1 */

	/* MCU Configuration--------------------------------------------------------*/

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* USER CODE BEGIN Init */

	/* USER CODE END Init */

	/* Configure the system clock */
	SystemClock_Config();

	/* USER CODE BEGIN SysInit */

	/* USER CODE END SysInit */

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_DMA_Init();
	MX_USART2_UART_Init();
	MX_TIM2_Init();
	MX_TIM1_Init();
	MX_TIM3_Init();
	MX_USART6_UART_Init();
	MX_TIM5_Init();
	MX_SPI3_Init();
	MX_TIM11_Init();
	MX_I2C2_Init();
	/* USER CODE BEGIN 2 */
	EndEffector_Event(Reset);
	HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_1 | TIM_CHANNEL_2);
	HAL_TIM_Base_Start_IT(&htim3);
	HAL_TIM_Base_Start(&htim1);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);

	HAL_TIM_Base_Start_IT(&htim5);

	t_Acce = traject_us / 1000000.00;
	t_DeAcce = traject_us / 1000000.00;
	t_Cons = traject_us / 1000000.00;
	t_count = traject_us / 1000000.00;
	t_diff = traject_us / 1000000.00;

	hmodbus.huart = &huart2;
	hmodbus.htim = &htim11;
	hmodbus.slaveAddress = 0x15;
	hmodbus.RegisterSize = 200;
	Modbus_init(&hmodbus, registerFrame);

	struct BaseSystemBit BaseSystemStatusData = { .SetPickTray = 1,
			.SetPlaceTray = 2, .Home = 4, .RunTrayMode = 8, .RunPointMode = 16 };

	struct EndEffectorStatusBit EndEffectorStatusData = { .LaserOff = 0,
			.LaserOn = 1, .GripperPower = 2, .GripperPicking = 6,
			.GripperPlacing = 10 };

	struct yaxisMovingStatusBit yaxisMovingStatusData =
			{ .JogPick = 1, .JogPlease = 2, .Home = 4, .GoPick = 8, .GoPlace =
					16, .Gopoint = 32 };

	struct xaxisMovingStatusBit xaxisMovingStatusData = { .Home = 1, .Run = 2,
			.JogLeft = 4, .JogRight = 8 };

	x_axis_Actual_Position= 0;
	x_axis_Target_Speed= 3000;
	x_axis_Target_Acceleration_Time= 1;
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {

		static uint64_t timestamp_traject = 0;
		static uint64_t timestamp_heartbeat = 0;
//		static uint64_t timestamp_Endeffecter = 0;

		int64_t GetTicku = micros();

		Modbus_Protocal_Worker();

		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */

		if (HAL_GetTick() >= timestamp_heartbeat) {
			timestamp_heartbeat = HAL_GetTick() + 200;

			Heartbeat_Protocol= 22881;

			y_axis_Actual_Position= (int32_t)(PosY*10);
			y_axis_Actual_Speed= fabs(current_velocity)*10;
			y_axis_Actual_Acceleration= fabs(acc)*10;

		}

		switch (State) {

		case INIT:
			State = INIT_HOMING;
			break;

		case INIT_HOMING:
			y_axis_Moving_Status= yaxisMovingStatusData.Home;
			x_axis_Moving_Status = xaxisMovingStatusData.Home;
			Init_Homing();

			break;

			case IDLE: //HOME

			if(End_Effector_Status != last_endeffecter_status) {
				if (End_Effector_Status == EndEffectorStatusData.LaserOn) {
					EndEffector_Event(Test_Start);
				} else if(End_Effector_Status == EndEffectorStatusData.LaserOff) {
					EndEffector_Event(Test_Stop);
				} else if (End_Effector_Status == EndEffectorStatusData.GripperPower) {
					EndEffector_Event(Run_Mode);
				}

				if (End_Effector_Status == EndEffectorStatusData.GripperPicking) {
					//				EndEffector_Event(Run_Mode);
					//				HAL_Delay(300);
					EndEffector_Event(Pick);
					End_Effector_Status = EndEffectorStatusData.GripperPower;
				} else if (End_Effector_Status == EndEffectorStatusData.GripperPlacing) {
					//				EndEffector_Event(Run_Mode);
					//				HAL_Delay(300);
					EndEffector_Event(Place);
					End_Effector_Status = EndEffectorStatusData.GripperPower;
				}
			}

			last_endeffecter_status = End_Effector_Status;

			if(Base_System_Status == BaseSystemStatusData.SetPickTray) {
				End_Effector_Status = EndEffectorStatusData.LaserOn;
				EndEffector_Event(Test_Start);
				HAL_GPIO_WritePin(Switch_Relay_1_GPIO_Port, Switch_Relay_1_Pin,
						RESET);
				HAL_GPIO_WritePin(Switch_Relay_2_GPIO_Port, Switch_Relay_2_Pin,
						SET);
				HAL_GPIO_WritePin(Switch_Relay_3_GPIO_Port, Switch_Relay_3_Pin,
						RESET);
				Base_System_Status = 0;
				y_axis_Moving_Status = yaxisMovingStatusData.JogPick;
				TRAY_STATUS = PICK;
				State = SETPICKTRAY;
			} else if(Base_System_Status == BaseSystemStatusData.SetPlaceTray) {
				End_Effector_Status = EndEffectorStatusData.LaserOn;
				EndEffector_Event(Test_Start);
				HAL_GPIO_WritePin(Switch_Relay_1_GPIO_Port, Switch_Relay_1_Pin,
						RESET);
				HAL_GPIO_WritePin(Switch_Relay_2_GPIO_Port, Switch_Relay_2_Pin,
						SET);
				HAL_GPIO_WritePin(Switch_Relay_3_GPIO_Port, Switch_Relay_3_Pin,
						RESET);
				Base_System_Status = 0;
				y_axis_Moving_Status = yaxisMovingStatusData.JogPlease;
				TRAY_STATUS = PLACE;
				State = SETPLACETRAY;
			}

			if(Base_System_Status == BaseSystemStatusData.RunPointMode) {
				Base_System_Status = 0;
//				EndEffector_Event(Run_Mode);
				x_axis_Target_Position = Goal_Point_x;
				x_axis_Moving_Status = xaxisMovingStatusData.Run;

				pos_i = PosY;

				if(Goal_Point_y >= 0 && Goal_Point_y <= 3500) {
					pos_f = (float)Goal_Point_y/10;
				} else if(Goal_Point_y >= 65535-3500 && Goal_Point_y <= 65535) {
					pos_f = -(float)(65536%Goal_Point_y)/10;
				}

				Trajectory_Gen(pos_i, pos_f, Max_Velocity, Max_Acceleration);

				y_axis_Moving_Status = yaxisMovingStatusData.Gopoint;
				State = RUNPOINTMODE;

			} else if(Base_System_Status == BaseSystemStatusData.RunTrayMode) {

				HAL_GPIO_WritePin(Switch_Relay_1_GPIO_Port, Switch_Relay_1_Pin, RESET);
				HAL_GPIO_WritePin(Switch_Relay_2_GPIO_Port, Switch_Relay_2_Pin, RESET);
				HAL_GPIO_WritePin(Switch_Relay_3_GPIO_Port, Switch_Relay_3_Pin, SET);

				Base_System_Status = 0;
				position_index = 0;
				End_Effector_Status = EndEffectorStatusData.GripperPower;
				EndEffector_Event(Run_Mode);
				State = RUNTRAYMODE;
				State_Control = TRAJECTGEN;
				State_RUNTRAYMODE = GOPICK;
			}

			if(Base_System_Status == BaseSystemStatusData.Home) {
				Base_System_Status = 0;
				State = INIT_HOMING;
			}
			break;

			case SETPICKTRAY:
			JoyStickControl();
			break;

			case SETPLACETRAY:
			JoyStickControl();
			break;

			case RUNPOINTMODE:
			if (GetTicku >= timestamp_traject) {
				timestamp_traject = GetTicku + traject_us;
				Trajectory_Eva();
				read_pos();
				PID(x);
			} else if (pos_f - PosY <= Boundary && pos_f - PosY >= -Boundary ) {
				__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
				Intregral = 0;
				Dutyfeedback = 0;
				v = 0;
				a = 0;

				pos_i = PosY;
				y_axis_Moving_Status = 0;

				State = IDLE;
			}
			break;

			case RUNTRAYMODE:
			switch (State_RUNTRAYMODE) {

				case GOPICK:
				y_axis_Moving_Status = yaxisMovingStatusData.GoPick;
				switch (State_Control) {
					case TRAJECTGEN:
					pos_i = PosY;
					pos_f = Pick_Point_Y[position_index];

					Trajectory_Gen(pos_i, pos_f, Max_Velocity, Max_Acceleration);

					if(x_axis_Moving_Status == 0) {
						x_axis_Target_Position= (int16_t)Pick_Point_X[position_index];
						x_axis_Moving_Status = xaxisMovingStatusData.Run;
						State_Control = TRAJECTEVA_PID;
					}

					break;
					case TRAJECTEVA_PID:
					if (GetTicku >= timestamp_traject) {
						timestamp_traject = GetTicku + traject_us;
						Trajectory_Eva();
						read_pos();
						PID(x);
					} else if (pos_f - PosY <= Boundary && pos_f - PosY >= -Boundary ) {
						__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
						Intregral = 0;
						Dutyfeedback = 0;
						v = 0;
						a = 0;

//						HAL_Delay(2000);
//						State_Control = TRAJECTGENNEXT;

						State_Control = TRAJECTGENNEXT;

					}
					break;

					case TRAJECTGENNEXT:
					if(flag == 0 && x_axis_Moving_Status == 0) {
						EndEffector_Event(Pick);
						timestamp_wait = HAL_GetTick()+2200;
						flag = 1;
					} else if(flag == 1) {
						if(HAL_GetTick() >= timestamp_wait) {
							flag = 0;
							pos_i = PosY;
							pos_f = Place_Point_Y[position_index];

							State_Control = TRAJECTGEN;
							State_RUNTRAYMODE = GOPLACE;
						}
					}

					break;

				}
				break;

				case GOPLACE:
				y_axis_Moving_Status = yaxisMovingStatusData.GoPlace;
				switch (State_Control) {
					case TRAJECTGEN:

					Trajectory_Gen(pos_i, pos_f, Max_Velocity, Max_Acceleration);

					if(x_axis_Moving_Status == 0) {
						x_axis_Target_Position= (int16_t)Place_Point_X[position_index];
						x_axis_Moving_Status = xaxisMovingStatusData.Run;
						State_Control = TRAJECTEVA_PID;
					}

					break;

					case TRAJECTEVA_PID:
					if (GetTicku >= timestamp_traject) {
						timestamp_traject = GetTicku + traject_us;
						Trajectory_Eva();
						read_pos();
						PID(x);
					} else if (pos_f - PosY <= Boundary && pos_f - PosY >= -Boundary) {
						__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
						Intregral = 0;
						Dutyfeedback = 0;
						v = 0;
						a = 0;

						pos_i = PosY;

//						HAL_Delay(2000);
						State_Control = TRAJECTGENNEXT;

					}
					break;

					case TRAJECTGENNEXT:

					if(flag == 0 && x_axis_Moving_Status == 0) {
						EndEffector_Event(Place);
						timestamp_wait = HAL_GetTick()+2200;
						flag = 1;
					} else if(flag == 1) {
						if(HAL_GetTick() >= timestamp_wait) {
							flag = 0;
							if(position_index < 8) {
								pos_i = PosY;
								position_index++;
								State_Control = TRAJECTGEN;
								State_RUNTRAYMODE = GOPICK;

							} else {
								y_axis_Moving_Status = 0;
								State = INIT_HOMING;
							}
						}
					}

					break;
				}
				break;
			}

			break;

			case EMERGENCY_LIMIT:
			if(HAL_GPIO_ReadPin(Emergency_GPIO_Port, Emergency_Pin) == 0) {
				EndEffector_Event(Out_Emergency);
				State = LAST_STATE;
				LAST_STATE = 0;
			}
			break;

			case SENSOR_CHECK:
			read_pos();
			GPIO_test.photo1 = HAL_GPIO_ReadPin(Photoelectric_sensor_1_GPIO_Port,Photoelectric_sensor_1_Pin);
			GPIO_test.photo2 = HAL_GPIO_ReadPin(Photoelectric_sensor_2_GPIO_Port,Photoelectric_sensor_2_Pin);
			GPIO_test.photo3 = HAL_GPIO_ReadPin(Photoelectric_sensor_3_GPIO_Port,Photoelectric_sensor_3_Pin);
			GPIO_test.emer = HAL_GPIO_ReadPin(Emergency_GPIO_Port,Emergency_Pin);

			if (GPIO_test.ramp == 1) {
				HAL_GPIO_WritePin(Switch_Relay_1_GPIO_Port, Switch_Relay_1_Pin,
						SET);
				HAL_GPIO_WritePin(Switch_Relay_2_GPIO_Port, Switch_Relay_2_Pin,
						RESET);
				HAL_GPIO_WritePin(Switch_Relay_3_GPIO_Port, Switch_Relay_3_Pin,
						RESET);
			} else if (GPIO_test.ramp == 2) {
				HAL_GPIO_WritePin(Switch_Relay_1_GPIO_Port, Switch_Relay_1_Pin,
						RESET);
				HAL_GPIO_WritePin(Switch_Relay_2_GPIO_Port, Switch_Relay_2_Pin,
						SET);
				HAL_GPIO_WritePin(Switch_Relay_3_GPIO_Port, Switch_Relay_3_Pin,
						RESET);
			} else if (GPIO_test.ramp == 3) {
				HAL_GPIO_WritePin(Switch_Relay_1_GPIO_Port, Switch_Relay_1_Pin,
						RESET);
				HAL_GPIO_WritePin(Switch_Relay_2_GPIO_Port, Switch_Relay_2_Pin,
						RESET);
				HAL_GPIO_WritePin(Switch_Relay_3_GPIO_Port, Switch_Relay_3_Pin,
						SET);
			} else {
				HAL_GPIO_WritePin(Switch_Relay_1_GPIO_Port, Switch_Relay_1_Pin,
						RESET);
				HAL_GPIO_WritePin(Switch_Relay_2_GPIO_Port, Switch_Relay_2_Pin,
						RESET);
				HAL_GPIO_WritePin(Switch_Relay_3_GPIO_Port, Switch_Relay_3_Pin,
						RESET);
			}

			if(go_next == 1 ) {
				State = INIT;
			}

			break;
		}

		if (Re == 1) {
			NVIC_SystemReset();
			break;
		}

	}
	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

	/** Configure the main internal regulator output voltage
	 */
	__HAL_RCC_PWR_CLK_ENABLE();
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLM = 4;
	RCC_OscInitStruct.PLL.PLLN = 100;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = 4;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK) {
		Error_Handler();
	}
}

/**
 * @brief I2C2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2C2_Init(void) {

	/* USER CODE BEGIN I2C2_Init 0 */

	/* USER CODE END I2C2_Init 0 */

	/* USER CODE BEGIN I2C2_Init 1 */

	/* USER CODE END I2C2_Init 1 */
	hi2c2.Instance = I2C2;
	hi2c2.Init.ClockSpeed = 100000;
	hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
	hi2c2.Init.OwnAddress1 = 0;
	hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	hi2c2.Init.OwnAddress2 = 0;
	hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
	if (HAL_I2C_Init(&hi2c2) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN I2C2_Init 2 */

	/* USER CODE END I2C2_Init 2 */

}

/**
 * @brief SPI3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_SPI3_Init(void) {

	/* USER CODE BEGIN SPI3_Init 0 */

	/* USER CODE END SPI3_Init 0 */

	/* USER CODE BEGIN SPI3_Init 1 */

	/* USER CODE END SPI3_Init 1 */
	/* SPI3 parameter configuration*/
	hspi3.Instance = SPI3;
	hspi3.Init.Mode = SPI_MODE_MASTER;
	hspi3.Init.Direction = SPI_DIRECTION_2LINES;
	hspi3.Init.DataSize = SPI_DATASIZE_8BIT;
	hspi3.Init.CLKPolarity = SPI_POLARITY_HIGH;
	hspi3.Init.CLKPhase = SPI_PHASE_2EDGE;
	hspi3.Init.NSS = SPI_NSS_SOFT;
	hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256;
	hspi3.Init.FirstBit = SPI_FIRSTBIT_LSB;
	hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
	hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
	hspi3.Init.CRCPolynomial = 10;
	if (HAL_SPI_Init(&hspi3) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN SPI3_Init 2 */

	/* USER CODE END SPI3_Init 2 */

}

/**
 * @brief TIM1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM1_Init(void) {

	/* USER CODE BEGIN TIM1_Init 0 */

	/* USER CODE END TIM1_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };
	TIM_OC_InitTypeDef sConfigOC = { 0 };
	TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = { 0 };

	/* USER CODE BEGIN TIM1_Init 1 */

	/* USER CODE END TIM1_Init 1 */
	htim1.Instance = TIM1;
	htim1.Init.Prescaler = 0;
	htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim1.Init.Period = 65535;
	htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim1.Init.RepetitionCounter = 0;
	htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim1) != HAL_OK) {
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_PWM_Init(&htim1) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
	sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
	if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1)
			!= HAL_OK) {
		Error_Handler();
	}
	sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
	sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
	sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
	sBreakDeadTimeConfig.DeadTime = 0;
	sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
	sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
	sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
	if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM1_Init 2 */

	/* USER CODE END TIM1_Init 2 */
	HAL_TIM_MspPostInit(&htim1);

}

/**
 * @brief TIM2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM2_Init(void) {

	/* USER CODE BEGIN TIM2_Init 0 */

	/* USER CODE END TIM2_Init 0 */

	TIM_Encoder_InitTypeDef sConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };

	/* USER CODE BEGIN TIM2_Init 1 */

	/* USER CODE END TIM2_Init 1 */
	htim2.Instance = TIM2;
	htim2.Init.Prescaler = 0;
	htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim2.Init.Period = 4294967295;
	htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
	sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
	sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
	sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
	sConfig.IC1Filter = 15;
	sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
	sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
	sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
	sConfig.IC2Filter = 15;
	if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM2_Init 2 */

	/* USER CODE END TIM2_Init 2 */

}

/**
 * @brief TIM3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM3_Init(void) {

	/* USER CODE BEGIN TIM3_Init 0 */

	/* USER CODE END TIM3_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };

	/* USER CODE BEGIN TIM3_Init 1 */

	/* USER CODE END TIM3_Init 1 */
	htim3.Instance = TIM3;
	htim3.Init.Prescaler = 99;
	htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim3.Init.Period = 9999;
	htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim3) != HAL_OK) {
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM3_Init 2 */

	/* USER CODE END TIM3_Init 2 */

}

/**
 * @brief TIM5 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM5_Init(void) {

	/* USER CODE BEGIN TIM5_Init 0 */

	/* USER CODE END TIM5_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };

	/* USER CODE BEGIN TIM5_Init 1 */

	/* USER CODE END TIM5_Init 1 */
	htim5.Instance = TIM5;
	htim5.Init.Prescaler = 99;
	htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim5.Init.Period = 4294967295;
	htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim5) != HAL_OK) {
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM5_Init 2 */

	/* USER CODE END TIM5_Init 2 */

}

/**
 * @brief TIM11 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM11_Init(void) {

	/* USER CODE BEGIN TIM11_Init 0 */

	/* USER CODE END TIM11_Init 0 */

	TIM_OC_InitTypeDef sConfigOC = { 0 };

	/* USER CODE BEGIN TIM11_Init 1 */

	/* USER CODE END TIM11_Init 1 */
	htim11.Instance = TIM11;
	htim11.Init.Prescaler = 99;
	htim11.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim11.Init.Period = 2005;
	htim11.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim11.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim11) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_OC_Init(&htim11) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_OnePulse_Init(&htim11, TIM_OPMODE_SINGLE) != HAL_OK) {
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_ACTIVE;
	sConfigOC.Pulse = 1433;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	if (HAL_TIM_OC_ConfigChannel(&htim11, &sConfigOC, TIM_CHANNEL_1)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM11_Init 2 */

	/* USER CODE END TIM11_Init 2 */

}

/**
 * @brief USART2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART2_UART_Init(void) {

	/* USER CODE BEGIN USART2_Init 0 */

	/* USER CODE END USART2_Init 0 */

	/* USER CODE BEGIN USART2_Init 1 */

	/* USER CODE END USART2_Init 1 */
	huart2.Instance = USART2;
	huart2.Init.BaudRate = 19200;
	huart2.Init.WordLength = UART_WORDLENGTH_9B;
	huart2.Init.StopBits = UART_STOPBITS_1;
	huart2.Init.Parity = UART_PARITY_EVEN;
	huart2.Init.Mode = UART_MODE_TX_RX;
	huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart2.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&huart2) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN USART2_Init 2 */

	/* USER CODE END USART2_Init 2 */

}

/**
 * @brief USART6 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART6_UART_Init(void) {

	/* USER CODE BEGIN USART6_Init 0 */

	/* USER CODE END USART6_Init 0 */

	/* USER CODE BEGIN USART6_Init 1 */

	/* USER CODE END USART6_Init 1 */
	huart6.Instance = USART6;
	huart6.Init.BaudRate = 115200;
	huart6.Init.WordLength = UART_WORDLENGTH_9B;
	huart6.Init.StopBits = UART_STOPBITS_1;
	huart6.Init.Parity = UART_PARITY_EVEN;
	huart6.Init.Mode = UART_MODE_TX_RX;
	huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart6.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&huart6) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN USART6_Init 2 */

	/* USER CODE END USART6_Init 2 */

}

/**
 * Enable DMA controller clock
 */
static void MX_DMA_Init(void) {

	/* DMA controller clock enable */
	__HAL_RCC_DMA1_CLK_ENABLE();

	/* DMA interrupt init */
	/* DMA1_Stream6_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };
	/* USER CODE BEGIN MX_GPIO_Init_1 */
	/* USER CODE END MX_GPIO_Init_1 */

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOH_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOD_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOA,
			Switch_Relay_3_Pin | Switch_Relay_1_Pin | Switch_Relay_2_Pin
					| DIR_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(JoyStick_SS_PIN_GPIO_Port, JoyStick_SS_PIN_Pin,
			GPIO_PIN_RESET);

	/*Configure GPIO pins : B1_Pin Photoelectric_sensor_1_Pin */
	GPIO_InitStruct.Pin = B1_Pin | Photoelectric_sensor_1_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/*Configure GPIO pin : Emergency_Pin */
	GPIO_InitStruct.Pin = Emergency_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(Emergency_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : Switch_Relay_3_Pin Switch_Relay_1_Pin Switch_Relay_2_Pin DIR_Pin */
	GPIO_InitStruct.Pin = Switch_Relay_3_Pin | Switch_Relay_1_Pin
			| Switch_Relay_2_Pin | DIR_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pins : Photoelectric_sensor_2_Pin Photoelectric_sensor_3_Pin */
	GPIO_InitStruct.Pin = Photoelectric_sensor_2_Pin
			| Photoelectric_sensor_3_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/*Configure GPIO pin : JoyStick_SS_PIN_Pin */
	GPIO_InitStruct.Pin = JoyStick_SS_PIN_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(JoyStick_SS_PIN_GPIO_Port, &GPIO_InitStruct);

	/* EXTI interrupt init*/
	HAL_NVIC_SetPriority(EXTI2_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(EXTI2_IRQn);

	HAL_NVIC_SetPriority(EXTI3_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(EXTI3_IRQn);

	HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

	HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

	/* USER CODE BEGIN MX_GPIO_Init_2 */
	/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void ramp_set(int number) {

	if (number == 1) {
		HAL_GPIO_WritePin(Switch_Relay_1_GPIO_Port, Switch_Relay_1_Pin, SET);
	} else {
		HAL_GPIO_WritePin(Switch_Relay_2_GPIO_Port, Switch_Relay_2_Pin, RESET);
		HAL_GPIO_WritePin(Switch_Relay_3_GPIO_Port, Switch_Relay_3_Pin, RESET);
	}

	if (number == 2) {
		HAL_GPIO_WritePin(Switch_Relay_2_GPIO_Port, Switch_Relay_2_Pin, SET);
	} else {
		HAL_GPIO_WritePin(Switch_Relay_1_GPIO_Port, Switch_Relay_1_Pin, RESET);
		HAL_GPIO_WritePin(Switch_Relay_3_GPIO_Port, Switch_Relay_3_Pin, RESET);
	}

	if (number == 3) {
		HAL_GPIO_WritePin(Switch_Relay_3_GPIO_Port, Switch_Relay_3_Pin, SET);
	} else {
		HAL_GPIO_WritePin(Switch_Relay_1_GPIO_Port, Switch_Relay_1_Pin, RESET);
		HAL_GPIO_WritePin(Switch_Relay_2_GPIO_Port, Switch_Relay_2_Pin, RESET);
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

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {

//	if (GPIO_Pin == Photoelectric_sensor_1_Pin) {
//		if (State == PID_STATE) {
//			Dutyfeedback = 0;
//			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
//			state_IT = 1;
//			State = EMERGENCY_LIMIT;
//		}
//	}
//
//	if (GPIO_Pin == Photoelectric_sensor_3_Pin) {
//		if (State == PID_STATE) {
//			Dutyfeedback = 0;
//			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
//			state_IT = 1;
//			State = EMERGENCY_LIMIT;
//		}
//	}

	if (GPIO_Pin == Emergency_Pin) {
		if (State != SENSOR_CHECK) {
			LAST_STATE = State;
			Dutyfeedback = 0;
			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);

			EndEffector_Event(In_Emergency);

			State = EMERGENCY_LIMIT;
		}

	}
}

uint64_t micros() {
	return __HAL_TIM_GET_COUNTER(&htim5) + _micros;
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if (htim == &htim5) {
		_micros += UINT32_MAX;
	}
}

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1) {
	}
	/* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
/* User can add his own implementation to report the file name and line number,
ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
