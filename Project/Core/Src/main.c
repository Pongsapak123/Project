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
float pos_i = 0;
float pos_f = 0;

uint8_t State_PID = 2;
uint8_t state_IT = 0;
uint8_t go_next = 0;
uint8_t Re = 0;

uint64_t traject_us = 1000;
uint64_t pid_us = 1000;

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

// for PID control///////////
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
float Dutyfeedback;

float Error = 0;
float Last_Error = 0;
float Last_ErrordeltaT;
float Intregral = 0;
float deltaT = 0.00001;
////////////////////////////////

// for JoyStick and Calibrate////
uint8_t TX[10] = { 0x01, 0x42 };
uint8_t RX[10];
uint8_t i = 0;
uint8_t workState = 0;

//Calibrate Data Set
uint8_t y_count = 0;
float y_c[3]; //Calibrate y-axis point
float x_pre_final[9];
float y_pre_final[9];
float x_final_joy[9] = { 0, 0, 0, 0, 0, 0, 0, 0, 0 };
float y_final[9] = { 0, 0, 0, 0, 0, 0, 0, 0, 0 };
float q;
uint8_t RX_last = 0x00;
uint8_t button_last = 0x00;
int imod3;
//Motor
uint16_t fast = 65536 * 0.23;
uint16_t slow = 65536 * 0.23;
uint8_t state_motor = 0; //0 is slow and 1 is fast

/////////////////////////////

///////Laser Status
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
int state_laser_test = 0;
/////////////////////////////

uint64_t _micros = 0;

ModbusHandleTypedef hmodbus;
u16u8_t registerFrame[200];

////////////read sensor/////////
int photo1;
int photo2;
int photo3;
int emer;

int32_t QEIReadRaw;
float PosY;
////////////read sensor/////////

int position_index = 0;
float position_test[18] = { 538.0, 142.9, 518.7, 122.6, 498.1, 102.1, 538.0,
		142.9, 518.7, 122.6, 498.1, 102.12, 538.0, 142.9, 518.7, 122.6, 498.1,
		102.1 };

enum State_Machine {
	INIT, INIT_HOMING, CALIBRATE, TRAJECT_GEN, PID_STATE, EMERGENCY_LIMIT, IDLE
} State = INIT;

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
	Place,
	Read,
} EndEffector_State = Init;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM3_Init(void);
static void MX_I2C2_Init(void);
static void MX_USART6_UART_Init(void);
static void MX_TIM5_Init(void);
static void MX_SPI3_Init(void);
static void MX_TIM11_Init(void);
/* USER CODE BEGIN PFP */
inline uint64_t micros();

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
	MX_I2C2_Init();
	MX_USART6_UART_Init();
	MX_TIM5_Init();
	MX_SPI3_Init();
	MX_TIM11_Init();
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

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {

		static uint64_t timestamp_traject = 0;
		static uint64_t timestamp_heartbeat = 0;
		int64_t GetTicku = micros();
		Modbus_Protocal_Worker();
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */

		if (HAL_GetTick() >= timestamp_heartbeat) {
			timestamp_heartbeat = HAL_GetTick() + 200;

			registerFrame[0x00].U16 = 22881;
		}

		switch (state_laser_test) {
		case 0:
			break;
		case 1:
			EndEffector_Event(Test_Start);
			state_laser_test = 0;
			break;
		case 2:
			EndEffector_Event(Run_Mode);
			state_laser_test = 0;
			break;
		case 3:
			EndEffector_Event(Pick);
			state_laser_test = 0;
			break;
		case 4:
			EndEffector_Event(Place);
			state_laser_test = 0;
			break;
		case 5:
			EndEffector_Event(Reset);
			state_laser_test = 0;
			break;
		case 6:
			EndEffector_Event(Test_Stop);
			state_laser_test = 0;
		}

		switch (State) {
		case INIT:
			read_pos();
			photo1 = HAL_GPIO_ReadPin(Photoelectric_sensor_1_GPIO_Port,
			Photoelectric_sensor_1_Pin);
			photo2 = HAL_GPIO_ReadPin(Photoelectric_sensor_2_GPIO_Port,
			Photoelectric_sensor_2_Pin);
			photo3 = HAL_GPIO_ReadPin(Photoelectric_sensor_3_GPIO_Port,
			Photoelectric_sensor_3_Pin);
			emer = HAL_GPIO_ReadPin(Emergency_GPIO_Port, Emergency_Pin);
			break;
		case INIT_HOMING:
			Init_Homing();
			break;
		case CALIBRATE:
			JoyStickControl();
			break;
		case TRAJECT_GEN:
			read_pos();
			Trajectory_Gen(pos_i, pos_f, 945, 4161);
			State = PID_STATE;
			break;
		case PID_STATE:

			if (GetTicku >= timestamp_traject) {
				timestamp_traject = GetTicku + traject_us;
				Trajectory_Eva();
				read_pos();
				PID(x);
			}
			if (State_PID == 1) {
				motor(0, 1);
				State = IDLE;
			}
			break;
		case IDLE:
			motor(0, 1);
			if (State_PID == 0) {
				State = TRAJECT_GEN;
			}
			break;
		case EMERGENCY_LIMIT:
			Photo_IT();
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
	Switch_Relay_1_Pin | Switch_Relay_2_Pin | Switch_Relay_3_Pin | DIR_Pin,
			GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(JoyStick_SS_PIN_GPIO_Port, JoyStick_SS_PIN_Pin,
			GPIO_PIN_RESET);

	/*Configure GPIO pins : B1_Pin Emergency_Pin Photoelectric_sensor_3_Pin */
	GPIO_InitStruct.Pin = B1_Pin | Emergency_Pin | Photoelectric_sensor_3_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/*Configure GPIO pins : Switch_Relay_1_Pin Switch_Relay_2_Pin Switch_Relay_3_Pin DIR_Pin */
	GPIO_InitStruct.Pin = Switch_Relay_1_Pin | Switch_Relay_2_Pin
			| Switch_Relay_3_Pin | DIR_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pins : Photoelectric_sensor_2_Pin Photoelectric_sensor_1_Pin */
	GPIO_InitStruct.Pin = Photoelectric_sensor_2_Pin
			| Photoelectric_sensor_1_Pin;
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

//	if(PosY != x){
//		State_PID = 1;
//		State = PID_TEST;
//	}
	motor(Dutyfeedback, dir);
	Last_Error = Error;
//	if (PosY >= pos_f * 1.01) {
//		overshoot_check = 1;
//	} pos_f >= PosY - 0.2 && pos_f <= PosY + 0.2

	if (pos_f - PosY <= 0.2 && pos_f - PosY >= -0.2) {
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
//		HAL_Delay(500);
//		overshoot_check = 0;
		Intregral = 0;
		Dutyfeedback = 0;
		v = 0;
		a = 0;

		if ((position_index + 2) % 2 == 0) {
			state_laser_test = 3;
		} else if ((position_index + 2) % 2 == 1) {
			state_laser_test = 4;
		}

		if (position_index < 17) {
			position_index++;
			read_pos();

			pos_f = position_test[position_index];
			State = TRAJECT_GEN;
			State_PID = 0;
		} else {
			State_PID = 2;
			position_index = 0;
			State = INIT_HOMING;
		}

	}

//	}
}

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
			EndEffector_State = Init;
			break;
		case Place:

			HAL_I2C_Master_Transmit(&hi2c2, End_Address << 1, Place_data, 2,
					10000);
			EndEffector_State = Init;
			break;
		case Read:
			HAL_I2C_Master_Receive(&hi2c2, End_Address << 1, Read_data, 1,
					10000);
			break;
		}

	}
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
		y_final[i] = y_pre_final[i] + (20 * sin_zeta);
	}
//Second Row
	x_final_joy[3] = x_pre_final[0] + (15 * 2 * sin_zeta);
	y_final[3] = y_pre_final[0] + (20 * 2 * sin_zeta);
	x_final_joy[4] = x_pre_final[1] + (15 * 2 * sin_zeta);
	y_final[4] = y_pre_final[1] + (20 * 2 * sin_zeta);
	x_final_joy[5] = x_pre_final[2] + (15 * 2 * sin_zeta);
	y_final[5] = y_pre_final[2] + (20 * 2 * sin_zeta);
//Third Row
	x_final_joy[6] = x_pre_final[0] + (15 * 3 * sin_zeta);
	y_final[6] = y_pre_final[0] + (20 * 3 * sin_zeta);
	x_final_joy[7] = x_pre_final[1] + (15 * 3 * sin_zeta);
	y_final[7] = y_pre_final[1] + (20 * 3 * sin_zeta);
	x_final_joy[8] = x_pre_final[2] + (15 * 3 * sin_zeta);
	y_final[8] = y_pre_final[2] + (20 * 3 * sin_zeta);
}

void JoyStickControl() {

	read_pos();

	HAL_GPIO_WritePin(JoyStick_SS_PIN_GPIO_Port, JoyStick_SS_PIN_Pin, 0);
	HAL_SPI_TransmitReceive(&hspi3, TX, RX, 10, 30);
	HAL_GPIO_WritePin(JoyStick_SS_PIN_GPIO_Port, JoyStick_SS_PIN_Pin, 1);

	if (RX[3] == 0xFE && RX_last == 0xFF) { //Select Speed Button
		if (state_motor == 1) {
			state_motor = 0;
		} else if (state_motor == 0) {
			state_motor = 1;
		}
	} else if (RX[4] == 0xBF && button_last == 0xFF) { //X Button

		if (y_count >= 4) {
			motor(0, 0);
		} else {
			y_count += 1;
		}
		y_c[y_count] = PosY;

	} else if (RX[4] == 0xEF && button_last == 0xFF) {
		workState = 1;
		State = INIT_HOMING;
	}

//motor speed Select
	switch (state_motor) {
	case 0:
		if (RX[3] == 0xFF) //Not be push
			motor(0, 1);
		else if (RX[3] == 0xEF) //UP
			motor(fast, -1);
		else if (RX[3] == 0xBF) //Down
			motor(fast, 1);
		break;
	case 1:
		if (RX[3] == 0xFF) //Not be push
			motor(0, 1);
		else if (RX[3] == 0xEF) //UP
			motor(slow, -1);
		else if (RX[3] == 0xBF) //Down
			motor(slow, 1);
		break;
	}

//X-axis
//		else if (RX[3] == 0x7F) //Left
//			printf("Left \r\n");
//		else if (RX[3] == 0xDF) //Right
//			printf("Right \r\n");
	RX_last = RX[3];
	button_last = RX[4];

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
			motor(Max_Counter_PWM * 0.25, -1);
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
			pos_f = position_test[position_index];
			State_PID = 2;
			state_homing = 0;
			EndEffector_Event(Run_Mode);
			State = IDLE;
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
		if (State == PID_STATE || State == CALIBRATE) {
			Dutyfeedback = 0;
			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
			state_IT = 1;
			State = EMERGENCY_LIMIT;
		}
	}

	if (GPIO_Pin == Photoelectric_sensor_3_Pin) {
		if (State == PID_STATE || State == CALIBRATE) {
			Dutyfeedback = 0;
			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
			state_IT = 1;
			State = EMERGENCY_LIMIT;
		}
	}

//	if (GPIO_Pin == Emergency_Pin) {
//		Dutyfeedback = 0;
//		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
//		state_IT = 1;
//		State = EMERGENCY_LIMIT;
//	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if (htim == &htim5) {
		_micros += UINT32_MAX;
	}
}

uint64_t micros() {
	return __HAL_TIM_GET_COUNTER(&htim5) + _micros;
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
