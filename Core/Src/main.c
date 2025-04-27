/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 STMicroelectronics.
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
#include "drive_model.h"
#include "h_bridge_control.h"
#include "chassis_param.h"
#include <math.h>
#include <stdlib.h>
#include <stdio.h>    // for sscanf, sprintf
#include <string.h>   // for strlen

extern UART_HandleTypeDef huart4;
#define RX_BUF_SIZE 64
char rxBuffer[RX_BUF_SIZE];
uint8_t rxChar;
uint8_t rxIndex = 0;
uint8_t msgReceived = 0;




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
RTC_HandleTypeDef hrtc;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim8;
TIM_HandleTypeDef htim16;

UART_HandleTypeDef huart4;
UART_HandleTypeDef huart5;
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;

PCD_HandleTypeDef hpcd_USB_FS;

/* USER CODE BEGIN PV */
// szj
uint8_t rx_byte;
//char message_hello[] = "Hello world";
uint8_t hello_world[11] = "Hello world";
uint8_t rx_data_11[11];  ////////
uint8_t rx_data[8];

uint8_t dir1[5]; // 4 bits + null terminator
uint8_t v1[5];   // 4 bits + null terminator

char temp_str[100];//// Print v and w

char rx_buffer[8];
uint8_t transfer_cplt;
volatile uint8_t lcd_clear_flag = 0;


char szj_buf[9];
char msg[20];  // Buffer for formatted message


// At board cart:
static float v_new = 0;
static float omega = 0;
char v_buffer[100], omega_buffer[100];

// lzr
const float EPSILON = 0.001;
const float V_DEAD_BAND = 0.05;
const float W_DEAD_BAND = 0.1;
const float V_MAX = 0.5;
const float W_MAX = 3.0;
const float W_AMP = 0.5;
const int LB = 0;
const int RB = 1;
const int LF = 2;
const int RF = 3;
const int FORWARD = 1;
const int BACKWARD = 2;
const int BREAK = 3;

const int TIM2_PRD = TIM2_FREQ / PWM_FREQ - 1;
const int TEST_MODE = 1;
const int PWM_MIN = 600;

static volatile int32_t counter = 0;
static volatile uint32_t last_interrupt_time = 0;

static volatile int duty_cycle_lb = 0;
static volatile int duty_cycle_rb = 0;
static volatile int duty_cycle_lf = 0;
static volatile int duty_cycle_rf = 0;

static volatile uint32_t lb_count_last = 0;
static volatile uint32_t rb_count_last = 0;
static volatile uint32_t lf_count_last = 0;
static volatile uint32_t rf_count_last = 0;

static volatile uint32_t lb_count_current = 0;
static volatile uint32_t rb_count_current = 0;
static volatile uint32_t lf_count_current = 0;
static volatile uint32_t rf_count_current = 0;

static volatile int32_t lb_count_diff = 0;
static volatile int32_t rb_count_diff = 0;
static volatile int32_t lf_count_diff = 0;
static volatile int32_t rf_count_diff = 0;

static volatile float lb_speed_real = 0.0;
static volatile float rb_speed_real = 0.0;
static volatile float lf_speed_real = 0.0;
static volatile float rf_speed_real = 0.0;

static volatile float lb_angle_diff = 0.0;
static volatile float rb_angle_diff = 0.0;
static volatile float lf_angle_diff = 0.0;
static volatile float rf_angle_diff = 0.0;

static volatile float lb_speed_ideal = 0.0;
static volatile float rb_speed_ideal = 0.0;
static volatile float lf_speed_ideal = 0.0;
static volatile float rf_speed_ideal = 0.0;

static volatile float lb_integral = 0.0;
static volatile float lb_previous_err = 0.0;
static volatile float rb_integral = 0.0;
static volatile float rb_previous_err = 0.0;
static volatile float lf_integral = 0.0;
static volatile float lf_previous_err = 0.0;
static volatile float rf_integral = 0.0;
static volatile float rf_previous_err = 0.0;

static volatile float v_desired = 0.0;
static volatile float w_desired = 0.0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_RTC_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_USB_PCD_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM8_Init(void);
static void MX_TIM16_Init(void);
static void MX_UART4_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_UART5_Init(void);
/* USER CODE BEGIN PFP */
void log_to_uart(const char *msg);
void compute_lb_real_speed(float time_gap);
int pwm_mapping(int duty_cycle);
void reset_pid(void);
void motor_control(int motor, int control, float v_desired);
void set_direction(int motor, int direction);
float compute_real_speed(float time_gap, int motor);
void compute_control(float time_gap);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


int parseVelocityData(const char* buffer, float* linear_vel, float* angular_vel)
{
    return sscanf(buffer, "%f,%f", linear_vel, angular_vel) == 2;
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

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
  MX_RTC_Init();
  MX_USART3_UART_Init();
  MX_USB_PCD_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM8_Init();
  MX_TIM16_Init();
  MX_UART4_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_UART5_Init();
  /* USER CODE BEGIN 2 */
	HAL_TIM_Base_Start_IT(&htim16);
	HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL);
	HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
	HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);
	HAL_TIM_Encoder_Start(&htim8, TIM_CHANNEL_ALL);

	HAL_UART_Receive_IT(&huart4, rx_data_11, 11);  // Give this back /////////////// UART 4
	HAL_UART_Receive_IT(&huart5, rx_data_11, 11);  // Use this UART 5
	//HAL_UART_Receive_IT(&huart1, rx_data_11, 11);

	//start PWM
//	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0);
//	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 0);
	set_direction(LB, BREAK);
	set_direction(RB, BREAK);
	set_direction(LF, BREAK);
	set_direction(RF, BREAK);
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0);
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 0);
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 0);
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, 0);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

	log_to_uart("Hello\n\r");
	char str[30];
//	sprintf(str, "%d", HAL_RCC_GetPCLK1Freq());
//	log_to_uart(str);

	lb_count_last = __HAL_TIM_GET_COUNTER(&htim3);
	rb_count_last = __HAL_TIM_GET_COUNTER(&htim4);
	lf_count_last = __HAL_TIM_GET_COUNTER(&htim1);
	rf_count_last = __HAL_TIM_GET_COUNTER(&htim8);

//	HAL_Delay(1000);
//	v_desired = 0.22;
//	w_desired = 0;
//	reset_pid();


	while (1) {
//		HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_0);
//		HAL_Delay(1000);


//		  HAL_UART_Transmit(&huart5, &hello_world, 11, 10);  // Timeout = 10ms
//		  HAL_Delay(2000); // 1000ms = 1s

		  /* Shift String for testing */
//		  int i;
//		  uint8_t temp = hello_world[10]; // Store last character
//		  for (i = 10; i > 0; i--) {
//		      hello_world[i] = hello_world[i - 1];
//		  }
//		  hello_world[0] = temp;


				/*v_desired = 1;
				reset_pid();
				HAL_Delay(1000);

				v_desired = 3;
				w_desired = 0.2;
				reset_pid();
				HAL_Delay(1000);

				v_desired = -1;
				w_desired = 0.0;
				reset_pid();
				HAL_Delay(1000);
				v_desired = -0.5;
				reset_pid();
				HAL_Delay(1000);*/
//				w_desired = 1.0;
//				reset_pid();
//				HAL_Delay(2000);



//		counter = (int16_t)__HAL_TIM_GET_COUNTER(&htim3);
//		angle = (float) counter / 1320.0 * 360.0;
//		sprintf(str, "count: %ld, angle: %.2f\r\n", counter, angle);
//		log_to_uart(str);
//		HAL_Delay(100);


//		v_desired = 0.0;
//		reset_pid();
//		HAL_Delay(2000);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	}
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB|RCC_PERIPHCLK_USART1
                              |RCC_PERIPHCLK_USART2|RCC_PERIPHCLK_USART3
                              |RCC_PERIPHCLK_UART4|RCC_PERIPHCLK_UART5
                              |RCC_PERIPHCLK_RTC|RCC_PERIPHCLK_TIM1
                              |RCC_PERIPHCLK_TIM16|RCC_PERIPHCLK_TIM8
                              |RCC_PERIPHCLK_TIM2|RCC_PERIPHCLK_TIM34;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.Usart3ClockSelection = RCC_USART3CLKSOURCE_PCLK1;
  PeriphClkInit.Uart4ClockSelection = RCC_UART4CLKSOURCE_PCLK1;
  PeriphClkInit.Uart5ClockSelection = RCC_UART5CLKSOURCE_PCLK1;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
  PeriphClkInit.USBClockSelection = RCC_USBCLKSOURCE_PLL_DIV1_5;
  PeriphClkInit.Tim1ClockSelection = RCC_TIM1CLK_HCLK;
  PeriphClkInit.Tim16ClockSelection = RCC_TIM16CLK_HCLK;
  PeriphClkInit.Tim8ClockSelection = RCC_TIM8CLK_HCLK;
  PeriphClkInit.Tim2ClockSelection = RCC_TIM2CLK_HCLK;
  PeriphClkInit.Tim34ClockSelection = RCC_TIM34CLK_HCLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */

  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = APB1_FREQ / TIM2_FREQ - 1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = TIM2_FREQ / PWM_FREQ - 1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim4, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief TIM8 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM8_Init(void)
{

  /* USER CODE BEGIN TIM8_Init 0 */

  /* USER CODE END TIM8_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM8_Init 1 */

  /* USER CODE END TIM8_Init 1 */
  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 0;
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = 65535;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 0;
  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim8, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM8_Init 2 */

  /* USER CODE END TIM8_Init 2 */

}

/**
  * @brief TIM16 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM16_Init(void)
{

  /* USER CODE BEGIN TIM16_Init 0 */

  /* USER CODE END TIM16_Init 0 */

  /* USER CODE BEGIN TIM16_Init 1 */

  /* USER CODE END TIM16_Init 1 */
  htim16.Instance = TIM16;
  htim16.Init.Prescaler = APB2_FREQ / TIM16_FREQ - 1;
  htim16.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim16.Init.Period = TIM16_FREQ / ENCODER_SAMPLE_FREQ -1;
  htim16.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim16.Init.RepetitionCounter = 0;
  htim16.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim16) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM16_Init 2 */

  /* USER CODE END TIM16_Init 2 */

}

/**
  * @brief UART4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART4_Init(void)
{

  /* USER CODE BEGIN UART4_Init 0 */

  /* USER CODE END UART4_Init 0 */

  /* USER CODE BEGIN UART4_Init 1 */

  /* USER CODE END UART4_Init 1 */
  huart4.Instance = UART4;
  huart4.Init.BaudRate = 115200;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  huart4.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart4.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART4_Init 2 */

  /* USER CODE END UART4_Init 2 */

}

/**
  * @brief UART5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART5_Init(void)
{

  /* USER CODE BEGIN UART5_Init 0 */

  /* USER CODE END UART5_Init 0 */

  /* USER CODE BEGIN UART5_Init 1 */

  /* USER CODE END UART5_Init 1 */
  huart5.Instance = UART5;
  huart5.Init.BaudRate = 115200;
  huart5.Init.WordLength = UART_WORDLENGTH_8B;
  huart5.Init.StopBits = UART_STOPBITS_1;
  huart5.Init.Parity = UART_PARITY_NONE;
  huart5.Init.Mode = UART_MODE_TX_RX;
  huart5.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart5.Init.OverSampling = UART_OVERSAMPLING_16;
  huart5.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart5.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart5) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART5_Init 2 */

  /* USER CODE END UART5_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 230400;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief USB Initialization Function
  * @param None
  * @retval None
  */
static void MX_USB_PCD_Init(void)
{

  /* USER CODE BEGIN USB_Init 0 */

  /* USER CODE END USB_Init 0 */

  /* USER CODE BEGIN USB_Init 1 */

  /* USER CODE END USB_Init 1 */
  hpcd_USB_FS.Instance = USB;
  hpcd_USB_FS.Init.dev_endpoints = 8;
  hpcd_USB_FS.Init.speed = PCD_SPEED_FULL;
  hpcd_USB_FS.Init.phy_itface = PCD_PHY_EMBEDDED;
  hpcd_USB_FS.Init.low_power_enable = DISABLE;
  hpcd_USB_FS.Init.lpm_enable = DISABLE;
  hpcd_USB_FS.Init.battery_charging_enable = DISABLE;
  if (HAL_PCD_Init(&hpcd_USB_FS) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USB_Init 2 */

  /* USER CODE END USB_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_4|GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LD1_Pin|LD3_Pin|GPIO_PIN_8|GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOF, GPIO_PIN_13|GPIO_PIN_14, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOG, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|USB_PowerSwitchOn_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8|GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pins : PE4 PE5 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : USER_Btn_Pin */
  GPIO_InitStruct.Pin = USER_Btn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USER_Btn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD1_Pin LD3_Pin PB8 PB9 */
  GPIO_InitStruct.Pin = LD1_Pin|LD3_Pin|GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PF13 PF14 */
  GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pins : PG1 PG2 PG3 USB_PowerSwitchOn_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|USB_PowerSwitchOn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_OverCurrent_Pin */
  GPIO_InitStruct.Pin = USB_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USB_OverCurrent_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PC8 PC9 */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
//wo de
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	//wo de:
    if (huart->Instance == UART5)
    {
//		reset_pid();

		HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_0);
		//HAL_UART_Transmit(&huart3, rx_data_11, 11, 10); // Send received byte to PC (UART3 -> COM port)




		/* Process receive buffer */
		// Map buffer in size 11 to size 8 to process
		for (int i = 0; i < 8; i++) {
			rx_data[i] = rx_data_11[i];
		}

		// Copy upper 4 bits (index 0 to 3) to dir1
		for (int i = 0; i < 4; i++) {
			dir1[i] = rx_data[i]; // '0' or '1'
		}
		dir1[4] = '\0'; // Null terminator

		// Copy lower 4 bits (index 4 to 7) to v1
		for (int i = 0; i < 4; i++) {
			v1[i] = rx_data[i + 4]; // '0' or '1'
		}
		v1[4] = '\0'; // Null terminator

		HAL_UART_Transmit(&huart3, (uint8_t*) "Buffer:", 7, 10);
        HAL_UART_Transmit(&huart3, rx_data, 8, 10); ////
		HAL_UART_Transmit(&huart3, (uint8_t*) "\r\n", 3, 10);

//		HAL_UART_Transmit(&huart3, (uint8_t*) "dir1:", 5, 10);
//		HAL_UART_Transmit(&huart3,  dir1, strlen(dir1), 10);
//		HAL_UART_Transmit(&huart3, (uint8_t*) "\r\n", 2, 10);
//
//		HAL_UART_Transmit(&huart3, (uint8_t*) "v1:", 3, 10);
//		HAL_UART_Transmit(&huart3,  v1, strlen(v1), 10);
//		HAL_UART_Transmit(&huart3, (uint8_t*) "\r\n", 2, 10);

		// float v without sign
		if (strcmp(v1, "0000") == 0) {
			v_new = 0.0;

//			HAL_UART_Transmit(&huart3, (uint8_t*) "v1=0000:", 8, 10);
//			HAL_UART_Transmit(&huart3, (uint8_t*) "\r\n", 2, 10);
		} else if (strcmp(v1, "0001") == 0) {
			v_new = 0.1;
//
//			HAL_UART_Transmit(&huart3, (uint8_t*) "v1=0001:", 8, 10);
//			HAL_UART_Transmit(&huart3, (uint8_t*) "\r\n", 2, 10);
		} else if (strcmp(v1, "0010") == 0) {
			v_new = 0.2;

//			HAL_UART_Transmit(&huart3, (uint8_t*) "v1=0010:", 8, 10);
//			HAL_UART_Transmit(&huart3, (uint8_t*) "\r\n", 2, 10);
		} else if (strcmp(v1, "0011") == 0) {
			v_new = 0.3;

//			HAL_UART_Transmit(&huart3, (uint8_t*) "v1=0011:", 8, 10);
//			HAL_UART_Transmit(&huart3, (uint8_t*) "\r\n", 2, 10);
		} else if (strcmp(v1, "0100") == 0) {
			v_new = 0.4;

//			HAL_UART_Transmit(&huart3, (uint8_t*) "v1=0100:", 8, 10);
//			HAL_UART_Transmit(&huart3, (uint8_t*) "\r\n", 2, 10);
		} else {
//			HAL_UART_Transmit(&huart3, (uint8_t*) "Error", 5, 10);
//			HAL_UART_Transmit(&huart3, (uint8_t*) "\r\n", 2, 10);
		}

		// Adding sign to float v
		// float w
		if (strcmp(dir1, "0001") == 0) {
			omega = 1 * v_new * 3;
			v_new = 1 * v_new;

//			HAL_UART_Transmit(&huart3, (uint8_t*) "dir1=0001:", 10, 10);
//			HAL_UART_Transmit(&huart3, (uint8_t*) "\r\n", 2, 10);
		} else if (strcmp(dir1, "0010") == 0) {
			omega = (-1) * v_new * 3;
			v_new = 1 * v_new;
//
//			HAL_UART_Transmit(&huart3, (uint8_t*) "dir1=0010:", 10, 10);
//			HAL_UART_Transmit(&huart3, (uint8_t*) "\r\n", 2, 10);
		} else if (strcmp(dir1, "0011") == 0) {
			omega = 0;
			v_new = (-1) * v_new;

//			HAL_UART_Transmit(&huart3, (uint8_t*) "dir1=0011:", 10, 10);
//			HAL_UART_Transmit(&huart3, (uint8_t*) "\r\n", 2, 10);
		} else if (strcmp(dir1, "0000") == 0) {
			omega = 0;
			v_new = 1 * v_new;

//			HAL_UART_Transmit(&huart3, (uint8_t*) "dir1=0000:", 10, 10);
//			HAL_UART_Transmit(&huart3, (uint8_t*) "\r\n", 2, 10);
		} else {
//			HAL_UART_Transmit(&huart3, (uint8_t*) "Error", 5, 10);
//			HAL_UART_Transmit(&huart3, (uint8_t*) "\r\n", 2, 10);
		}

		v_desired = v_new;
		w_desired = omega;
		sprintf(temp_str, "v: %+6.2f | w: %+6.2fd\r\n", v_desired, w_desired);
		log_to_uart(temp_str);

//		HAL_UART_Transmit(&huart3, (uint8_t*) "\n\n", 2, 10);



		// Flag
		HAL_UART_Receive_IT(&huart5, rx_data_11, 11);
    }

//    // lhz:
//	if ((huart->Instance == UART4)  && 0){
//
//		if (rxChar == '\n' || rxChar == '\r') {
//			if (rxIndex > 0) {
//				rxBuffer[rxIndex] = '\0';
//				msgReceived = 1;
//				float v = 0.0f, w = 0.0f;
//				if (parseVelocityData(rxBuffer, &v, &w)) {
//					//HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_0);
//					v_desired = v;
//					if (w < -1 * W_DEAD_BAND) {
//						w_desired = w - W_AMP;
//					} else if (w > W_DEAD_BAND) {
//						w_desired = w + W_AMP;
//					} else {
//						w_desired = w;
//					}
//				}
//				msgReceived = 0;
//				rxIndex = 0;
//			}
//		} else {
//			if (rxIndex < RX_BUF_SIZE - 1) {
//				rxBuffer[rxIndex++] = rxChar;
//			} else {
//				rxIndex = 0;  // overflow, reset
//			}
//		}
//	}


	if (huart->Instance == UART4) {

		HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_0);
		//HAL_UART_Transmit(&huart3, rx_data_11, 11, 10); // Send received byte to PC (UART3 -> COM port)




		/* Process receive buffer */
		// Map buffer in size 11 to size 8 to process
		for (int i = 0; i < 8; i++) {
			rx_data[i] = rx_data_11[i];
		}

		// Copy upper 4 bits (index 0 to 3) to dir1
		for (int i = 0; i < 4; i++) {
			dir1[i] = rx_data[i]; // '0' or '1'
		}
		dir1[4] = '\0'; // Null terminator

		// Copy lower 4 bits (index 4 to 7) to v1
		for (int i = 0; i < 4; i++) {
			v1[i] = rx_data[i + 4]; // '0' or '1'
		}
		v1[4] = '\0'; // Null terminator

		HAL_UART_Transmit(&huart3, (uint8_t*) "Buffer:", 7, 10);
        HAL_UART_Transmit(&huart3, rx_data, 8, 10); ////
		HAL_UART_Transmit(&huart3, (uint8_t*) "\r\n", 3, 10);

//		HAL_UART_Transmit(&huart3, (uint8_t*) "dir1:", 5, 10);
//		HAL_UART_Transmit(&huart3,  dir1, strlen(dir1), 10);
//		HAL_UART_Transmit(&huart3, (uint8_t*) "\r\n", 2, 10);
//
//		HAL_UART_Transmit(&huart3, (uint8_t*) "v1:", 3, 10);
//		HAL_UART_Transmit(&huart3,  v1, strlen(v1), 10);
//		HAL_UART_Transmit(&huart3, (uint8_t*) "\r\n", 2, 10);

		// float v without sign
		if (strcmp(v1, "0000") == 0) {
			v_new = 0.0;

			HAL_UART_Transmit(&huart3, (uint8_t*) "v1=0000:", 8, 10);
			HAL_UART_Transmit(&huart3, (uint8_t*) "\r\n", 2, 10);
		} else if (strcmp(v1, "0001") == 0) {
			v_new = 0.3;

			HAL_UART_Transmit(&huart3, (uint8_t*) "v1=0001:", 8, 10);
			HAL_UART_Transmit(&huart3, (uint8_t*) "\r\n", 2, 10);
		} else if (strcmp(v1, "0010") == 0) {
			v_new = 0.6;

			HAL_UART_Transmit(&huart3, (uint8_t*) "v1=0010:", 8, 10);
			HAL_UART_Transmit(&huart3, (uint8_t*) "\r\n", 2, 10);
		} else if (strcmp(v1, "0011") == 0) {
			v_new = 0.9;

			HAL_UART_Transmit(&huart3, (uint8_t*) "v1=0011:", 8, 10);
			HAL_UART_Transmit(&huart3, (uint8_t*) "\r\n", 2, 10);
		} else if (strcmp(v1, "0100") == 0) {
			v_new = 1.0;

			HAL_UART_Transmit(&huart3, (uint8_t*) "v1=0100:", 8, 10);
			HAL_UART_Transmit(&huart3, (uint8_t*) "\r\n", 2, 10);
		} else {
			HAL_UART_Transmit(&huart3, (uint8_t*) "Error", 5, 10);
			HAL_UART_Transmit(&huart3, (uint8_t*) "\r\n", 2, 10);
		}

		// Adding sign to float v
		// float w
		if (strcmp(dir1, "0001") == 0) {
			omega = 1 * v_new;
			v_new = 1 * v_new;

			HAL_UART_Transmit(&huart3, (uint8_t*) "dir1=0001:", 10, 10);
			HAL_UART_Transmit(&huart3, (uint8_t*) "\r\n", 2, 10);
		} else if (strcmp(dir1, "0010") == 0) {
			omega = (-1) * v_new;
			v_new = 1 * v_new;

			HAL_UART_Transmit(&huart3, (uint8_t*) "dir1=0010:", 10, 10);
			HAL_UART_Transmit(&huart3, (uint8_t*) "\r\n", 2, 10);
		} else if (strcmp(dir1, "0011") == 0) {
			omega = 0;
			v_new = 0;

			HAL_UART_Transmit(&huart3, (uint8_t*) "dir1=0011:", 10, 10);
			HAL_UART_Transmit(&huart3, (uint8_t*) "\r\n", 2, 10);
		} else if (strcmp(dir1, "0000") == 0) {
			omega = 0;
			v_new = 1 * v_new;

			HAL_UART_Transmit(&huart3, (uint8_t*) "dir1=0000:", 10, 10);
			HAL_UART_Transmit(&huart3, (uint8_t*) "\r\n", 2, 10);
		} else {
			HAL_UART_Transmit(&huart3, (uint8_t*) "Error", 5, 10);
			HAL_UART_Transmit(&huart3, (uint8_t*) "\r\n", 2, 10);
		}

		v_desired = v_new;
		w_desired = omega;

		HAL_UART_Transmit(&huart3, (uint8_t*) "\n\n", 2, 10);



		// Flag
		HAL_UART_Receive_IT(&huart4, rx_data_11, 11);

	}



    //HAL_UART_Receive_IT(&huart1, &rx_byte, 1);
	//HAL_UART_Receive_IT(&huart4, &rxChar, 1);  // re-arm
}


// LZR de
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if (htim->Instance == TIM16) {
//		HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_0);
		float time_gap = (float) (HAL_GetTick() - last_interrupt_time) / 1000.0;
		if (time_gap < EPSILON) {
			time_gap = EPSILON;
		}

		last_interrupt_time = HAL_GetTick();
		compute_control(time_gap);

//		compute_ideal_speed(v_desired, w_desired, &lb_speed_ideal,
//				&rb_speed_ideal);
//		lb_speed_real = compute_real_speed(time_gap, LB);
//		rb_speed_real = compute_real_speed(time_gap, RB);
//
//		float lb_error = fabs(lb_speed_ideal) - fabs(lb_speed_real);
//		lb_integral += lb_error * time_gap;
//		float lb_derivative = (lb_error - lb_previous_err) / time_gap;
//		lb_previous_err = lb_error;
//		const float Kp = 1000, Ki = 7500, Kd = 0.0;
//		int lb_control = (int) (Kp * lb_error + Ki * lb_integral
//				+ Kd * lb_derivative);
//
//		motor_control(LB, lb_control, lb_speed_ideal);
//		motor_control(RB, rb_control, rb_speed_ideal);
//		if(fabs(v_desired) < 0.1 && fabs(w_desired) < 0.1) {
//			lb_stop();
//			duty_cycle_lb = 0;
//			HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_0);
//		}
//		else if(lb_control / abs(lb_control) > 0) {
//			lb_forward();
//			duty_cycle_lb = lb_control;
//		}
//		else {
////			lb_backward();
////			duty_cycle_lb = abs(lb_control);
//			duty_cycle_lb = 0;
//			lb_stop();
//		}
//		if (duty_cycle_lb > TIM2_PRD) {
//			duty_cycle_lb = TIM2_PRD;
//		}
	}
}

int pwm_mapping(int duty_cycle) {
//	if(duty_cycle < PWM_MIN) {
//		return PWM_MIN;
//	}
	return (int) ((float) duty_cycle / (float) TIM2_PRD
			* (float) (TIM2_PRD - PWM_MIN) + (float) PWM_MIN);
}

void motor_control(int motor, int control, float v_desired) {
	int duty_cycle = 0;
	if (fabs(v_desired) > V_DEAD_BAND) {
		if (control > 0) {
			duty_cycle = control;
			if (v_desired > 0) {
				set_direction(motor, FORWARD);
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
			} else {
				set_direction(motor, BACKWARD);
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
			}
		} else {
			duty_cycle = 0;
			set_direction(motor, BREAK);
		}
	} else {
		set_direction(motor, BREAK);
	}

	if (duty_cycle > TIM2_PRD) {
		duty_cycle = TIM2_PRD;
	}

	duty_cycle = pwm_mapping(duty_cycle);
	if (motor == LB) {
		duty_cycle_lb = duty_cycle;
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, duty_cycle);

	}
	else if (motor == RB) {
		duty_cycle_rb = duty_cycle;
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, duty_cycle);
	}
	else if (motor == LF) {
		duty_cycle_lf = duty_cycle;
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, duty_cycle);

	}
	else if (motor == RF) {
		duty_cycle_rf = duty_cycle;
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, duty_cycle);

	}
}

void set_direction(int motor, int direction) {
	if (motor == LB){
		if (direction == FORWARD) {
			lb_forward();
		}
		else if (direction == BACKWARD) {
			lb_backward();
		}
		else if (direction == BREAK) {
			lb_stop();
		}
	}
	else if (motor == RB) {
		if (direction == FORWARD) {
			rb_forward();
		}
		else if (direction == BACKWARD) {
			rb_backward();
		}
		else if (direction == BREAK) {
			rb_stop();
		}
	}
	else if (motor == LF) {
		if (direction == FORWARD) {
			lf_forward();
		} else if (direction == BACKWARD) {
			lf_backward();
		} else if (direction == BREAK) {
			lf_stop();
		}
	}
	else if (motor == RF) {
		if (direction == FORWARD) {
			rf_forward();
		}
		else if (direction == BACKWARD) {
			rf_backward();
		}
		else if (direction == BREAK) {
			rf_stop();
		}
	}
}

void reset_pid(void) {
	lb_integral = 0.0;
	lb_previous_err = 0.0;
	rb_integral = 0.0;
	rb_previous_err = 0.0;
}

void compute_control(float time_gap) {
	int lb_control = 0;
	int rb_control = 0;
	int lf_control = 0;
	int rf_control = 0;
	const float Kp = 1000, Ki=7500.0; //Ki = 7500;
	const float Kd = 0.0;
	if (fabs(v_desired) > V_MAX) {
 		v_desired = V_MAX * (v_desired / fabs(v_desired));
 	}

	if (w_desired > W_MAX) {
		w_desired = W_MAX * (w_desired / fabs(w_desired));
	}


	compute_ideal_speed(v_desired, w_desired, &lb_speed_ideal,
			&rb_speed_ideal, &lf_speed_ideal, &rf_speed_ideal);

	// compute control for left side
	lb_speed_real = compute_real_speed(time_gap, LB);
	float lb_error = fabs(lb_speed_ideal) - fabs(lb_speed_real);
	lb_integral += lb_error * time_gap;
	float lb_derivative = (lb_error - lb_previous_err) / time_gap;
	lb_previous_err = lb_error;
	lb_control = (int) (Kp * lb_error + Ki * lb_integral
			+ Kd * lb_derivative);

	lf_speed_real = compute_real_speed(time_gap, LF);
	float lf_error = fabs(lf_speed_ideal) - fabs(lf_speed_real);
	lf_integral += lf_error * time_gap;
	float lf_derivative = (lf_error - lf_previous_err) / time_gap;
	lf_previous_err = lf_error;
	lf_control = (int) (Kp * lf_error + Ki * lf_integral
			+ Kd * lf_derivative);

	//compute control for right speed
	rb_speed_real = compute_real_speed(time_gap, RB);
	float rb_error = fabs(rb_speed_ideal) - fabs(rb_speed_real);
	rb_integral += rb_error * time_gap;
	float rb_derivative = (rb_error - rb_previous_err) / time_gap;
	rb_previous_err = rb_error;
	rb_control = (int) (Kp * rb_error + Ki * rb_integral
			+ Kd * rb_derivative);

	rf_speed_real = compute_real_speed(time_gap, RF);
	float rf_error = fabs(rf_speed_ideal) - fabs(rf_speed_real);
	rf_integral += rf_error * time_gap;
	float rf_derivative = (rf_error - rf_previous_err) / time_gap;
	rf_previous_err = rf_error;
	rf_control = (int) (Kp * rf_error + Ki * rf_integral
			+ Kd * rf_derivative);

	motor_control(LB, lb_control, lb_speed_ideal);
	motor_control(LF, lf_control, lf_speed_ideal);
	motor_control(RB, rb_control, rb_speed_ideal);
	motor_control(RF, rf_control, rf_speed_ideal);

	char temp_str[200];

	lb_count_current = __HAL_TIM_GET_COUNTER(&htim3);
	rb_count_current = __HAL_TIM_GET_COUNTER(&htim4);

		sprintf(temp_str,
				"[Δt: %6.2f s] v: %+6.2f | w: %+6.2f | err: %+6.2f | int: %+6.2f | der: %+6.2f | LB Ideal: %+6.2f | LB Real: %+6.2f | LB Control: %04d | LB Duty Cycle: %03d\r\n",
				time_gap, v_desired, w_desired, lb_error, lb_integral,
				lb_derivative, lb_speed_ideal, lb_speed_real, lb_control,
				duty_cycle_lb);
//		log_to_uart(temp_str);
//
		sprintf(temp_str,
				"[Δt: %6.2f s] v: %+6.2f | w: %+6.2f | err: %+6.2f | int: %+6.2f | der: %+6.2f | RB Ideal: %+6.2f | RB Real: %+6.2f | RB Control: %04d | RB Duty Cycle: %03d\r\n",
				time_gap, v_desired, w_desired, rb_error, rb_integral,
				rb_derivative, rb_speed_ideal, rb_speed_real, rb_control,
				duty_cycle_rb);
//		log_to_uart(temp_str);

//		sprintf(temp_str,
//				"[Δt: %6.2f s] v: %+6.2f | w: %+6.2f | err: %+6.2f | int: %+6.2f | der: %+6.2f | LF Ideal: %+6.2f | LF Real: %+6.2f | LF Control: %04d | LF Duty Cycle: %03d\r\n",
//				time_gap, v_desired, w_desired, lb_error, lf_integral,
//				lf_derivative, lf_speed_ideal, lf_speed_real, lf_control,
//				duty_cycle_lf);
//		log_to_uart(temp_str);
//
//		sprintf(temp_str,
//				"[Δt: %6.2f s] v: %+6.2f | w: %+6.2f | err: %+6.2f | int: %+6.2f | der: %+6.2f | RF Ideal: %+6.2f | RF Real: %+6.2f | RF Control: %04d | RF Duty Cycle: %03d\r\n",
//				time_gap, v_desired, w_desired, rf_error, rf_integral,
//				rf_derivative, rf_speed_ideal, rf_speed_real, rf_control,
//				duty_cycle_rf);
//		log_to_uart(temp_str);
//
		sprintf(temp_str, "\r\n");
		log_to_uart(temp_str);
	}


float compute_real_speed(float time_gap, int motor) {
//	char temp_str[50];
	int16_t count_diff = 0;
	float speed_real = 0.0;
	if (motor == LB) {
		count_diff = (int16_t) (__HAL_TIM_GET_COUNTER(&htim3) - lb_count_last);
		speed_real = -1 * count_diff / 1320.0 * 2 * M_PI * MOTOR_RADIUS / time_gap;
		lb_count_last = __HAL_TIM_GET_COUNTER(&htim3);
//		sprintf(temp_str, "LB count diff: %d, speed: %.4f\r\n", count_diff, speed_real);
//		log_to_uart(temp_str);
	}
	else if (motor == RB) {
		count_diff = (int16_t) (__HAL_TIM_GET_COUNTER(&htim4) - rb_count_last);
		speed_real = count_diff / 1320.0 * 2 * M_PI * MOTOR_RADIUS / time_gap;
		rb_count_last = __HAL_TIM_GET_COUNTER(&htim4);
//		sprintf(temp_str, "RB count diff: %d, speed: %.4f\r\n", count_diff, speed_real);
//		log_to_uart(temp_str);
	}
	else if (motor == LF) {
		count_diff = (int16_t) (__HAL_TIM_GET_COUNTER(&htim1) - lf_count_last);
		speed_real = -1 * count_diff / 1320.0 * 2 * M_PI * MOTOR_RADIUS / time_gap;
		lf_count_last = __HAL_TIM_GET_COUNTER(&htim1);
	}
	else if (motor == RF) {
		count_diff = (int16_t) (__HAL_TIM_GET_COUNTER(&htim8) - rf_count_last);
		speed_real = count_diff / 1320.0 * 2 * M_PI * MOTOR_RADIUS / time_gap;
		rf_count_last = __HAL_TIM_GET_COUNTER(&htim8);
	}
	return speed_real;
}

void log_to_uart(const char *msg) {
	HAL_UART_Transmit(&huart3, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
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
