/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include <math.h>
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
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

/* USER CODE BEGIN PV */
const double PI = 3.1415926;
const double dt = 0.001;
volatile double duty = 0;
uint32_t t1;
uint32_t t2;
uint32_t flag_start;
uint32_t debug;
volatile uint32_t time_lns;
volatile uint32_t time_lms;
volatile uint32_t time_1s;
volatile uint32_t Tim;
volatile double property;

volatile double deg = 0;
volatile uint32_t cnt_1 = 0;
volatile uint32_t flag_1 = 0;
volatile uint32_t cnt_2 = 0;
volatile uint32_t flag_2 = 0;

double int_error;
double diff_error;
double error_old;
double error_current;
int timer_entire;
int timer_start;
double targetval;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);
double MotorPIDControl();
double MotorErrorFunction(double target, double output);
double MotorIntegralErrorFunction();
double MotorDiffErrorFunction();
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_TIM2_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
	
	HAL_TIM_PWM_Start (&htim2, TIM_CHANNEL_1);
	duty = 0;
	TIM2->CCR1 = duty;
	if ( HAL_TIM_Base_Start_IT(&htim3)!= HAL_OK )
	{
		Error_Handler();
	}
	
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);
	flag_start = 1;
	time_lns = 0;
	time_lms = 0;
	time_1s = 0;
	deg = 0;
	duty = 0;
	int_error = 0;
	diff_error = 0;
	error_old = 0;
	error_current = 0;
	timer_entire = 0;
	timer_start = 0;
	if(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_8) == GPIO_PIN_SET){
		flag_2 = 1;
	}
	else{
		flag_2 = 0;
	}
	if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_10) == GPIO_PIN_SET){
		flag_1 = 1;
	}
	else{
		flag_1 = 0;
	}
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 63;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 63;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4|GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC2 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PC6 */
  GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PC8 */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PA10 */
  GPIO_InitStruct.Pin = GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB4 PB5 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

//------------------PID
double MotorPIDControl(){
	//read property
	//read current position
	double K_p = 0.85;
	double K_i = 0.015;
	double K_d = 0.08;
	//double K_p = 0.375/1.3;
	//double K_i = 0.194/1.4 * dt;
	//double K_d = 0.001775 * 5 / dt;
	return (K_p * MotorErrorFunction(property, deg) + K_i * MotorIntegralErrorFunction() + K_d * MotorDiffErrorFunction());
}
double MotorErrorFunction(double target, double output){
	//property - current
	//store old value to calculate differential
	error_old = error_current;
	error_current = target - output;
	return (error_current);
}
double MotorIntegralErrorFunction(){
	//Euler integral?
	if( (time_lms % 500) == 0 && (time_lms / 500)%2 == 1)
		int_error = 0;
	int_error += error_current;
	if (int_error < 0)
		return - int_error;
	else
		return int_error;
}
double MotorDiffErrorFunction(){
	//differential
	diff_error = error_current - error_old;
	return diff_error;
}
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	t1=1000;
	t2=1;
  if(htim->Instance == TIM3) 
	{
		time_lns ++;
	}
	if( (time_lns % t2) == 0 ) time_lms++; 
	if( (time_lms % t1) == 0 ) time_1s++; 
	

	if(flag_start == 0)
      {
         if(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_6) == GPIO_PIN_SET)
         {
            flag_start = 1;
						time_lns = 0;
						time_lms = 0;
						time_1s = 0;
            deg = 0;
						duty = 0;
						int_error = 0;
						diff_error = 0;
						error_old = 0;
						error_current = 0;
						timer_entire = 0;
						timer_start = 0;
						if(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_8) == GPIO_PIN_SET){
							flag_1 = 0;
						}
						else{
							flag_1 = 1;
						}
						if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_10) == GPIO_PIN_SET){
							flag_2 = 0;
						}
						else{
							flag_2 = 1;
						}
         }
      }
		else {
			//---------------------------------property
			if(time_1s == 1) property =180;
			if(time_1s == 2) property =0;
			if(time_1s == 3) property =-180;
			if(time_1s == 4) property =0;
			if(time_1s == 5) property =360;
			if(time_1s == 6) property =0;
			if(time_1s == 7) property =-360;
			if(time_1s == 8) property =0;
			if(time_1s == 9) property =120;
			if(time_1s == 10) property =240;
			if(time_1s == 11) property =360;
			if(time_1s == 12) property =240;
			if(time_1s == 13) property =120;
			if(time_1s == 14) property =0;
			if(time_1s == 15) property =-120;
			if(time_1s == 16) property =-240;
			if(time_1s == 17) property =-360;
			if(time_1s == 18) property =-240;
			if(time_1s == 19) property =-120;
			if(time_1s == 20) property =0;
			if(time_lns >= 21*t1*t2 && time_lns < 27*t1*t2){
				Tim = time_lns - 21*t1*t2;
				property = -120*sin(2*PI*Tim/(6*t1*t2));
			}
			if(time_lns >= 27*t1*t2 && time_lns < 33*t1*t2){
				Tim = time_lns - 27*t1*t2;
				property = -240*sin(2*PI*Tim/(6*t1*t2));
			}
			if(time_lns >= 33*t1*t2 && time_lns < 39*t1*t2){
				Tim = time_lns - 33*t1*t2;
				property = -360*sin(2*PI*Tim/(6*t1*t2));
			}
			//------------------------------PID
			duty = MotorPIDControl() / dt;
			if (duty > 999) {
				duty = 999;
			}
			else if (duty < -999) {
				duty = -999;
			}
			// 듀티 값에 따라 PWM 출력 설정
			if (duty < 0) {
				TIM2->CCR1 = -duty + 500;
			}
			else {
				TIM2->CCR1 = duty + 500;
			}
			if(deg > property){
				debug = 1;
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);
			}
			else if(deg == property){
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);
			}
			else if(deg < property){
				debug = -1;
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);
			}
		}
}
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	
	 if (GPIO_Pin == GPIO_PIN_10) { 
				cnt_1++;
//deg = deg + 36.0f/(4*13);
			if(flag_1 == 0) //flag_1 : 0 -> 1
			{
				flag_1 = 1;
				if(flag_2 == 0) // (0 , 0) -> (1 , 0)
				{
					deg = deg + 36.0f/(4*13) ;
				}
				else // (0 , 1) -> (1 , 1)
				{
					deg = deg - 36.0f/(4*13) ;
				}
			}
			else //flag_1 : 1 -> 0
			{ 
				flag_1 = 0;
				if(flag_2 == 0) // (1 , 0) -> (0 , 0)
				{
					deg = deg - 36.0f/(4*13) ;
				}
				else // (1 , 1) -> (0 , 1)
				{
					deg = deg + 36.0f/(4*13) ;
				}
				
			}
		}
		if (GPIO_Pin == GPIO_PIN_8)
		{
			 cnt_2++;
			if(flag_2 == 0) //flag_2 : 0 -> 1
			{
				flag_2 = 1;
				if(flag_1 == 0) // (0 , 0) -> (0 , 1)
				{
					deg = deg - 36.0f/(4*13) ;
				}
				else // (1 , 0) -> (1 , 1)
				{
					deg = deg + 36.0f/(4*13) ;
				}
			}
			else //flag_2 : 1 -> 0
			{ 
				flag_2 = 0;
				if(flag_1 == 0) // (0 , 1) -> (0 , 0)
				{
					deg = deg + 36.0f/(4*13) ;
				}
				else // (1 , 1) -> (1 , 0)
				{
					deg = deg - 36.0f/(4*13) ;
				}
				
			}
		}
   
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
  while (1)
  {
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
