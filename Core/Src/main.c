/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim8;

UART_HandleTypeDef huart4;
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM8_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_UART4_Init(void);
/* USER CODE BEGIN PFP */
int step=1;
int X=0;
int Y=0;
int Z=0;
int turn=0;
int Xflag=1;
int Yflag=1;
int Zflag=1;
int turnflag=1;
uint8_t YY[10];
uint8_t YY1[]={0XBA,0XEC,0XC9,0XAB};//R
uint8_t YY2[]={0XC0,0XB6,0XC9,0XAB};//B
uint8_t YY3[]={0XC2,0XCC,0XC9,0XAB};//G

uint8_t ADC_Value;
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void adjust(int n)
{
	if(n==1)
	{
		__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,20);
		__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,0);

		__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_4,23);
		__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3,0);

		__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_2,10);//22
		__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_1,0);

		__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_3,10);
		__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_4,0);
		HAL_Delay(300);
		  __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,0);
		__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,10);

		__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_4,0);
		__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3,10);

		__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_2,0);
		__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_1,19);

		__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_3,0);
		__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_4,26);
		HAL_Delay(300);

	}
}
void move(int n)
{
	if(n==1)//直行
	{
		if(HAL_GPIO_ReadPin(GPIOG,GPIO_PIN_11)==1&&HAL_GPIO_ReadPin(GPIOG,GPIO_PIN_9)==1)
		{
			__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,20);
			__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,0);

			__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_4,20);
			__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3,0);

			__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_2,24);//22
			__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_1,0);

			__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_3,18);
			__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_4,0);
		}
		if(HAL_GPIO_ReadPin(GPIOG,GPIO_PIN_11)==1&&HAL_GPIO_ReadPin(GPIOG,GPIO_PIN_9)==0)
		{
			__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,20);
			__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,0);

			__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_4,20);
			__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3,0);

			__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_2,40);
			__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_1,0);

			__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_3,18);
			__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_4,0);
		}
		if(HAL_GPIO_ReadPin(GPIOG,GPIO_PIN_11)==0&&HAL_GPIO_ReadPin(GPIOG,GPIO_PIN_9)==1)
		{
			__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,20);
			__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,0);

			__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_4,20);
			__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3,0);

			__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_2,22);
			__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_1,0);

			__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_3,30);
			__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_4,0);
		}
		if(HAL_GPIO_ReadPin(GPIOG,GPIO_PIN_11)==0&&HAL_GPIO_ReadPin(GPIOG,GPIO_PIN_9)==0)
		{
			__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,0);
			__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,0);

			__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_4,0);
			__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3,0);

			__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_2,0);
			__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_1,0);

			__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_3,0);
			__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_4,0);
		}
	}
	if(n==2)//右行
	{
		if(HAL_GPIO_ReadPin(GPIOD,GPIO_PIN_6)==1&&HAL_GPIO_ReadPin(GPIOD,GPIO_PIN_4)==1)
		{
			__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,0);
			__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,21);

			__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_4,25);
			__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3,0);

			__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_2,0);
			__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_1,19);

			__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_3,20);
			__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_4,0);
		}
		if(HAL_GPIO_ReadPin(GPIOD,GPIO_PIN_6)==0&&HAL_GPIO_ReadPin(GPIOD,GPIO_PIN_4)==1)
		{
			__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,0);
			__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,24);

			__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_4,25);
			__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3,0);

			__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_2,0);
			__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_1,24);

			__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_3,22);
			__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_4,0);
		}
		if(HAL_GPIO_ReadPin(GPIOD,GPIO_PIN_6)==1&&HAL_GPIO_ReadPin(GPIOD,GPIO_PIN_4)==0)
		{
			__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,0);
			__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,21);//16

			__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_4,28);//18
			__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3,0);

			__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_2,0);
			__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_1,19);

			__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_3,24);
			__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_4,0);
		}
		if(HAL_GPIO_ReadPin(GPIOD,GPIO_PIN_6)==0&&HAL_GPIO_ReadPin(GPIOD,GPIO_PIN_4)==0)
		{
			adjust(1);
		}
	}
	if(n==3)//旋转
	{
		__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,20);
		__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,0);

		__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_4,0);
		__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3,20);

		__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_2,0);
		__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_1,20);

		__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_3,20);
		__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_4,0);
	}
	if(n==4)//刹车
	{
		__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,0);
		__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,0);

		__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_4,0);
		__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3,0);

		__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_2,0);
		__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_1,0);

		__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_3,0);
		__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_4,0);
	}
	if(n==5)//倒车入库
	{
		__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,0);
		__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,0);

		__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_4,0);
		__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3,25);

		__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_1,0);
		__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_2,0);

		__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_3,0);
		__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_4,30);
	}
}

void shuxian()
{
	if(step==1)
	{
		if(HAL_GPIO_ReadPin(GPIOD,GPIO_PIN_6)==0)
		{
			Xflag=1;
		}
		if(HAL_GPIO_ReadPin(GPIOD,GPIO_PIN_6)==1&&Xflag==1)
		{
			X+=1;
			Xflag=0;
		}
	}

	if(step==2)
	{
		if(HAL_GPIO_ReadPin(GPIOG,GPIO_PIN_9)==1&&Yflag==1)
		{
			Y+=1;
			Yflag=0;
		}
		if(HAL_GPIO_ReadPin(GPIOG,GPIO_PIN_9)==0)
		{
			Yflag=1;
		}
	}

	if(step==4)
	{
		if(HAL_GPIO_ReadPin(GPIOG,GPIO_PIN_9)==1&&Zflag==1)
		{
			Z+=1;
			Zflag=0;
		}
		if(HAL_GPIO_ReadPin(GPIOG,GPIO_PIN_9)==0)
		{
			Zflag=1;
		}
	}
}

void buzhou()
{
	if(step==1)
	{
		move(1);
		shuxian();
		if(X==3)
		{
			move(4);
			HAL_Delay(1000);
			step=2;
		}
	}
	if(step==2)
	{
		move(2);
		shuxian();
		if(Y==3)
		{
			move(4);
			HAL_Delay(1000);
			step=3;
		}
	}
	if(step==3)
	{
		move(4);
		HAL_Delay(2000);
		step=4;
	}
	if(step==4)
	{
		move(5);
		shuxian();
		if(Z==5)
		{
			move(4);
			step=5;
		}
	}
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
  MX_TIM1_Init();
  MX_TIM8_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_UART4_Init();
  /* USER CODE BEGIN 2 */
   HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);
   HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_2);
   HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_3);
   HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_4);
   HAL_TIM_PWM_Start(&htim8,TIM_CHANNEL_1);
   HAL_TIM_PWM_Start(&htim8,TIM_CHANNEL_2);
   HAL_TIM_PWM_Start(&htim8,TIM_CHANNEL_3);
   HAL_TIM_PWM_Start(&htim8,TIM_CHANNEL_4);
//   HAL_UART_Receive_IT(&huart1,TLY,1000);
     HAL_UART_Receive_IT(&huart1,YY,1000);
//   HAL_GPIO_WritePin(GPIOB,GPIO_PIN_11,0);
//   HAL_GPIO_WritePin(GPIOE,GPIO_PIN_10,0);
//   HAL_GPIO_WritePin(GPIOE,GPIO_PIN_15,0);
   //HAL_UART_Transmit(&huart1,TLYwork,sizeof(TLYwork),1000);
   //HAL_ADC_Start(&hadc2);     //启动ADC转换
   //HAL_ADC_PollForConversion(&hadc3, 50);   //等待转换完成
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */


	    __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,5);
		__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,0);

		__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_4,0);
		__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3,0);

		__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_1,0);
		__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_2,0);

		__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_3,0);
		__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_4,0);


	  //buzhou();
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
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);
  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 120;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_USART2|RCC_PERIPHCLK_UART4
                              |RCC_PERIPHCLK_USART1;
  PeriphClkInitStruct.Usart234578ClockSelection = RCC_USART234578CLKSOURCE_D2PCLK1;
  PeriphClkInitStruct.Usart16ClockSelection = RCC_USART16CLKSOURCE_D2PCLK2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Enables the Clock Security System
  */
  HAL_RCC_EnableCSS();
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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 119;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 99;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
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
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM8_Init 1 */

  /* USER CODE END TIM8_Init 1 */
  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 119;
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = 99;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 0;
  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim8, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim8) != HAL_OK)
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
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim8, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM8_Init 2 */

  /* USER CODE END TIM8_Init 2 */
  HAL_TIM_MspPostInit(&htim8);

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
  huart4.Init.BaudRate = 9600;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  huart4.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart4.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart4.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart4, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart4, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART4_Init 2 */

  /* USER CODE END UART4_Init 2 */

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
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK)
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
  huart2.Init.BaudRate = 9600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart2, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart2, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOF, GPIO_PIN_12|GPIO_PIN_15, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOG, GPIO_PIN_0|GPIO_PIN_1, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_10|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_RESET);

  /*Configure GPIO pins : PE4 PE5 PE6 PE0
                           PE1 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_0
                          |GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : PC14 PC4 */
  GPIO_InitStruct.Pin = GPIO_PIN_14|GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PF2 PF3 PF5 PF7
                           PF9 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_5|GPIO_PIN_7
                          |GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pin : PA6 */
  GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB2 PB3 PB4
                           PB5 PB8 PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4
                          |GPIO_PIN_5|GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PF12 PF15 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pins : KEY1_Pin KEY2_Pin */
  GPIO_InitStruct.Pin = KEY1_Pin|KEY2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pins : PG0 PG1 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pins : PE10 PE15 */
  GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : PB11 */
  GPIO_InitStruct.Pin = GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PD4 PD6 PD7 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : PG9 PG10 PG11 PG12
                           PG13 PG14 PG15 */
  GPIO_InitStruct.Pin = GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12
                          |GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */

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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
