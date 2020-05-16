/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
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
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;
ADC_HandleTypeDef hadc3;

TIM_HandleTypeDef htim1;

UART_HandleTypeDef huart5;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
static void MX_UART5_Init(void);
static void MX_ADC1_Init(void);
static void MX_ADC2_Init(void);
static void MX_ADC3_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

int last_state = 0;
uint16_t T = 1000;
volatile uint32_t counter = 0;

volatile uint16_t prescaler = 6;
volatile uint8_t sector = 7;
volatile uint8_t zeroed = 0;
volatile uint16_t angle = 0;
volatile uint16_t angle_rotor = 0;
volatile float incr = 0.10;
volatile float angle_temp = 0;
volatile float m =0.0;  // commanded vector magnitude
float dir = 1;
int overflow = 0;
float ki = 0.005; // 0.5
float kp = 0.1; // 40
float kd = 0;
float integral = 0;
float prev_error = 0;
int error = 0;
uint32_t last_time = 0;
uint32_t this_time = 0;
int vel = 0;
uint32_t elapsed_ms = 0;
int underflow = 0;
int fault = 0;

float set_v = 14;
uint32_t adc[3];

volatile uint_fast16_t u = 0;
volatile uint_fast16_t v = 0;
volatile uint_fast16_t w = 0;

volatile uint_fast16_t t0 = 0;
volatile uint_fast16_t t1 = 0;
volatile uint_fast16_t t2 = 0;
float sin_table[359];
#define MAX_TX 1000
uint16_t cycle_count = 0;
uint32_t tx_bytes = 0;
int started = 0;
uint8_t tx_buffer[MAX_TX];

void fill_sin_table()
{
    for(int i=0; i<360; i++)
        sin_table[i] = sin(0.0174532925*i);
}


void pid(){
	error = set_v - 4000.0/vel;
	float prop = error*kp;

	integral += error*ki;
	if (integral > 500){
		integral = 500;
	}
	if (integral < -500){
		integral = -500;
	}

	float derivative = (error - prev_error)*kd;
	float res = prop + integral + derivative;

	prev_error = error;
//	if (error == 0) // completely kill
//	{
//		return 0;
//	}
	res = (res+500)/1000.0;
	if (res < 0){
		res = 0; // left
	}
	else if (res > 1)
	{
		res = 1.0; // right
	}
	m = res;
//	m = 0.4;
}

void cal_pwms()
{

	if (angle < 0)
	{
		angle = 360 + angle;
	}

	if (angle > 360)
	{
		angle = angle - 360;
	}


	if (angle <= 60)
	{
		sector = 1;
		angle_temp = angle;
	}
	else if (angle <= 120)
	{
		sector = 2;
		angle_temp = angle-60;
	}
	else if (angle <= 180)
	{
		sector = 3;
		angle_temp = angle-120;
	}
	else if (angle <= 240)
	{
		sector = 4;
		angle_temp = angle-180;
	}
	else if (angle <= 300)
	{
		sector = 5;
		angle_temp = angle-240;
	}
	else
	{
		sector = 6;
		angle_temp = angle-300;
	}

	t1 = T*m*sin_table[(int)(60-angle_temp)];
	t2 = T*m*sin_table[(int)angle_temp];

	t0 = T-t1-t2;

	switch (sector)
	{
		case 1:
	    	u = t1 + t2 +0.5*t0;
	    	v = t2 + 0.5*t0;
	    	w = 0.5*t0;
	      break;
	    case 2:
	    	u = t1 + 0.5*t0;
	    	v = t1 + t2 + 0.5*t0;
	    	w = 0.5*t0;
	      break;
	    case 3:
	    	u = 0.5*t0;
	    	v = t1 + t2 + 0.5*t0;
	    	w = t2 + 0.5*t0;
	      break;
	    case 4:
	    	u = 0.5*t0;
	    	v = t1 + 0.5*t0;
	    	w = t1 + t2 + 0.5*t0;
	      break;
	    case 5:
	    	u = t2 + 0.5*t0;
	    	v = 0.5*t0;
	    	w = t1 + t2 + 0.5*t0;
	      break;
	    case 6:
	    	u = t1 + t2 + 0.5*t0;
	    	v = 0.5*t0;
	    	w = t1 + 0.5*t0;
	      break;
	    default:
	    	u = 0;
	    	v = 0;
	    	w = 0;
	}
	if (fault == 1)
	{
    	u = 0;
    	v = 0;
    	w = 0;
	}

	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, u);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, v);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, w);
}


void start(int dir)
{
	started = 0;
	HAL_GPIO_WritePin(Relay1_GPIO_Port, Relay1_Pin, 0);
	HAL_GPIO_WritePin(Relay0_GPIO_Port, Relay0_Pin, 1);
	HAL_Delay(3000);
	HAL_GPIO_WritePin(Relay1_GPIO_Port, Relay1_Pin, 1);

//	if (HAL_GPIO_ReadPin(hall_0_GPIO_Port, hall_0_Pin))
//	{
//		__HAL_TIM_SET_COUNTER(&htim8, 600);
//	}
//	else
//	{
//		__HAL_TIM_SET_COUNTER(&htim8, 1800);
//	}
//	HAL_Delay(100);

	int a = HAL_GPIO_ReadPin(hall_0_GPIO_Port, hall_0_Pin);
	int b = HAL_GPIO_ReadPin(hall_1_GPIO_Port, hall_1_Pin);
	int c = HAL_GPIO_ReadPin(hall_2_GPIO_Port, hall_2_Pin);

	if (a && c){
		angle_rotor = 30;
	}
	else if (!b && !c){
		angle_rotor = 90;
	}
	else if (a && b){
		angle_rotor = 150;
	}
	else if (!a && !c){
		angle_rotor = 210;
	}
	else if (b && c){
		angle_rotor = 270;
	}
	else if (!a && !b){
		angle_rotor = 330;
	}


	started = 1;
//	return;
//	m = 0.01;
//	for (int x = 0; x<360; x++)
//	{
//		angle = x;
//		cal_pwms();
//		m+=0.01;
//		HAL_Delay(1);
//	}
//	for (int x = 0; x<360; x++)
//	{
//		angle = x;
//		cal_pwms();
//		m+=0.1;
//		HAL_Delay(1);
//	}
//	m = 0;
//	started = 1;
}

void myisr()
{
	if (started)
	{
//		angle_rotor =(uint16_t)__HAL_TIM_GET_COUNTER(&htim8)/6.66666666666666;

		angle = angle_rotor + 90;
		cal_pwms();
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
	fill_sin_table();
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
  MX_UART5_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_ADC3_Init();
  /* USER CODE BEGIN 2 */
  // arr = freq
  // ccr = duty



  HAL_TIM_Base_Start_IT(&htim1);

  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);

  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);

  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);

  HAL_TIM_OC_Start_IT(&htim1, TIM_CHANNEL_4);


  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);


//  HAL_TIM_Base_Start(&htim8);  // encoder timer
//  HAL_TIM_Encoder_Start(&htim8,TIM_CHANNEL_ALL);

  HAL_ADCEx_InjectedStart_IT(&hadc1);

//  HAL_ADC_Start_DMA(&hadc2, (uint32_t *)adc, 2);
//  HAL_ADC_Start_IT(&hadc2);

  HAL_ADCEx_InjectedStart_IT(&hadc2);
  HAL_ADCEx_InjectedStart_IT(&hadc3);

//  HAL_AD
//  HAL_UART_Transmit_DMA(&huart5, tx_buffer, 100);
//  HAL_UART_Transmit(&huart5, tx_buffer, 100);

//  HAL_ADC_IRQHandler(hadc)
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  __HAL_TIM_SET_PRESCALER(&htim1, prescaler);

  start(1);
  while (1)
  {

	  counter+=1;
	  if (counter < 45000)
	  {
		  HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, 1);
		  if (counter == 42000)
		  {
			  char str[80];

		  }
	  }
	  else
	  {
		  HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, 0);
	  }
	  if (counter > 90000)
	  {
		  counter = 0;
	  }

	  int val = HAL_GPIO_ReadPin(B1_GPIO_Port, B1_Pin);
	  if (val != last_state)
	  {
		  if (fault == 1)  // fault happened
		  {
			  if (HAL_GPIO_ReadPin(VFO_GPIO_Port, VFO_Pin))  // no fault now
				{
				  fault = 0;
	      		  HAL_GPIO_WritePin(LD4_GPIO_Port, LD4_Pin, 0);
				}
		  }
		  if (val)
		  {
//			  HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, val);
//			  HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);

//			  incr+=0.001;
//			  if (incr > 1.0)
//			  {
//			  m = 1.0;
//			  }
//				__HAL_TIM_SET_COUNTER(&htim8, 0);
//			  m+=0.1;
//			  if (m>1.0)
//			  {
//				  m=0;
//			  }
			  set_v+=1;
			  if (set_v>50.0)
			  {
				  set_v=2;
			  }

//			  angle+=10;
		  }
		  last_state = val;
	  }


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

  /** Configure the main internal regulator output voltage 
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 3;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Activate the Over-Drive mode 
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};
  ADC_InjectionConfTypeDef sConfigInjected = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure the ADC multi-mode 
  */
  multimode.Mode = ADC_TRIPLEMODE_INJECSIMULT;
  multimode.TwoSamplingDelay = ADC_TWOSAMPLINGDELAY_5CYCLES;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configures for the selected ADC injected channel its corresponding rank in the sequencer and its sample time 
  */
  sConfigInjected.InjectedChannel = ADC_CHANNEL_1;
  sConfigInjected.InjectedRank = 1;
  sConfigInjected.InjectedNbrOfConversion = 1;
  sConfigInjected.InjectedSamplingTime = ADC_SAMPLETIME_3CYCLES;
  sConfigInjected.ExternalTrigInjecConvEdge = ADC_EXTERNALTRIGINJECCONVEDGE_FALLING;
  sConfigInjected.ExternalTrigInjecConv = ADC_EXTERNALTRIGINJECCONV_T1_TRGO;
  sConfigInjected.AutoInjectedConv = DISABLE;
  sConfigInjected.InjectedDiscontinuousConvMode = DISABLE;
  sConfigInjected.InjectedOffset = 0;
  if (HAL_ADCEx_InjectedConfigChannel(&hadc1, &sConfigInjected) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief ADC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};
  ADC_InjectionConfTypeDef sConfigInjected = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc2.Init.Resolution = ADC_RESOLUTION_12B;
  hadc2.Init.ScanConvMode = DISABLE;
  hadc2.Init.ContinuousConvMode = DISABLE;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.NbrOfConversion = 1;
  hadc2.Init.DMAContinuousRequests = DISABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configures for the selected ADC injected channel its corresponding rank in the sequencer and its sample time 
  */
  sConfigInjected.InjectedChannel = ADC_CHANNEL_2;
  sConfigInjected.InjectedRank = 1;
  sConfigInjected.InjectedNbrOfConversion = 1;
  sConfigInjected.InjectedSamplingTime = ADC_SAMPLETIME_3CYCLES;
  sConfigInjected.AutoInjectedConv = DISABLE;
  sConfigInjected.InjectedDiscontinuousConvMode = DISABLE;
  sConfigInjected.InjectedOffset = 0;
  if (HAL_ADCEx_InjectedConfigChannel(&hadc2, &sConfigInjected) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

}

/**
  * @brief ADC3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC3_Init(void)
{

  /* USER CODE BEGIN ADC3_Init 0 */

  /* USER CODE END ADC3_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};
  ADC_InjectionConfTypeDef sConfigInjected = {0};

  /* USER CODE BEGIN ADC3_Init 1 */

  /* USER CODE END ADC3_Init 1 */
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
  */
  hadc3.Instance = ADC3;
  hadc3.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc3.Init.Resolution = ADC_RESOLUTION_12B;
  hadc3.Init.ScanConvMode = DISABLE;
  hadc3.Init.ContinuousConvMode = DISABLE;
  hadc3.Init.DiscontinuousConvMode = DISABLE;
  hadc3.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc3.Init.NbrOfConversion = 1;
  hadc3.Init.DMAContinuousRequests = DISABLE;
  hadc3.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  if (HAL_ADC_Init(&hadc3) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configures for the selected ADC injected channel its corresponding rank in the sequencer and its sample time 
  */
  sConfigInjected.InjectedChannel = ADC_CHANNEL_3;
  sConfigInjected.InjectedRank = 1;
  sConfigInjected.InjectedNbrOfConversion = 1;
  sConfigInjected.InjectedSamplingTime = ADC_SAMPLETIME_3CYCLES;
  sConfigInjected.AutoInjectedConv = DISABLE;
  sConfigInjected.InjectedDiscontinuousConvMode = DISABLE;
  sConfigInjected.InjectedOffset = 0;
  if (HAL_ADCEx_InjectedConfigChannel(&hadc3, &sConfigInjected) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC3_Init 2 */

  /* USER CODE END ADC3_Init 2 */

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
  htim1.Init.Prescaler = 45000;
  htim1.Init.CounterMode = TIM_COUNTERMODE_CENTERALIGNED2;
  htim1.Init.Period = 1000;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
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
  if (HAL_TIM_OC_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_OC4REF;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 500;
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
  sConfigOC.OCMode = TIM_OCMODE_TOGGLE;
  sConfigOC.Pulse = 400;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_LOW;
  if (HAL_TIM_OC_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
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
  huart5.Init.BaudRate = 19200;
  huart5.Init.WordLength = UART_WORDLENGTH_8B;
  huart5.Init.StopBits = UART_STOPBITS_1;
  huart5.Init.Parity = UART_PARITY_NONE;
  huart5.Init.Mode = UART_MODE_TX;
  huart5.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart5.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart5) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART5_Init 2 */

  /* USER CODE END UART5_Init 2 */

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
  HAL_GPIO_WritePin(GPIOC, NCS_MEMS_SPI_Pin|CSX_Pin|OTG_FS_PSO_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, Relay0_Pin|Relay1_Pin|RDX_Pin|WRX_DCX_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, myio_Pin|myio2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOG, LD3_Pin|LD4_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : VFO_Pin */
  GPIO_InitStruct.Pin = VFO_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(VFO_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : A0_Pin A1_Pin A2_Pin A3_Pin 
                           A4_Pin A5_Pin SDNRAS_Pin A6_Pin 
                           A7_Pin A8_Pin A9_Pin */
  GPIO_InitStruct.Pin = A0_Pin|A1_Pin|A2_Pin|A3_Pin 
                          |A4_Pin|A5_Pin|SDNRAS_Pin|A6_Pin 
                          |A7_Pin|A8_Pin|A9_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF12_FMC;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pins : SPI5_SCK_Pin SPI5_MISO_Pin SPI5_MOSI_Pin */
  GPIO_InitStruct.Pin = SPI5_SCK_Pin|SPI5_MISO_Pin|SPI5_MOSI_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI5;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pin : ENABLE_Pin */
  GPIO_InitStruct.Pin = ENABLE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF14_LTDC;
  HAL_GPIO_Init(ENABLE_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SDNWE_Pin */
  GPIO_InitStruct.Pin = SDNWE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF12_FMC;
  HAL_GPIO_Init(SDNWE_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : NCS_MEMS_SPI_Pin CSX_Pin OTG_FS_PSO_Pin */
  GPIO_InitStruct.Pin = NCS_MEMS_SPI_Pin|CSX_Pin|OTG_FS_PSO_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : VSYNC_Pin G2_Pin */
  GPIO_InitStruct.Pin = VSYNC_Pin|G2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF14_LTDC;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : OTG_FS_OC_Pin */
  GPIO_InitStruct.Pin = OTG_FS_OC_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(OTG_FS_OC_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : R3_Pin */
  GPIO_InitStruct.Pin = R3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF9_LTDC;
  HAL_GPIO_Init(R3_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PB1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF3_TIM8;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : BOOT1_Pin */
  GPIO_InitStruct.Pin = BOOT1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BOOT1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : A10_Pin A11_Pin BA0_Pin BA1_Pin 
                           SDCLK_Pin SDNCAS_Pin */
  GPIO_InitStruct.Pin = A10_Pin|A11_Pin|BA0_Pin|BA1_Pin 
                          |SDCLK_Pin|SDNCAS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF12_FMC;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pins : D4_Pin D5_Pin D6_Pin D11_Pin 
                           D12_Pin */
  GPIO_InitStruct.Pin = D4_Pin|D5_Pin|D6_Pin|D11_Pin 
                          |D12_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF12_FMC;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : G4_Pin G5_Pin B6_Pin B7_Pin */
  GPIO_InitStruct.Pin = G4_Pin|G5_Pin|B6_Pin|B7_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF14_LTDC;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : OTG_HS_ID_Pin OTG_HS_DM_Pin OTG_HS_DP_Pin */
  GPIO_InitStruct.Pin = OTG_HS_ID_Pin|OTG_HS_DM_Pin|OTG_HS_DP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF12_OTG_HS_FS;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : VBUS_HS_Pin */
  GPIO_InitStruct.Pin = VBUS_HS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(VBUS_HS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : D13_Pin D0_Pin D1_Pin D2_Pin 
                           D3_Pin */
  GPIO_InitStruct.Pin = D13_Pin|D0_Pin|D1_Pin|D2_Pin 
                          |D3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF12_FMC;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : Relay0_Pin Relay1_Pin RDX_Pin WRX_DCX_Pin */
  GPIO_InitStruct.Pin = Relay0_Pin|Relay1_Pin|RDX_Pin|WRX_DCX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : TE_Pin */
  GPIO_InitStruct.Pin = TE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(TE_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : R7_Pin DOTCLK_Pin B3_Pin */
  GPIO_InitStruct.Pin = R7_Pin|DOTCLK_Pin|B3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF14_LTDC;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pins : hall_1_Pin hall_2_Pin */
  GPIO_InitStruct.Pin = hall_1_Pin|hall_2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : STLINK_RX_Pin STLINK_TX_Pin */
  GPIO_InitStruct.Pin = STLINK_RX_Pin|STLINK_TX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : myio_Pin myio2_Pin */
  GPIO_InitStruct.Pin = myio_Pin|myio2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA15 */
  GPIO_InitStruct.Pin = GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : R2_Pin */
  GPIO_InitStruct.Pin = R2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF14_LTDC;
  HAL_GPIO_Init(R2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : G7_Pin B2_Pin */
  GPIO_InitStruct.Pin = G7_Pin|B2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF14_LTDC;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : G3_Pin B4_Pin */
  GPIO_InitStruct.Pin = G3_Pin|B4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF9_LTDC;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pins : LD3_Pin LD4_Pin */
  GPIO_InitStruct.Pin = LD3_Pin|LD4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pins : SDCKE1_Pin SDNE1_Pin */
  GPIO_InitStruct.Pin = SDCKE1_Pin|SDNE1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF12_FMC;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : hall_0_Pin */
  GPIO_InitStruct.Pin = hall_0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(hall_0_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

}

/* USER CODE BEGIN 4 */

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{

	int vel2 = this_time - last_time ;

//	vel2 = 10;
	if (vel2 < 100) // too fast prolly a glitch
	{
		return;
	}


	int a = HAL_GPIO_ReadPin(hall_0_GPIO_Port, hall_0_Pin);
	int b = HAL_GPIO_ReadPin(hall_1_GPIO_Port, hall_1_Pin);
	int c = HAL_GPIO_ReadPin(hall_2_GPIO_Port, hall_2_Pin);
	int last_rotor_angle = angle_rotor;

	if (a && c){
		if (last_rotor_angle == 330 || last_rotor_angle == 90)
		{
			angle_rotor = 30;
			last_time = this_time;
			vel = vel2;
			HAL_GPIO_WritePin(myio2_GPIO_Port, myio2_Pin, 1);
			HAL_GPIO_WritePin(myio2_GPIO_Port, myio2_Pin, 0);
		}
	}
	else if (!b && !c){
		if (last_rotor_angle == 30 || last_rotor_angle == 150)
		{
			angle_rotor = 90;
			last_time = this_time;
			vel = vel2;

			HAL_GPIO_WritePin(myio2_GPIO_Port, myio2_Pin, 1);
			HAL_GPIO_WritePin(myio2_GPIO_Port, myio2_Pin, 0);
		}
	}
	else if (a && b){
		if (last_rotor_angle == 90 || last_rotor_angle == 210)
		{
			angle_rotor = 150;
			last_time = this_time;
			vel = vel2;

			HAL_GPIO_WritePin(myio2_GPIO_Port, myio2_Pin, 1);
			HAL_GPIO_WritePin(myio2_GPIO_Port, myio2_Pin, 0);
		}
	}
	else if (!a && !c){
		if (last_rotor_angle == 150 || last_rotor_angle == 270)
		{
			angle_rotor = 210;
			last_time = this_time;
			vel = vel2;

			HAL_GPIO_WritePin(myio2_GPIO_Port, myio2_Pin, 1);
			HAL_GPIO_WritePin(myio2_GPIO_Port, myio2_Pin, 0);
		}
	}
	else if (b && c){
		if (last_rotor_angle == 210 || last_rotor_angle == 330)
		{
			angle_rotor = 270;
			last_time = this_time;
			vel = vel2;

			HAL_GPIO_WritePin(myio2_GPIO_Port, myio2_Pin, 1);
			HAL_GPIO_WritePin(myio2_GPIO_Port, myio2_Pin, 0);
		}
	}
	else if (!a && !b){
		if (last_rotor_angle == 270 || last_rotor_angle == 30)
		{
			angle_rotor = 330;
			last_time = this_time;
			vel = vel2;

			HAL_GPIO_WritePin(myio2_GPIO_Port, myio2_Pin, 1);
			HAL_GPIO_WritePin(myio2_GPIO_Port, myio2_Pin, 0);
		}
	}




//
//	if (GPIO_Pin == hall_0_Pin)  // hall sensor
//	{
//		if (zeroed == 0 && started==1 )
//		{
//			if (HAL_GPIO_ReadPin(hall_0_GPIO_Port, hall_0_Pin)) //high
//			{
//				angle_rotor = 0;
//			}
//			else
//			{
//				angle_rotor = 180;
//			}
//		}
//	}
//
//	else if (GPIO_Pin == hall_1_Pin)  // hall sensor
//	{
//		if (zeroed == 0 && started==1 )
//		{
//			if (HAL_GPIO_ReadPin(hall_1_GPIO_Port, hall_1_Pin)) //high
//			{
//				angle_rotor = 120;
//			}
//			else
//			{
//				angle_rotor = 300;
//			}
//		}
//	}
//	else if (GPIO_Pin == hall_2_Pin)  // hall sensor
//	{
//		if (zeroed == 0 && started==1 )
//		{
//			if (HAL_GPIO_ReadPin(hall_2_GPIO_Port, hall_2_Pin)) //high
//			{
//				angle_rotor = 240;
//			}
//			else
//			{
//				angle_rotor = 60;
//			}
//		}
//	}
}


void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef* hadc)
{
		HAL_GPIO_WritePin(myio_GPIO_Port, myio_Pin, 1);
		HAL_GPIO_WritePin(myio_GPIO_Port, myio_Pin, 0);
//		HAL_GPIO_TogglePin(myio_GPIO_Port, myio_Pin);
//		adc[0] = HAL_ADCEx_InjectedGetValue(&hadc2, 1);
		adc[0] += HAL_ADCEx_InjectedGetValue(&hadc1, 1);
		adc[1] += HAL_ADCEx_InjectedGetValue(&hadc2, 1);
		adc[2] += HAL_ADCEx_InjectedGetValue(&hadc3, 1);


		if (cycle_count == 100 )
		{
//			if(adc[0]+adc[1]+adc[2] > 50000)
//			{
//				fault = 1;
//				HAL_GPIO_WritePin(LD4_GPIO_Port, LD4_Pin, 1);
//				set_v = 5;
//				error = 0;
//
//			}
			int length = sprintf(tx_buffer, "%d,%d,%f,%d,%d,%f,%u,%u,%u\r\n", vel, error, integral, angle, angle_rotor, m, adc[0], adc[1], adc[2]);

			HAL_UART_Transmit_IT(&huart5, tx_buffer, length);
			cycle_count = 0;
			adc[0] = 0;
			adc[1] = 0;
			adc[2] = 0;
//			HAL_GPIO_WritePin(myio_GPIO_Port, myio_Pin, 1);

		}
		cycle_count+=1;
//		HAL_GPIO_WritePin(myio_GPIO_Port, myio_Pin, 0);

//		if (tx_bytes + length+1 >= MAX_TX)
//		{
////			HAL_UART_Transmit_IT(&huart5, tx_buffer, tx_bytes);
//			// send it and
//			tx_bytes = 0;
//			cycle_count = 0;

//
//		}
}



//void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim)
//{
//	if (htim->Instance == TIM1)
//	{
//		if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_4)
//		{	//      HAL_IncTick();
//			HAL_GPIO_WritePin(myio_GPIO_Port, myio_Pin, 1);
//			HAL_GPIO_WritePin(myio_GPIO_Port, myio_Pin, 0);
//		}
//	}
//}

/* USER CODE END 4 */

 /**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  if (htim->Instance == TIM6) {

    elapsed_ms++;
//    if (elapsed_ms == 4)
//    {
//    	int old_vel = vel;
//    	elapsed_ms = 0;
//    	if (underflow)
//    	{
//    		dir = 1; // adasd
//    	}
//    	else
//    	{
//    		dir = 1;
//    	}
//    	if (dir)  // clockwise rotation
//		{
//			if (angle >= last_angle)
//			{
//				vel = angle - last_angle;
//			}
//			else
//			{
//				vel = angle + 360 -last_angle;
//			}
//			if (vel > 200)
//			{
//				vel = old_vel;
//			}
//    	}
//    	else  // anti-clockwise rotation
//		{
//			if (last_angle >= angle)
//			{
//				vel = last_angle - angle;
//			}
//			else
//			{
//				vel =  360 - angle + last_angle;
//			}
//    	}
//		last_angle = angle;
//
//	//    pid();
//    }
    if (fault == 0)
    {
//		if (!HAL_GPIO_ReadPin(VFO_GPIO_Port, VFO_Pin))
//		{
//			HAL_GPIO_WritePin(LD4_GPIO_Port, LD4_Pin, 1);
//			fault = 1;
//
//		}
		pid();
    }
  }

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */
  if (htim->Instance == TIM1) {
	  myisr();
	  this_time++;
//      HAL_IncTick();
//		HAL_GPIO_WritePin(myio_GPIO_Port, myio_Pin, 1);
//		HAL_GPIO_WritePin(myio_GPIO_Port, myio_Pin, 0);

    }

//  if (htim->Instance == TIM8)
//  {
////	  if (__HAL_TIM_GET_COUNTER(&htim8)>1200)
////	  {
////		  underflow = 1;
////	  }
////	  else
////	  {
////		  underflow = 0;
////	  }
//  }
  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
