/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
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
#include "timer_pwm_lib.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define DUTY_CYCLE_SWEEP_DURATION 5000
#define DUTY_CYCLE_FINAL 100
#define DUTY_CYCLE_START 30
#define DUTY_CYCLE_STEP 1
#define DUTY_CYCLE_STEP_PERIOD DUTY_CYCLE_SWEEP_DURATION / ( ( DUTY_CYCLE_FINAL - DUTY_CYCLE_START ) / DUTY_CYCLE_STEP )

#define DUTY_CYCLE_MAX 100
#define DUTY_CYCLE_MIN 0


enum DCDC_STATES{
	INITIALIZING,
	DCDC_VALID,
	PWM_SWEEP,
	DCDC_INVALID
};

//Values to be returned by function that would otherwise return void
enum RETURN_CODES{
	VALID,				// Proper operation i.e. no error
	INVALID_ARGUMENT,	// Argument does not satisfy function requirements
	TIMEOUT_ERROR,		// Time taken above set threshold
	VALUE_ERROR			// Value causes logic error
};

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

CAN_HandleTypeDef hcan1;
/* USER CODE BEGIN PV */
enum DCDC_STATES DCDC_State;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_CAN1_Init(void);
/* USER CODE BEGIN PFP */
uint8_t PWM_Sweep_Nonblocking(uint8_t timer, uint8_t channel, uint8_t duty_target, uint8_t duty_step_size, uint8_t is_positive_logic, uint8_t initialize, uint8_t duty_init);
uint8_t PWM_Sweep_Blocking(uint8_t timer, uint8_t channel, uint16_t duty_sweep_duration, uint8_t duty_start, uint8_t duty_final, uint8_t duty_step_size, uint8_t is_positive_logic);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim) //ISR triggered by timer overflow
{
	if(htim == &htim2 && timer_2_repetition_counter == TIMER_2_PERIOD_MULTIPLIER-1)
    {
		timer_2_repetition_counter = 0;
		//ISR for Timer 2
        //User code here
		if (DCDC_State == PWM_SWEEP)
		{
			//In this case, the sweep is not being reinitialized, therefore duty_init not used
			PWM_Sweep_Nonblocking(2, 1, DUTY_CYCLE_FINAL, DUTY_CYCLE_STEP, 0, 0, DUTY_CYCLE_START);
		}

        //User code ends

    }
    else if(htim == &htim2)
    {
    	timer_2_repetition_counter++;
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
  MX_ADC1_Init();
  MX_CAN1_Init();
  /* USER CODE BEGIN 2 */

  DCDC_State = INITIALIZING;

  Timer_Init_Base(2, 1, 1, 1, -1, -1, -1, DUTY_CYCLE_STEP_PERIOD);
  PWM_Init(2, 1, DUTY_CYCLE_MAX);

  //Set pins to initial state
  HAL_GPIO_WritePin(POWERTRAIN_FAN_EN_GPIO_Port, POWERTRAIN_FAN_EN_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(POWERTRAIN_PUMP_EN_GPIO_Port, POWERTRAIN_PUMP_EN_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(FRONT_CONTROLLER_AND_ACC_EN_GPIO_Port, FRONT_CONTROLLER_AND_ACC_EN_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(MOTOR_CONTROLLER_EN_GPIO_Port, MOTOR_CONTROLLER_EN_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(DCDC_ON_LED_EN_GPIO_Port,DCDC_ON_LED_EN_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(TSAL_EN_GPIO_Port, TSAL_EN_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(SHUTDOWN_CIRCUIT_EN_GPIO_Port,SHUTDOWN_CIRCUIT_EN_Pin , GPIO_PIN_RESET);
  HAL_GPIO_WritePin(MOTOR_CONTROLLER_PRECHARGE_EN_GPIO_Port, MOTOR_CONTROLLER_PRECHARGE_EN_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(DCDC_EN_GPIO_Port,DCDC_EN_Pin , GPIO_PIN_SET);

  //Set select pins ON, one by one, with delays
  HAL_GPIO_WritePin(TSAL_EN_GPIO_Port, TSAL_EN_Pin, GPIO_PIN_SET);

  HAL_Delay(50);

  HAL_GPIO_WritePin(FRONT_CONTROLLER_AND_ACC_EN_GPIO_Port, FRONT_CONTROLLER_AND_ACC_EN_Pin, GPIO_PIN_SET);

  HAL_Delay(200);

  HAL_GPIO_WritePin(MOTOR_CONTROLLER_PRECHARGE_EN_GPIO_Port, MOTOR_CONTROLLER_PRECHARGE_EN_Pin, GPIO_PIN_SET);

  HAL_Delay(1000);

  HAL_GPIO_WritePin(MOTOR_CONTROLLER_EN_GPIO_Port, MOTOR_CONTROLLER_EN_Pin, GPIO_PIN_SET);

  HAL_Delay(50);

  HAL_GPIO_WritePin(MOTOR_CONTROLLER_PRECHARGE_EN_GPIO_Port, MOTOR_CONTROLLER_PRECHARGE_EN_Pin, GPIO_PIN_RESET);

  HAL_Delay(1000);

  HAL_GPIO_WritePin(SHUTDOWN_CIRCUIT_EN_GPIO_Port, SHUTDOWN_CIRCUIT_EN_Pin, GPIO_PIN_SET);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  //Poll until MUX_DCDC_VALID goes low
	  while(HAL_GPIO_ReadPin(MUX_DCDC_VALID_GPIO_Port, MUX_DCDC_VALID_Pin) == GPIO_PIN_SET){  };
	  DCDC_State = DCDC_VALID;
	  // Set powertrain pump/fans pins with delays;
	  HAL_GPIO_WritePin(DCDC_ON_LED_EN_GPIO_Port, DCDC_ON_LED_EN_Pin, GPIO_PIN_SET);

	  HAL_Delay(50);

	  HAL_GPIO_WritePin(POWERTRAIN_PUMP_EN_GPIO_Port, POWERTRAIN_PUMP_EN_Pin, GPIO_PIN_SET);

	  HAL_Delay(100);

	  HAL_GPIO_WritePin(POWERTRAIN_FAN_EN_GPIO_Port, POWERTRAIN_FAN_EN_Pin, GPIO_PIN_SET);

	  HAL_Delay(50);

	  //Start PWM Sweep
	  PWM_Sweep_Nonblocking(2, 1, DUTY_CYCLE_FINAL, DUTY_CYCLE_STEP, 0, 1, DUTY_CYCLE_START);
	  //Only change now, otherwise the pwm sweep might begin unitialized
	  DCDC_State = PWM_SWEEP;


	  //Poll until MUX_DCDC_VALID goes high
	  while(HAL_GPIO_ReadPin(MUX_DCDC_VALID_GPIO_Port, MUX_DCDC_VALID_Pin) == GPIO_PIN_RESET){  };
	  DCDC_State = DCDC_INVALID;

	  //Turn off pump and fans
	  HAL_GPIO_WritePin(DCDC_ON_LED_EN_GPIO_Port, DCDC_ON_LED_EN_Pin, GPIO_PIN_RESET);
	  HAL_GPIO_WritePin(POWERTRAIN_PUMP_EN_GPIO_Port, POWERTRAIN_PUMP_EN_Pin, GPIO_PIN_RESET);
	  HAL_GPIO_WritePin(POWERTRAIN_FAN_EN_GPIO_Port, POWERTRAIN_FAN_EN_Pin, GPIO_PIN_RESET);
	  PWM_Stop(2, 1);
	  PWM_Init(2, 1, DUTY_CYCLE_MAX);


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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
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

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_10;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief CAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 16;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_1TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_1TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */

  /* USER CODE END CAN1_Init 2 */

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOG, POWERTRAIN_FAN_EN_Pin|FRONT_CONTROLLER_AND_ACC_EN_Pin|SHUTDOWN_CIRCUIT_EN_Pin|MOTOR_CONTROLLER_PRECHARGE_EN_Pin
                          |DCDC_EN_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOG, POWERTRAIN_PUMP_EN_Pin|MOTOR_CONTROLLER_EN_Pin|DCDC_ON_LED_EN_Pin|TSAL_EN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : PE2 PE3 MUX_DCDC_VALID_Pin MUX_LVBATT_VALID_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_3|MUX_DCDC_VALID_Pin|MUX_LVBATT_VALID_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : POWERTRAIN_FAN_EN_Pin POWERTRAIN_PUMP_EN_Pin FRONT_CONTROLLER_AND_ACC_EN_Pin MOTOR_CONTROLLER_EN_Pin
                           DCDC_ON_LED_EN_Pin TSAL_EN_Pin SHUTDOWN_CIRCUIT_EN_Pin MOTOR_CONTROLLER_PRECHARGE_EN_Pin
                           DCDC_EN_Pin */
  GPIO_InitStruct.Pin = POWERTRAIN_FAN_EN_Pin|POWERTRAIN_PUMP_EN_Pin|FRONT_CONTROLLER_AND_ACC_EN_Pin|MOTOR_CONTROLLER_EN_Pin
                          |DCDC_ON_LED_EN_Pin|TSAL_EN_Pin|SHUTDOWN_CIRCUIT_EN_Pin|MOTOR_CONTROLLER_PRECHARGE_EN_Pin
                          |DCDC_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
uint8_t PWM_Sweep_Blocking(uint8_t timer, uint8_t channel, uint16_t duty_sweep_duration, uint8_t duty_start, uint8_t duty_final, uint8_t duty_step_size, uint8_t is_positive_logic)
{
	//check for valid arguments
	if(!(timer==1 || timer == 2 || timer == 4) ) 												{return INVALID_ARGUMENT;}
	else if( !( (channel>=1) && (channel<=4) ) ) 												{return INVALID_ARGUMENT;}
	else if( !( (duty_start >= DUTY_CYCLE_MIN) && (duty_start <= DUTY_CYCLE_MAX) ) ) 			{return INVALID_ARGUMENT;}
	else if( !( (duty_final >= DUTY_CYCLE_MIN) && (duty_final <= DUTY_CYCLE_MAX) ) ) 			{return INVALID_ARGUMENT;}
	else if( !( (duty_step_size >= DUTY_CYCLE_MIN) && ( duty_step_size <= DUTY_CYCLE_MAX ) ) ) 	{return INVALID_ARGUMENT;}
	else if( !( (is_positive_logic == 0) || (is_positive_logic == 1) ) )						{return INVALID_ARGUMENT;}

	//check for any logic issues
	if( !( ( (duty_final - duty_start) % duty_step_size ) == 0 ) ) 								{return VALUE_ERROR;}

	//DUTY_CYCLE_STEP_DELAY = ( DUTY_CYCLE_SWEEP_DURATION / TOTAL_NUMBER_OF_STEPS )
	uint16_t duty_step_delay = ( duty_sweep_duration / ( (duty_final - duty_start) / duty_step_size ) );

	for(uint8_t duty = duty_start; duty<duty_final; duty+=duty_step_size)
	{
		PWM_Stop(timer, channel);

		if(is_positive_logic)
		{
			PWM_Init(timer, channel, duty);
		}
		else
		{
			PWM_Init(timer, channel, DUTY_CYCLE_MAX - duty);
		}

		HAL_Delay(duty_step_delay);
	}
}

uint8_t PWM_Sweep_Nonblocking(uint8_t timer, uint8_t channel, uint8_t duty_target, uint8_t duty_step_size, uint8_t is_positive_logic, uint8_t initialize, uint8_t duty_init)
{
	static current_duty = 0;

	if (initialize==1)
	{
		//check for valid arguments
		if(!(timer==1 || timer == 2 || timer == 4) ) 												{return INVALID_ARGUMENT;}
		else if( !( (channel>=1) && (channel<=4) ) ) 												{return INVALID_ARGUMENT;}
		else if( !( (duty_init >= DUTY_CYCLE_MIN) && (duty_init <= DUTY_CYCLE_MAX) ) ) 				{return INVALID_ARGUMENT;}
		else if( !( (duty_target >= DUTY_CYCLE_MIN) && (duty_target <= DUTY_CYCLE_MAX) ) ) 			{return INVALID_ARGUMENT;}
		else if( !( (duty_step_size >= DUTY_CYCLE_MIN) && ( duty_step_size <= DUTY_CYCLE_MAX ) ) ) 	{return INVALID_ARGUMENT;}
		else if( !( (duty_init >= DUTY_CYCLE_MIN) && ( duty_init <= DUTY_CYCLE_MAX ) ) ) 			{return INVALID_ARGUMENT;}
		else if( !( (is_positive_logic == 0) || (is_positive_logic == 1) ) )						{return INVALID_ARGUMENT;}

		current_duty = duty_init;
	}

	if( current_duty == duty_target )
	{
		return VALID;
	}
	else if( current_duty > duty_target )
	{
		//make sure duty does not go below min
		current_duty = ( (current_duty - duty_step_size) >= DUTY_CYCLE_MIN ) ? (current_duty - duty_step_size) : DUTY_CYCLE_MIN;
	}
	else if( current_duty < duty_target )
	{
		//make sure duty does not go above max
		current_duty = ( (current_duty + duty_step_size) <= DUTY_CYCLE_MAX ) ? (current_duty + duty_step_size) : DUTY_CYCLE_MAX;
	}

	PWM_Stop(timer, channel);

	if (is_positive_logic)
	{
		PWM_Init(timer, channel, current_duty);
	}
	else
	{
		PWM_Init(timer, channel, DUTY_CYCLE_MAX - current_duty);
	}

	return VALID;
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
