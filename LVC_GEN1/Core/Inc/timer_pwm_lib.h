/*
 *  timer_pwm_lib.h
 *
 *  Created on: Jan 6, 2021
 *  Author: Samuel Parent and Dorian Knight
 */

#ifndef INC_TIMER_PWM_LIB_H_
#define INC_TIMER_PWM_LIB_H_

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;

const uint16_t COUNTER_PERIOD = 20000 - 1; //Global Maximum number that the stm32 will count to before resetting

/* Private function prototypes -----------------------------------------------*/
static void MX_TIM2_Init(uint32_t prescaler, uint8_t channel_1, uint8_t channel_2, uint8_t channel_3, uint8_t channel_4);
static void MX_TIM1_Init(uint32_t prescaler, uint8_t channel_1, uint8_t channel_2, uint8_t channel_3, uint8_t channel_4);
static void Timer_Init_Base(uint8_t timer, uint16_t period, uint8_t do_trigger_ISR, uint8_t channel_1, uint8_t channel_2, uint8_t channel_3, uint8_t channel_4, uint16_t ISR_period);
uint32_t Calculate_Prescaler(uint16_t period);
static void Calculate_Timer_Period_Multiplier(uint8_t timer,uint16_t period, uint16_t ISR_period);
static void PWM_Init(uint8_t timer, uint8_t channel, uint8_t duty_cycle);
uint16_t Calculate_Ticks_On_Per_Cycle(uint8_t duty_cycle);
static void PWM_Stop(uint8_t timer,uint8_t channel);
static void Timer_Stop(uint8_t timer);

//Macro definitions
#define Timer_Init(...) Var_Timer_Init((Timer_Init_args){__VA_ARGS__});

/* Private user code ---------------------------------------------------------*/
// Used for ISR on timer overflow function
uint16_t timer_1_repetition_counter = 0;
uint16_t timer_2_repetition_counter = 0;

//Values below are calculated in the Calculate_Timer_Period_Multiplier
uint16_t TIMER_1_PERIOD_MULTIPLIER; //(Timer period) x (TIMER_1_PERIOD_MULTIPLIER) = period for timer 1 ISR
uint16_t TIMER_2_PERIOD_MULTIPLIER; //(Timer period) x (TIMER_2_PERIOD_MULTIPLIER) = period for timer 2 ISR
//Setting up the default variables
typedef struct
{
	uint8_t timer;
	uint16_t period;
	uint8_t do_trigger_ISR;
	uint8_t channel_1;
	uint8_t channel_2;
	uint8_t channel_3;
	uint8_t channel_4;
	uint16_t ISR_period;
} Timer_Init_args;

/**
  * @brief set up default values for Timer_Init function
  * @param Timer_Init_args
  * @retval None
  */
static void Var_Timer_Init(Timer_Init_args in)
{
	uint8_t timer_out = in.timer;
	uint16_t period_out = in.period;
	uint8_t do_trigger_ISR_out = in.do_trigger_ISR ? in.do_trigger_ISR :1;
	uint8_t channel_1_out = in.channel_1 ? in.channel_1 :1;
	uint8_t channel_2_out = in.channel_2 ? in.channel_2 :1;
	uint8_t channel_3_out = in.channel_3 ? in.channel_3 :1;
	uint8_t channel_4_out = in.channel_4 ? in.channel_4 :1;
	uint16_t ISR_period_out = in.ISR_period ? in.ISR_period :in.period;
	Timer_Init_Base(timer_out,period_out,do_trigger_ISR_out,channel_1_out,channel_2_out,channel_3_out,channel_4_out,ISR_period_out);
}

/**
  * @brief TIM1 Initialization Function
  * @param Prescaler, Channel 1 init, Channel 2 init, Channel 3 init, Channel 4 init,
  * @retval None
  */
static void MX_TIM1_Init(uint32_t prescaler, uint8_t channel_1, uint8_t channel_2, uint8_t channel_3, uint8_t channel_4)
{
  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  htim1.Init.Prescaler = prescaler;
  htim1.Init.Period = COUNTER_PERIOD;

  htim1.Instance = TIM1;
  htim1.Init.Prescaler = prescaler;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = COUNTER_PERIOD;
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

  //Channel configuration
  if (channel_1==1)
  {
	  HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1);
  }
  if (channel_2==1)
  {
  	  HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2);
  }
  if (channel_3==1)
  {
  	  HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3);
  }
  if (channel_4==1)
  {
  	  HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4);
  }

  HAL_TIM_MspPostInit(&htim1);
}

/**
  * @brief TIM2 Initialization Function
  * @param Prescaler, Channel 1 init, Channel 2 init, Channel 3 init, Channel 4 init,
  * @retval None
  */
static void MX_TIM2_Init(uint32_t prescaler, uint8_t channel_1, uint8_t channel_2, uint8_t channel_3, uint8_t channel_4)
{
  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = prescaler;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = COUNTER_PERIOD;
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

  //Channel configuration
  if (channel_1==1)
  {
	  HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1);
  }
  if (channel_2==1)
  {
	  HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2);
  }
  if (channel_3==1)
  {
	  HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3);
  }
  if (channel_4==1)
  {
	  HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_4);
  }

  HAL_TIM_MspPostInit(&htim2);
}

/**
  * @brief Initialize timer n on channel m for s milli seconds with ISR
  * @param Timer number, period (milliseconds), Trigger the Interrupt Service Routine(2=True 1=False), Channel 1 init, Channel 2 init, Channel 3 init, Channel 4 init, ISR_period (milliseconds)
  * @retval None
  */
static void Timer_Init_Base(uint8_t timer, uint16_t period, uint8_t do_trigger_ISR, uint8_t channel_1, uint8_t channel_2, uint8_t channel_3, uint8_t channel_4, uint16_t ISR_period)
{
	//Checks to see that all arguments passed to the function are valid (*note that -1 evaluates to 255)
	if ((timer >= 1 && timer <= 2) && (period > 0 && period < 65536) && (do_trigger_ISR == 255 || do_trigger_ISR == 1 || do_trigger_ISR == 0) && (channel_1 == 255 || channel_1 == 1 || channel_1 == 0) && (channel_2 == 255 || channel_2 == 1 || channel_2 == 0) && (channel_3 == 255 || channel_3 == 1 || channel_3 == 0) && (channel_4 == 255 || channel_4 == 1 || channel_4 == 0) && ISR_period % period == 0)
	{
		uint32_t prescaler = Calculate_Prescaler(period); //max period value is 2^16-1
		Calculate_Timer_Period_Multiplier(timer, period, ISR_period); //This allows you to slow down the ISR_period in relation to the timer period

		switch (timer)
		{
			case (1):
				timer_1_repetition_counter = 0; //Because the timer is being reinitialized the timer count must be reset
				MX_TIM1_Init(prescaler,channel_1,channel_2,channel_3,channel_4);
				if(do_trigger_ISR==1)
				{
					HAL_TIM_Base_Start_IT(&htim1);
				}
				break;
			case (2):
				timer_2_repetition_counter = 0;
				MX_TIM2_Init(prescaler,channel_1,channel_2,channel_3,channel_4);
				if(do_trigger_ISR==1)
				{
					HAL_TIM_Base_Start_IT(&htim2);
				}
				break;
			default:
				printf("'Timer_Init(%d, %d, ...)' - This is the default message for timer switch, check timer_pwm_lib.h\n", timer, period);

		}
	}

	//Argument specific error statements
	if (!(timer >= 1 && timer <= 2))
	{
		printf("'Timer_Init(%d, %d, ...)' - Invalid timer argument. Should be either '1' or '2'.\n", timer, period);
	}
	if (!(period > 0 && period < 65536))
	{
		printf("'Timer_Init(%d, %d, ...)' - Invalid period argument. Should be an int in range of (0,65536).\n", timer, period);
	}
	if (!(do_trigger_ISR == 255 || do_trigger_ISR == 1 || do_trigger_ISR == 0))
	{
		printf("'Timer_Init(%d, %d, ...)' - Invalid do_trigger_ISR argument. Should be either '-1' or '1'.\n", timer, period);
	}
	if (!(ISR_period % period == 0))
	{
		printf("'Timer_Init(%d, %d, ...)' - Invalid ISR_period argument. Should be a multiple of the timer period.\n", timer, period);
	}
	if (!((channel_1 == 255 || channel_1 == 1 || channel_1 == 0) && (channel_2 == 255 || channel_2 == 1 || channel_2 == 0) && (channel_3 == 255 || channel_3 == 1 || channel_3 == 0) && (channel_4 == 255 || channel_4 == 1 || channel_4 == 0)))
	{
		printf("'Timer_Init(%d, %d, ...)' - Invalid channel_x argument. Should be either '-1' or '1'.\n", timer, period);
	}
}

/**
  * @brief Calculate the prescaler necessary to operate desired PWM
  * @param period
  * @retval prescaler
  */
uint32_t Calculate_Prescaler(uint16_t period)
{
	uint32_t prescaler = ((HAL_RCC_GetSysClockFreq()*(float)period)/(COUNTER_PERIOD+1))-1;
	prescaler /= 1000;
	return prescaler;
}

/**
  * @brief Calculate the value stored in TIMER_1_PERIOD_MULTIPLIER for the ISR
  * @param timer number, period and ISR_period
  * @retval None
  * @note: copy this into the user code 4 section
  */
static void Calculate_Timer_Period_Multiplier(uint8_t timer,uint16_t period, uint16_t ISR_period)
{
	switch (timer)
	{
		case(1):
			TIMER_1_PERIOD_MULTIPLIER = ISR_period/period;
			break;
		case(2):
			TIMER_2_PERIOD_MULTIPLIER = ISR_period/period;
			break;
	}
}

/**
  * @brief Take initialized timer, channel and desired duty cycle and initialize PWM
  * @param Timer, channel, duty cycle
  * @retval None
  */
static void PWM_Init(uint8_t timer, uint8_t channel, uint8_t duty_cycle)
{
	if (timer >= 1 && timer <= 2 && channel >= 1 && channel <= 4 && duty_cycle >= 0 && duty_cycle <= 100)
	{
		TIM_HandleTypeDef* desired_timer;
		uint32_t desired_channel;

		//Calculate the pulse
		uint16_t ticks_on_per_cycle = Calculate_Ticks_On_Per_Cycle(duty_cycle);

		//Find desired timer
		switch(timer)
		{
			case(1):
				desired_timer = &htim1;
				break;
			case(2):
				desired_timer = &htim2;
				break;
			default:
				printf("'PWM_Init(%d, %d, %d)' - This is the default message for timer switch, check timer_pwm_lib.h.\n", timer, channel, duty_cycle);
		}

		//Find desired channel
		switch(channel)
		{
			case(1):
				desired_channel = TIM_CHANNEL_1;
				break;
			case(2):
				desired_channel = TIM_CHANNEL_2;
				break;
			case(3):
				desired_channel = TIM_CHANNEL_3;
				break;
			case(4):
				desired_channel = TIM_CHANNEL_4;
				break;
			default:
				printf("'PWM_Init(%d, %d, %d)' - This is the default message for channel switch, check timer_pwm_lib.h.\n", timer, channel, duty_cycle);
		}

		//Initiate the PWM and desired duty cycle
		HAL_TIM_PWM_Start(desired_timer, desired_channel);
		__HAL_TIM_SET_COMPARE(desired_timer, desired_channel, ticks_on_per_cycle);
	}

	if (!(timer >= 1 && timer <= 2))
		{
			printf("'PWM_Init(%d, %d, %d)' - Invalid timer argument. Should be either '1' or '2'.\n", timer, channel, duty_cycle);
		}
	if (!(channel >= 1 && channel <= 4))
	{
		printf("'PWM_Init(%d, %d, %d)' - Invalid Channel argument. Should be an integer within range [1,4].\n", timer, channel, duty_cycle);
	}
	if (!(duty_cycle >= 0 && duty_cycle <= 100))
	{
		printf("'PWM_Init(.duty_cycle = %d)' - Invalid Duty Cycle argument. Should be within range [0,100].\n", duty_cycle);
	}
}

/**
  * @brief Calculate the duty cycle in ticks
  * @param duty cycle
  * @retval Ticks per cycle
  */
uint16_t Calculate_Ticks_On_Per_Cycle(uint8_t duty_cycle)
{
	uint16_t ticks_on_per_cycle;

	ticks_on_per_cycle = (duty_cycle*COUNTER_PERIOD)/100;

	return ticks_on_per_cycle;
}

/**
  * @brief Stop PWM signal from timer n on channel m
  * @param Timer, channel
  * @retval None
  */
static void PWM_Stop(uint8_t timer, uint8_t channel)
{
	TIM_HandleTypeDef* desired_timer;
	uint32_t desired_channel;

	//Error flag
	_Bool all_clear = 1;

	//Find timer
	switch(timer)
	{
		case(1):
			desired_timer = &htim1;
			break;
		case(2):
			desired_timer = &htim2;
			break;
		default:
			all_clear = 0;
			printf("'PWM_Stop(%d, %d)' - Invalid Timer argument. Should be either '1' or '2'.\n", timer, channel);
	}

	//Find channel
	switch(channel)
	{
		case(1):
			desired_channel = TIM_CHANNEL_1;
			break;
		case(2):
			desired_channel = TIM_CHANNEL_2;
			break;
		case(3):
			desired_channel = TIM_CHANNEL_3;
			break;
		case(4):
			desired_channel = TIM_CHANNEL_4;
			break;
		default:
			all_clear = 0;
			printf("'PWM_Stop(%d, %d)' - Invalid Channel argument. Should be an integer within range [1,4].\n", timer, channel);
	}

	if (all_clear)
	{
		//Stop PWM signal
		HAL_TIM_PWM_Stop(desired_timer, desired_channel);
	}

}

/**
  * @brief Stop timer n
  * @param Timer
  * @retval None
  */
static void Timer_Stop(uint8_t timer)
{
	TIM_HandleTypeDef* desired_timer;

	//Error flag
	_Bool all_clear = 1;

	//Find timer
	switch(timer)
	{
		case(1):
			desired_timer = &htim1;
			break;
		case(2):
			desired_timer = &htim2;
			break;
		default:
			all_clear = 0;
			printf("Timer_Stop(%d)' - Invalid Timer argument. Should be either '1' or '2'.\n");
	}

	if(all_clear)
	{
		//Stop timer
		HAL_TIM_Base_DeInit(desired_timer);
	}
}

#endif /* INC_TIMER_PWM_LIB_H_ */
