/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f7xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LV_BATTERY_VOLTAGE_Pin GPIO_PIN_0
#define LV_BATTERY_VOLTAGE_GPIO_Port GPIOC
#define LV_BATTERY_ISENSE_Pin GPIO_PIN_1
#define LV_BATTERY_ISENSE_GPIO_Port GPIOC
#define DCDC_VSENSE_Pin GPIO_PIN_2
#define DCDC_VSENSE_GPIO_Port GPIOC
#define DCDC_ISENSE_Pin GPIO_PIN_3
#define DCDC_ISENSE_GPIO_Port GPIOC
#define FAN_PWM_Pin GPIO_PIN_0
#define FAN_PWM_GPIO_Port GPIOA
#define POWERTRAIN_FAN_EN_Pin GPIO_PIN_0
#define POWERTRAIN_FAN_EN_GPIO_Port GPIOG
#define POWERTRAIN_PUMP_EN_Pin GPIO_PIN_1
#define POWERTRAIN_PUMP_EN_GPIO_Port GPIOG
#define FRONT_CONTROLLER_AND_ACC_EN_Pin GPIO_PIN_2
#define FRONT_CONTROLLER_AND_ACC_EN_GPIO_Port GPIOG
#define MOTOR_CONTROLLER_EN_Pin GPIO_PIN_3
#define MOTOR_CONTROLLER_EN_GPIO_Port GPIOG
#define DCDC_ON_LED_EN_Pin GPIO_PIN_4
#define DCDC_ON_LED_EN_GPIO_Port GPIOG
#define TSAL_EN_Pin GPIO_PIN_5
#define TSAL_EN_GPIO_Port GPIOG
#define SHUTDOWN_CIRCUIT_EN_Pin GPIO_PIN_6
#define SHUTDOWN_CIRCUIT_EN_GPIO_Port GPIOG
#define MOTOR_CONTROLLER_PRECHARGE_EN_Pin GPIO_PIN_7
#define MOTOR_CONTROLLER_PRECHARGE_EN_GPIO_Port GPIOG
#define DCDC_EN_Pin GPIO_PIN_8
#define DCDC_EN_GPIO_Port GPIOG
#define MUX_DCDC_VALID_Pin GPIO_PIN_0
#define MUX_DCDC_VALID_GPIO_Port GPIOE
#define MUX_LVBATT_VALID_Pin GPIO_PIN_1
#define MUX_LVBATT_VALID_GPIO_Port GPIOE
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
