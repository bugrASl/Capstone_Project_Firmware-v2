/**
 *  @file   main.h
 *  @brief  BSAU main header — HAL includes, error handler, pin definitions.
 */

/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
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
#include "stm32l4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "bsau_config.h"
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

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define ADC_CHANNEL_0_Pin GPIO_PIN_0
#define ADC_CHANNEL_0_GPIO_Port GPIOA
#define ADC_CHANNEL_1_Pin GPIO_PIN_1
#define ADC_CHANNEL_1_GPIO_Port GPIOA
#define ADC_CHANNEL_2_Pin GPIO_PIN_2
#define ADC_CHANNEL_2_GPIO_Port GPIOA
#define ADC_CHANNEL_3_Pin GPIO_PIN_3
#define ADC_CHANNEL_3_GPIO_Port GPIOA
#define ADC_CHANNEL_4_Pin GPIO_PIN_4
#define ADC_CHANNEL_4_GPIO_Port GPIOA
#define ADC_CHANNEL_5_Pin GPIO_PIN_5
#define ADC_CHANNEL_5_GPIO_Port GPIOA
#define ADC_CHANNEL_6_Pin GPIO_PIN_6
#define ADC_CHANNEL_6_GPIO_Port GPIOA
#define ADC_CHANNEL_7_Pin GPIO_PIN_7
#define ADC_CHANNEL_7_GPIO_Port GPIOA
#define ADC_CHANNEL_BATT_Pin GPIO_PIN_0
#define ADC_CHANNEL_BATT_GPIO_Port GPIOB
#define NRF_CE_Pin GPIO_PIN_8
#define NRF_CE_GPIO_Port GPIOA
#define STATUS_LED_Pin GPIO_PIN_15
#define STATUS_LED_GPIO_Port GPIOA
#define NRF_SCK_Pin GPIO_PIN_3
#define NRF_SCK_GPIO_Port GPIOB
#define NRF_MISO_Pin GPIO_PIN_4
#define NRF_MISO_GPIO_Port GPIOB
#define NRF_MOSI_Pin GPIO_PIN_5
#define NRF_MOSI_GPIO_Port GPIOB
#define NRF_IRQ_Pin GPIO_PIN_6
#define NRF_IRQ_GPIO_Port GPIOB
#define NRF_CSN_Pin GPIO_PIN_7
#define NRF_CSN_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

