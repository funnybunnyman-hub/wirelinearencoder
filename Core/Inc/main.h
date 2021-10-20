/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

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

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define ADC0_Pin GPIO_PIN_0
#define ADC0_GPIO_Port GPIOC
#define ADC1_Pin GPIO_PIN_1
#define ADC1_GPIO_Port GPIOC
#define ADC2_Pin GPIO_PIN_2
#define ADC2_GPIO_Port GPIOC
#define ADC3_Pin GPIO_PIN_3
#define ADC3_GPIO_Port GPIOC
#define ADC4_Pin GPIO_PIN_0
#define ADC4_GPIO_Port GPIOA
#define ADC5_Pin GPIO_PIN_1
#define ADC5_GPIO_Port GPIOA
#define ADC6_Pin GPIO_PIN_2
#define ADC6_GPIO_Port GPIOA
#define ADC7_Pin GPIO_PIN_3
#define ADC7_GPIO_Port GPIOA
#define ADC8_Pin GPIO_PIN_4
#define ADC8_GPIO_Port GPIOA
#define ADC9_Pin GPIO_PIN_5
#define ADC9_GPIO_Port GPIOA
#define ADC10_Pin GPIO_PIN_6
#define ADC10_GPIO_Port GPIOA
#define ADC11_Pin GPIO_PIN_7
#define ADC11_GPIO_Port GPIOA
#define ADC12_Pin GPIO_PIN_4
#define ADC12_GPIO_Port GPIOC
#define ADC13_Pin GPIO_PIN_5
#define ADC13_GPIO_Port GPIOC
#define ADC14_Pin GPIO_PIN_0
#define ADC14_GPIO_Port GPIOB
#define ADC15_Pin GPIO_PIN_1
#define ADC15_GPIO_Port GPIOB
#define WE_0A_Pin GPIO_PIN_2
#define WE_0A_GPIO_Port GPIOB
#define WE_0B_Pin GPIO_PIN_10
#define WE_0B_GPIO_Port GPIOB
#define WE_1A_Pin GPIO_PIN_11
#define WE_1A_GPIO_Port GPIOB
#define WE_1B_Pin GPIO_PIN_12
#define WE_1B_GPIO_Port GPIOB
#define WE_2A_Pin GPIO_PIN_13
#define WE_2A_GPIO_Port GPIOB
#define WE_2B_Pin GPIO_PIN_14
#define WE_2B_GPIO_Port GPIOB
#define WE_3A_Pin GPIO_PIN_15
#define WE_3A_GPIO_Port GPIOB
#define WE_3B_Pin GPIO_PIN_6
#define WE_3B_GPIO_Port GPIOC
#define WE_4A_Pin GPIO_PIN_7
#define WE_4A_GPIO_Port GPIOC
#define WE_4B_Pin GPIO_PIN_8
#define WE_4B_GPIO_Port GPIOC
#define WE_5A_Pin GPIO_PIN_9
#define WE_5A_GPIO_Port GPIOC
#define WE_5B_Pin GPIO_PIN_8
#define WE_5B_GPIO_Port GPIOA
#define WE_6A_Pin GPIO_PIN_9
#define WE_6A_GPIO_Port GPIOA
#define WE_6B_Pin GPIO_PIN_10
#define WE_6B_GPIO_Port GPIOA
#define WE_7A_Pin GPIO_PIN_11
#define WE_7A_GPIO_Port GPIOA
#define WE_7B_Pin GPIO_PIN_12
#define WE_7B_GPIO_Port GPIOA
/* USER CODE BEGIN Private defines */
#define BUFFER_SIZE_SPI3 8
#define ENCODER_NUMBER 3
#define ADC_NUMBER 3

typedef enum {
  AIRPRESSURE = 0x01,
  WIREENCODER = 0x02,
  READ_CONFIG = 0x03,
  CHECKING_SPI = 0x04,
} Controller_State;

struct Encoder_SIGNAL_Info {
  uint16_t A_pin;
  uint16_t B_pin;
  GPIO_TypeDef* A_port;
  GPIO_TypeDef* B_port;
};

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
