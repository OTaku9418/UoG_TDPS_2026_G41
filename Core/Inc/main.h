/* USER CODE BEGIN Header */
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
#include "stm32f4xx_hal.h"

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
#define radar_ot2_Pin GPIO_PIN_13
#define radar_ot2_GPIO_Port GPIOC
#define User_Key_Pin GPIO_PIN_0
#define User_Key_GPIO_Port GPIOA
#define BT_TX_Pin GPIO_PIN_2
#define BT_TX_GPIO_Port GPIOA
#define BT_RX_Pin GPIO_PIN_3
#define BT_RX_GPIO_Port GPIOA
#define IR_X5_Pin GPIO_PIN_7
#define IR_X5_GPIO_Port GPIOA
#define IR_X4_Pin GPIO_PIN_5
#define IR_X4_GPIO_Port GPIOC
#define IR_X3_Pin GPIO_PIN_1
#define IR_X3_GPIO_Port GPIOB
#define IR_X2_Pin GPIO_PIN_8
#define IR_X2_GPIO_Port GPIOE
#define IR_X1_Pin GPIO_PIN_10
#define IR_X1_GPIO_Port GPIOE
#define TOF_TX_Pin GPIO_PIN_10
#define TOF_TX_GPIO_Port GPIOB
#define TOF_RX_Pin GPIO_PIN_11
#define TOF_RX_GPIO_Port GPIOB
#define lora_m1_Pin GPIO_PIN_8
#define lora_m1_GPIO_Port GPIOC
#define lora_aux_Pin GPIO_PIN_9
#define lora_aux_GPIO_Port GPIOC
#define radar_TX_Pin GPIO_PIN_10
#define radar_TX_GPIO_Port GPIOC
#define radar_RX_Pin GPIO_PIN_11
#define radar_RX_GPIO_Port GPIOC
#define lora_TX_Pin GPIO_PIN_12
#define lora_TX_GPIO_Port GPIOC
#define lora_RX_Pin GPIO_PIN_2
#define lora_RX_GPIO_Port GPIOD
#define lora_m0_Pin GPIO_PIN_3
#define lora_m0_GPIO_Port GPIOD

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
