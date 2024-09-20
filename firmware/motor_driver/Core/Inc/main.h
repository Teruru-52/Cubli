/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.h
 * @brief          : Header for main.c file.
 *                   This file contains the common defines of the application.
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2023 STMicroelectronics.
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
#include "stm32g4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <math.h>
#include "arm_math.h"

#include "cmsis_os.h"
#include "adc.h"
#include "fdcan.h"
#include "spi.h"
#include "tim.h"
#include "gpio.h"
#include "SEGGER_RTT.h"
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
#define INLC_Pin GPIO_PIN_13
#define INLC_GPIO_Port GPIOC
#define INLB_Pin GPIO_PIN_14
#define INLB_GPIO_Port GPIOC
#define INLA_Pin GPIO_PIN_15
#define INLA_GPIO_Port GPIOC
#define CAL_Pin GPIO_PIN_3
#define CAL_GPIO_Port GPIOA
#define ENABLE_Pin GPIO_PIN_4
#define ENABLE_GPIO_Port GPIOA
#define SPI_CS_DRV_Pin GPIO_PIN_0
#define SPI_CS_DRV_GPIO_Port GPIOB
#define nFAULT_Pin GPIO_PIN_1
#define nFAULT_GPIO_Port GPIOB
#define SW_Pin GPIO_PIN_2
#define SW_GPIO_Port GPIOB
#define LED1_Pin GPIO_PIN_10
#define LED1_GPIO_Port GPIOB
#define LED2_Pin GPIO_PIN_11
#define LED2_GPIO_Port GPIOB
#define LED3_Pin GPIO_PIN_12
#define LED3_GPIO_Port GPIOB
#define LED4_Pin GPIO_PIN_14
#define LED4_GPIO_Port GPIOB
#define LED_CAN_TX_Pin GPIO_PIN_8
#define LED_CAN_TX_GPIO_Port GPIOA
#define LED_CAN_RX_Pin GPIO_PIN_9
#define LED_CAN_RX_GPIO_Port GPIOA
#define Hall_W_Pin GPIO_PIN_15
#define Hall_W_GPIO_Port GPIOA
#define Hall_W_EXTI_IRQn EXTI15_10_IRQn
#define Hall_U_Pin GPIO_PIN_3
#define Hall_U_GPIO_Port GPIOB
#define Hall_U_EXTI_IRQn EXTI3_IRQn
#define Hall_V_Pin GPIO_PIN_4
#define Hall_V_GPIO_Port GPIOB
#define Hall_V_EXTI_IRQn EXTI4_IRQn
#define SPI_CS_ENC_Pin GPIO_PIN_5
#define SPI_CS_ENC_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
