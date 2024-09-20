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
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <math.h>
#include "arm_math.h"

// #include "can.h"
// #include "dma.h"
// #include "fatfs.h"
#include "sdio.h"
// #include "spi.h"
#include "tim.h"
// #include "usart.h"
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
#define SPI_CS_IMU2_Pin GPIO_PIN_13
#define SPI_CS_IMU2_GPIO_Port GPIOC
#define SW_Pin GPIO_PIN_0
#define SW_GPIO_Port GPIOA
#define LCD_RST_Pin GPIO_PIN_1
#define LCD_RST_GPIO_Port GPIOA
#define LCD_DC_Pin GPIO_PIN_2
#define LCD_DC_GPIO_Port GPIOA
#define SPI_CS_LCD_Pin GPIO_PIN_3
#define SPI_CS_LCD_GPIO_Port GPIOA
#define LED1_Pin GPIO_PIN_5
#define LED1_GPIO_Port GPIOC
#define LED2_Pin GPIO_PIN_0
#define LED2_GPIO_Port GPIOB
#define LED3_Pin GPIO_PIN_2
#define LED3_GPIO_Port GPIOB
#define LED4_Pin GPIO_PIN_11
#define LED4_GPIO_Port GPIOB
#define LED_CAN_TX_Pin GPIO_PIN_12
#define LED_CAN_TX_GPIO_Port GPIOB
#define LED_CAN_RX_Pin GPIO_PIN_15
#define LED_CAN_RX_GPIO_Port GPIOB
#define SPI_CS_IMU5_Pin GPIO_PIN_3
#define SPI_CS_IMU5_GPIO_Port GPIOB
#define SPI_CS_IMU6_Pin GPIO_PIN_4
#define SPI_CS_IMU6_GPIO_Port GPIOB
#define SPI_CS_IMU1_Pin GPIO_PIN_5
#define SPI_CS_IMU1_GPIO_Port GPIOB
#define SPI_CS_IMU4_Pin GPIO_PIN_6
#define SPI_CS_IMU4_GPIO_Port GPIOB
#define SPI_CS_IMU3_Pin GPIO_PIN_7
#define SPI_CS_IMU3_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
