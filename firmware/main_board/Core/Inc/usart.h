/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file    usart.h
 * @brief   This file contains all the function prototypes for
 *          the usart.c file
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
#ifndef __USART_H__
#define __USART_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

extern UART_HandleTypeDef huart1;

/* USER CODE BEGIN Private defines */
#define RCV_BUFF_SIZE (20)
#define SND_BUFF_SIZE (20)
#define CHAR_LF (0x0a)
#define TRUE (1)
#define FALSE (0)
/* USER CODE END Private defines */

void MX_USART1_UART_Init(void);

/* USER CODE BEGIN Prototypes */
  uint8_t RdBuff[RCV_BUFF_SIZE];
  uint8_t SdBuff[SND_BUFF_SIZE];
  uint16_t Rd_ptr;
  uint16_t rcvLength;
  uint8_t rdData;

  uint16_t getUartWrPtr(UART_HandleTypeDef *huart);
  uint8_t isUartRcv(void);
  uint8_t rdUart(uint8_t *rdData);
  uint8_t Compare_Stop(uint8_t *sdbuff);
/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __USART_H__ */

