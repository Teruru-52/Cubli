/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file    fdcan.h
 * @brief   This file contains all the function prototypes for
 *          the fdcan.c file
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
#ifndef __FDCAN_H__
#define __FDCAN_H__

#ifdef __cplusplus
extern "C"
{
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

  /* USER CODE BEGIN Includes */
  /* USER CODE END Includes */

  extern FDCAN_HandleTypeDef hfdcan1;

/* USER CODE BEGIN Private defines */
/* MB -> BLMD */
#define CAN_ID_MD1 0x101
#define CAN_ID_MD2 0x102
#define CAN_ID_MD3 0x103
/* BLMD -> MB */
#define CAN_ID_MB 0x100
  // FDCAN_RxHeaderTypeDef RxHeader;
  /* USER CODE END Private defines */

  void MX_FDCAN1_Init(void);

  /* USER CODE BEGIN Prototypes */
  void FDCAN_Send(uint8_t *pTxData);
  // uint8_t RxData[8];
  /* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __FDCAN_H__ */
