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
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */
#define BLMD_Serial_Number 1
// #define BLMD_Serial_Number 2
// #define BLMD_Serial_Number 3
/* MB -> BLMD */
#define CAN_ID_TX (BLMD_Serial_Number + 10)
/* Lightning5 -> MB */
#define CAN_ID_RX (CAN_ID_TX + 100)
/* S/N */
#define SerialNumber (BLMD_Serial_Number + 500)
/* USER CODE END Includes */

extern FDCAN_HandleTypeDef hfdcan1;

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

void MX_FDCAN1_Init(void);

/* USER CODE BEGIN Prototypes */
  void FDCAN_Send(uint8_t *pTxData, uint8_t Size);
  uint8_t FDCAN_Receive(uint32_t *id, uint8_t *pRxData);
  uint32_t FDCAN_Get_data_length_code(uint8_t length);
  uint8_t FDCAN_Get_data_length(uint32_t code);
/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __FDCAN_H__ */

