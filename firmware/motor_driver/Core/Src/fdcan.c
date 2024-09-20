/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file    fdcan.c
 * @brief   This file provides code for the configuration
 *          of the FDCAN instances.
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
/* Includes ------------------------------------------------------------------*/
#include "fdcan.h"

/* USER CODE BEGIN 0 */
extern void FDCANReceiveCallback(uint8_t *pRxData);
/* USER CODE END 0 */

FDCAN_HandleTypeDef hfdcan1;

/* FDCAN1 init function */
void MX_FDCAN1_Init(void)
{

  /* USER CODE BEGIN FDCAN1_Init 0 */
  FDCAN_FilterTypeDef FDCAN1_sFilterConfig;
  /* USER CODE END FDCAN1_Init 0 */

  /* USER CODE BEGIN FDCAN1_Init 1 */

  /* USER CODE END FDCAN1_Init 1 */
  hfdcan1.Instance = FDCAN1;
  hfdcan1.Init.ClockDivider = FDCAN_CLOCK_DIV1;
  hfdcan1.Init.FrameFormat = FDCAN_FRAME_CLASSIC;
  hfdcan1.Init.Mode = FDCAN_MODE_NORMAL;
  hfdcan1.Init.AutoRetransmission = DISABLE;
  hfdcan1.Init.TransmitPause = DISABLE;
  hfdcan1.Init.ProtocolException = DISABLE;
  hfdcan1.Init.NominalPrescaler = 12;
  hfdcan1.Init.NominalSyncJumpWidth = 1;
  hfdcan1.Init.NominalTimeSeg1 = 11;
  hfdcan1.Init.NominalTimeSeg2 = 2;
  hfdcan1.Init.DataPrescaler = 12;
  hfdcan1.Init.DataSyncJumpWidth = 1;
  hfdcan1.Init.DataTimeSeg1 = 11;
  hfdcan1.Init.DataTimeSeg2 = 2;
  hfdcan1.Init.StdFiltersNbr = 1;
  hfdcan1.Init.ExtFiltersNbr = 0;
  hfdcan1.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
  if (HAL_FDCAN_Init(&hfdcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN FDCAN1_Init 2 */
  FDCAN1_sFilterConfig.IdType = FDCAN_STANDARD_ID;
  FDCAN1_sFilterConfig.FilterIndex = 0;
  // FDCAN1_sFilterConfig.FilterType = FDCAN_FILTER_DUAL;
  FDCAN1_sFilterConfig.FilterType = FDCAN_FILTER_MASK;
  FDCAN1_sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
  // FDCAN1_sFilterConfig.FilterID1 = CAN_ID_MB;
  FDCAN1_sFilterConfig.FilterID1 = 0;
  FDCAN1_sFilterConfig.FilterID2 = 0;
  if (HAL_FDCAN_ConfigFilter(&hfdcan1, &FDCAN1_sFilterConfig) != HAL_OK)
  {
    Error_Handler();
  }

  // if (HAL_FDCAN_ConfigInterruptLines(&hfdcan1, FDCAN_IT_GROUP_RX_FIFO0, FDCAN_INTERRUPT_LINE0) != HAL_OK)
  // {
  //   Error_Handler();
  // }
  // if (HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, FDCAN_TX_BUFFER0) != HAL_OK)
  // {
  //   Error_Handler();
  // }
  // if (HAL_FDCAN_ConfigInterruptLines(&hfdcan1, FDCAN_IT_GROUP_SMSG, FDCAN_INTERRUPT_LINE1) != HAL_OK)
  // {
  //   Error_Handler();
  // }
  // if (HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_TX_COMPLETE, FDCAN_TX_BUFFER0 | FDCAN_TX_BUFFER1 | FDCAN_TX_BUFFER2) != HAL_OK)
  // {
  //   Error_Handler();
  // }

  if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE END FDCAN1_Init 2 */

}

void HAL_FDCAN_MspInit(FDCAN_HandleTypeDef* fdcanHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};
  if(fdcanHandle->Instance==FDCAN1)
  {
  /* USER CODE BEGIN FDCAN1_MspInit 0 */

  /* USER CODE END FDCAN1_MspInit 0 */

  /** Initializes the peripherals clocks
  */
    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_FDCAN;
    PeriphClkInit.FdcanClockSelection = RCC_FDCANCLKSOURCE_PCLK1;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
    {
      Error_Handler();
    }

    /* FDCAN1 clock enable */
    __HAL_RCC_FDCAN_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**FDCAN1 GPIO Configuration
    PA11     ------> FDCAN1_RX
    PA12     ------> FDCAN1_TX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_11|GPIO_PIN_12;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF9_FDCAN1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* FDCAN1 interrupt Init */
    HAL_NVIC_SetPriority(FDCAN1_IT0_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(FDCAN1_IT0_IRQn);
  /* USER CODE BEGIN FDCAN1_MspInit 1 */

  /* USER CODE END FDCAN1_MspInit 1 */
  }
}

void HAL_FDCAN_MspDeInit(FDCAN_HandleTypeDef* fdcanHandle)
{

  if(fdcanHandle->Instance==FDCAN1)
  {
  /* USER CODE BEGIN FDCAN1_MspDeInit 0 */

  /* USER CODE END FDCAN1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_FDCAN_CLK_DISABLE();

    /**FDCAN1 GPIO Configuration
    PA11     ------> FDCAN1_RX
    PA12     ------> FDCAN1_TX
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_11|GPIO_PIN_12);

    /* FDCAN1 interrupt Deinit */
    HAL_NVIC_DisableIRQ(FDCAN1_IT0_IRQn);
  /* USER CODE BEGIN FDCAN1_MspDeInit 1 */

  /* USER CODE END FDCAN1_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */
void FDCAN_Send(uint8_t *pTxData)
{
  FDCAN_TxHeaderTypeDef FDCAN_TxHeader;
  FDCAN_TxHeader.Identifier = CAN_ID_MD1;
  FDCAN_TxHeader.IdType = FDCAN_STANDARD_ID;
  FDCAN_TxHeader.TxFrameType = FDCAN_DATA_FRAME;
  FDCAN_TxHeader.DataLength = FDCAN_DLC_BYTES_8;
  FDCAN_TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
  // FDCAN_TxHeader.BitRateSwitch = FDCAN_BRS_ON;
  FDCAN_TxHeader.BitRateSwitch = FDCAN_BRS_OFF;
  // FDCAN_TxHeader.FDFormat = FDCAN_FD_CAN;
  FDCAN_TxHeader.FDFormat = FDCAN_CLASSIC_CAN;
  FDCAN_TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
  FDCAN_TxHeader.MessageMarker = 0;
  uint8_t TxData[8];
  for (uint8_t i = 0; i < 8; i++)
  {
    TxData[i] = pTxData[i];
  }
  // printf("FreeLevel = %ld\n", HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan1));
  if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &FDCAN_TxHeader, TxData) != HAL_OK)
  {
    Error_Handler();
  }
  BLMD_Access_Lamp.FDCAN_TX = ENABLE;
}

// これなぜか呼ばれてない
void HAL_FDCAN_TxBufferCompleteCallback(FDCAN_HandleTypeDef *hfdcan, uint32_t BufferIndexes)
{
  BLMD_Access_Lamp.FDCAN_TX = ENABLE;
}

uint8_t RxData[8];
FDCAN_RxHeaderTypeDef RxHeader;

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
  if (hfdcan->Instance == hfdcan1.Instance)
  {
    if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != RESET)
    {
      if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &RxHeader, RxData) != HAL_OK)
      {
        Error_Handler();
      }

      switch (RxHeader.Identifier)
      {
      case CAN_ID_MB:
        printf("id=%ld, data=%d\r\n", RxHeader.Identifier, RxData[0]);
        FDCANReceiveCallback(RxData);
        break;
      default:
        break;
      }
      BLMD_Access_Lamp.FDCAN_RX = ENABLE;
    }
  }
}
/* USER CODE END 1 */
