/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file    gpio.h
 * @brief   This file contains all the function prototypes for
 *          the gpio.c file
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
#ifndef __GPIO_H__
#define __GPIO_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* USER CODE BEGIN Private defines */
  typedef struct _GPIO_Value
  {
    GPIO_TypeDef *GPIOx;
    uint16_t GPIO_PIN_x;
  } GPIO_Value;

  extern GPIO_Value LED_WHITE;
  extern GPIO_Value LED_BLUE;
  extern GPIO_Value LED_GREEN;
  extern GPIO_Value LED_YELLOW;
  extern GPIO_Value LED_CAN_TX;
  extern GPIO_Value LED_CAN_RX;

  extern GPIO_Value USER_SW;
  extern GPIO_Value SPI1_CS_DRV;
  extern GPIO_Value SPI1_CS_ENC;

  extern GPIO_Value HALL_U;
  extern GPIO_Value HALL_V;
  extern GPIO_Value HALL_W;

  extern GPIO_Value DRV_nFAULT;
  extern GPIO_Value DRV_ENABLE;
  extern GPIO_Value DRV_CAL;
  extern GPIO_Value DRV_INLx;

  typedef struct _Access_Lamp
  {
    FunctionalState FDCAN_TX;
    FunctionalState FDCAN_RX;
  } Access_Lamp;

  extern Access_Lamp BLMD_Access_Lamp;
/* USER CODE END Private defines */

void MX_GPIO_Init(void);

/* USER CODE BEGIN Prototypes */
  GPIO_PinState Read_GPIO(GPIO_Value GPIO);
  void Toggle_GPIO(GPIO_Value GPIO);
  void Write_GPIO(GPIO_Value GPIO, GPIO_PinState PinState);

  void SetTxLED(GPIO_PinState PinState);
  void ActivateTxLED(void);
  void ResetTxLED(void);

  void SetRxLED(GPIO_PinState PinState);
  void ActivateRxLED(void);
  void ResetRxLED(void);
/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif
#endif /*__ GPIO_H__ */

