/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file    gpio.c
 * @brief   This file provides code for the configuration
 *          of all used GPIO pins.
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
#include "gpio.h"

/* USER CODE BEGIN 0 */
GPIO_Value LED1 = {LED1_GPIO_Port, LED1_Pin};
GPIO_Value LED2 = {LED2_GPIO_Port, LED2_Pin};
GPIO_Value LED3 = {LED3_GPIO_Port, LED3_Pin};
GPIO_Value LED4 = {LED4_GPIO_Port, LED4_Pin};
GPIO_Value LED_CAN_TX = {LED_CAN_TX_GPIO_Port, LED_CAN_TX_Pin};
GPIO_Value LED_CAN_RX = {LED_CAN_RX_GPIO_Port, LED_CAN_RX_Pin};

GPIO_Value USER_SW = {SW_GPIO_Port, SW_Pin};
GPIO_Value SPI1_CS_DRV = {SPI_CS_DRV_GPIO_Port, SPI_CS_DRV_Pin};
GPIO_Value SPI1_CS_ENC = {SPI_CS_ENC_GPIO_Port, SPI_CS_ENC_Pin};

GPIO_Value HALL_U = {Hall_U_GPIO_Port, Hall_U_Pin};
GPIO_Value HALL_V = {Hall_V_GPIO_Port, Hall_V_Pin};
GPIO_Value HALL_W = {Hall_W_GPIO_Port, Hall_W_Pin};

GPIO_Value DRV_nFAULT = {nFAULT_GPIO_Port, nFAULT_Pin};
GPIO_Value DRV_ENABLE = {ENABLE_GPIO_Port, ENABLE_Pin};
GPIO_Value DRV_CAL = {CAL_GPIO_Port, CAL_Pin};
GPIO_Value DRV_INLx = {INLx_GPIO_Port, INLx_Pin};

Access_Lamp BLMD_Access_Lamp = {DISABLE, DISABLE};
/* USER CODE END 0 */

/*----------------------------------------------------------------------------*/
/* Configure GPIO                                                             */
/*----------------------------------------------------------------------------*/
/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/** Configure pins as
        * Analog
        * Input
        * Output
        * EVENT_OUT
        * EXTI
*/
void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, nFAULT_Pin|SPI_CS_DRV_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, CAL_Pin|INLx_Pin|LED_CAN_TX_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LED1_Pin|LED2_Pin|LED3_Pin|LED4_Pin
                          |LED_CAN_RX_Pin|SPI_CS_ENC_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : PtPin */
  GPIO_InitStruct.Pin = ENABLE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(ENABLE_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PCPin PCPin */
  GPIO_InitStruct.Pin = nFAULT_Pin|SPI_CS_DRV_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PAPin PAPin PAPin */
  GPIO_InitStruct.Pin = CAL_Pin|INLx_Pin|LED_CAN_TX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PBPin PBPin PBPin */
  GPIO_InitStruct.Pin = SW_Pin|Hall_U_Pin|Hall_V_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PBPin PBPin PBPin PBPin
                           PBPin PBPin */
  GPIO_InitStruct.Pin = LED1_Pin|LED2_Pin|LED3_Pin|LED4_Pin
                          |LED_CAN_RX_Pin|SPI_CS_ENC_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PtPin */
  GPIO_InitStruct.Pin = Hall_W_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(Hall_W_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 2 */
GPIO_PinState Read_GPIO(GPIO_Value GPIO)
{
  return HAL_GPIO_ReadPin(GPIO.GPIOx, GPIO.GPIO_PIN_x);
}

void Toggle_GPIO(GPIO_Value GPIO)
{
  HAL_GPIO_TogglePin(GPIO.GPIOx, GPIO.GPIO_PIN_x);
}

void Write_GPIO(GPIO_Value GPIO, GPIO_PinState PinState)
{
  HAL_GPIO_WritePin(GPIO.GPIOx, GPIO.GPIO_PIN_x, PinState);
}
/* USER CODE END 2 */
