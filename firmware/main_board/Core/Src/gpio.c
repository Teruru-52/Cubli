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
GPIO_Value LED_YELLOW = {LED1_GPIO_Port, LED1_Pin};
GPIO_Value LED_GREEN = {LED2_GPIO_Port, LED2_Pin};
GPIO_Value LED_BLUE = {LED3_GPIO_Port, LED3_Pin};
GPIO_Value LED_WHITE = {LED4_GPIO_Port, LED4_Pin};
GPIO_Value LED_CAN_TX = {LED_CAN_TX_GPIO_Port, LED_CAN_TX_Pin};
GPIO_Value LED_CAN_RX = {LED_CAN_RX_GPIO_Port, LED_CAN_RX_Pin};

GPIO_Value USER_SW = {SW_GPIO_Port, SW_Pin};
GPIO_Value SPI_CS_IMU1 = {SPI_CS_IMU1_GPIO_Port, SPI_CS_IMU1_Pin};
GPIO_Value SPI_CS_IMU2 = {SPI_CS_IMU2_GPIO_Port, SPI_CS_IMU2_Pin};
GPIO_Value SPI_CS_IMU3 = {SPI_CS_IMU3_GPIO_Port, SPI_CS_IMU3_Pin};
GPIO_Value SPI_CS_IMU4 = {SPI_CS_IMU4_GPIO_Port, SPI_CS_IMU4_Pin};
GPIO_Value SPI_CS_IMU5 = {SPI_CS_IMU5_GPIO_Port, SPI_CS_IMU5_Pin};
GPIO_Value SPI_CS_IMU6 = {SPI_CS_IMU6_GPIO_Port, SPI_CS_IMU6_Pin};

GPIO_Value SPI_CS_LCD = {SPI_CS_LCD_GPIO_Port, SPI_CS_LCD_Pin};
GPIO_Value LCD_DC = {LCD_DC_GPIO_Port, LCD_DC_Pin};
GPIO_Value LCD_RST = {LCD_RST_GPIO_Port, LCD_RST_Pin};

Access_Lamp MB_Access_Lamp = {DISABLE, DISABLE};
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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, SPI_CS_IMU2_Pin|LED1_Pin|LED_CAN_TXC7_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, SW1_Pin|LCD_RST_Pin|LCD_DC_Pin|SPI_CS_LCD_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LED2_Pin|LED3_Pin|LED4_Pin|LED_CAN_RX_Pin
                          |LED_CAN_TX_Pin|SPI_CS_IMU5_Pin|SPI_CS_IMU6_Pin|SPI_CS_IMU1_Pin
                          |SPI_CS_IMU4_Pin|SPI_CS_IMU3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : PCPin PCPin PCPin */
  GPIO_InitStruct.Pin = SPI_CS_IMU2_Pin|LED1_Pin|LED_CAN_TXC7_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PAPin PAPin PAPin PAPin */
  GPIO_InitStruct.Pin = SW1_Pin|LCD_RST_Pin|LCD_DC_Pin|SPI_CS_LCD_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PBPin PBPin PBPin PBPin
                           PBPin PBPin PBPin PBPin
                           PBPin PBPin */
  GPIO_InitStruct.Pin = LED2_Pin|LED3_Pin|LED4_Pin|LED_CAN_RX_Pin
                          |LED_CAN_TX_Pin|SPI_CS_IMU5_Pin|SPI_CS_IMU6_Pin|SPI_CS_IMU1_Pin
                          |SPI_CS_IMU4_Pin|SPI_CS_IMU3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PA15 */
  GPIO_InitStruct.Pin = GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

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

void SetTxLED(GPIO_PinState PinState)
{
  Write_GPIO(LED_CAN_TX, PinState);
}

void ActivateTxLED(void)
{
  if (MB_Access_Lamp.CAN_TX == ENABLE)
  {
    Write_GPIO(LED_CAN_TX, GPIO_PIN_SET);
  }
}

void ResetTxLED(void)
{
  MB_Access_Lamp.CAN_TX = DISABLE;
}

void SetRxLED(GPIO_PinState PinState)
{
  Write_GPIO(LED_CAN_RX, PinState);
}

void ActivateRxLED(void)
{
  if (MB_Access_Lamp.CAN_RX == ENABLE)
  {
    Write_GPIO(LED_CAN_RX, GPIO_PIN_SET);
  }
}

void ResetRxLED(void)
{
  MB_Access_Lamp.CAN_RX = DISABLE;
}
/* USER CODE END 2 */
