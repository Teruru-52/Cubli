/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file    spi.h
 * @brief   This file contains all the function prototypes for
 *          the spi.c file
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
#ifndef __SPI_H__
#define __SPI_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

extern SPI_HandleTypeDef hspi1;

extern SPI_HandleTypeDef hspi2;

/* USER CODE BEGIN Private defines */
  typedef struct _SPI_Value
  {
    SPI_HandleTypeDef *hspi;
    GPIO_Value *SPI_CS;
    uint16_t size;
    uint32_t timeout;
  } SPI_Value;

  extern SPI_Value spi_imu1;
  extern SPI_Value spi_imu2;
  extern SPI_Value spi_imu3;
  extern SPI_Value spi_imu4;
  extern SPI_Value spi_imu5;
  extern SPI_Value spi_imu6;
/* USER CODE END Private defines */

void MX_SPI1_Init(void);
void MX_SPI2_Init(void);

/* USER CODE BEGIN Prototypes */
  uint8_t Read_1byte(SPI_Value *spi_value, uint8_t reg);
  void Write_1byte(SPI_Value *spi_value, uint8_t reg, uint8_t data);
/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __SPI_H__ */

