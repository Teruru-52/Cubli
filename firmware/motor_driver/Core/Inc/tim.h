/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file    tim.h
 * @brief   This file contains all the function prototypes for
 *          the tim.c file
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
#ifndef __TIM_H__
#define __TIM_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

extern TIM_HandleTypeDef htim1;

extern TIM_HandleTypeDef htim3;

extern TIM_HandleTypeDef htim4;

/* USER CODE BEGIN Private defines */
#define TIM_CLOCK_DIVIDER 1
#define ADV_TIM_CLK_MHz 168
#define PWM_FREQUENCY 30000
#define PWM_PERIOD_CYCLES (uint16_t)(ADV_TIM_CLK_MHz * \
                                     (uint32_t)1000000u / ((uint32_t)(PWM_FREQUENCY)))
/* USER CODE END Private defines */

void MX_TIM1_Init(void);
void MX_TIM3_Init(void);
void MX_TIM4_Init(void);

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* USER CODE BEGIN Prototypes */
  typedef struct _BLDC_PWM
  {
    TIM_HandleTypeDef *htim;
    uint32_t Channel_U;
    uint32_t Channel_V;
    uint32_t Channel_W;
  } BLDC_PWM;

  void Blmd_TIM_Init(void);
  void _configure3PWM(BLDC_PWM *bldc_pwm);
  void _writeDutyCycle3PWM(BLDC_PWM *bldc_pwm, float duty_u, float duty_v, float duty_w);

  extern BLDC_PWM blcd_pwm;
/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __TIM_H__ */

