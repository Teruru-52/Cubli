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

extern TIM_HandleTypeDef htim15;

/* USER CODE BEGIN Private defines */
#define TIM_CLOCK_DIVIDER 1
#define ADV_TIM_CLK_MHz 150
#define PWM_FREQUENCY 30000
#define PWM_PERIOD_CYCLES (uint16_t)(ADV_TIM_CLK_MHz * \
                                     (uint32_t)1000000u / ((uint32_t)(PWM_FREQUENCY)))
/* USER CODE END Private defines */

void MX_TIM1_Init(void);
void MX_TIM3_Init(void);
void MX_TIM4_Init(void);
void MX_TIM15_Init(void);

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* USER CODE BEGIN Prototypes */
  typedef struct _BLDC_PWM
  {
    FunctionalState OutputPWM;
    TIM_HandleTypeDef *htim;
    uint32_t Channel_U;
    uint32_t Channel_V;
    uint32_t Channel_W;
  } BLDC_PWM;

  void Blmd_TIM_Init(void);
  void PWM_Start(BLDC_PWM *bldc_pwm);
  void PWM_Stop(BLDC_PWM *bldc_pwm);
  void PWM_Set(BLDC_PWM *bldc_pwm, float CHu, float CHv, float CHw);
  void PWM_Update(BLDC_PWM *bldc_pwm, float Duty_u, float Duty_v, float Duty_w);

  extern BLDC_PWM blcd_pwm;
/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __TIM_H__ */

