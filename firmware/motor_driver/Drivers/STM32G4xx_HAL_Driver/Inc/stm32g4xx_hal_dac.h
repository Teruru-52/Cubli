/**
  ******************************************************************************
  * @file    stm32g4xx_hal_dac.h
  * @author  MCD Application Team
  * @brief   Header file of DAC HAL module.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef STM32G4xx_HAL_DAC_H
#define STM32G4xx_HAL_DAC_H

#ifdef __cplusplus
extern "C" {
#endif

/** @addtogroup STM32G4xx_HAL_Driver
  * @{
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32g4xx_hal_def.h"

#if defined(DAC1) || defined(DAC2) || defined(DAC3) ||defined (DAC4)

/** @addtogroup DAC
  * @{
  */

/* Exported types ------------------------------------------------------------*/

/** @defgroup DAC_Exported_Types DAC Exported Types
  * @{
  */

/**
  * @brief  HAL State structures definition
  */
typedef enum
{
  HAL_DAC_STATE_RESET             = 0x00U,  /*!< DAC not yet initialized or disabled  */
  HAL_DAC_STATE_READY             = 0x01U,  /*!< DAC initialized and ready for use    */
  HAL_DAC_STATE_BUSY              = 0x02U,  /*!< DAC internal processing is ongoing   */
  HAL_DAC_STATE_TIMEOUT           = 0x03U,  /*!< DAC timeout state                    */
  HAL_DAC_STATE_ERROR             = 0x04U   /*!< DAC error state                      */

} HAL_DAC_StateTypeDef;

/**
  * @brief  DAC handle Structure definition
  */
#if (USE_HAL_DAC_REGISTER_CALLBACKS == 1)
typedef struct __DAC_HandleTypeDef
#else
typedef struct
#endif /* USE_HAL_DAC_REGISTER_CALLBACKS */
{
  DAC_TypeDef                 *Instance;     /*!< Register base address             */

  __IO HAL_DAC_StateTypeDef   State;         /*!< DAC communication state           */

  HAL_LockTypeDef             Lock;          /*!< DAC locking object                */

  DMA_HandleTypeDef           *DMA_Handle1;  /*!< Pointer DMA handler for channel 1 */

  DMA_HandleTypeDef           *DMA_Handle2;  /*!< Pointer DMA handler for channel 2 */

  __IO uint32_t               ErrorCode;     /*!< DAC Error code                    */

#if (USE_HAL_DAC_REGISTER_CALLBACKS == 1)
  void (* ConvCpltCallbackCh1)(struct __DAC_HandleTypeDef *hdac);
  void (* ConvHalfCpltCallbackCh1)(struct __DAC_HandleTypeDef *hdac);
  void (* ErrorCallbackCh1)(struct __DAC_HandleTypeDef *hdac);
  void (* DMAUnderrunCallbackCh1)(struct __DAC_HandleTypeDef *hdac);

  void (* ConvCpltCallbackCh2)(struct __DAC_HandleTypeDef *hdac);
  void (* ConvHalfCpltCallbackCh2)(struct __DAC_HandleTypeDef *hdac);
  void (* ErrorCallbackCh2)(struct __DAC_HandleTypeDef *hdac);
  void (* DMAUnderrunCallbackCh2)(struct __DAC_HandleTypeDef *hdac);


  void (* MspInitCallback)(struct __DAC_HandleTypeDef *hdac);
  void (* MspDeInitCallback)(struct __DAC_HandleTypeDef *hdac);
#endif /* USE_HAL_DAC_REGISTER_CALLBACKS */

} DAC_HandleTypeDef;

/**
  * @brief   DAC Configuration sample and hold Channel structure definition
  */
typedef struct
{
  uint32_t DAC_SampleTime ;          /*!< Specifies the Sample time for the selected channel.
                                          This parameter applies when DAC_SampleAndHold is DAC_SAMPLEANDHOLD_ENABLE.
                                          This parameter must be a number between Min_Data = 0 and Max_Data = 1023 */

  uint32_t DAC_HoldTime ;            /*!< Specifies the hold time for the selected channel
                                          This parameter applies when DAC_SampleAndHold is DAC_SAMPLEANDHOLD_ENABLE.
                                          This parameter must be a number between Min_Data = 0 and Max_Data = 1023 */

  uint32_t DAC_RefreshTime ;         /*!< Specifies the refresh time for the selected channel
                                          This parameter applies when DAC_SampleAndHold is DAC_SAMPLEANDHOLD_ENABLE.
                                          This parameter must be a number between Min_Data = 0 and Max_Data = 255 */
} DAC_SampleAndHoldConfTypeDef;

/**
  * @brief   DAC Configuration regular Channel structure definition
  */
typedef struct
{
  uint32_t DAC_HighFrequency;            /*!< Specifies the frequency interface mode
                                              This parameter can be a value of @ref DAC_HighFrequency */

  FunctionalState DAC_DMADoubleDataMode; /*!< Specifies if DMA double data mode should be enabled or not for the selected channel.
                                              This parameter can be ENABLE or DISABLE */

  FunctionalState DAC_SignedFormat;      /*!< Specifies if signed format should be used or not for the selected channel.
                                              This parameter can be ENABLE or DISABLE */

  uint32_t DAC_SampleAndHold;            /*!< Specifies whether the DAC mode.
                                              This parameter can be a value of @ref DAC_SampleAndHold */

  uint32_t DAC_Trigger;                  /*!< Specifies the external trigger for the selected DAC channel.
                                              This parameter can be a value of @ref DAC_trigger_selection.
                                              Note: In case of sawtooth wave generation, this
                                              trigger corresponds to the reset trigger. */

  uint32_t DAC_Trigger2;                 /*!< Specifies the external secondary trigger for the selected DAC channel.
                                              This parameter can be a value of @ref DAC_trigger_selection.
                                              Note: In case of sawtooth wave generation, this
                                              trigger corresponds to the step trigger.*/

  uint32_t DAC_OutputBuffer;             /*!< Specifies whether the DAC channel output buffer is enabled or disabled.
                                               This parameter can be a value of @ref DAC_output_buffer */

  uint32_t DAC_ConnectOnChipPeripheral ; /*!< Specifies whether the DAC output is connected or not to on chip peripheral.
                                              This parameter can be a value of @ref DAC_ConnectOnChipPeripheral */

  uint32_t DAC_UserTrimming;             /*!< Specifies the trimming mode
                                              This parameter must be a value of @ref DAC_UserTrimming
                                              DAC_UserTrimming is either factory or user trimming */

  uint32_t DAC_TrimmingValue;             /*!< Specifies the offset trimming value
                                               i.e. when DAC_SampleAndHold is DAC_TRIMMING_USER.
                                               This parameter must be a number between Min_Data = 1 and Max_Data = 31 */
  DAC_SampleAndHoldConfTypeDef  DAC_SampleAndHoldConfig;  /*!< Sample and Hold settings */
} DAC_ChannelConfTypeDef;

#if (USE_HAL_DAC_REGISTER_CALLBACKS == 1)
/**
  * @brief  HAL DAC Callback ID enumeration definition
  */
typedef enum
{
  HAL_DAC_CH1_COMPLETE_CB_ID                 = 0x00U,  /*!< DAC CH1 Complete Callback ID      */
  HAL_DAC_CH1_HALF_COMPLETE_CB_ID            = 0x01U,  /*!< DAC CH1 half Complete Callback ID */
  HAL_DAC_CH1_ERROR_ID                       = 0x02U,  /*!< DAC CH1 error Callback ID         */
  HAL_DAC_CH1_UNDERRUN_CB_ID                 = 0x03U,  /*!< DAC CH1 underrun Callback ID      */

  HAL_DAC_CH2_COMPLETE_CB_ID                 = 0x04U,  /*!< DAC CH2 Complete Callback ID      */
  HAL_DAC_CH2_HALF_COMPLETE_CB_ID            = 0x05U,  /*!< DAC CH2 half Complete Callback ID */
  HAL_DAC_CH2_ERROR_ID                       = 0x06U,  /*!< DAC CH2 error Callback ID         */
  HAL_DAC_CH2_UNDERRUN_CB_ID                 = 0x07U,  /*!< DAC CH2 underrun Callback ID      */

  HAL_DAC_MSPINIT_CB_ID                      = 0x08U,  /*!< DAC MspInit Callback ID           */
  HAL_DAC_MSPDEINIT_CB_ID                    = 0x09U,  /*!< DAC MspDeInit Callback ID         */
  HAL_DAC_ALL_CB_ID                          = 0x0AU   /*!< DAC All ID                        */
} HAL_DAC_CallbackIDTypeDef;

/**
  * @brief  HAL DAC Callback pointer definition
  */
typedef void (*pDAC_CallbackTypeDef)(DAC_HandleTypeDef *hdac);
#endif /* USE_HAL_DAC_REGISTER_CALLBACKS */

/**
  * @}
  */

/* Exported constants --------------------------------------------------------*/

/** @defgroup DAC_Exported_Constants DAC Exported Constants
  * @{
  */

/** @defgroup DAC_Error_Code DAC Error Code
  * @{
  */
#define  HAL_DAC_ERROR_NONE              0x00U    /*!< No error                          */
#define  HAL_DAC_ERROR_DMAUNDERRUNCH1    0x01U    /*!< DAC channel1 DMA underrun error   */
#define  HAL_DAC_ERROR_DMAUNDERRUNCH2    0x02U    /*!< DAC channel2 DMA underrun error   */
#define  HAL_DAC_ERROR_DMA               0x04U    /*!< DMA error                         */
#define  HAL_DAC_ERROR_TIMEOUT           0x08U    /*!< Timeout error                     */
#if (USE_HAL_DAC_REGISTER_CALLBACKS == 1)
#define HAL_DAC_ERROR_INVALID_CALLBACK   0x10U    /*!< Invalid callback error            */
#endif /* USE_HAL_DAC_REGISTER_CALLBACKS */

/**
  * @}
  */

/** @defgroup DAC_trigger_selection DAC trigger selection
  * @{
  */
#define DAC_TRIGGER_NONE                0x00000000UL                                                                      /*!< DAC (all) conversion is automatic once the DAC_DHRxxxx register has been loaded, and not by external trigger */
#define DAC_TRIGGER_SOFTWARE            (                                                                    DAC_CR_TEN1) /*!< DAC (all) conversion started by software trigger for DAC channel */
#define DAC_TRIGGER_T1_TRGO             (                                                   DAC_CR_TSEL1_0 | DAC_CR_TEN1) /*!< DAC3: TIM1 TRGO selected as external conversion trigger for DAC channel. */
#define DAC_TRIGGER_T8_TRGO             (                                                   DAC_CR_TSEL1_0 | DAC_CR_TEN1) /*!< DAC1/2/4: TIM8 TRGO selected as external conversion trigger for DAC channel. Refer to device datasheet for DACx availability. */
#define DAC_TRIGGER_T7_TRGO             (                                  DAC_CR_TSEL1_1                  | DAC_CR_TEN1) /*!< DAC (all): TIM7 TRGO selected as external conversion trigger for DAC channel */
#define DAC_TRIGGER_T15_TRGO            (                                  DAC_CR_TSEL1_1 | DAC_CR_TSEL1_0 | DAC_CR_TEN1) /*!< DAC (all): TIM15 TRGO selected as external conversion trigger for DAC channel */
#define DAC_TRIGGER_T2_TRGO             (                 DAC_CR_TSEL1_2                                   | DAC_CR_TEN1) /*!< DAC (all): TIM2 TRGO selected as external conversion trigger for DAC channel */
#define DAC_TRIGGER_T4_TRGO             (                 DAC_CR_TSEL1_2                  | DAC_CR_TSEL1_0 | DAC_CR_TEN1) /*!< DAC (all): TIM4 TRGO selected as external conversion trigger for DAC channel */
#define DAC_TRIGGER_EXT_IT9             (                 DAC_CR_TSEL1_2 | DAC_CR_TSEL1_1                  | DAC_CR_TEN1) /*!< DAC (all): EXTI Line9 event selected as external conversion trigger for DAC channel. Note: only to be used as update or reset (sawtooth generation) trigger */
#define DAC_TRIGGER_EXT_IT10            (                 DAC_CR_TSEL1_2 | DAC_CR_TSEL1_1                  | DAC_CR_TEN1) /*!< DAC (all): EXTI Line10 event selected as external conversion trigger for DAC channel. Note: only to be used as step (sawtooth generation) trigger */
#define DAC_TRIGGER_T6_TRGO             (                 DAC_CR_TSEL1_2 | DAC_CR_TSEL1_1 | DAC_CR_TSEL1_0 | DAC_CR_TEN1) /*!< DAC (all): TIM6 TRGO selected as external conversion trigger for DAC channel */
#define DAC_TRIGGER_T3_TRGO             (DAC_CR_TSEL1_3                                                    | DAC_CR_TEN1) /*!< DAC (all): TIM3 TRGO selected as external conversion trigger for DAC channel */
#define DAC_TRIGGER_HRTIM_RST_TRG1      (DAC_CR_TSEL1_3                                   | DAC_CR_TSEL1_0 | DAC_CR_TEN1) /*!< DAC (all): HRTIM RST TRIG 1 selected as external conversion trigger for DAC channel. Note: only to be used as reset (sawtooth generation) trigger. On this STM32 series, parameter only available if HRTIM feature is supported (refer to device datasheet for supported features list) */
#define DAC_TRIGGER_HRTIM_STEP_TRG1     (DAC_CR_TSEL1_3                                   | DAC_CR_TSEL1_0 | DAC_CR_TEN1) /*!< DAC (all): HRTIM STEP TRIG 1 selected as external conversion trigger for DAC channel. Note: only to be used as step (sawtooth generation) trigger. On this STM32 series, parameter only available if HRTIM feature is supported (refer to device datasheet for supported features list) */
#define DAC_TRIGGER_HRTIM_RST_TRG2      (DAC_CR_TSEL1_3                  | DAC_CR_TSEL1_1                  | DAC_CR_TEN1) /*!< DAC (all): HRTIM RST TRIG 2 selected as external conversion trigger for DAC channel. Note: only to be used as reset (sawtooth generation) trigger. On this STM32 series, parameter only available if HRTIM feature is supported (refer to device datasheet for supported features list) */
#define DAC_TRIGGER_HRTIM_STEP_TRG2     (DAC_CR_TSEL1_3                  | DAC_CR_TSEL1_1                  | DAC_CR_TEN1) /*!< DAC (all): HRTIM STEP TRIG 2 selected as external conversion trigger for DAC channel. Note: only to be used as step (sawtooth generation) trigger. On this STM32 series, parameter only available if HRTIM feature is supported (refer to device datasheet for supported features list) */
#define DAC_TRIGGER_HRTIM_RST_TRG3      (DAC_CR_TSEL1_3                  | DAC_CR_TSEL1_1 | DAC_CR_TSEL1_0 | DAC_CR_TEN1) /*!< DAC (all): HRTIM RST TRIG 3 selected as external conversion trigger for DAC channel. Note: only to be used as reset (sawtooth generation) trigger. On this STM32 series, parameter only available if HRTIM feature is supported (refer to device datasheet for supported features list) */
#define DAC_TRIGGER_HRTIM_STEP_TRG3     (DAC_CR_TSEL1_3                  | DAC_CR_TSEL1_1 | DAC_CR_TSEL1_0 | DAC_CR_TEN1) /*!< DAC (all): HRTIM STEP TRIG 3 selected as external conversion trigger for DAC channel. Note: only to be used as step (sawtooth generation) trigger. On this STM32 series, parameter only available if HRTIM feature is supported (refer to device datasheet for supported features list) */
#define DAC_TRIGGER_HRTIM_RST_TRG4      (DAC_CR_TSEL1_3 | DAC_CR_TSEL1_2                                   | DAC_CR_TEN1) /*!< DAC (all): HRTIM RST TRIG 4 selected as external conversion trigger for DAC channel. Note: only to be used as reset (sawtooth generation) trigger. On this STM32 series, parameter only available if HRTIM feature is supported (refer to device datasheet for supported features list) */
#define DAC_TRIGGER_HRTIM_STEP_TRG4     (DAC_CR_TSEL1_3 | DAC_CR_TSEL1_2                                   | DAC_CR_TEN1) /*!< DAC (all): HRTIM STEP TRIG 4 selected as external conversion trigger for DAC channel. Note: only to be used as step (sawtooth generation) trigger. On this STM32 series, parameter only available if HRTIM feature is supported (refer to device datasheet for supported features list) */
#define DAC_TRIGGER_HRTIM_RST_TRG5      (DAC_CR_TSEL1_3 | DAC_CR_TSEL1_2                  | DAC_CR_TSEL1_0 | DAC_CR_TEN1) /*!< DAC (all): HRTIM RST TRIG 5 selected as external conversion trigger for DAC channel. Note: only to be used as reset (sawtooth generation) trigger. On this STM32 series, parameter only available if HRTIM feature is supported (refer to device datasheet for supported features list) */
#define DAC_TRIGGER_HRTIM_STEP_TRG5     (DAC_CR_TSEL1_3 | DAC_CR_TSEL1_2                  | DAC_CR_TSEL1_0 | DAC_CR_TEN1) /*!< DAC (all): HRTIM STEP TRIG 5 selected as external conversion trigger for DAC channel. Note: only to be used as step (sawtooth generation) trigger. On this STM32 series, parameter only available if HRTIM feature is supported (refer to device datasheet for supported features list) */
#define DAC_TRIGGER_HRTIM_RST_TRG6      (DAC_CR_TSEL1_3 | DAC_CR_TSEL1_2 | DAC_CR_TSEL1_1                  | DAC_CR_TEN1) /*!< DAC (all): HRTIM RST TRIG 6 selected as external conversion trigger for DAC channel. Note: only to be used as reset (sawtooth generation) trigger. On this STM32 series, parameter only available if HRTIM feature is supported (refer to device datasheet for supported features list) */
#define DAC_TRIGGER_HRTIM_STEP_TRG6     (DAC_CR_TSEL1_3 | DAC_CR_TSEL1_2 | DAC_CR_TSEL1_1                  | DAC_CR_TEN1) /*!< DAC (all): HRTIM STEP TRIG 6 selected as external conversion trigger for DAC channel. Note: only to be used as step (sawtooth generation) trigger. On this STM32 series, parameter only available if HRTIM feature is supported (refer to device datasheet for supported features list) */
#define DAC_TRIGGER_HRTIM_TRG01         (DAC_CR_TSEL1_3 | DAC_CR_TSEL1_2 | DAC_CR_TSEL1_1 | DAC_CR_TSEL1_0 | DAC_CR_TEN1) /*!< DAC1&4: HRTIM TRIG OUT 1 selected as external conversion trigger for DAC channel. Note: only to be used as update or reset (sawtooth generation) trigger. Refer to device datasheet for DACx instance availability. On this STM32 series, parameter only available if HRTIM feature is supported (refer to device datasheet for supported features list) */
#define DAC_TRIGGER_HRTIM_TRG02         (DAC_CR_TSEL1_3 | DAC_CR_TSEL1_2 | DAC_CR_TSEL1_1 | DAC_CR_TSEL1_0 | DAC_CR_TEN1) /*!< DAC2: HRTIM TRIG OUT 1 selected as external conversion trigger for DAC channel. Note: only to be used as update or reset (sawtooth generation) trigger. On this STM32 series, parameter only available if HRTIM feature is supported and DAC2 instance present (refer to device datasheet for supported features list and DAC2 instance availability) */
#define DAC_TRIGGER_HRTIM_TRG03         (DAC_CR_TSEL1_3 | DAC_CR_TSEL1_2 | DAC_CR_TSEL1_1 | DAC_CR_TSEL1_0 | DAC_CR_TEN1) /*!< DAC3: HRTIM TRIG OUT 1 selected as external conversion trigger for DAC channel. Note: only to be used as update or reset (sawtooth generation) trigger. On this STM32 series, parameter only available if HRTIM feature is supported (refer to device datasheet for supported features list) */

/**
  * @}
  */

/** @defgroup DAC_output_buffer DAC output buffer
  * @{
  */
#define DAC_OUTPUTBUFFER_ENABLE            0x00000000U
#define DAC_OUTPUTBUFFER_DISABLE           (DAC_MCR_MODE1_1)

/**
  * @}
  */

/** @defgroup DAC_Channel_selection DAC Channel selection
  * @{
  */
#define DAC_CHANNEL_1                      0x00000000U

#define DAC_CHANNEL_2                      0x00000010U

/**
  * @}
  */

/** @defgroup DAC_data_alignment DAC data alignment
  * @{
  */
#define DAC_ALIGN_12B_R                    0x00000000U
#define DAC_ALIGN_12B_L                    0x00000004U
#define DAC_ALIGN_8B_R                     0x00000008U

/**
  * @}
  */

/** @defgroup DAC_flags_definition DAC flags definition
  * @{
  */
#define DAC_FLAG_DMAUDR1                   (DAC_SR_DMAUDR1)

#define DAC_FLAG_DMAUDR2                   (DAC_SR_DMAUDR2)

#define DAC_FLAG_DAC1RDY                   (DAC_SR_DAC1RDY)

#define DAC_FLAG_DAC2RDY                   (DAC_SR_DAC2RDY)


/**
  * @}
  */

/** @defgroup DAC_IT_definition  DAC IT definition
  * @{
  */
#define DAC_IT_DMAUDR1                   (DAC_SR_DMAUDR1)

#define DAC_IT_DMAUDR2                   (DAC_SR_DMAUDR2)


/**
  * @}
  */

/** @defgroup DAC_ConnectOnChipPeripheral DAC ConnectOnChipPeripheral
  * @{
  */
#define DAC_CHIPCONNECT_EXTERNAL       (1UL << 0)
#define DAC_CHIPCONNECT_INTERNAL       (1UL << 1)
#define DAC_CHIPCONNECT_BOTH           (1UL << 2)

/**
  * @}
  */

/** @defgroup DAC_UserTrimming DAC User Trimming
  * @{
  */
#define DAC_TRIMMING_FACTORY        (0x00000000UL)        /*!< Factory trimming */
#define DAC_TRIMMING_USER           (0x00000001UL)        /*!< User trimming */
/**
  * @}
  */

/** @defgroup DAC_SampleAndHold DAC power mode
  * @{
  */
#define DAC_SAMPLEANDHOLD_DISABLE     (0x00000000UL)
#define DAC_SAMPLEANDHOLD_ENABLE      (DAC_MCR_MODE1_2)

/**
  * @}
  */
/** @defgroup DAC_HighFrequency DAC high frequency interface mode
  * @{
  */
#define DAC_HIGH_FREQUENCY_INTERFACE_MODE_DISABLE        0x00000000UL       /*!< High frequency interface mode disabled */
#define DAC_HIGH_FREQUENCY_INTERFACE_MODE_ABOVE_80MHZ    (DAC_MCR_HFSEL_0)  /*!< High frequency interface mode compatible to AHB>80MHz enabled */
#define DAC_HIGH_FREQUENCY_INTERFACE_MODE_ABOVE_160MHZ   (DAC_MCR_HFSEL_1)  /*!< High frequency interface mode compatible to AHB>160MHz enabled */
#define DAC_HIGH_FREQUENCY_INTERFACE_MODE_AUTOMATIC      0x00000002UL       /*!< High frequency interface mode automatic */

/**
  * @}
  */

/**
  * @}
  */

/* Delay for DAC channel voltage settling time from DAC channel startup       */
/* (transition from disable to enable).                                       */
/* Note: DAC channel startup time depends on board application environment:   */
/*       impedance connected to DAC channel output.                           */
/*       The delay below is specified under conditions:                       */
/*        - voltage maximum transition (lowest to highest value)              */
/*        - until voltage reaches final value +-1LSB                          */
/*        - DAC channel output buffer enabled                                 */
/*        - load impedance of 5kOhm (min), 50pF (max)                         */
/* Literal set to maximum value (refer to device datasheet,                   */
/* parameter "tWAKEUP").                                                      */
/* Unit: us                                                                   */
#define DAC_DELAY_STARTUP_US          (15UL)  /*!< Delay for DAC channel voltage settling time from DAC channel startup (transition from disable to enable) */

/* Exported macro ------------------------------------------------------------*/

/** @defgroup DAC_Exported_Macros DAC Exported Macros
  * @{
  */

/** @brief Reset DAC handle state.
  * @param  __HANDLE__ specifies the DAC handle.
  * @retval None
  */
#if (USE_HAL_DAC_REGISTER_CALLBACKS == 1)
#define __HAL_DAC_RESET_HANDLE_STATE(__HANDLE__) do {                                                        \
                                                      (__HANDLE__)->State             = HAL_DAC_STATE_RESET; \
                                                      (__HANDLE__)->MspInitCallback   = NULL;                \
                                                      (__HANDLE__)->MspDeInitCallback = NULL;                \
                                                     } while(0)
#else
#define __HAL_DAC_RESET_HANDLE_STATE(__HANDLE__) ((__HANDLE__)->State = HAL_DAC_STATE_RESET)
#endif /* USE_HAL_DAC_REGISTER_CALLBACKS */

/** @brief Enable the DAC channel.
  * @param  __HANDLE__ specifies the DAC handle.
  * @param  __DAC_Channel__ specifies the DAC channel
  * @retval None
  */
#define __HAL_DAC_ENABLE(__HANDLE__, __DAC_Channel__) \
  ((__HANDLE__)->Instance->CR |=  (DAC_CR_EN1 << ((__DAC_Channel__) & 0x10UL)))

/** @brief Disable the DAC channel.
  * @param  __HANDLE__ specifies the DAC handle
  * @param  __DAC_Channel__ specifies the DAC channel.
  * @retval None
  */
#define __HAL_DAC_DISABLE(__HANDLE__, __DAC_Channel__) \
  ((__HANDLE__)->Instance->CR &=  ~(DAC_CR_EN1 << ((__DAC_Channel__) & 0x10UL)))

/** @brief Set DHR12R1 alignment.
  * @param  __ALIGNMENT__ specifies the DAC alignment
  * @retval None
  */
#define DAC_DHR12R1_ALIGNMENT(__ALIGNMENT__) (0x00000008UL + (__ALIGNMENT__))


/** @brief  Set DHR12R2 alignment.
  * @param  __ALIGNMENT__ specifies the DAC alignment
  * @retval None
  */
#define DAC_DHR12R2_ALIGNMENT(__ALIGNMENT__) (0x00000014UL + (__ALIGNMENT__))


/** @brief  Set DHR12RD alignment.
  * @param  __ALIGNMENT__ specifies the DAC alignment
  * @retval None
  */
#define DAC_DHR12RD_ALIGNMENT(__ALIGNMENT__) (0x00000020UL + (__ALIGNMENT__))

/** @brief Enable the DAC interrupt.
  * @param  __HANDLE__ specifies the DAC handle
  * @param  __INTERRUPT__ specifies the DAC interrupt.
  *          This parameter can be any combination of the following values:
  *            @arg DAC_IT_DMAUDR1 DAC channel 1 DMA underrun interrupt
  *            @arg DAC_IT_DMAUDR2 DAC channel 2 DMA underrun interrupt (1)
  *
  *         (1) On this STM32 series, parameter not available on all instances.
  *             Refer to device datasheet for channels availability.
  * @retval None
  */
#define __HAL_DAC_ENABLE_IT(__HANDLE__, __INTERRUPT__) (((__HANDLE__)->Instance->CR) |= (__INTERRUPT__))

/** @brief Disable the DAC interrupt.
  * @param  __HANDLE__ specifies the DAC handle
  * @param  __INTERRUPT__ specifies the DAC interrupt.
  *          This parameter can be any combination of the following values:
  *            @arg DAC_IT_DMAUDR1 DAC channel 1 DMA underrun interrupt
  *            @arg DAC_IT_DMAUDR2 DAC channel 2 DMA underrun interrupt (1)
  *
  *         (1) On this STM32 series, parameter not available on all instances.
  *             Refer to device datasheet for channels availability.
  * @retval None
  */
#define __HAL_DAC_DISABLE_IT(__HANDLE__, __INTERRUPT__) (((__HANDLE__)->Instance->CR) &= ~(__INTERRUPT__))

/** @brief  Check whether the specified DAC interrupt source is enabled or not.
  * @param __HANDLE__ DAC handle
  * @param __INTERRUPT__ DAC interrupt source to check
  *          This parameter can be any combination of the following values:
  *            @arg DAC_IT_DMAUDR1 DAC channel 1 DMA underrun interrupt
  *            @arg DAC_IT_DMAUDR2 DAC channel 2 DMA underrun interrupt (1)
  *
  *         (1) On this STM32 series, parameter not available on all instances.
  *             Refer to device datasheet for channels availability.
  * @retval State of interruption (SET or RESET)
  */
#define __HAL_DAC_GET_IT_SOURCE(__HANDLE__, __INTERRUPT__) (((__HANDLE__)->Instance->CR\
                                                             & (__INTERRUPT__)) == (__INTERRUPT__))

/** @brief  Get the selected DAC's flag status.
  * @param  __HANDLE__ specifies the DAC handle.
  * @param  __FLAG__ specifies the DAC flag to get.
  *          This parameter can be any combination of the following values:
  *            @arg DAC_FLAG_DMAUDR1 DAC channel 1 DMA underrun flag
  *            @arg DAC_FLAG_DMAUDR2 DAC channel 2 DMA underrun flag (1)
  *            @arg DAC_FLAG_DAC1RDY DAC channel 1 ready status flag
  *            @arg DAC_FLAG_DAC2RDY DAC channel 2 ready status flag (1)
  *
  *         (1) On this STM32 series, parameter not available on all instances.
  *             Refer to device datasheet for channels availability.
  * @retval None
  */
#define __HAL_DAC_GET_FLAG(__HANDLE__, __FLAG__) ((((__HANDLE__)->Instance->SR) & (__FLAG__)) == (__FLAG__))

/** @brief  Clear the DAC's flag.
  * @param  __HANDLE__ specifies the DAC handle.
  * @param  __FLAG__ specifies the DAC flag to clear.
  *          This parameter can be any combination of the following values:
  *            @arg DAC_FLAG_DMAUDR1 DAC channel 1 DMA underrun flag
  *            @arg DAC_FLAG_DMAUDR2 DAC channel 2 DMA underrun flag (1)
  *
  *         (1) On this STM32 series, parameter not available on all instances.
  *             Refer to device datasheet for channels availability.
  * @retval None
  */
#define __HAL_DAC_CLEAR_FLAG(__HANDLE__, __FLAG__) (((__HANDLE__)->Instance->SR) = (__FLAG__))

/**
  * @}
  */

/* Private macro -------------------------------------------------------------*/

/** @defgroup DAC_Private_Macros DAC Private Macros
  * @{
  */
#define IS_DAC_OUTPUT_BUFFER_STATE(STATE) (((STATE) == DAC_OUTPUTBUFFER_ENABLE) || \
                                           ((STATE) == DAC_OUTPUTBUFFER_DISABLE))

#if defined(STM32G474xx) || defined(STM32G484xx) || defined(STM32G473xx)
#define IS_DAC_CHANNEL(DACX, CHANNEL)        \
  (((DACX) == DAC2) ?                  \
   ((CHANNEL) == DAC_CHANNEL_1)        \
   :                                    \
   (((CHANNEL) == DAC_CHANNEL_1)    || \
    ((CHANNEL) == DAC_CHANNEL_2)))
#else
#define IS_DAC_CHANNEL(DACX, CHANNEL)        \
  (((CHANNEL) == DAC_CHANNEL_1)     || \
   ((CHANNEL) == DAC_CHANNEL_2))
#endif /* STM32G474xx || STM32G484xx || STM32G473xx */

#define IS_DAC_ALIGN(ALIGN) (((ALIGN) == DAC_ALIGN_12B_R) || \
                             ((ALIGN) == DAC_ALIGN_12B_L) || \
                             ((ALIGN) == DAC_ALIGN_8B_R))

#define IS_DAC_DATA(DATA) ((DATA) <= 0xFFF0UL)

#define IS_DAC_REFRESHTIME(TIME)   ((TIME) <= 0x000000FFUL)

/**
  * @}
  */

/* Include DAC HAL Extended module */
#include "stm32g4xx_hal_dac_ex.h"

/* Exported functions --------------------------------------------------------*/

/** @addtogroup DAC_Exported_Functions
  * @{
  */

/** @addtogroup DAC_Exported_Functions_Group1
  * @{
  */
/* Initialization and de-initialization functions *****************************/
HAL_StatusTypeDef HAL_DAC_Init(DAC_HandleTypeDef *hdac);
HAL_StatusTypeDef HAL_DAC_DeInit(DAC_HandleTypeDef *hdac);
void HAL_DAC_MspInit(DAC_HandleTypeDef *hdac);
void HAL_DAC_MspDeInit(DAC_HandleTypeDef *hdac);

/**
  * @}
  */

/** @addtogroup DAC_Exported_Functions_Group2
  * @{
  */
/* IO operation functions *****************************************************/
HAL_StatusTypeDef HAL_DAC_Start(DAC_HandleTypeDef *hdac, uint32_t Channel);
HAL_StatusTypeDef HAL_DAC_Stop(DAC_HandleTypeDef *hdac, uint32_t Channel);
HAL_StatusTypeDef HAL_DAC_Start_DMA(DAC_HandleTypeDef *hdac, uint32_t Channel, const uint32_t *pData, uint32_t Length,
                                    uint32_t Alignment);
HAL_StatusTypeDef HAL_DAC_Stop_DMA(DAC_HandleTypeDef *hdac, uint32_t Channel);
void HAL_DAC_IRQHandler(DAC_HandleTypeDef *hdac);
HAL_StatusTypeDef HAL_DAC_SetValue(DAC_HandleTypeDef *hdac, uint32_t Channel, uint32_t Alignment, uint32_t Data);

void HAL_DAC_ConvCpltCallbackCh1(DAC_HandleTypeDef *hdac);
void HAL_DAC_ConvHalfCpltCallbackCh1(DAC_HandleTypeDef *hdac);
void HAL_DAC_ErrorCallbackCh1(DAC_HandleTypeDef *hdac);
void HAL_DAC_DMAUnderrunCallbackCh1(DAC_HandleTypeDef *hdac);

#if (USE_HAL_DAC_REGISTER_CALLBACKS == 1)
/* DAC callback registering/unregistering */
HAL_StatusTypeDef     HAL_DAC_RegisterCallback(DAC_HandleTypeDef *hdac, HAL_DAC_CallbackIDTypeDef CallbackID,
                                               pDAC_CallbackTypeDef pCallback);
HAL_StatusTypeDef     HAL_DAC_UnRegisterCallback(DAC_HandleTypeDef *hdac, HAL_DAC_CallbackIDTypeDef CallbackID);
#endif /* USE_HAL_DAC_REGISTER_CALLBACKS */

/**
  * @}
  */

/** @addtogroup DAC_Exported_Functions_Group3
  * @{
  */
/* Peripheral Control functions ***********************************************/
uint32_t HAL_DAC_GetValue(const DAC_HandleTypeDef *hdac, uint32_t Channel);
HAL_StatusTypeDef HAL_DAC_ConfigChannel(DAC_HandleTypeDef *hdac,
                                        const DAC_ChannelConfTypeDef *sConfig, uint32_t Channel);
/**
  * @}
  */

/** @addtogroup DAC_Exported_Functions_Group4
  * @{
  */
/* Peripheral State and Error functions ***************************************/
HAL_DAC_StateTypeDef HAL_DAC_GetState(const DAC_HandleTypeDef *hdac);
uint32_t HAL_DAC_GetError(const DAC_HandleTypeDef *hdac);

/**
  * @}
  */

/**
  * @}
  */

/** @defgroup DAC_Private_Functions DAC Private Functions
  * @{
  */
void DAC_DMAConvCpltCh1(DMA_HandleTypeDef *hdma);
void DAC_DMAErrorCh1(DMA_HandleTypeDef *hdma);
void DAC_DMAHalfConvCpltCh1(DMA_HandleTypeDef *hdma);
/**
  * @}
  */

/**
  * @}
  */

#endif /* DAC1 || DAC2 || DAC3 || DAC4 */

/**
  * @}
  */

#ifdef __cplusplus
}
#endif


#endif /* STM32G4xx_HAL_DAC_H */
