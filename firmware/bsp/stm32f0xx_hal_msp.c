/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file         stm32f0xx_hal_msp.c
  * @brief        This file provides code for the MSP Initialization
  *               and de-Initialization codes.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "main.h"
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN Define */

/* USER CODE END Define */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN Macro */

/* USER CODE END Macro */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* External functions --------------------------------------------------------*/
/* USER CODE BEGIN ExternalFunctions */

/* USER CODE END ExternalFunctions */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */
/**
  * Initializes the Global MSP.
  */
void HAL_MspInit(void)
{

  /* USER CODE BEGIN MspInit 0 */

  /* USER CODE END MspInit 0 */

  __HAL_RCC_SYSCFG_CLK_ENABLE();
  __HAL_RCC_PWR_CLK_ENABLE();

  /* System interrupt init*/

  /* USER CODE BEGIN MspInit 1 */

  /* USER CODE END MspInit 1 */
}

/**
  * @brief  Enables the clocks of the PWM timers. Also lost in the split - note
  *         HAL_TIM_MspPostInit() below only configures the output pins, so
  *         without this the timers have no clock at all.
  */
void HAL_TIM_PWM_MspInit(TIM_HandleTypeDef* htim_pwm)
{
  GPIO_InitTypeDef GPIO_InitStruct;

  if (htim_pwm->Instance == TIM3)
  {
    __HAL_RCC_TIM3_CLK_ENABLE();

    /* PB4 ------> TIM3_CH1, PB5 ------> TIM3_CH2 */
    GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF1_TIM3;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
  }
  else if (htim_pwm->Instance == TIM15)
  {
    __HAL_RCC_TIM15_CLK_ENABLE();
  }
}

/**
  * @brief  Configures the GPIO pins driven by the timer compare outputs.
  * @note   Called by the MX_TIMx_Init() functions after the PWM channels are
  *         configured, so the pins only leave GPIO mode once the timer is set
  *         up. Restored from the original CubeMX-generated MSP.
  */
void HAL_TIM_MspPostInit(TIM_HandleTypeDef* htim)
{
  GPIO_InitTypeDef GPIO_InitStruct;

  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;

  if (htim->Instance == TIM1)
  {
    /* PA8 ------> TIM1_CH1  (PA7 / CH1N left as GPIO) */
    GPIO_InitStruct.Pin = GPIO_PIN_8;
    GPIO_InitStruct.Alternate = GPIO_AF2_TIM1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  }
  else if (htim->Instance == TIM3)
  {
    /* PB0 ------> TIM3_CH3 */
    GPIO_InitStruct.Pin = GPIO_PIN_0;
    GPIO_InitStruct.Alternate = GPIO_AF1_TIM3;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
  }
  else if (htim->Instance == TIM14)
  {
    /* PA7 ------> TIM14_CH1 */
    GPIO_InitStruct.Pin = GPIO_PIN_7;
    GPIO_InitStruct.Alternate = GPIO_AF4_TIM14;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  }
  else if (htim->Instance == TIM15)
  {
    // TODO - remove. Now tim15 uses for ADC trigger
//    /* PB14 ------> TIM15_CH1, PB15 ------> TIM15_CH2 */
//    GPIO_InitStruct.Pin = GPIO_PIN_14 | GPIO_PIN_15;
//    GPIO_InitStruct.Alternate = GPIO_AF1_TIM15;
//    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
  }
  else if (htim->Instance == TIM17)
  {
    /* PB9 ------> TIM17_CH1 */
    GPIO_InitStruct.Pin = GPIO_PIN_9;
    GPIO_InitStruct.Alternate = GPIO_AF2_TIM17;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
  }
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
