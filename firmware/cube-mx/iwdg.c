/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    iwdg.c
  * @brief   This file provides code for the configuration
  *          of the IWDG instances.
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
#include "iwdg.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

IWDG_HandleTypeDef hiwdg;

/* IWDG init function */
void MX_IWDG_Init(void)
{
  /*
   * 1 s timeout at the nominal 40 kHz LSI: 625 * 64 / 40000. LSI is an RC spread over
   * 30..60 kHz, so the real window is 0.67..1.33 s - the refresh period must fit the
   * fast end, not the nominal one.
   */
  hiwdg.Instance       = IWDG;
  hiwdg.Init.Prescaler = IWDG_PRESCALER_64;
  hiwdg.Init.Reload    = 625;
  hiwdg.Init.Window    = IWDG_WINDOW_DISABLE;

  /* HAL_IWDG_Init() also starts the counter - there is no separate start. */
  if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
  {
    // Initialization Error
    Error_Handler();
  }
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

