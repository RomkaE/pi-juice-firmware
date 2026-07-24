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
/* IWDG init function */
void MX_IWDG_Init(void)
{

  // ##-3- Configure the IWDG peripheral ######################################*/
  // Set counter reload value to obtain 250ms IWDG TimeOut.
  //   IWDG counter clock Frequency = LsiFreq / 32
  //   Counter Reload Value = 250ms / IWDG counter clock period
  //                     = 0.25s / (32/LsiFreq)
  //                      = LsiFreq / (32 * 4)
  //                      = LsiFreq / 128
  hiwdg.Instance = IWDG;
  hiwdg.Init.Prescaler = IWDG_PRESCALER_256;
  hiwdg.Init.Reload    = 1300;//LSI_VALUE / 4; // 8 seconds
  hiwdg.Init.Window    = IWDG_WINDOW_DISABLE;

  // TODO - !?
//  DelayUs(100);
  if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
  {
    // Initialization Error
    Error_Handler();
  }
/*
  IWDG->KR = 0x0000CCCC; // (1)
  DelayUs(100);
  IWDG->KR = 0x00005555; // (2)
  DelayUs(100);
  IWDG->PR = IWDG_PRESCALER_256; // (3)
  //DelayUs(100);
  IWDG->RLR = 1300; // (4)
  //DelayUs(100);
  while (IWDG->SR) // (5)
  {
   // add time out here for a robust application
  }
  IWDG->KR = 0x0000AAAA; // (6)
  //DelayUs(100);

  //##-4- Start the IWDG #####################################################
  if (__HAL_IWDG_START(&hiwdg) != HAL_OK)
  {
    Error_Handler();
  }
*/
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

