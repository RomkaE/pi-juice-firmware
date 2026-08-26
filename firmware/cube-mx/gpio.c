/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    gpio.c
  * @brief   This file provides code for the configuration
  *          of all used GPIO pins.
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
#include "gpio.h"
#include "board.h"

/* USER CODE BEGIN 0 */

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
  GPIO_InitTypeDef GPIO_InitStruct = { 0 };

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /* Unused and not-yet-claimed pins go to analog. A driver that needs one overrides the
   * mode itself later (TIM3 for the LED, IoConfigure for EXT_IO). */
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Pin = GPIO_PARK_A_PINS;
  HAL_GPIO_Init(GPIO_PARK_A_PORT, &GPIO_InitStruct);
  GPIO_InitStruct.Pin = GPIO_PARK_B_PINS;
  HAL_GPIO_Init(GPIO_PARK_B_PORT, &GPIO_InitStruct);

  /*Configure charger interrupt input */
  GPIO_InitStruct.Pin = CHG_INT_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;//GPIO_PULLDOWN;
  HAL_GPIO_Init(CHG_INT_PORT, &GPIO_InitStruct);

  /*Configure VSYS switch current limit select */
  GPIO_InitStruct.Pin = PWR_VSYS_ILIM_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(PWR_VSYS_ILIM_PORT, &GPIO_InitStruct);

  /*Configure VSYS switch enable, active LOW - kept HIGH, the switch stays off. Nothing pulls
    the switch EN input on the board, so the pin has to drive it. Level first, so the pin never
    dips low on the way to output. */
  GPIO_InitStruct.Pin = PWR_VSYS_EN_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_WritePin(PWR_VSYS_EN_PORT, PWR_VSYS_EN_PIN, GPIO_PIN_SET);
  HAL_GPIO_Init(PWR_VSYS_EN_PORT, &GPIO_InitStruct);

  /*Configure 5V detection LDO enable and charger NTC control outputs */
  GPIO_InitStruct.Pin = PWR_5V_LDO_EN_PIN|CHG_NTC_CTRL1_PIN|CHG_NTC_CTRL2_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(PWR_5V_LDO_EN_PORT, &GPIO_InitStruct);

  // Configure SW3, SW1 as button inputs
  GPIO_InitStruct.Pin = BTN_SW3_PIN | BTN_SW1_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;//GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(BTN_SW3_PORT, &GPIO_InitStruct);

  // Configure SW2 - carries the power button role, see TODO(hw) in button.c
  GPIO_InitStruct.Pin = BTN_SW2_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;//GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(BTN_SW2_PORT, &GPIO_InitStruct);

  //Configure ID EEPROM control outputs
  /*
  GPIO_InitStruct.Pin = EE_ADDR_SEL_PIN|EE_WP_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(EE_ADDR_SEL_PORT, &GPIO_InitStruct);
  */

  /*Configure open drain, fuel gauge int for HW<2.3, 5V regulator power good for HW>=2.3*/
  GPIO_InitStruct.Pin = FG_INT_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_WritePin(FG_INT_PORT, FG_INT_PIN, GPIO_PIN_SET);
  HAL_GPIO_Init(FG_INT_PORT, &GPIO_InitStruct);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(CHG_NTC_CTRL1_PORT, CHG_NTC_CTRL1_PIN|PWR_5V_LDO_EN_PIN|CHG_NTC_CTRL2_PIN, GPIO_PIN_RESET);

  // TODO: Move IRQ management to the corresponding driver/component:
  {
    /* Enable and set EXTI line 12,13 Interrupt to the lowest priority */
    HAL_NVIC_SetPriority(EXTI4_15_IRQn, 2, 0);
    HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);

    /* Enable and set EXTI line charger interrupt to the lowest priority */
    HAL_NVIC_SetPriority(EXTI0_1_IRQn, 2, 0);
    HAL_NVIC_EnableIRQ(EXTI0_1_IRQn);

    /* Enable and set EXTI line 2, SW2 Interrupt to the lowest priority */
    HAL_NVIC_SetPriority(EXTI2_3_IRQn, 2, 0);
    HAL_NVIC_EnableIRQ(EXTI2_3_IRQn);
  }
}

/* USER CODE BEGIN 2 */

/* USER CODE END 2 */
