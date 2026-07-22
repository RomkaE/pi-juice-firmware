/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : stm32f0xx_hal_msp.c
  * Description        : This file provides code for the MSP Initialization 
  *                      and de-Initialization codes.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
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

/* DMA channel feeding analogIn[] from the ADC, linked to hadc in
 * HAL_ADC_MspInit() below. */
static DMA_HandleTypeDef DmaHandle;

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
  * @brief  ADC low level init: clocks, analog pins, DMA and interrupts.
  * @note   Restored from the original CubeMX MSP - it was lost when the monolith
  *         was split up, which made HAL_ADC_Init() fail: without
  *         __HAL_RCC_ADC1_CLK_ENABLE() the peripheral is unclocked and enabling
  *         it times out.
  */
void HAL_ADC_MspInit(ADC_HandleTypeDef* hadc)
{
  GPIO_InitTypeDef GPIO_InitStruct;

  if (hadc->Instance == ADC1)
  {
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_ADC1_CLK_ENABLE();
    __HAL_RCC_DMA1_CLK_ENABLE();

    /*
     * PA0..PA5 into analog mode. Only PA0, PA1, PA2 and PA4 are scanned (see the channel
     * table in analog.c); PA3 (battery NTC) and PA5 (config resistor, read once at init)
     * are parked here because analog is the right idle state for an otherwise unused pin.
     *
     * PA7 is not here: it is the user-configurable IO1 pin, owned by io_control.c, which
     * switches it between analog / digital in / out / PWM at run time.
     */
    GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 |
                          GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /*
     * Circular DMA into analogIn[].
     *
     * NOTE: the original used WORD alignment because analogIn[] was uint32_t.
     * It is uint16_t now, so both sides must be HALFWORD - the ADC data
     * register is 16 bit anyway. Getting this wrong would scatter every second
     * sample across the buffer.
     */
    DmaHandle.Instance                 = DMA1_Channel1;
    DmaHandle.Init.Direction           = DMA_PERIPH_TO_MEMORY;
    DmaHandle.Init.PeriphInc           = DMA_PINC_DISABLE;
    DmaHandle.Init.MemInc              = DMA_MINC_ENABLE;
    DmaHandle.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
    DmaHandle.Init.MemDataAlignment    = DMA_MDATAALIGN_HALFWORD;
    DmaHandle.Init.Mode                = DMA_CIRCULAR;
    DmaHandle.Init.Priority            = DMA_PRIORITY_MEDIUM;

    HAL_DMA_DeInit(&DmaHandle);
    HAL_DMA_Init(&DmaHandle);

    __HAL_LINKDMA(hadc, DMA_Handle, DmaHandle);

    HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 3, 0);
    HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

    /* Analog watchdog interrupt */
    HAL_NVIC_SetPriority(ADC1_IRQn, 3, 0);
    HAL_NVIC_EnableIRQ(ADC1_IRQn);
  }
}

void HAL_ADC_MspDeInit(ADC_HandleTypeDef* hadc)
{
  if (hadc->Instance == ADC1)
  {
    HAL_NVIC_DisableIRQ(ADC1_IRQn);
    HAL_NVIC_DisableIRQ(DMA1_Channel1_IRQn);
    HAL_DMA_DeInit(&DmaHandle);
    __HAL_RCC_DMA1_CLK_DISABLE();
    __HAL_RCC_ADC1_CLK_DISABLE();

    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3
                          |GPIO_PIN_4|GPIO_PIN_5);
  }
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
  * @brief  Enables the clocks of the timers used in base mode.
  */
void HAL_TIM_Base_MspInit(TIM_HandleTypeDef* htim_base)
{
  if (htim_base->Instance == TIM1)
  {
    __HAL_RCC_TIM1_CLK_ENABLE();
  }
  else if (htim_base->Instance == TIM17)
  {
    __HAL_RCC_TIM17_CLK_ENABLE();
  }
  else if (htim_base->Instance == TIM14)
  {
    __HAL_RCC_TIM14_CLK_ENABLE();
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
    /* PB14 ------> TIM15_CH1, PB15 ------> TIM15_CH2 */
    GPIO_InitStruct.Pin = GPIO_PIN_14 | GPIO_PIN_15;
    GPIO_InitStruct.Alternate = GPIO_AF1_TIM15;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
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
