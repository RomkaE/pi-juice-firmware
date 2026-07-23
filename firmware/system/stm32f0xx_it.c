/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f0xx_it.c
  * @brief   Interrupt Service Routines.
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

/* USER CODE BEGIN 0 */
//extern void SysTickCb();
/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern ADC_HandleTypeDef hadc;


extern I2C_HandleTypeDef hi2c1;
extern I2C_HandleTypeDef hi2c2;
//extern SMBUS_HandleTypeDef hsmbus;
extern void I2C_EV_IRQHandler(I2C_HandleTypeDef *hi2c);

extern RTC_HandleTypeDef hrtc;
extern ADC_HandleTypeDef hadc;
//extern WWDG_HandleTypeDef hwwdg;
extern TIM_HandleTypeDef htim6;
/******************************************************************************/
/*            Cortex-M0 Processor Interruption and Exception Handlers         */
/******************************************************************************/

/**
* @brief This function handles Non maskable interrupt.
*/
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */

  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
* @brief This function handles Hard fault interrupt.
*/
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
  }
  /* USER CODE BEGIN HardFault_IRQn 1 */

  /* USER CODE END HardFault_IRQn 1 */
}
/*
 * SVC_Handler, PendSV_Handler and SysTick_Handler are provided by the FreeRTOS
 * Cortex-M0 port (portasm.c / port.c) and must not be defined here.
 *
 * The HAL time base is delegated to the scheduler as well: HAL_InitTick() is
 * overridden to a no-op and HAL_GetTick() returns xTaskGetTickCount(), both in
 * src/main.c. So there is no HAL_IncTick() to call from the tick interrupt.
 */

/******************************************************************************/
/* STM32F0xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f0xx.s).                    */
/******************************************************************************/
/* USER CODE BEGIN 1 */
/**
  * @brief  This function handles I2C event and error interrupt request.  
  * @param  None
  * @retval None
  * @Note   This function is redefined in "main.h" and related to I2C data transmission     
  */
void I2C1_IRQHandler(void)
{
  HAL_I2C_EV_IRQHandler(&hi2c1);
  HAL_I2C_ER_IRQHandler(&hi2c1);
}

void I2C2_IRQHandler(void)
{
  HAL_I2C_EV_IRQHandler(&hi2c2);
  HAL_I2C_ER_IRQHandler(&hi2c2);
}

void DMA1_Channel1_IRQHandler(void)
{
  HAL_DMA_IRQHandler(hadc.DMA_Handle);
}

/**
  * @brief  This function handles DMA interrupt request.
  * @param  None
  * @retval None
  * @Note   This function is redefined in "main.h" and related to DMA Channel
  *         used for I2C data transmission
  */
void DMA1_Channel2_3_IRQHandler(void)
{
  HAL_DMA_IRQHandler(hi2c1.hdmarx);
  HAL_DMA_IRQHandler(hi2c1.hdmatx);
}

void DMA1_Channel4_5_IRQHandler(void)
{
  HAL_DMA_IRQHandler(hi2c2.hdmarx);
  HAL_DMA_IRQHandler(hi2c2.hdmatx);
}


/**
  * @brief  This function handles RTC Alarm interrupt request.
  * @param  None
  * @retval None
  */
void RTC_IRQHandler(void)
{
  HAL_RTC_AlarmIRQHandler(&hrtc);
  HAL_RTCEx_WakeUpTimerIRQHandler(&hrtc);
}

void ADC1_IRQHandler(void)
{
	HAL_ADC_IRQHandler(&hadc);
}

/**
  * @brief  This function handles external line 0 interrupt request.
  * @param  None
  * @retval None
  */
void EXTI4_15_IRQHandler(void)
{
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_7);
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_12);
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_13);
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_8); // IO2
}

void EXTI0_1_IRQHandler(void)
{
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_0);
}

void EXTI2_3_IRQHandler(void)
{
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_2);
}

/*void WWDG_IRQHandler(void)
{
	HAL_WWDG_IRQHandler(&hwwdg);
}*/


/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
