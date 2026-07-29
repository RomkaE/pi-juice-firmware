/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    i2c.c
  * @brief   This file provides code for the configuration
  *          of the I2C instances.
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
#include "i2c.h"

/* USER CODE BEGIN 0 */

#include "board.h"
#include "nv.h"

/*
 * I2C2 TIMINGR, recomputed for the clock the board actually runs at.
 *
 * The previous value 0x20000A0D was carried over from a configuration with the
 * PLL enabled. The PLL is off (bsp/board_ver0.c), so SYSCLK = HCLK = PCLK =
 * I2C2CLK = 8 MHz, t_I2CCLK = 125 ns.
 *
 *   PRESC  = 7  -> t_PRESC = 8 * 125 ns = 1 us
 *   SCLDEL = 2  -> data setup   (SCLDEL+1) * t_PRESC = 3 us
 *   SDADEL = 1  -> data hold    SDADEL * t_PRESC     = 1 us
 *   SCLH   = 8  -> t_HIGH       (SCLH+1) * t_PRESC   = 9 us
 *   SCLL   = 10 -> t_LOW        (SCLL+1) * t_PRESC   = 11 us
 *
 * SCL lands near 46 kHz instead of the previous 107 kHz. The bus to the fuel
 * gauge crosses the whole board past the switching regulator with only a 10k
 * pull-up, so the extra margin on every edge is worth far more than the speed:
 * a FuelGaugeReadWord() goes from ~0.5 ms to ~1.2 ms, which nothing depends on.
 *
 * SDADEL and SCLDEL were both 0 before. ST advises against a zero data hold
 * delay with slow edges, and SCLDEL 0 left the setup time at 375 ns against a
 * 250 ns minimum.
 */
#define I2C2_TIMING				0x7021080A

/*
 * Digital noise filter length for I2C2, in t_I2CCLK units: 4 * 125 ns = 500 ns
 * of spike suppression on SCL and SDA. The analogue filter alone (~50 ns) is
 * no match for a switching regulator coupling into a long trace. At 46 kHz
 * t_LOW is 11 us, so 500 ns costs nothing in the timing budget.
 *
 * The value also has to stay below the data hold delay: SDADEL (1 us) must be
 * at least (DNF + 3) * t_I2CCLK = 875 ns, which holds.
 */
#define I2C2_DIGITAL_FILTER		4

/* USER CODE END 0 */

I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;

/*
 * No DMA handles here on purpose. CubeMX wired DMA1 channels 2..5 to both
 * buses, but every transfer in this firmware goes through the interrupt API
 * (HAL_I2C_*_IT), which never looks at hdmatx/hdmarx. Dropped along with the
 * two DMA interrupt handlers in system/stm32f0xx_it.c, whose NVIC lines were
 * never enabled anyway - only DMA1_Channel1 is, for the ADC.
 */

/* I2C1 init function */
void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /*
   * Both own addresses come from the emulated EEPROM when the stored copy is
   * valid (low byte and its complement in the high byte), otherwise from the
   * defaults in i2c.h. The command server can rewrite them at runtime through
   * registers 124/125.
   */
  uint16_t var = 0;

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x00201D2B;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_ENABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;

  /* NOTE: hand written, CubeMX cannot express address-from-NV. Re-apply after
   * regenerating this file, otherwise both slave addresses come out as 0. */
  EE_ReadVariable(OWN_ADDRESS1_NV_ADDR, &var);
  hi2c1.Init.OwnAddress1 = (((~var) & 0xFF) == (var >> 8)) ? (var & 0xFF)
                                                           : (OWN1_I2C_ADDRESS << 1);
  EE_ReadVariable(OWN_ADDRESS2_NV_ADDR, &var);
  hi2c1.Init.OwnAddress2 = (((~var) & 0xFF) == (var >> 8)) ? (var & 0xFF)
                                                           : (OWN2_I2C_ADDRESS << 1);

  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}
/* I2C2 init function */
void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.Timing = I2C2_TIMING;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, I2C2_DIGITAL_FILTER) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

void HAL_I2C_MspInit(I2C_HandleTypeDef* hi2c)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(hi2c->Instance==I2C1)
  {
  /* USER CODE BEGIN I2C1_MspInit 0 */

  /* USER CODE END I2C1_MspInit 0 */

    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**I2C1 GPIO Configuration
    PB6     ------> I2C1_SCL
    PB7     ------> I2C1_SDA
    */
    GPIO_InitStruct.Pin = I2C1_SCL_PIN|I2C1_SDA_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = I2C1_GPIO_AF;
    HAL_GPIO_Init(I2C1_SCL_PORT, &GPIO_InitStruct);

    /* Peripheral clock enable */
    __HAL_RCC_I2C1_CLK_ENABLE();

    /* I2C1 interrupt Init */
    HAL_NVIC_SetPriority(I2C1_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(I2C1_IRQn);
  /* USER CODE BEGIN I2C1_MspInit 1 */

  /* USER CODE END I2C1_MspInit 1 */
  }
  else if(hi2c->Instance==I2C2)
  {
  /* USER CODE BEGIN I2C2_MspInit 0 */

  /* USER CODE END I2C2_MspInit 0 */

    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**I2C2 GPIO Configuration
    PB10     ------> I2C2_SCL
    PB11     ------> I2C2_SDA
    */
    GPIO_InitStruct.Pin = I2C2_SCL_PIN|I2C2_SDA_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = I2C2_GPIO_AF;
    HAL_GPIO_Init(I2C2_SCL_PORT, &GPIO_InitStruct);

    /* Peripheral clock enable */
    __HAL_RCC_I2C2_CLK_ENABLE();

    /* I2C2 interrupt Init */
    HAL_NVIC_SetPriority(I2C2_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(I2C2_IRQn);
  /* USER CODE BEGIN I2C2_MspInit 1 */

  /* USER CODE END I2C2_MspInit 1 */
  }
}

void HAL_I2C_MspDeInit(I2C_HandleTypeDef* i2cHandle)
{

  if(i2cHandle->Instance==I2C1)
  {
  /* USER CODE BEGIN I2C1_MspDeInit 0 */

  /* USER CODE END I2C1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_I2C1_CLK_DISABLE();

    /**I2C1 GPIO Configuration
    PB6     ------> I2C1_SCL
    PB7     ------> I2C1_SDA
    */
    HAL_GPIO_DeInit(I2C1_SCL_PORT, I2C1_SCL_PIN);

    HAL_GPIO_DeInit(I2C1_SDA_PORT, I2C1_SDA_PIN);

  /* USER CODE BEGIN I2C1_MspDeInit 1 */

  /* USER CODE END I2C1_MspDeInit 1 */
  }
  else if(i2cHandle->Instance==I2C2)
  {
  /* USER CODE BEGIN I2C2_MspDeInit 0 */

  /* USER CODE END I2C2_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_I2C2_CLK_DISABLE();

    /**I2C2 GPIO Configuration
    PB10     ------> I2C2_SCL
    PB11     ------> I2C2_SDA
    */
    HAL_GPIO_DeInit(I2C2_SCL_PORT, I2C2_SCL_PIN);

    HAL_GPIO_DeInit(I2C2_SDA_PORT, I2C2_SDA_PIN);

  /* USER CODE BEGIN I2C2_MspDeInit 1 */

  /* USER CODE END I2C2_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

