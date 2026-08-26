/*
 * board_ver0.c
 *
 *  Created on:
 *      Author: Roman Egoshin
 */

#include "board.h"

// ST HAL/CubeMX:
#include "stm32f0xx_hal.h"
#include "cube-mx/main.h"


void bsp_ClockConfig(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

  /**Initializes the CPU, AHB and APB busses clocks
  */
  // LSI is listed on purpose: it clocks the IWDG, and once the watchdog is running the
  // hardware holds LSI on regardless. Requesting it OFF here would hang waiting for LSIRDY.
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSI14
                              |RCC_OSCILLATORTYPE_LSE|RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSI14State = RCC_HSI14_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.HSI14CalibrationValue = 16;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  // SYSCLK is HSI (8 MHz) directly; the PLL stays off, so its source/mul fields are omitted.
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /**Initializes the CPU, AHB and APB busses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }

  /*
   * Selects the RTC clock source. This is NOT optional: HAL_RTC_MspInit() only
   * opens the RTC clock gate (RCC_BDCR.RTCEN), while the source itself lives in
   * RCC_BDCR.RTCSEL, whose reset value is 00 = no clock.
   *
   * The bug this guards against is latent: RTCSEL sits in the backup domain,
   * which survives a system reset and a reflash, so a board that once ran a
   * firmware setting it to LSE keeps working. It only dies on a virgin part or
   * after the backup domain loses power.
   *
   * I2C1 is listed for completeness - RCC_CFGR3.I2C1SW already defaults to HSI,
   * so that half only pins down the default explicitly.
   */
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_I2C1|RCC_PERIPHCLK_RTC;
  PeriphClkInit.I2c1ClockSelection   = RCC_I2C1CLKSOURCE_HSI;
  PeriphClkInit.RTCClockSelection    = RCC_RTCCLKSOURCE_LSE;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}
