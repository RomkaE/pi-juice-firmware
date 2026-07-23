/*
 * app.h
 *
 *  Created on: 2026
 *      Author: Roman Egoshin
 */

/*============================ INCLUDES ======================================*/

#include "config.h"
#include "board.h"
#include "app-error/app_error.h"

// ST HAL/CubeMX:
#include "stm32f0xx_hal.h"
#include "cube-mx/gpio.h"
#include "cube-mx/dma.h"

/*============================ TYPES =========================================*/

typedef void (*pFunction)(void);

/*============================ VARIABLES =====================================*/


/*============================ PRIVATE DEFINITIONS ===========================*/


/*============================ PRIVATE PROTOTYPES ============================*/


/*============================ IMPLEMENTATION (PRIVATE FUNCTIONS) ============*/

/*============================ IMPLEMENTATION (PUBLIC FUNCTIONS) =============*/

void bsp_Init(void)
{
  HAL_Init();
  HAL_SetTickFreq(HAL_TICK_FREQ_100HZ);
  SystemClock_Config();

  MX_GPIO_Init();
  MX_DMA_Init();
}

void bsp_DeInit(void)
{
}
