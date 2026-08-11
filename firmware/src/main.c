/*
 * main.c
 *
 *  Created on: Jul 20, 2026
 *      Author: Roman Egoshin
 */

#include <stdint.h>
#include <stdbool.h>
#include <errno.h>
#include "app.h"
#include "retained_memory.h"
#include "board.h"
#include "app-error/app_error.h"
#include "app-error/app_assert.h"

// ST HAL/CubeMX:
#include "stm32f0xx_hal.h"

// FreeRTOS:
#include "FreeRTOS.h"
#include "task.h"

// RealTime monitor:
#include "rtmon.h"

// LOG:
#include "log/log.h"

int main(void)
{
  bsp_Init();
  retained_mem_Check();
  LOG_INIT(LOG_LEVEL_DEBUG);

  // CRITICAL: Restore the 5V DC-DC converter state:
  bsp_Pwr5V_Restore();

  #if RTMON_ENABLED
    rtmon_Init();
  #endif
  app_Init();
  vTaskStartScheduler();
  APP_ERROR(APP_ERR);
  while (1);
}

// adapter for logs:
uint32_t app_timer_get_ms(void)
{
  return HAL_GetTick();
}

// Overwrite the HAL tick initialization interface:
HAL_StatusTypeDef HAL_InitTick(uint32_t TickPriority)
{
  (void)TickPriority;
  return HAL_OK;
}

uint32_t HAL_GetTick(void)
{
  return (uint32_t)(xTaskGetTickCount() * (1000 / configTICK_RATE_HZ));
}

// HAL error handler:
void Error_Handler(void)
{
  APP_ERROR(APP_HAL_ERROR);
}

// Overrides the _sbrk():
void *_sbrk(int incr)
{
  (void)incr;
  APP_ERROR(APP_ERR_NOMEM);
  errno = ENOMEM;
  return (void *)-1;
}

// FreeRTOS stack overflow:
#if ( configCHECK_FOR_STACK_OVERFLOW > 0 )

void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName)
{
  (void) xTask;
  (void) pcTaskName;
  APP_ERROR(APP_ERR_NOMEM);
}

#endif /* #if ( configCHECK_FOR_STACK_OVERFLOW > 0 ) */
