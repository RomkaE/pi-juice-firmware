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
#include "app-error/app_error.h"
#include "app-error/app_assert.h"
#include "board.h"

// ST HAL:
#include "stm32f0xx_hal.h"

// FreeRTOS:
#include "FreeRTOS.h"
#include "task.h"

#include "rtmon_config.h"
#if RTMON_ENABLED
#include "rtmon.h"
#endif

int main(void)
{
  #if RTMON_ENABLED
    rtmon_Init();
  #endif
  app_Init();
  vTaskStartScheduler();
  while (1);
}

// TODO - remove
// LOG adapter:
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
