/*
 * time_count.c
 *
 *  Created on: 12.12.2016.
 *      Author: milan
 */
#include <to_refactor/time_count.h>
#include "stm32f0xx_hal.h"

/*
 * The HAL time base belongs to the scheduler now: HAL_InitTick() and
 * HAL_GetTick() are overridden in src/main.c on top of xTaskGetTickCount(),
 * and SysTick_Handler comes from the FreeRTOS port. The SysTick-based versions
 * that used to live here - together with HAL_IncTick(), the msTickCnt counter
 * they maintained, and TimeTickCb() which topped that counter up after STOP
 * mode - are gone, because nothing reads that counter any more.
 *
 * TODO: the sleep-time catch-up TimeTickCb() used to do still has to come back
 * in FreeRTOS terms. Waking from STOP leaves the scheduler tick behind by a
 * full sleep period; vTaskStepTick() is the replacement, ideally by moving the
 * STOP-mode block into portSUPPRESS_TICKS_AND_SLEEP() with tickless idle.
 */

/*
 * Overrides the __weak HAL_Delay() from stm32f0xx_hal.c.
 *
 * TODO: this busy-waits while the scheduler is running, so it starves every
 * lower priority task for the whole delay. Callers that are allowed to block
 * should move to vTaskDelay().
 */
void HAL_Delay(__IO uint32_t Delay)
{
	DelayUs(Delay*1000);
}
