
#include "bsp_rtmon.h"
#include "stm32f0xx_hal.h"

/*
 * Free-running counter backing the FreeRTOS run-time statistics
 * (portGET_RUN_TIME_COUNTER_VALUE, see config/FreeRTOSConfig.h).
 *
 * TIM7 is the natural pick on STM32F030CC. Of the two unused timers left
 * (TIM7 and TIM16) it is the basic one - no compare channels, no GPIO pins,
 * it exists purely as a time base. TIM6 is left alone because the excluded
 * cube-mx/stm32f0xx_hal_timebase_tim.c is written around it.
 *
 * The previous version of this file drove TIM2, carried over from another
 * board: STM32F030CC has no TIM2 at all, so it could never have worked.
 *
 * Timing: PCLK is 8 MHz with the APB prescaler at 1, so the timer clock is
 * 8 MHz too (the x2 rule only applies when the APB prescaler is > 1). PSC = 7
 * gives a 1 MHz count - 1 us resolution - and the 16-bit counter rolls over
 * every 65.536 ms.
 *
 * configRUN_TIME_COUNTER_TYPE is uint32_t while every timer on this part is
 * 16-bit (the 32-bit TIM2 does not exist here), so the counter is widened in
 * software: the update interrupt carries the upper half. Returning the raw CNT
 * instead would corrupt every delta taken across a rollover.
 */

#define RTMON_TIM              TIM7
#define RTMON_TIM_IRQn         TIM7_IRQn
#define RTMON_TIM_PRESCALER    (8U - 1U)   /* 8 MHz -> 1 MHz, 1 us per count */

static volatile uint32_t s_OverflowCount;

void bsp_rtmon_InitRunTimer(void)
{
  __HAL_RCC_TIM7_CLK_ENABLE();

  s_OverflowCount = 0;

  RTMON_TIM->CR1  = 0;
  RTMON_TIM->PSC  = RTMON_TIM_PRESCALER;
  RTMON_TIM->ARR  = 0xFFFFU;
  RTMON_TIM->EGR  = TIM_EGR_UG;    /* latch PSC/ARR */
  RTMON_TIM->SR   = 0;             /* UG above raises UIF - drop it */
  RTMON_TIM->DIER = TIM_DIER_UIE;
  RTMON_TIM->CR1  = TIM_CR1_CEN;

  /* Lowest priority: profiling must never delay I2C or the application. */
  HAL_NVIC_SetPriority(RTMON_TIM_IRQn, 3, 0);
  HAL_NVIC_EnableIRQ(RTMON_TIM_IRQn);
}

void TIM7_IRQHandler(void)
{
  if (RTMON_TIM->SR & TIM_SR_UIF)
  {
    RTMON_TIM->SR = ~TIM_SR_UIF;
    s_OverflowCount++;
  }
}

configRUN_TIME_COUNTER_TYPE bsp_rtmon_GetRunTimer(void)
{
  uint32_t hi, lo, hiAgain;

  /*
   * Re-read the upper half to catch a rollover landing between the two reads.
   * This assumes the update interrupt is serviced reasonably promptly - it
   * fires only ~15 times a second, so that holds unless something blocks
   * interrupts for tens of milliseconds.
   */
  do
  {
    hi      = s_OverflowCount;
    lo      = RTMON_TIM->CNT;
    hiAgain = s_OverflowCount;
  } while (hi != hiAgain);

  return (configRUN_TIME_COUNTER_TYPE)((hi << 16) | lo);
}
