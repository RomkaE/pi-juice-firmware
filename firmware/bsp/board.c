/*
 * board.c
 *
 *  Created on: 2026
 *      Author: Roman Egoshin
 */

#include <stddef.h>

#include "config.h"
#include "board.h"
#include "retained_memory.h"

// ST HAL/CubeMX:
#include "stm32f0xx_hal.h"
#include "cube-mx/gpio.h"
#include "cube-mx/dma.h"
#include "cube-mx/iwdg.h"

// FreeRTOS:
#include "FreeRTOS.h"
#include "task.h"

// LOG:
#include "log/log.h"

// System memory STM32F030xC:
#define SYSMEM_BOOT_BASE          0x1FFFD800

static bool s_StatePwr5V __attribute__((section("no_init")));

void bsp_Init(void)
{
  HAL_Init();
  HAL_SetTickFreq(HAL_TICK_FREQ_100HZ);

  MX_GPIO_Init();
  MX_DMA_Init();
}

void bsp_WdtStart(void)
{
#ifndef NDEBUG
  // Without this the counter keeps running while the core is halted on a breakpoint:
  __HAL_RCC_DBGMCU_CLK_ENABLE();
  __HAL_DBGMCU_FREEZE_IWDG();
#endif

  MX_IWDG_Init();
}

void bsp_WdtRefresh(void)
{
  if (hiwdg.Instance)
    HAL_IWDG_Refresh(&hiwdg);
}

void bsp_Pwr5V_SetState(bool _state)
{
  static bool gpio_inited = false;
  if (_state)
  {
    HAL_GPIO_WritePin(PWR_5V_BOOST_EN_PORT, PWR_5V_BOOST_EN_PIN, GPIO_PIN_SET);
    s_StatePwr5V = true;
    LOG_INFO("[BSP] 5V DC-DC ENABLED");
  }
  else
  {
    HAL_GPIO_WritePin(PWR_5V_BOOST_EN_PORT, PWR_5V_BOOST_EN_PIN, GPIO_PIN_RESET);
    s_StatePwr5V = false;
    LOG_INFO("[BSP] 5V DC-DC DISABLED");
  }

  // Configure the pin as an output only after setting
  // its value to prevent an incorrect output state:
  if (!gpio_inited)
  {
    GPIO_InitTypeDef GPIO_InitStruct = { 0 };
    GPIO_InitStruct.Pin = PWR_5V_BOOST_EN_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(PWR_5V_BOOST_EN_PORT, &GPIO_InitStruct);
    gpio_inited = true;
  }
}

bool bsp_Pwr5V_GetState(void)
{
  return s_StatePwr5V;
}

void bsp_Pwr5V_Restore(void)
{
  if (retained_mem_GetStatus())
    bsp_Pwr5V_SetState(s_StatePwr5V);
  else
  {
    bool def_state = false;
    bsp_Pwr5V_SetState(def_state);
  }
}

__NO_RETURN void bsp_ResetCPU(void)
{
  /*
   * The IWDG is not reset by SYSRESETREQ - it keeps counting across the reboot. Without this kick a
   * counter that was already near expiry could bite before the new image reaches bsp_WdtStart(),
   * turning a deliberate reset into a watchdog one and adding a bogus DIAG_RESET_IWDG.
   */
  bsp_WdtRefresh();

  NVIC_SystemReset();
  while(1);
}

__NO_RETURN void bsp_StartBootloader(void)
{
  vTaskSuspendAll();

  // Disable and clear interrupts:
  __disable_irq();
  for (int i = 0; i <= 29; i++)
  {
    HAL_NVIC_DisableIRQ(i);
    HAL_NVIC_ClearPendingIRQ(i);
  }

  HAL_DeInit();

  __HAL_SYSCFG_REMAPMEMORY_SYSTEMFLASH();

  uint32_t stack = *(uint32_t*)SYSMEM_BOOT_BASE;
  uint32_t entry = *(uint32_t*)(SYSMEM_BOOT_BASE + 4);

  __set_MSP(stack);

  void (*bootloader)(void) = (void*)entry;
  bootloader();
  while(1);
}

// Bootloader "Go" jumps to the app without reset, leaving system memory mapped at 0x0.
// On Cortex-M0, non-zero MEM_MODE means this path was used; reset for a clean flash boot.
void bsp_EnsureCleanBoot(void)
{
  __HAL_RCC_SYSCFG_CLK_ENABLE();
  if ((SYSCFG->CFGR1 & SYSCFG_CFGR1_MEM_MODE) != 0u)
    bsp_ResetCPU();
}
