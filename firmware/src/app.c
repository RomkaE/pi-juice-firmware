/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  *
  * COPYRIGHT(c) 2016 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include <stdbool.h>

#include "iosystem/analog.h"
#include <to_refactor/button.h>
#include <to_refactor/command_server.h>
#include <to_refactor/execution.h>
#include <to_refactor/fuel_gauge_lc709203f.h>
#include <to_refactor/io_control.h>
#include <to_refactor/power_management.h>
#include <to_refactor/power_source.h>
#include <to_refactor/rtc_ds1339_emu.h>
#include <to_refactor/time_count.h>
#include "main.h"
#include "eeprom.h"
#include "retained_memory.h"
#include "charger_bq2416x.h"
#include "led.h"

#include "app-error/app_assert.h"

// ST HAL/CubeMX:
#include "stm32f0xx_hal.h"
#include "board.h"
#include "cube-mx/i2c.h"
#include "driver/i2c/i2c_slave.h"
#include "driver/i2c/i2c_master.h"
#include "cube-mx/iwdg.h"
#include "cube-mx/tim.h"
#include "cube-mx/rtc.h"

// FreeRTOS:
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

// LOG:
#include "log/log.h"

/* Slave addresses live in cube-mx/i2c.h together with the I2C1 setup. */

#define NEED_EVENT_POLL()		((chargerNeedPoll \
								|| extiFlag \
								|| rtcWakeupEventFlag \
								|| PWR_SOURCE_NEED_POLL() \
								|| alarmEventFlag ))

/* Private variables ---------------------------------------------------------*/

/* Owned by cube-mx/i2c.c, cube-mx/rtc.c and cube-mx/iwdg.c since the split.
 * hi2c1/hi2c2 are declared by cube-mx/i2c.h. */

//bool resetStatus = 0;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* Buffer used for I2C transfer */
  //uint8_t i2cTrfBuffer[256];

static uint32_t lowPowerDealyTimer;
static uint32_t mainPollMsCounter;

PowerState_T state = STATE_INIT;

uint32_t executionState __attribute__((section("no_init"))); // used to indicate if there was unpredictable reset like watchdog expired

uint32_t lastHostCommandTimer;

uint8_t i2cErrorCounter = 0;

extern uint32_t lastWakeupTimer;
extern uint8_t alarmEventFlag;

static TaskHandle_t s_TaskHandleApp;
static StaticTask_t TaskTCBApp;
static StackType_t TaskStackApp[1024 * 1];

/* Private function prototypes -----------------------------------------------*/

void ButtonDualLongPressEventCb(void) {
	// Reset to default
	NvEreaseAllVariables();

	executionState = EXECUTION_STATE_CONFIG_RESET;
	while(1) {
	  LedSetRGB(LED_D1, 150, 0, 0);
	  HAL_Delay(500);
	  LedSetRGB(LED_D1, 0, 0, 150);
	  HAL_Delay(500);
	}
}

uint8_t extiFlag = 0;
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
  if (GPIO_Pin == CHG_INT_PIN)
  {
	  // CH_INT
	  chargerInterruptFlag = 1;
	  extiFlag = 1;
  } else if (GPIO_Pin == I2C1_SDA_PIN)
  {
	  // I2C SDA
	  extiFlag = 2;
  } else if (GPIO_Pin == EXT_IO2_PIN) {
	  extiFlag = 4;
	  ioWakeupEvent = 1;
  } else {
	  // SW1, SW2, SW3
	  extiFlag = 3;
  }
}

static void main_init(void)
{
  LOG_INFO("1. executionState=0x%08X", executionState);

	if (!RetainedMemoryCheck()) {
	  LOG_WARNING("Retained Memory is INVALID!");
		executionState = EXECUTION_STATE_COLD_START;
	}
	else
	  LOG_INFO("Retained Memory is VALID!");

	if (executionState != EXECUTION_STATE_NORMAL && executionState != EXECUTION_STATE_CONFIG_RESET)
	{
		if (__HAL_RCC_GET_FLAG(RCC_FLAG_PORRST))
			executionState = EXECUTION_STATE_POR_RESET;
	}
	__HAL_RCC_CLEAR_RESET_FLAGS();

	// TODO - check/remove:
	bool reset_init = false;
	if ( executionState != EXECUTION_STATE_NORMAL )
	  reset_init = true;

  // Cold start detection:
  if (HAL_GPIO_ReadPin(PWR_5V_BOOST_EN_PORT, PWR_5V_BOOST_EN_PIN) == GPIO_PIN_RESET && executionState == EXECUTION_STATE_POR_RESET)
  {
    executionState = EXECUTION_STATE_COLD_START;
  }
  LOG_INFO("2. executionState=0x%08X", executionState);

	NvInit();

	// Initialize all configured peripherals
  // TODO:
	MX_RTC_Init();
	MX_TIM3_Init();
	MX_TIM1_Init();

	// TODO - move/remove
	extern void MX_TIM14_Init(void);
	MX_TIM14_Init();

	HAL_InitTick(TICK_INT_PRIORITY);

	MS_TIME_COUNTER_INIT(lastHostCommandTimer);
	MS_TIME_COUNTER_INIT(mainPollMsCounter);
	MS_TIME_COUNTER_INIT(lowPowerDealyTimer);

	// TODO - move to bsp/drivers
	MX_IWDG_Init();

	PowerSourceInit(reset_init);
	FuelGaugeInit();
	PowerManagementInit(reset_init);
	LedInit();
	ButtonInit();

	IoControlInit();

	HAL_GPIO_WritePin(EE_WP_PORT, EE_WP_PIN, GPIO_PIN_SET); // ee write protect
	uint16_t var = 0;
	EE_ReadVariable(ID_EEPROM_ADR_NV_ADDR, &var);
	if ( (((~var)&0xFF) == (var>>8)) ) {
		HAL_GPIO_WritePin(EE_ADDR_SEL_PORT, EE_ADDR_SEL_PIN, (var&0x02) ? GPIO_PIN_SET : GPIO_PIN_RESET);
	} else {
		HAL_GPIO_WritePin(EE_ADDR_SEL_PORT, EE_ADDR_SEL_PIN, GPIO_PIN_RESET); // default ee Adr
	}

	state = STATE_NORMAL;

	// I2C master and 2 devices init:
  {
    i2c_master_Init();
    BatteryInit();

    // TODO - WTF?
    if (executionState == EXECUTION_STATE_COLD_START) {
      HAL_Delay(100);  // after power-on, charger and fuel gauge requires initialization time
      FuelGaugeIcPreInit();
    }
    ChargerInit(reset_init);
  }

  // I2C slave and device init:
  {
    i2c_slave_Init();
    RtcInit(reset_init);
  }

	executionState = EXECUTION_STATE_NORMAL; // after initialization indicate it for future wd resets

}

static void main_poll(void)
{
  // Do not disturb i2c transfer if this is i2c interrupt wakeup
  if ( MS_TIME_COUNT(mainPollMsCounter) >= TICK_PERIOD_MS || NEED_EVENT_POLL())
  {
    PowerSource5vIoDetectionTask();
    ChargerTask();
    FuelGaugeTask();
    BatteryTask();
    PowerSourceTask();

    // TODO - move to bsp/drivers
    extern RTC_HandleTypeDef hrtc;
    if (alarmEventFlag
        || __HAL_RTC_ALARM_GET_FLAG(&hrtc, RTC_FLAG_ALRAF) != RESET)
    {
      EvaluateAlarm();
      alarmEventFlag = 0;
      __HAL_RTC_ALARM_CLEAR_FLAG(&hrtc, RTC_FLAG_ALRAF);
    }
    LedTask();
    //if (MS_TIME_COUNT(mainPollMsCounter) > 98) {
    ButtonTask();
    PowerManagementTask();
    //}
    if (NEED_EVENT_POLL())
    {
      state = STATE_RUN;
    }
    // Load-current gating removed with the current-sensing feature; the 5V-rail-low test
    // below is the remaining low-power entry condition.
    else if ((analog_Get5vPi() < 4600 /*&& !PWR_VSYS_OUTPUT_EN_STATUS()*/)
        && MS_TIME_COUNT(lastHostCommandTimer) > 5000
        && MS_TIME_COUNT(lowPowerDealyTimer) >= 22
        && MS_TIME_COUNT(lastWakeupTimer) > 20000
        && chargerStatus == CHG_NO_VALID_SOURCE && !IsButtonActive())
    {
      state = STATE_LOWPOWER;
    }
    else
    {
      state = STATE_NORMAL;
    }

    if (extiFlag == 2)
    {
      MS_TIME_COUNTER_INIT(lastHostCommandTimer);
    }
    extiFlag = 0;

    // Refresh IWDG: reload counter
    // TODO - move to bsp/drivers
    extern IWDG_HandleTypeDef hiwdg;
    if (HAL_IWDG_Refresh(&hiwdg) != HAL_OK)
    {
      Error_Handler(); // Refresh Error
    }

    MS_TIME_COUNTER_INIT(mainPollMsCounter);
  }
}

static void TaskApp(void *parameters)
{
  (void)parameters;

  LOG_INIT(LOG_LEVEL_DEBUG);
  LOG_INFO("APP task started");

  // TODO - move/remove
  main_init();

  // Subsystems initialization:
  analog_Init();

  while(1)
  {
	  main_poll();  // TODO - move/remove
  }
}

void app_Init(void)
{
  s_TaskHandleApp = xTaskCreateStatic(TaskApp, "APP", sizeof(TaskStackApp)/sizeof(StackType_t),
                           NULL, 5,
                           TaskStackApp, &TaskTCBApp);
  ASSERT(s_TaskHandleApp != NULL);
}
