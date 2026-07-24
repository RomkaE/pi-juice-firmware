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

// ST HAL/CubeMX:
#include "stm32f0xx_hal.h"
#include "cube-mx/i2c.h"
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
								|| POW_SOURCE_NEED_POLL() \
								|| alarmEventFlag ))

/* Private variables ---------------------------------------------------------*/

/* Owned by cube-mx/i2c.c, cube-mx/rtc.c and cube-mx/iwdg.c since the split.
 * hi2c1/hi2c2 are declared by cube-mx/i2c.h. */



uint8_t resetStatus = 0;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* Buffer used for I2C transfer */
  //uint8_t i2cTrfBuffer[256];

static uint32_t lowPowerDealyTimer;
static uint32_t mainPollMsCounter;

PowerState_T state = STATE_INIT;

uint32_t executionState __attribute__((section("no_init"))); // used to indicate if there was unpredictable reset like watchdog expired

uint32_t lastHostCommandTimer __attribute__((section("no_init")));

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
	  LedSetRGB(LED1, 150, 0, 0);
	  LedSetRGB(LED2, 150, 0, 0);
	  HAL_Delay(500);
	  LedSetRGB(LED1, 0, 0, 150);
	  LedSetRGB(LED2, 0, 0, 150);
	  HAL_Delay(500);
	}
}

uint8_t extiFlag = 0;
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
  if (GPIO_Pin == GPIO_PIN_0)
  {
	  // CH_INT
	  chargerInterruptFlag = 1;
	  extiFlag = 1;
  } else if (GPIO_Pin == GPIO_PIN_7)
  {
	  // I2C SDA
	  extiFlag = 2;
  } else if (GPIO_Pin == GPIO_PIN_8) {
	  extiFlag = 4;
	  ioWakeupEvent = 1;
  } else {
	  // SW1, SW2, SW3
	  extiFlag = 3;
  }
}

static void main_init(void)
{
	/*
	 * Before touching any no_init variable: decide whether the retained section belongs to
	 * this image at all. If it does not, drop executionState so the tests below fall
	 * through to the cold-boot path and every retained value is re-read from NV.
	 */
	if (!RetainedMemoryCheck()) {
		executionState = 0;
	}

	if (executionState != EXECUTION_STATE_NORMAL && executionState != EXECUTION_STATE_UPDATE && executionState != EXECUTION_STATE_CONFIG_RESET) {
		if (__HAL_RCC_GET_FLAG(RCC_FLAG_PORRST)) {
			executionState = EXECUTION_STATE_POWER_RESET;
		} else {//if (__HAL_RCC_GET_FLAG(RCC_FLAG_SFTRST)){
			// updating from old firmware without executionState defined
			executionState = EXECUTION_STATE_UPDATE;
		} // else if (__HAL_RCC_GET_FLAG(RCC_FLAG_IWDGRST) {
	}
	__HAL_RCC_CLEAR_RESET_FLAGS();

	if ( executionState == EXECUTION_STATE_NORMAL ) {
		resetStatus = 1;
	} else {
		// initialize globals
		resetStatus = 0;
	}

	__HAL_FLASH_PREFETCH_BUFFER_ENABLE();

	NvInit();

	// Initialize all configured peripherals
	// 
	if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_10) == GPIO_PIN_RESET && executionState == EXECUTION_STATE_POWER_RESET)
	{
		executionState = EXECUTION_STATE_POWER_ON;
	}

	// TOD:
	MX_I2C1_Init();
	MX_I2C2_Init();
	MX_RTC_Init();
	MX_TIM3_Init();
	MX_TIM17_Init();
	MX_TIM1_Init();

	// TODO - fix
	extern void MX_TIM14_Init(void);
	MX_TIM14_Init();

	HAL_InitTick(TICK_INT_PRIORITY);

	if (!resetStatus) MS_TIME_COUNTER_INIT(lastHostCommandTimer);

	MS_TIME_COUNTER_INIT(mainPollMsCounter);
	MS_TIME_COUNTER_INIT(lowPowerDealyTimer);

	// TODO - move to bsp/drivers
	MX_IWDG_Init();

	BatteryInit();
	if (executionState == EXECUTION_STATE_POWER_ON) {
		HAL_Delay(100);  // after power-on, charger and fuel gauge requires initialization time
		FuelGaugeIcPreInit();
	}
	ChargerInit();
	PowerSourceInit();
	FuelGaugeInit();
	PowerManagementInit();
	LedInit();
	ButtonInit();
	RtcInit();
	IoControlInit();

	NvSetDataInitialized();
	/*if ( executionState == EXECUTION_STATE_CONFIG_RESET ) {
		LedSetRGB(1, 0, 255, 0);
	} else if (executionState == EXECUTION_STATE_POWER_RESET) {
		LedSetRGB(1, 255, 0, 0);
	} else if (executionState == EXECUTION_STATE_UPDATE) {
		LedSetRGB(1, 0, 0, 255);
	} else {
		LedSetRGB(1, 0, 0, 0);
	}*/

	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET); // ee write protect
	uint16_t var = 0;
	EE_ReadVariable(ID_EEPROM_ADR_NV_ADDR, &var);
	if ( (((~var)&0xFF) == (var>>8)) ) {
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, (var&0x02) ? GPIO_PIN_SET : GPIO_PIN_RESET);
	} else {
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_RESET); // default ee Adr
	}

	state = STATE_NORMAL;

	HAL_I2C_EnableListen_IT(&hi2c1);

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
    if ((hi2c2.ErrorCode
        & (HAL_I2C_ERROR_TIMEOUT | HAL_I2C_ERROR_BERR | HAL_I2C_ERROR_ARLO))
        || hi2c2.State != HAL_I2C_STATE_READY || hi2c2.XferCount)
    {
      HAL_I2C_DeInit(&hi2c2);
      MX_I2C2_Init();
      chargerI2cErrorCounter = 1;
    }
    if (chargerI2cErrorCounter > 10)
    {
      HAL_I2C_DeInit(&hi2c2);
      MX_I2C2_Init();
      chargerI2cErrorCounter = 1;
    }

    if (NEED_EVENT_POLL())
    {
      state = STATE_RUN;
    }
    // Load-current gating removed with the current-sensing feature; the 5V-rail-low test
    // below is the remaining low-power entry condition.
    else if ((analog_Get5vPi() < 4600 && !POW_VSYS_OUTPUT_EN_STATUS())
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

  LOG_INIT(LOG_LEVEL_VERBOSE);
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
