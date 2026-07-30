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
#include <to_refactor/fuel_gauge_lc709203f.h>
#include <to_refactor/io_control.h>
#include <to_refactor/power_management.h>
#include <to_refactor/power_source.h>
#include <to_refactor/rtc_ds1339_emu.h>
#include <to_refactor/time_count.h>
#include "main.h"
#include "nv.h"
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

// TODO !!??
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
	nv_Erase();

//	executionState = EXECUTION_STATE_CONFIG_RESET;
	/* Terminal indication: never returns, and HAL_Delay() spins at the APP task priority, so
	 * the IWDG is no longer refreshed and resets the board after ~8 s. Pre-existing - TODO.
	 * led_SetRGB() only queues, but the LED task outranks APP and applies it right away. */
	while(1) {
	  led_SetRGB(LED_D1, 150, 0, 0);
	  HAL_Delay(500);
	  led_SetRGB(LED_D1, 0, 0, 150);
	  HAL_Delay(500);
	}
}

// TODO
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
	if (retained_mem_GetStatus())
	  LOG_INFO("Retained Memory is VALID!");
	else
    LOG_WARNING("Retained Memory is INVALID!");

	// Reset reason:
  bool reset_init = false;        // TODO - check/remove
  // TODO add logs and processing all reset reasons:
  if (__HAL_RCC_GET_FLAG(RCC_FLAG_PORRST))
    reset_init = true;
  __HAL_RCC_CLEAR_RESET_FLAGS();

	/* Not fatal, but worth shouting about: with no usable emulated EEPROM every nv_read_U8()
	 * fails and the whole configuration silently falls back to its compile time default -
	 * including the I2C own addresses, which makes the board look dead to the host. */
	if (nv_Init() != NV_OK)
	  LOG_ERROR("[NV] Init failed, configuration falls back to defaults");

	// Initialize all configured peripherals
  // TODO:
	MX_RTC_Init();
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
	FuelGaugeInit(reset_init);
	PowerManagementInit(reset_init);
	ButtonInit();

	IoControlInit();

	// EEPROM IC management:
	HAL_GPIO_WritePin(EE_WP_PORT, EE_WP_PIN, GPIO_PIN_SET); // ee write protect
	uint8_t eeAddr = 0;   // nothing valid stored -> 0, which selects the default ee address
	(void)nv_read_U8(NV_ADDR_ID_EEPROM_ADR, &eeAddr);
	HAL_GPIO_WritePin(EE_ADDR_SEL_PORT, EE_ADDR_SEL_PIN,
	                  (eeAddr & 0x02) ? GPIO_PIN_SET : GPIO_PIN_RESET);

	// I2C master and 2 devices init:
  {
    i2c_master_Init();
    BatteryInit();


//    if (executionState == EXECUTION_STATE_COLD_START)
    {
      HAL_Delay(100);  // after power-on, charger and fuel gauge requires initialization time
      FuelGaugeIcPreInit(); // // TODO - check
    }

    ChargerInit(reset_init);
  }

  // I2C slave and device init:
  {
    i2c_slave_Init();
    RtcInit(reset_init);
  }
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
    ButtonTask();
    PowerManagementTask();

    // TODO -!?
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

  // Configure the system clock after SysTick initialization
  // because the HAL tick and delay functions depend on the FreeRTOS tick:
  bsp_ClockConfig();

  LOG_INFO("APP task started");

  // TODO - move/remove
  main_init();

  // Subsystems initialization:
  analog_Init();
  led_Init();   // after main_init(): the task needs nv_Init() and bsp_ClockConfig() done

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
