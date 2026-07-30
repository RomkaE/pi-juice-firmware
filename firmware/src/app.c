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
#include "iosystem/button.h"
#include <to_refactor/command_server.h>
#include <to_refactor/fuel_gauge_lc709203f.h>
#include <to_refactor/io_control.h>
#include <to_refactor/power_management.h>
#include <to_refactor/power_source.h>
#include <to_refactor/rtc_ds1339_emu.h>
#include <to_refactor/time_count.h>
#include "app.h"
#include "main.h"
#include "nv.h"
#include "retained_memory.h"
#include "charger_bq2416x.h"
#include "led.h"
#include "board.h"
#include "app-error/app_assert.h"

#include "driver/i2c/i2c_slave.h"
#include "driver/i2c/i2c_master.h"

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

static uint32_t lowPowerDealyTimer;
static uint32_t mainPollMsCounter;

// TODO !!??
uint32_t lastHostCommandTimer;

uint8_t i2cErrorCounter = 0;

extern uint32_t lastWakeupTimer;
extern uint8_t alarmEventFlag;

static TaskHandle_t s_TaskHandleApp;
static StaticTask_t TaskTCBApp;
static StackType_t TaskStackApp[256];

// System wide event queue - see the contract in app.h:
static QueueHandle_t s_EvtQueHandle;
static StaticQueue_t s_EvtQue;
static AppEvent_t s_EvtQueBuf[16];   // TODO - remove magic number

/*
 * What a configured button function does. The mapping lives here rather than in button.c on
 * purpose: the button module detects and knows how a button is configured, this decides what
 * that configuration means. Indexed by ButtonFunction_T, which is host visible - the codes
 * arrive raw from registers 0x110/0x112/0x114.
 */
static const ButtonEventCb_T buttonEventCbs[BUTTON_EVENT_FUNC_NUMBER] = {
  NULL,                         // BUTTON_EVENT_NO_FUNC
  button_OnEvent_PowerOn,       // BUTTON_EVENT_FUNC_POWER_ON
  button_OnEvent_PowerOff,      // BUTTON_EVENT_FUNC_POWER_OFF
  button_OnEvent_PowerReset,    // BUTTON_EVENT_FUNC_POWER_RESET
};

void button_OnEvent_DualLongPress(void) {
	// Reset to default
	nv_Erase();

	/* Terminal indication: never returns, and HAL_Delay() spins at the APP task priority, so
	 * the IWDG is no longer refreshed and resets the board after ~8 s. Pre-existing - TODO.
	 * led_SetRGB() only queues, but the LED task outranks APP and applies it right away. */
	// TODO - non return! WDT - is DISABLED!!!
	while(1) {
	  led_SetRGB(LED_D1, 150, 0, 0);
	  HAL_Delay(500);
	  led_SetRGB(LED_D1, 0, 0, 150);
	  HAL_Delay(500);
	}
}

// TODO - move to bsp:
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if (GPIO_Pin == CHG_INT_PIN)
  {
    charger_NotifyFromISR();
  }
  else if (GPIO_Pin == EXT_IO2_PIN)
  {
    ioWakeupEvent = 1;			    // TODO - change to notify
  }
  else if (GPIO_Pin == BTN_SW1_PIN || GPIO_Pin == BTN_SW2_PIN || GPIO_Pin == BTN_SW3_PIN)
  {
    // A press edge, wake the button task out of its idle block.
    button_NotifyFromISR();
  }
}

static bool main_init(void)
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
//	MX_IWDG_Init();

	PowerSourceInit(reset_init);
	PowerManagementInit(reset_init);

	IoControlInit();

	// EEPROM IC management:
	HAL_GPIO_WritePin(EE_WP_PORT, EE_WP_PIN, GPIO_PIN_SET); // ee write protect
	uint8_t eeAddr = 0;   // nothing valid stored -> 0, which selects the default ee address
	(void)nv_read_U8(NV_ADDR_ID_EEPROM_ADR, &eeAddr);
	HAL_GPIO_WritePin(EE_ADDR_SEL_PORT, EE_ADDR_SEL_PIN,
	                  (eeAddr & 0x02) ? GPIO_PIN_SET : GPIO_PIN_RESET);

	// I2C master init - battery/fuel_gauge/charger do their own device init once their tasks
	// are created, see TaskApp()'s subsystem-init block below.
  i2c_master_Init();

  // I2C slave and device init:
  {
    i2c_slave_Init();
    RtcInit(reset_init);
  }

  return reset_init;
}

static void main_poll(void)
{
  // TODO - move to rtc
  /*
  extern RTC_HandleTypeDef hrtc;
  if (alarmEventFlag || __HAL_RTC_ALARM_GET_FLAG(&hrtc, RTC_FLAG_ALRAF) != RESET)
  {
    EvaluateAlarm();
    alarmEventFlag = 0;
    __HAL_RTC_ALARM_CLEAR_FLAG(&hrtc, RTC_FLAG_ALRAF);
  }
  */

  // TODO - move to i2c slave proto/logic
  /*
  if (extiFlag == 2)
  {
    MS_TIME_COUNTER_INIT(lastHostCommandTimer);
  }
  extiFlag = 0;
  */

  // Refresh IWDG: reload counter
  // TODO - move to bsp/drivers
  /*
  extern IWDG_HandleTypeDef hiwdg;
  if (HAL_IWDG_Refresh(&hiwdg) != HAL_OK)
  {
    Error_Handler(); // Refresh Error
  }
  */
}

/*
 * Battery applied a profile change (in whichever of the three app_ProcessEvent() cases below);
 * this is what actually tells charger/fuel_gauge/power_source about it. Called unconditionally
 * after any of the three - redundantly re-sending the same profile to a module that isn't
 * affected by what changed is harmless (they only cache a copy, no expensive or destructive
 * work is gated on "did it really change"), and it keeps this simple: one fan-out point instead
 * of three slightly different ones.
 */
static void app_FanOutBatteryProfile(void)
{
  BatteryProfile_T profile;
  bool valid = battery_GetProfile(&profile);
  const BatteryProfile_T *p = valid ? &profile : NULL;
  charger_CmdSetBatProfile(p);
  fuel_gauge_CmdSetBatProfile(p);
  PowerSourceSetBatProfile(p);
}

static void app_ProcessEvent(const AppEvent_t *_evt)
{
  switch (_evt->type)
  {
    case APP_EVT_BUTTON:
    {
      uint8_t func = _evt->data.button.func;
      if (func < BUTTON_EVENT_FUNC_NUMBER && buttonEventCbs[func] != NULL)
        buttonEventCbs[func](_evt->data.button.index,
                             (ButtonEvent_T)_evt->data.button.event);
      break;
    }

    case APP_EVT_BUTTON_RESET_CONFIG:
      button_OnEvent_DualLongPress();
      break;

    case APP_EVT_BATTERY_SET_PROFILE:
      battery_ApplySetProfile(_evt->data.batterySetProfile.id, _evt->data.batterySetProfile.seq);
      app_FanOutBatteryProfile();
      break;

    case APP_EVT_BATTERY_WRITE_CUSTOM_PROFILE:
      battery_ApplyWriteCustomProfile(&_evt->data.batteryCustomProfile.profile, _evt->data.batteryCustomProfile.seq);
      app_FanOutBatteryProfile();
      break;

    case APP_EVT_BATTERY_WRITE_CUSTOM_EXTENDED_PROFILE:
      battery_ApplyWriteCustomExtendedProfile(&_evt->data.batteryCustomExtProfile.profile);
      app_FanOutBatteryProfile();
      break;

    case APP_EVT_CHARGER_INPUT_PRESENCE:
      charger_OnEvent_InputPresenceChanged(_evt->data.chargerInput.present);
      break;

    default:
      LOG_ERROR("[APP] Unknown event type %u", _evt->type);
      break;
  }
}

bool app_PostEvent(const AppEvent_t *_pEvent)
{
  if (s_EvtQueHandle == NULL)
    return false;

  BaseType_t sent;

  if (xPortIsInsideInterrupt() != pdFALSE)
  {
    BaseType_t woken = pdFALSE;
    sent = xQueueSendFromISR(s_EvtQueHandle, _pEvent, &woken);
    portYIELD_FROM_ISR(woken);
  }
  else
  {
    sent = xQueueSend(s_EvtQueHandle, _pEvent, 0);
  }

  return (sent == pdTRUE);
}

static void TaskApp(void *parameters)
{
  (void)parameters;

  // Configure the system clock after SysTick initialization
  // because the HAL tick and delay functions depend on the FreeRTOS tick:
  bsp_ClockConfig();

  LOG_INFO("APP task started");

  // TODO - move/remove
  bool reset_init = main_init();

  /*
   * Subsystems initialization. battery_Init() has no task of its own - it's a plain synchronous
   * NV load - so by the time it returns, battery_GetProfile() already reflects real data, and it
   * just needs to run before fuel_gauge_Init()/charger_Init(), which read it during their own
   * startup. Those two, in turn, each run at a priority above APP's, so xTaskCreateStatic()
   * inside them preempts and runs the new task to its first blocking wait before returning -
   * see led_Init()'s comment for the same property.
   */
  analog_Init();
  led_Init();
  button_Init();
  battery_Init();
  fuel_gauge_Init();
  charger_Init(reset_init);

  /*
   * No dedicated battery task to drive the charge-status LED anymore (see battery.h) - APP does
   * it here, deadline driven exactly like led.c's own blink phases, so the event queue wait is
   * a heartbeat rather than the only pacing.
   */
  TickType_t ledDeadline = xTaskGetTickCount() + pdMS_TO_TICKS(BATTERY_CHARGE_LED_PERIOD_MS);

  while(1)
  {
    AppEvent_t evt;
    int32_t remain = (int32_t)(ledDeadline - xTaskGetTickCount());
    TickType_t wait = (remain > 0) ? (TickType_t)remain : 0;

    if (xQueueReceive(s_EvtQueHandle, &evt, wait) == pdTRUE)
      app_ProcessEvent(&evt);

    if ((int32_t)(xTaskGetTickCount() - ledDeadline) >= 0) {
      battery_UpdateChargeLed();
      ledDeadline = xTaskGetTickCount() + pdMS_TO_TICKS(BATTERY_CHARGE_LED_PERIOD_MS);
    }
  }
}

void app_Init(void)
{
  // Create the event queue first: producers are started by TaskApp and post into it.
  s_EvtQueHandle = xQueueCreateStatic(sizeof(s_EvtQueBuf)/sizeof(s_EvtQueBuf[0]),
                           sizeof(s_EvtQueBuf[0]), (uint8_t*)s_EvtQueBuf, &s_EvtQue);
  ASSERT(s_EvtQueHandle != NULL);

  s_TaskHandleApp = xTaskCreateStatic(TaskApp, "APP", sizeof(TaskStackApp)/sizeof(StackType_t),
                           NULL, 5,
                           TaskStackApp, &TaskTCBApp);
  ASSERT(s_TaskHandleApp != NULL);
}
