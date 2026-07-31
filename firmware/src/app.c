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
#include "power/fuel_gauge_lc709203f.h"
#include "power/power_management.h"
#include "power/power_source.h"
#include "power/charger_bq2416x.h"

#include <to_refactor/command_server.h>
#include <to_refactor/rtc_ds1339_emu.h>
#include <to_refactor/io_control.h>
#include "app.h"
#include "main.h"
#include "nv.h"
#include "retained_memory.h"
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

// TODO !!??
uint32_t lastHostCommandTimer;

uint8_t i2cErrorCounter = 0;

extern uint8_t alarmEventFlag;

static TaskHandle_t s_TaskHandleApp;
static StaticTask_t TaskTCBApp;
static StackType_t TaskStackApp[256];

// System wide event queue - see the contract in app.h:
static QueueHandle_t s_EvtQueHandle;
static StaticQueue_t s_EvtQue;
static AppEvent_t s_EvtQueBuf[16];   // TODO - remove magic number

/*
 * Request mirror for the fuel gauge configuration register, same idiom as led.c/charger.c: a host
 * write is applied asynchronously, so until it has been, reads answer with what was asked for
 * rather than with what is still in effect.
 */
static uint8_t s_FgReqConfig;
static volatile uint8_t s_FgReqConfigSeq;
static volatile uint8_t s_FgAppliedConfigSeq;

/* Same mirror for the charger's two configuration registers. */
static uint8_t s_ChgReqInputsConfig, s_ChgInputsReadout;
static volatile uint8_t s_ChgReqInputsSeq, s_ChgAppliedInputsSeq;
static uint8_t s_ChgReqChargingConfig, s_ChgChargingReadout;
static volatile uint8_t s_ChgReqChargingSeq, s_ChgAppliedChargingSeq;

/*
 * What a configured button function does. The mapping lives here rather than in button.c on
 * purpose: the button module detects and knows how a button is configured, this decides what
 * that configuration means. Indexed by ButtonFunction_T, which is host visible - the codes
 * arrive raw from registers 0x110/0x112/0x114.
 */
static void app_OnButtonPowerOn(uint8_t b, ButtonEvent_T event)
{
  (void)event;
  power_HostTurnOn();
  button_ClearEvent(b);
}

static void app_OnButtonPowerOff(uint8_t b, ButtonEvent_T event)
{
  (void)event;
  power_HostTurnOff();
  button_ClearEvent(b);
}

static void app_OnButtonPowerReset(uint8_t b, ButtonEvent_T event)
{
  (void)event;
  power_HostReset();
  button_ClearEvent(b);
}

static const ButtonEventCb_T buttonEventCbs[BUTTON_EVENT_FUNC_NUMBER] = {
  NULL,                         // BUTTON_EVENT_NO_FUNC
  app_OnButtonPowerOn,          // BUTTON_EVENT_FUNC_POWER_ON
  app_OnButtonPowerOff,         // BUTTON_EVENT_FUNC_POWER_OFF
  app_OnButtonPowerReset,       // BUTTON_EVENT_FUNC_POWER_RESET
};

static void button_OnEvent_DualLongPress(void) {
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

//	MS_TIME_COUNTER_INIT(lastHostCommandTimer);
//	MS_TIME_COUNTER_INIT(mainPollMsCounter);
//	MS_TIME_COUNTER_INIT(lowPowerDealyTimer);

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

//static void main_poll(void)
//{
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
//}

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
  charger_SetBatProfile(p);
  fuel_gauge_SetBatProfile(p);
  PowerSourceSetBatProfile(p);
}

/*
 * Battery presence: battery.c aggregates it, this hands the answer to whoever needs to act on it.
 * Called both on the charger's BATSTAT edge and on the periodic tick below, because the second
 * source of the aggregate - the pack voltage - has no edge of its own.
 */
static void app_UpdateBatteryPresence(void)
{
  if (battery_UpdatePresence())
    fuel_gauge_SetBatteryPresent(battery_IsPresent());
}

/*
 * Battery temperature verdict: battery.c combines the profile's thresholds with whatever
 * temperature source is alive, and the charger is told the answer. The charger never reads the
 * fuel gauge itself - that was the last cross-dependency inside components/power.
 */
static void app_UpdateThermalState(void)
{
  if (battery_UpdateThermalState())
    charger_SetThermalState(battery_GetThermalState());
}

/*
 * Persist the fuel gauge configuration and only then hand it to the driver. The read-back is what
 * decides: whatever ends up in NV is what the fuel gauge is told to run with, so a store that
 * silently went wrong cannot leave the two disagreeing.
 */
static void app_ApplyFuelGaugeConfig(uint8_t config, uint8_t seq)
{
  if (nv_write_U8(NV_ADDR_FUEL_GAUGE_CONFIG_MASK, config) != NV_OK)
    LOG_ERROR("[APP] NV write of the fuel gauge config failed");

  uint8_t stored;
  if (nv_read_U8(NV_ADDR_FUEL_GAUGE_CONFIG_MASK, &stored) != NV_OK) {
    LOG_ERROR("[APP] NV read-back of the fuel gauge config failed, falling back to auto detect");
    stored = FUEL_GAUGE_CONFIG_DEFAULT;
  }

  fuel_gauge_SetConfig(stored);
  s_FgAppliedConfigSeq = seq;
}

/*
 * The charger's two configuration bytes follow the same rule, with one wrinkle of their own that
 * comes straight from the host protocol: bit 7 asks for the value to be stored, and a read-back
 * only counts as verified when the low seven bits match what was requested.
 */
static uint8_t app_PersistChargerConfig(uint8_t _nvAddr, uint8_t _config)
{
  if (_config & 0x80) {
    if (nv_write_U8(_nvAddr, _config) != NV_OK)
      LOG_ERROR("[APP] NV write of charger config %u failed", _nvAddr);
  }

  uint8_t stored = 0;
  bool verified = (nv_read_U8(_nvAddr, &stored) == NV_OK)
               && ((_config & 0x7F) == (stored & 0x7F));

  return verified ? stored : _config;
}

static void app_ApplyChargerInputsConfig(uint8_t config, uint8_t seq)
{
  s_ChgInputsReadout = app_PersistChargerConfig(NV_ADDR_CHARGER_INPUTS_CONFIG, config);
  charger_SetInputsConfig(config);
  s_ChgAppliedInputsSeq = seq;
}

static void app_ApplyChargerChargingConfig(uint8_t config, uint8_t seq)
{
  s_ChgChargingReadout = app_PersistChargerConfig(NV_ADDR_CHARGING_CONFIG, config);
  charger_SetChargingConfig(config);
  s_ChgAppliedChargingSeq = seq;
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

    case APP_EVT_BATTERY_PRESENCE:
      // The flag itself is one of two sources - battery.c re-reads both and decides:
      app_UpdateBatteryPresence();
      break;

    case APP_EVT_FUEL_GAUGE_SET_CONFIG:
      app_ApplyFuelGaugeConfig(_evt->data.fuelGaugeConfig.config, _evt->data.fuelGaugeConfig.seq);
      break;

    case APP_EVT_CHARGER_SET_INPUTS_CONFIG:
      app_ApplyChargerInputsConfig(_evt->data.chargerConfig.config, _evt->data.chargerConfig.seq);
      break;

    case APP_EVT_CHARGER_SET_CHARGING_CONFIG:
      app_ApplyChargerChargingConfig(_evt->data.chargerConfig.config, _evt->data.chargerConfig.seq);
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

int8_t app_FuelGaugeCmdSetConfig(uint8_t *data, uint16_t len)
{
  (void)len;

  if (!fuel_gauge_IsConfigValid(data[0]))
    return 1;

  uint8_t seq = (uint8_t)(s_FgReqConfigSeq + 1);
  AppEvent_t evt = { .type = APP_EVT_FUEL_GAUGE_SET_CONFIG };
  evt.data.fuelGaugeConfig.config = data[0];
  evt.data.fuelGaugeConfig.seq = seq;

  s_FgReqConfig = data[0];
  if (app_PostEvent(&evt))
    s_FgReqConfigSeq = seq;

  return 0;
}

void app_FuelGaugeReadConfig(uint8_t data[], uint16_t *len)
{
  data[0] = (s_FgReqConfigSeq != s_FgAppliedConfigSeq)
      ? s_FgReqConfig
      : fuel_gauge_GetConfig();
  *len = 1;
}

void app_ChargerCmdWriteInputsConfig(uint8_t config)
{
  uint8_t seq = (uint8_t)(s_ChgReqInputsSeq + 1);
  AppEvent_t evt = { .type = APP_EVT_CHARGER_SET_INPUTS_CONFIG };
  evt.data.chargerConfig.config = config;
  evt.data.chargerConfig.seq = seq;

  s_ChgReqInputsConfig = config;
  if (app_PostEvent(&evt))
    s_ChgReqInputsSeq = seq;
}

uint8_t app_ChargerReadInputsConfig(void)
{
  return (s_ChgReqInputsSeq != s_ChgAppliedInputsSeq) ? s_ChgReqInputsConfig : s_ChgInputsReadout;
}

void app_ChargerCmdWriteChargingConfig(uint8_t config)
{
  uint8_t seq = (uint8_t)(s_ChgReqChargingSeq + 1);
  AppEvent_t evt = { .type = APP_EVT_CHARGER_SET_CHARGING_CONFIG };
  evt.data.chargerConfig.config = config;
  evt.data.chargerConfig.seq = seq;

  s_ChgReqChargingConfig = config;
  if (app_PostEvent(&evt))
    s_ChgReqChargingSeq = seq;
}

uint8_t app_ChargerReadChargingConfig(void)
{
  return (s_ChgReqChargingSeq != s_ChgAppliedChargingSeq)
      ? s_ChgReqChargingConfig : s_ChgChargingReadout;
}

static void TaskApp(void *parameters)
{
  (void)parameters;

  // Configure the system clock after SysTick initialization
  // because the HAL tick and delay functions depend on the FreeRTOS tick:
  bsp_ClockConfig();

  LOG_INFO("APP task started");

  main_init();

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

  /* The fuel gauge does not read NV itself - see app_ApplyFuelGaugeConfig(). */
  uint8_t fg_cfg_mask;
  if (nv_read_U8(NV_ADDR_FUEL_GAUGE_CONFIG_MASK, &fg_cfg_mask) != NV_OK ||
      !fuel_gauge_IsConfigValid(fg_cfg_mask))
    fg_cfg_mask = FUEL_GAUGE_CONFIG_DEFAULT;
  fuel_gauge_Init(fg_cfg_mask);

  /* Nor does the charger. Both bytes are used only on a power-on reset - see charger_Init(). */
  uint8_t chgInputs, chgCharging;
  if (nv_read_U8(NV_ADDR_CHARGER_INPUTS_CONFIG, &chgInputs) != NV_OK)
    chgInputs = CHARGER_INPUTS_CONFIG_DEFAULT;
  if (nv_read_U8(NV_ADDR_CHARGING_CONFIG, &chgCharging) != NV_OK)
    chgCharging = CHARGER_CHARGING_CONFIG_DEFAULT;
  s_ChgInputsReadout = chgInputs;
  s_ChgChargingReadout = chgCharging;
  charger_Init(chgInputs, chgCharging);

  /*
   * Neither device asks anybody for anything - they are told. The profile goes in here; battery
   * presence and the thermal verdict arrive on the charger's first BATSTAT edge or on the
   * periodic tick below, whichever comes first.
   */
  {
    BatteryProfile_T profile;
    const BatteryProfile_T *p = battery_GetProfile(&profile) ? &profile : NULL;
    fuel_gauge_SetBatProfile(p);
    charger_SetBatProfile(p);
  }

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
      app_UpdateBatteryPresence();
      app_UpdateThermalState();
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
