
#include <stdint.h>
#include <stdbool.h>

#include "iosystem/analog.h"
#include "iosystem/button.h"
#include "power/fuel_gauge_lc709203f.h"
#include "power/power_manager.h"
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
  pwr_mngr_HostTurnOn();
  button_ClearEvent(b);
}

static void app_OnButtonPowerOff(uint8_t b, ButtonEvent_T event)
{
  (void)event;
  pwr_mngr_HostTurnOff();
  button_ClearEvent(b);
}

static void app_OnButtonPowerReset(uint8_t b, ButtonEvent_T event)
{
  (void)event;
  pwr_mngr_HostReset();
  button_ClearEvent(b);
}

static const ButtonEventCb_T buttonEventCbs[BUTTON_EVENT_FUNC_NUMBER] = {
  NULL,                         // BUTTON_EVENT_NO_FUNC
  app_OnButtonPowerOn,          // BUTTON_EVENT_FUNC_POWER_ON
  app_OnButtonPowerOff,         // BUTTON_EVENT_FUNC_POWER_OFF
  app_OnButtonPowerReset,       // BUTTON_EVENT_FUNC_POWER_RESET
};

static void button_OnEvent_DualLongPress(void)
{
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
    // TODO
//    pwr_mngr_SetIoWakeupEvent();
  }
  else if (GPIO_Pin == BTN_SW1_PIN || GPIO_Pin == BTN_SW2_PIN || GPIO_Pin == BTN_SW3_PIN)
  {
    // A press edge, wake the button task out of its idle block.
    button_NotifyFromISR();
  }
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
  charger_SetBatProfile(p);
  fuel_gauge_SetBattProfile(p);
  // TODO
//  pwr_mngr_SetBatProfile(p);
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
//      battery_ApplyWriteCustomExtendedProfile(&_evt->data.batteryCustomExtProfile.profile);
      app_FanOutBatteryProfile();
      break;

    case APP_EVT_CHARGER_INPUT_PRESENCE:
      //
//      TODO
//      charger_OnEvent_InputPresenceChanged(_evt->data.chargerInput.present);
      break;

    case APP_EVT_CHRGR_BATT_PRESENCE:
      // The flag itself is one of two sources - battery.c re-reads both and decides:
//      app_UpdateBatteryPresence();
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

    /*
     * Everything else the charger publishes. Nothing acts on these yet, so they share one weak
     * hook rather than seven empty ones - the event type is passed along, so a consumer that
     * appears later switches on it without the producer side changing at all.
     */
    case APP_EVT_CHARGER_STATUS:
    case APP_EVT_CHARGER_FAULT:
    case APP_EVT_CHARGER_IN_STAT:
    case APP_EVT_CHARGER_USB_STAT:
    case APP_EVT_CHARGER_TS_FAULT:
    case APP_EVT_CHARGER_DPM:
//      TODO
//      charger_OnEvent_ValueChanged(_evt->type, _evt->data.chargerValue.value);
      break;

    case APP_EVT_POWER_PROTECTION:
//      pwr_mngr_OnEvent_Protection(_evt->data.powerTrip.trip);
      break;

    default:
      LOG_ERROR("[APP] Unknown event type %u", _evt->type);
      break;
  }
}

// TODO - set static and move
void app_PostEvent(const AppEvent_t *_pEvent)
{
  if (s_EvtQueHandle == NULL)
    return;

  BaseType_t sent;
  BaseType_t woken = pdFALSE;
  if (xPortIsInsideInterrupt() != pdFALSE)
    sent = xQueueSendFromISR(s_EvtQueHandle, _pEvent, &woken);
  else
    sent = xQueueSend(s_EvtQueHandle, _pEvent, 0);

  if (!sent)
  {
    LOG_CRITICAL("[CHG] APP queue full. Event %u dropped");
    // TODO!!!
  }

  // Call this at the end of a potential interrupt, because on some FreeRTOS
  // ports, this function may trigger an immediate context switch:
  portYIELD_FROM_ISR(woken);
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
  s_FgReqConfigSeq = seq;
  app_PostEvent(&evt);
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
  /* Stripped here, where the host byte enters, so that NV, the read-back and the charger all carry
   * the same value. The host's verifying write of a refused bit will fail, and that is the point. */
  config = charger_SanitizeInputsConfig(config);

  uint8_t seq = (uint8_t)(s_ChgReqInputsSeq + 1);
  AppEvent_t evt = { .type = APP_EVT_CHARGER_SET_INPUTS_CONFIG };
  evt.data.chargerConfig.config = config;
  evt.data.chargerConfig.seq = seq;

  s_ChgReqInputsConfig = config;
  s_ChgReqInputsSeq = seq;
  app_PostEvent(&evt);
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
  s_ChgReqChargingSeq = seq;
  app_PostEvent(&evt);
}

uint8_t app_ChargerReadChargingConfig(void)
{
  return (s_ChgReqChargingSeq != s_ChgAppliedChargingSeq)
      ? s_ChgReqChargingConfig : s_ChgChargingReadout;
}

static bool Init(void)
{
  bool cold_start = false;
  if (retained_mem_GetStatus())
    LOG_INFO("Retained Memory is VALID!");
  else
  {
    LOG_WARNING("Retained Memory is INVALID!");
    cold_start = true;
  }

  // TODO - move to BSP:
  if (__HAL_RCC_GET_FLAG(RCC_FLAG_PORRST))
  {
    // TODO add logs and processing all reset reasons
  }
  __HAL_RCC_CLEAR_RESET_FLAGS();

  /* Not fatal, but worth shouting about: with no usable emulated EEPROM every nv_read_U8()
   * fails and the whole configuration silently falls back to its compile time default -
   * including the I2C own addresses, which makes the board look dead to the host. */
  if (nv_Init() != NV_OK)
    LOG_ERROR("[NV] Init failed, configuration falls back to defaults");

  // Initialize all configured peripherals
  // TODO move:
  MX_RTC_Init();
  extern void MX_TIM1_Init(void);
  MX_TIM1_Init();

  // TODO - move/remove
  extern void MX_TIM14_Init(void);
  MX_TIM14_Init();

  // TODO - move to bsp/drivers
//  MX_IWDG_Init();

  pwr_mngr_Init(cold_start);

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
    RtcInit(cold_start);
  }

  analog_Init();
  led_Init();
  button_Init();

  {
    battery_Init();

    BatteryProfile_T batt_profile;
    bool valid = battery_GetProfile(&batt_profile);

    if (valid)
      fuel_gauge_Init(&batt_profile);
    else
      fuel_gauge_Init(NULL);

    /* Nor does the charger read NV itself. Both bytes are handed over below, together with the
     * profile - charger_SetBatProfile() cannot be used before charger_Init() has made the queue. */
    uint8_t chgInputs, chgCharging;
    if (nv_read_U8(NV_ADDR_CHARGER_INPUTS_CONFIG, &chgInputs) != NV_OK)
      chgInputs = CHARGER_INPUTS_CONFIG_DEFAULT;
    if (nv_read_U8(NV_ADDR_CHARGING_CONFIG, &chgCharging) != NV_OK)
      chgCharging = CHARGER_CHARGING_CONFIG_DEFAULT;

    // Whatever NV holds may predate the precedence becoming board policy.
    chgInputs = charger_SanitizeInputsConfig(chgInputs);

    s_ChgInputsReadout = chgInputs;
    s_ChgChargingReadout = chgCharging;
    charger_Init(valid ? &batt_profile : NULL, chgInputs, chgCharging);

    // TODO
    // pwr_mngr_Init() ran in main_init(), before battery_Init(), so its cutoff is still the default.
//    pwr_mngr_SetBatProfile(valid ? &batt_profile : NULL);
  }

  return cold_start;
}

typedef enum
{
  SM_APP_OFF = 0,
  SM_APP_ON,
  SM_APP_LOW_BATT
} SM_App_t;

static void TaskApp(void *parameters)
{
  (void)parameters;

  // Clock configuration after SysTick initialization, since in this project
  // the HAL tick and delay functions depend on the FreeRTOS system tick:
  bsp_ClockConfig();

  LOG_INFO("APP task started");
  Init();

  // Init app fsm:
  static SM_App_t state = SM_APP_OFF;
  if (bsp_Pwr5V_GetState())
    state = SM_APP_ON;
  SM_App_t next = SM_APP_OFF;
  while(1)
  {
    AppEvent_t evt;
    BaseType_t res = xQueueReceive(s_EvtQueHandle, &evt, portMAX_DELAY);
    if (res != pdPASS)
      continue;

    bool repeat;
    do {
      repeat = false;
      switch (state)
      {
        case SM_APP_OFF:
        break;
        case SM_APP_ON:
        break;
        case SM_APP_LOW_BATT:
        break;
      }

      if (state != next)
      {
        state = next;
        repeat = true;
        evt.type = APP_EVT_SM_ENTRY;
      }
    } while(repeat);
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
