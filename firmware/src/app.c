/*
 * main.c
 *
 *  Created on: Jul 20, 2026
 *      Author: Roman Egoshin
 */

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
#include "utils/time_count.h"
#include "app-error/app_error.h"

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
#include "timers.h"

// LOG:
#include "log/log.h"

typedef enum
{
  SM_APP_OFF = 0,    // rail down, waiting for a reason to bring the host up
  SM_APP_POWER_UP,   // rail raised, giving it time before its output is judged
  SM_APP_ON,         // host powered
  SM_APP_LOW_BATT,   // rail down, cut by a protection, waiting for energy to come back
  SM_APP_COUNT       // also bounds an entry chain - see fsm_Dispatch()
} AppState_t;

/* Assigned here once, at startup, and only by fsm_Dispatch() after that. */
static AppState_t s_State = SM_APP_OFF;

uint8_t i2cErrorCounter = 0;

extern uint8_t alarmEventFlag;

static TaskHandle_t s_TaskHandleApp;
static StaticTask_t TaskTCBApp;
static StackType_t TaskStackApp[256];

/*
 * One shot, armed for exactly the delay the host asked for in register 0x62 - there is nothing to
 * poll for, so nothing polls.
 */
static TimerHandle_t s_TimerPowerOffHandle;
static StaticTimer_t s_TimerPowerOff;

/*
 * "The host asked to be cut, and it has not happened yet." The deadline itself is the timer's -
 * nothing here duplicates it. Set from the I2C interrupt, cleared by the APP task.
 */
static volatile bool s_PowerOffPending;

/*
 * The wake triggers and the host watchdog: unlike the power off, these have nothing to arm a one
 * shot from - they answer to readings that arrive on their own (charge level, how long the host
 * has been quiet), so they are re-evaluated on a beat.
 */
#define APP_POWER_POLICY_PERIOD_MS  500

static TimerHandle_t s_PolicyTimerHandle;
static StaticTimer_t s_PolicyTimer;

// One shot that completes a power cycle: the rail comes back up when it fires.
static TimerHandle_t s_RailOnTimerHandle;
static StaticTimer_t s_RailOnTimer;

/*
 * One shot that ends SM_APP_POWER_UP. The boost needs time before its output means anything, so
 * the rail is brought up, left alone for this long, and only then judged.
 */
#define APP_POWER_UP_TIME_MS    50

static TimerHandle_t s_TimerPowerUpHandle;
static StaticTimer_t s_TimerPowerUp;

/* How long the host must have been silent before an armed trigger may restart it. */
/*
#define APP_HOST_QUIET_MS         15000
#define APP_TURNON_HOST_QUIET_MS  11000
#define APP_TURNON_WAKE_QUIET_MS  12000
#define APP_WAKE_REARM_MS         30000
*/

#define APP_WAKEUP_ON_CHARGE_OFF  0xFFFF
#define APP_WAKEUP_ON_CHARGE_MIN  5     // RSOC 5%, armed when a protection cuts the rail

static uint8_t s_WakeupOnChargeConfig;      // register 0x63 byte
static bool s_WakeupOnChargeArmed __attribute__((section("no_init")));

static uint16_t s_WdtHostConfig;        // persisted minutes
static uint32_t s_WdtHostPeriodMs;
static uint32_t s_WdtHostTimer;

// System wide event queue - see the contract in app.h:
static QueueHandle_t s_EvtQueHandle;
static StaticQueue_t s_EvtQue;
static AppEvent_t s_EvtQueBuf[16];      // TODO - remove magic number

/*
 * Request mirror for the fuel gauge configuration register, same idiom as led.c/charger.c: a host
 * write is applied asynchronously, so until it has been, reads answer with what was asked for
 * rather than with what is still in effect.
 */
static uint8_t s_FgReqConfig;
static uint8_t s_FgReqConfigSeq;
static uint8_t s_FgAppliedConfigSeq;

/* Same mirror for the charger's two configuration registers. */
static uint8_t s_ChgReqInputsConfig, s_ChgInputsReadout;
static uint8_t s_ChgReqInputsSeq, s_ChgAppliedInputsSeq;
static uint8_t s_ChgReqChargingConfig, s_ChgChargingReadout;
static uint8_t s_ChgReqChargingSeq, s_ChgAppliedChargingSeq;

// TODO - move to bsp:
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if (GPIO_Pin == CHG_INT_PIN)
  {
    charger_NotifyFromISR();
  }
  else if (GPIO_Pin == EXT_IO2_PIN)
  {
    app_SetIoWakeupEvent();
  }
  else if (GPIO_Pin == BTN_SW1_PIN || GPIO_Pin == BTN_SW2_PIN || GPIO_Pin == BTN_SW3_PIN)
  {
    // A press edge, wake the button task out of its idle block.
    button_NotifyFromISR();
  }
}

void button_ButtonFuncCallback(ButtonFunction_T _btn_func)
{
  LOG_INFO("[APP]: Button event: func=%d", _btn_func);
  AppEvent_t event = { .type = APP_EVT_BUTTON,
                       .button.func = _btn_func };
  app_PostEvent(&event);
}

void button_ButtonRstCfgCallback(void)
{
  LOG_INFO("[APP]: Button RESET_CONFIG event");
  AppEvent_t event = { .type = APP_EVT_BUTTON_RESET_CONFIG,
                       .button.func = BUTTON_EVENT_NO_FUNC };
  app_PostEvent(&event);
}

void charger_SnapshotChangedCallback(const ChargerSnapshot_t *_p_snapshot, uint8_t _changed)
{
  if (_changed & CHG_CHANGED_BATT_PRESENT)
  {
    AppEvent_t event = { .type = APP_EVT_CHRGR_BATT_PRESENCE,
                         .batteryPresence.present = _p_snapshot->batt_present };
    app_PostEvent(&event);
  }

  if (_changed & CHG_CHANGED_STATUS)
  {
    AppEvent_t event = { .type = APP_EVT_CHRGR_STATUS,
                         .chargerStatus.status = _p_snapshot->status };
    app_PostEvent(&event);
  }
}

void fuel_gauge_Temp_Callback(int8_t _temp)
{
  AppEvent_t event = { .type = APP_EVT_FG_TEMP,
                       .fg.temperature = _temp };
  app_PostEvent(&event);
}

void fuel_gauge_Rsoc_Callback(uint16_t _rsoc)
{
  AppEvent_t event = { .type = APP_EVT_FG_RSOC,
                       .fg.rsoc = _rsoc };
  app_PostEvent(&event);
}

static void OnTimerPowerOff(TimerHandle_t _timer)
{
  (void)_timer;
  AppEvent_t event = { .type = APP_EVT_TIMER_POWER_OFF };
  app_PostEvent(&event);
}

static void OnTimerPowerUp(TimerHandle_t _timer)
{
  (void)_timer;
  AppEvent_t event = { .type = APP_EVT_TIMER_POWER_UP };
  app_PostEvent(&event);
}

/*
static void PolicyTimerCallback(TimerHandle_t _timer)
{
  (void)_timer;
  AppEvent_t event = { .type = APP_EVT_POWER_POLICY };
  app_PostEvent(&event);
}
*/

/*
static void RailOnTimerCallback(TimerHandle_t _timer)
{
  (void)_timer;
  AppEvent_t event = { .type = APP_EVT_RAIL_ON };
  app_PostEvent(&event);
}
*/

static void FanOutBatteryProfile(void)
{
  BatteryProfile_T profile;
  bool valid = battery_GetProfile(&profile);
  const BatteryProfile_T *p = valid ? &profile : NULL;
  charger_SetBatProfile(p);
  fuel_gauge_SetBattProfile(p);
  pwr_mngr_SetBatProfile(p);
}

/*
 * Persist the fuel gauge configuration and only then hand it to the driver. The read-back is what
 * decides: whatever ends up in NV is what the fuel gauge is told to run with, so a store that
 * silently went wrong cannot leave the two disagreeing.
 */
static void ApplyFuelGaugeConfig(uint8_t config, uint8_t seq)
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
static uint8_t PersistChargerConfig(uint8_t _nvAddr, uint8_t _config)
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

static void ApplyChargerInputsConfig(uint8_t config, uint8_t seq)
{
  s_ChgInputsReadout = PersistChargerConfig(NV_ADDR_CHARGER_INPUTS_CONFIG, config);
  charger_SetInputsConfig(config);
  s_ChgAppliedInputsSeq = seq;
}

static void ApplyChargerChargingConfig(uint8_t config, uint8_t seq)
{
  s_ChgChargingReadout = PersistChargerConfig(NV_ADDR_CHARGING_CONFIG, config);
  charger_SetChargingConfig(config);
  s_ChgAppliedChargingSeq = seq;
}

/*
 * The configured threshold in x10 percent, back from the register byte. 0x7F in it means "never".
 * Bit 7 says the value lives in NV: the host sets it to ask for a store and reads it back as
 * "non volatile". It says nothing about the wake policy - see CheckWakeupOnCharge().
 */
static uint16_t WakeupChargeConfig2Percent(uint8_t _config)
{
  uint8_t cfg = _config & 0x7F;
  return cfg <= 100 ? (uint16_t)(cfg) * 10 : APP_WAKEUP_ON_CHARGE_OFF;
}

static void SetWakeupOnChargeArmed(bool _state)
{
  if (s_WakeupOnChargeArmed == _state)
    return;

  LOG_DEBUG("[APP] Wakeup On Charge: %s", _state ? "ARMED" : "DISARMED");
  s_WakeupOnChargeArmed = _state;
}

/* Check only. State machine controls arming and wake-up action.
 * Arming prevents continuous wake-up while charging remains active. */
static bool IsWakeupOnChargeAllowed(ChargerStatus_t _chrgr_status, uint16_t _rsoc)
{
  if (!s_WakeupOnChargeArmed)
    return false;

  if (_chrgr_status != CHG_STATUS_CHARGING_FROM_IN && _chrgr_status != CHG_STATUS_CHARGE_DONE)
    return false;

  // FUEL_GAUGE_RSOC_UNKNOWN and "never" are the same value, so it has to be ruled out first.
  return _rsoc != FUEL_GAUGE_RSOC_UNKNOWN
      && _rsoc >= WakeupChargeConfig2Percent(s_WakeupOnChargeConfig);
}

/*
 * Restarts the host. The rail has to come back on its own afterwards, so the one shot is armed
 * here - power_manager only says how long the cycle needs.
 */
static bool HostRestart(void)
{
  uint32_t cycle_ms = pwr_mngr_HostRestart();
  if (cycle_ms == 0)
    return false;   // the rail was already down, there was nothing to cycle

  if (xTimerChangePeriod(s_RailOnTimerHandle, pdMS_TO_TICKS(cycle_ms), 0) != pdPASS)
  {
    LOG_ERROR("[APP] could not arm the power cycle timer, rail stays down");
    return false;
  }

  return true;
}

/* Arms the one shot for what the host asked for. APP task only - it talks to the timer service. */
static void ArmPowerOffTimer(uint8_t _delay_sec)
{
  if (_delay_sec == 0xFF)
  {
    xTimerStop(s_TimerPowerOffHandle, 0);
    return;
  }

  uint32_t delay_ticks = pdMS_TO_TICKS(_delay_sec * 1000u);
  if (delay_ticks == 0)
    delay_ticks = 1;   // a period of zero is not a valid

  // Changing the period is what starts a one shot with a fresh deadline.
  if (xTimerChangePeriod(s_TimerPowerOffHandle, delay_ticks, 0) != pdPASS)
    APP_ERROR(APP_ERR_RTOS_TIMER);
  else
    LOG_INFO("[APP] POWER OFF in %usec.", (unsigned)_delay_sec);
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

  // Initialize retained variables on cold start:
  if (cold_start)
  {
    s_WakeupOnChargeArmed = true;   // Force initial state log
    SetWakeupOnChargeArmed(false);
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
  uint8_t nv_res = nv_Init();
  if (nv_res == NV_OK)
    LOG_INFO("NV inited");
  else
    LOG_ERROR("NV init failed, configuration falls back to defaults");

  // Get WakeupOnCharge config:
  {
    nv_res = nv_read_U8(NV_ADDR_WAKEUPONCHARGE_CONFIG, &s_WakeupOnChargeConfig);
    if (nv_res != NV_OK)
    {
      LOG_WARNING("[APP] Set default WAKEUP ON CHARGE config");
      s_WakeupOnChargeConfig = 0x7F;
      nv_write_U8(NV_ADDR_WAKEUPONCHARGE_CONFIG, s_WakeupOnChargeConfig);
    }
    else if ((s_WakeupOnChargeConfig & 0x7F) > 100)
      s_WakeupOnChargeConfig = 0x7F;   // out of range reads as "never"
    else
      s_WakeupOnChargeConfig |= 0x80;  // came out of NV, so the host reads it back as non-volatile
  }

  // Get HostWDT config:
  {
    uint8_t valueH, valueL;
    nv_res = nv_read_U8(NV_ADDR_HOST_WDT_CONFIGL, &valueL);
    if (nv_res == NV_OK)
      nv_res = nv_read_U8(NV_ADDR_HOST_WDT_CONFIGH, &valueH);

    if (nv_res == NV_OK)
      s_WdtHostConfig = valueH << 8 | valueL;
    else
    {
      LOG_WARNING("[APP] Set default WDT HOST config");
      s_WdtHostConfig = 0;
      nv_write_U8(NV_ADDR_HOST_WDT_CONFIGL, (uint8_t)s_WdtHostConfig);
      nv_write_U8(NV_ADDR_HOST_WDT_CONFIGH, (uint8_t)(s_WdtHostConfig >> 8));
    }
  }

  // Initialize all configured peripherals:
//  MX_IWDG_Init();   // TODO - move to bsp/drivers
  pwr_mngr_Init(cold_start);
  analog_Init();
  IoControlInit();
  led_Init();
  button_Init();

  // EEPROM IC management:
  {
    HAL_GPIO_WritePin(EE_WP_PORT, EE_WP_PIN, GPIO_PIN_SET); // ee write protect
    uint8_t ee_addr = 0;   // nothing valid stored -> 0, which selects the default ee address
    uint8_t nv_res = nv_read_U8(NV_ADDR_ID_EEPROM_ADR, &ee_addr);
    if (nv_res != NV_OK)
    {
      LOG_WARNING("[APP] Set default EEPROM ADDR");
      nv_write_U8(NV_ADDR_ID_EEPROM_ADR, ee_addr);
    }
    HAL_GPIO_WritePin(EE_ADDR_SEL_PORT, EE_ADDR_SEL_PIN,
                      (ee_addr & 0x02) ? GPIO_PIN_SET : GPIO_PIN_RESET);
  }

  // I2C master init:
  i2c_master_Init();

  // I2C slave and device init:
  {
    i2c_slave_Init();
    RtcInit(cold_start);
  }

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
    nv_res = nv_read_U8(NV_ADDR_CHARGER_INPUTS_CONFIG, &chgInputs);
    if (nv_res != NV_OK)
      chgInputs = CHARGER_INPUTS_CONFIG_DEFAULT;
    nv_res = nv_read_U8(NV_ADDR_CHARGING_CONFIG, &chgCharging);
    if (nv_res != NV_OK)
      chgCharging = CHARGER_CHARGING_CONFIG_DEFAULT;

    // Whatever NV holds may predate the precedence becoming board policy.
    chgInputs = charger_SanitizeInputsConfig(chgInputs);

    s_ChgInputsReadout = chgInputs;
    s_ChgChargingReadout = chgCharging;
    charger_Init(valid ? &batt_profile : NULL, chgInputs, chgCharging);
  }

  return cold_start;
}

static void ProcessEvent(const AppEvent_t *_evt)
{
  switch (_evt->type)
  {
    // Answered by the FSM alone - listed so they do not read as unhandled below.
    /*
    case APP_EVT_TIMER_POWER_OFF:
    case APP_EVT_RAIL_ON:
    case APP_EVT_TIMER_POWER_UP:
    case APP_EVT_POWER_PROTECTION:
    case APP_EVT_BUTTON:
      break;
  */

    case APP_EVT_BUTTON_RESET_CONFIG:
    {
      // Reset to default
      nv_Erase();
      bsp_ResetCPU();
    }
    break;

    case APP_EVT_FG_TEMP:
      break;   // TODO - nothing consumes the battery temperature yet

    case APP_EVT_FG_RSOC:
      break;   // the wake policy owns it, see state_Off()

    case APP_EVT_CHRGR_BATT_PRESENCE:
      LOG_WARNING("[APP] Batter present status=%u", (unsigned)_evt->batteryPresence.present);

      // Forward battery present status:
      fuel_gauge_SetBatteryPresent(_evt->batteryPresence.present);
      battery_UpdatePresence();
    break;

    case APP_EVT_CHRGR_STATUS:
      LOG_WARNING("[APP] Charger status=%u", (unsigned)_evt->chargerStatus.status);

      // Check incorrect charging status:
      if (_evt->chargerStatus.status == CHG_STATUS_CHARGING_FROM_USB)
        LOG_CRITICAL("[APP] Rcvd charger status: CHARGING_FROM_USB");
    break;

    case APP_EVT_CMD_BATT_SET_PROFILE:
      battery_ApplySetProfile(_evt->batterySetProfile.id, _evt->batterySetProfile.seq);
      FanOutBatteryProfile();
      break;

    case APP_EVT_CMD_BATT_WRITE_CUSTOM_PROFILE:
      battery_ApplyWriteCustomProfile(&_evt->batteryCustomProfile.profile, _evt->batteryCustomProfile.seq);
      FanOutBatteryProfile();
      break;

    case APP_EVT_CMD_BATT_WRITE_CUSTOM_EXTENDED_PROFILE:
      battery_ApplyWriteCustomExtendedProfile(&_evt->batteryCustomExtProfile.profile);
      FanOutBatteryProfile();
      break;

    case APP_EVT_CMD_FUEL_GAUGE_SET_CONFIG:
      ApplyFuelGaugeConfig(_evt->fuelGaugeConfig.config, _evt->fuelGaugeConfig.seq);
      break;

    case APP_EVT_CHARGER_SET_INPUTS_CONFIG:
      ApplyChargerInputsConfig(_evt->chargerConfig.config, _evt->chargerConfig.seq);
      break;

    case APP_EVT_CHARGER_SET_CHARGING_CONFIG:
      ApplyChargerChargingConfig(_evt->chargerConfig.config, _evt->chargerConfig.seq);
      break;

    default:
      LOG_DEBUG("[APP] Unhandled event: type=%u", _evt->type);
    break;
  }
}

static AppState_t state_Off(const AppEvent_t *_evt)
{
  AppState_t next = SM_APP_OFF;
  switch (_evt->type)
  {
    case APP_EVT_SM_ENTRY:
      LOG_INFO("[APP] state OFF");
      pwr_mngr_HostOff();
    break;

    case APP_EVT_BUTTON:
      if (_evt->button.func == BUTTON_EVENT_FUNC_POWER_ON)
      {
        LOG_INFO("[APP] POWER UP triggered by button");
        next = SM_APP_POWER_UP;
      }
    break;

    case APP_EVT_CHRGR_STATUS:
      // The source is gone, so the next one that appears may raise the host:
      if (_evt->chargerStatus.status == CHG_STATUS_NO_VALID_SOURCE)
        SetWakeupOnChargeArmed(true);
      else if (IsWakeupOnChargeAllowed(_evt->chargerStatus.status, fuel_gauge_GetRsoc()))
      {
        LOG_INFO("[APP] POWER UP triggered by charge");
        next = SM_APP_POWER_UP;
      }
    break;

    case APP_EVT_FG_RSOC:
      // The charge may have grown past the threshold with the charger already in:
      if (IsWakeupOnChargeAllowed(charger_GetStatus(), _evt->fg.rsoc))
      {
        LOG_INFO("[APP] POWER UP triggered by charge");
        next = SM_APP_POWER_UP;
      }
    break;

    default:
    break;
  }

  return next;
}

static AppState_t state_PowerUp(const AppEvent_t *_evt)
{
  BaseType_t res;
  switch (_evt->type)
  {
    case APP_EVT_SM_ENTRY:
    {
      LOG_INFO("[APP] state POWER_UP");

      // Wake-up handled, clear the arm:
      SetWakeupOnChargeArmed(false);

      // Enable 5V boost:
      pwr_mngr_HostOn();

      // Start PowerUp timer:
      res = xTimerChangePeriod(s_TimerPowerUpHandle, pdMS_TO_TICKS(APP_POWER_UP_TIME_MS), 0);
      if (res != pdPASS)
        APP_ERROR(APP_ERR_RTOS_TIMER);
    }
    break;

    case APP_EVT_TIMER_POWER_UP:
    {
      return SM_APP_ON;
    }
    break;


    default:
    break;   // the host has not even booted yet, nothing it asks for applies
  }

  return SM_APP_POWER_UP;
}

static AppState_t state_On(const AppEvent_t *_evt)
{
  AppState_t next = SM_APP_ON;
  switch (_evt->type)
  {
    case APP_EVT_CHRGR_STATUS:
      // While the host is up, arm follows the source:
      SetWakeupOnChargeArmed(_evt->chargerStatus.status == CHG_STATUS_NO_VALID_SOURCE);
    break;

    case APP_EVT_SM_ENTRY:
      LOG_INFO("[APP] state ON");
      // TODO - enable 5V power protection
//      pwr_mngr_Arm5vCheck(true);
//      OnWakeAccepted();   // whatever asked for this is satisfied now
    break;

    case APP_EVT_POWER_PROTECTION:
      // TODO
//      LOG_ERROR("[APP] protection cut, trip=%u", (unsigned)_ev->powerTrip.trip);
//      pwr_mngr_SetStatusFlags(PWR_STATUS_FORCED_POWER_OFF);
//      next = SM_APP_LOW_BATT;
    break;


    case APP_EVT_BUTTON:
      if (_evt->button.func == BUTTON_EVENT_FUNC_POWER_OFF)
      {
        LOG_WARNING("[APP] State ON rcvd button FUNC_POWER_OFF");
        pwr_mngr_SetStatusFlags(PWR_STATUS_POWER_OFF_BTN);
        next = SM_APP_OFF;
      }
      else if (_evt->button.func == BUTTON_EVENT_FUNC_POWER_RESET)
      {
        LOG_WARNING("[APP] State ON rcvd button FUNC_POWER_RESET");
//        if (HostRestart())
//          next = SM_APP_OFF;
      }
      else if (_evt->button.func == BUTTON_EVENT_FUNC_POWER_ON)
      {
        LOG_WARNING("[APP] State ON rcvd button FUNC_POWER_ON");
//         HostRestart();
//         next = SM_APP_OFF;
      }
    break;

    case APP_EVT_CMD_SCHEDULE_POWER_OFF:
      ArmPowerOffTimer(_evt->powerOff.delay_sec);
    break;

    case APP_EVT_TIMER_POWER_OFF:
      s_PowerOffPending = false;
      LOG_INFO("[APP] POWER OFF timer expired. Switch off 5V...");
      next = SM_APP_OFF;
    break;

    default:
    break;
  }

  return next;
}

static AppState_t state_LowBatt(const AppEvent_t *_evt)
{
  switch (_evt->type)
  {
    case APP_EVT_SM_ENTRY:
      LOG_INFO("[APP] state LOW_BATT");
//      CancelScheduledPowerOff();
//      pwr_mngr_Arm5vCheck(false);
//      pwr_mngr_RailOff();
//      s_WakeupOnCharge = APP_WAKEUP_ON_CHARGE_MIN;   // come back once there is any energy
    break;

    /*
    case APP_EVT_POWER_POLICY:
      // The only way out: enough charge to be worth trying again.
//      if (IsWakeDue())
//        return SM_APP_POWER_UP;
    break;

*/

    default:
    break;
  }

  return SM_APP_LOW_BATT;
}

// The only place that maps a state to its handler.
static AppState_t fsm_Handle(AppState_t _state, const AppEvent_t *_evt)
{
  switch (_state)
  {
    case SM_APP_OFF:       return state_Off(_evt);
    case SM_APP_POWER_UP:  return state_PowerUp(_evt);
    case SM_APP_ON:        return state_On(_evt);
    case SM_APP_LOW_BATT:  return state_LowBatt(_evt);
    default:
      APP_ERROR(APP_ERR);
      return _state;
  }
}

// The only place that assigns the state. Handlers decide, this moves.
// An entry action may itself ask to move on, hence the loop; the guard catches a chain that
// does not settle.
static void fsm_Dispatch(const AppEvent_t *_evt)
{
  AppState_t next = fsm_Handle(s_State, _evt);

  uint8_t guard = SM_APP_COUNT;
  while (next != s_State)
  {
    if (guard-- == 0)
    {
      LOG_ERROR("[APP] entry chain does not settle at state %u, event %u",
          (unsigned)next, (unsigned)_evt->type);
      APP_ERROR(APP_ERR);
    }

    s_State = next;
    static const AppEvent_t entryEv = { .type = APP_EVT_SM_ENTRY };
    next = fsm_Handle(s_State, &entryEv);
  }
}

static void Task(void *parameters)
{
  (void)parameters;

  // Clock configuration after SysTick initialization, since in this project
  // the HAL tick and delay functions depend on the FreeRTOS system tick:
  bsp_ClockConfig();

  LOG_INFO("APP task started");
  Init();

  // Restore correct FSM state:
  s_State = pwr_mngr_IsHostPowered() ? SM_APP_ON : SM_APP_OFF;

  // Entry event to start FSM:
  const AppEvent_t entry_evt = { .type = APP_EVT_SM_ENTRY };
  fsm_Dispatch(&entry_evt);

//  if (xTimerStart(s_PolicyTimerHandle, 0) != pdPASS)
//    LOG_ERROR("[APP] power policy timer start failed, wake triggers will not run");

  while(1)
  {
    AppEvent_t evt;
    if (xQueueReceive(s_EvtQueHandle, &evt, portMAX_DELAY) != pdPASS)
      continue;

    ProcessEvent(&evt);
    fsm_Dispatch(&evt);
  }
}

void app_Init(void)
{
  // Create the event queue first: producers are started by TaskApp and post into it.
  s_EvtQueHandle = xQueueCreateStatic(sizeof(s_EvtQueBuf)/sizeof(s_EvtQueBuf[0]),
                           sizeof(s_EvtQueBuf[0]), (uint8_t*)s_EvtQueBuf, &s_EvtQue);
  ASSERT(s_EvtQueHandle != NULL);

  s_TimerPowerOffHandle = xTimerCreateStatic("PWR_OFF", 1, pdFALSE,
                           NULL, OnTimerPowerOff, &s_TimerPowerOff);
  ASSERT(s_TimerPowerOffHandle != NULL);

  s_TimerPowerUpHandle = xTimerCreateStatic("PWR_UP", 1, pdFALSE,
                           NULL, OnTimerPowerUp, &s_TimerPowerUp);
  ASSERT(s_TimerPowerUpHandle != NULL);

  // TODO
  /*
  s_PolicyTimerHandle = xTimerCreateStatic("PWR_POL", pdMS_TO_TICKS(APP_POWER_POLICY_PERIOD_MS),
                           pdTRUE, NULL, PolicyTimerCallback, &s_PolicyTimer);
  ASSERT(s_PolicyTimerHandle != NULL);
  */

  /*
  s_RailOnTimerHandle = xTimerCreateStatic("RAIL_ON", 1, pdFALSE,
                           NULL, RailOnTimerCallback, &s_RailOnTimer);
  ASSERT(s_RailOnTimerHandle != NULL);
  */

  s_TaskHandleApp = xTaskCreateStatic(Task, "APP", sizeof(TaskStackApp)/sizeof(StackType_t),
                           NULL, 7,
                           TaskStackApp, &TaskTCBApp);
  ASSERT(s_TaskHandleApp != NULL);
}

// TODO - set static and move
void app_PostEvent(const AppEvent_t *_p_event)
{
  if (s_EvtQueHandle == NULL)
    return;

  BaseType_t sent;
  BaseType_t woken = pdFALSE;
  if (xPortIsInsideInterrupt() != pdFALSE)
    sent = xQueueSendFromISR(s_EvtQueHandle, _p_event, &woken);
  else
    sent = xQueueSend(s_EvtQueHandle, _p_event, 0);

  if (!sent)
  {
    LOG_CRITICAL("[APP] APP queue full. Event %u dropped", _p_event->type);
    APP_ERROR(APP_ERR_RTOS_QUEUE);
  }

  // Call this at the end of a potential interrupt, because on some FreeRTOS
  // ports, this function may trigger an immediate context switch:
  portYIELD_FROM_ISR(woken);
}

void app_SetRtcWakeupEvent(void)
{
  // TODO - not implemented now
}

void app_SetIoWakeupEvent(void)
{
  // TODO - not implemented now
}

void app_OnCmdSetFuelGaugeConfig(uint8_t *_data, uint16_t _len)
{
  LOG_WARNING("[APP] Rcvd CMD SetFuelGaugeConfig: data[0]=%u, len=%u",
      (unsigned)_data[0], (unsigned)_len);
  (void)_len;

  if (!fuel_gauge_IsConfigValid(_data[0]))
    return;

  uint8_t seq = (uint8_t)(s_FgReqConfigSeq + 1);
  AppEvent_t evt = { .type = APP_EVT_CMD_FUEL_GAUGE_SET_CONFIG };
  evt.fuelGaugeConfig.config = _data[0];
  evt.fuelGaugeConfig.seq = seq;

  s_FgReqConfig = _data[0];
  s_FgReqConfigSeq = seq;
  app_PostEvent(&evt);
}

void app_OnCmdGetFuelGaugeConfig(uint8_t _data[], uint16_t *_p_len)
{
  LOG_DEBUG("[APP] Rcvd CMD GetFuelGaugeConfig");
  _data[0] = fuel_gauge_GetConfig();
  *_p_len = 1;
}

void app_OnCmdSchedulePowerOff(uint8_t _delay_code)
{
  LOG_WARNING("[APP] Rcvd CMD OnCmdSchedulePowerOff: delay=%usec.",
      (unsigned)_delay_code);
  if (_delay_code > 250 && _delay_code != 0xFF)
    return;   // 251 - 254 are reserved

  s_PowerOffPending = (_delay_code != 0xFF);

  /* The timer is armed by the APP task, not here: this runs in the I2C interrupt. */
  AppEvent_t event = { .type = APP_EVT_CMD_SCHEDULE_POWER_OFF };
  event.powerOff.delay_sec = _delay_code;
  app_PostEvent(&event);
}

uint8_t app_OnCmdGetPowerOffCounter(void)
{
  LOG_DEBUG("[APP] Rcvd CMD OnCmdGetPowerOffCounter");
  if (!s_PowerOffPending)
    return 0xFF;

  /* Whole seconds left, straight off the timer. Anything out of range means the deadline has
   * passed, or the APP task has not armed it yet - both answer "any moment now". */
  TickType_t left = xTimerGetExpiryTime(s_TimerPowerOffHandle) - xTaskGetTickCount();
  return (left <= pdMS_TO_TICKS(250000)) ? (uint8_t)(left / configTICK_RATE_HZ) : 0;
}

void app_OnCmdSetHostWDTConfig(uint8_t _data[], uint16_t _len)
{
  LOG_WARNING("[APP] Rcvd CMD SetHostWDTConfig: data[0]=%u, data[1]=%u, len=%u",
      (unsigned)_data[0], (unsigned)_data[1], (unsigned)_len);
  // TODO
  /*
  if (_len < 2)
    return;

  uint16_t cfg = ((uint16_t)_data[1] << 8) | _data[0];
  uint16_t d = cfg & 0x3FFF;
  d <<= ((cfg & 0x4000) >> 13);   // 4 minute resolution over the 16384-65536 range

  if (!(_data[1] & 0x80))
  {
    s_WdtHostPeriodMs = d * (uint32_t)60000;
    s_WdtHostTimer = MS_TIME_COUNT(lastHostCommandTimer) + s_WdtHostPeriodMs;
    return;
  }

  s_WdtHostConfig = d;
  nv_write_U8(NV_ADDR_WATCHDOG_CONFIGL, s_WdtHostConfig);
  nv_write_U8(NV_ADDR_WATCHDOG_CONFIGH, s_WdtHostConfig >> 8);

  if (nv_read_U8(NV_ADDR_WATCHDOG_CONFIGL, (uint8_t*)&s_WdtHostConfig) != NV_OK
   || nv_read_U8(NV_ADDR_WATCHDOG_CONFIGH, (uint8_t*)&s_WdtHostConfig + 1) != NV_OK)
  {
    s_WdtHostConfig = 0;
  }

  if (s_WdtHostConfig == 0)
  {
    s_WdtHostPeriodMs = 0;
    s_WdtHostTimer = 0;
  }*/
}

void app_OnCmdGetHostWDTConfig(uint8_t _data[], uint16_t *_p_len)
{
  LOG_DEBUG("[APP] Rcvd CMD GetHostWDTConfig");

  uint16_t d = s_WdtHostConfig ? s_WdtHostConfig : (uint16_t)(s_WdtHostPeriodMs / 60000);
  if (d >= 0x4000)
    d = (d >> 2) | 0x4000;

  _data[0] = d;
  _data[1] = (d >> 8) | (s_WdtHostConfig ? 0x80 : 0x00);
  *_p_len = 2;
}

void app_OnCmdSetWakeupOnCharge(uint8_t _data[], uint16_t _len)
{
  LOG_WARNING("[APP] Rcvd CMD SetWakeupOnCharge: data[0]=%u, len=%u",
      (unsigned)_data[0], (unsigned)_len);
  (void)_len;

  bool store = _data[0] & 0x80;   // MSB is used for "store to NV" status
  s_WakeupOnChargeConfig = (_data[0] & 0x7F) <= 100 ? _data[0] : 0x7F;
  if (store)
    nv_write_U8(NV_ADDR_WAKEUPONCHARGE_CONFIG, s_WakeupOnChargeConfig);
}

void app_OnCmdGetWakeupOnCharge(uint8_t _data[], uint16_t *_p_len)
{
  LOG_DEBUG("[APP] Rcvd CMD GetWakeupOnCharge");
  _data[0] = s_WakeupOnChargeConfig;
  *_p_len = 1;
}

void app_OnCmdSetChargerInputsConfig(uint8_t _config)
{
  LOG_WARNING("[APP] Rcvd CMD SetChargerInputsConfig: config=%u",
      (unsigned)_config);
  /* Stripped here, where the host byte enters, so that NV, the read-back and the charger all carry
   * the same value. The host's verifying write of a refused bit will fail, and that is the point. */
  _config = charger_SanitizeInputsConfig(_config);

  uint8_t seq = (uint8_t)(s_ChgReqInputsSeq + 1);
  AppEvent_t evt = { .type = APP_EVT_CHARGER_SET_INPUTS_CONFIG };
  evt.chargerConfig.config = _config;
  evt.chargerConfig.seq = seq;

  s_ChgReqInputsConfig = _config;
  s_ChgReqInputsSeq = seq;
  app_PostEvent(&evt);
}

uint8_t app_OnCmdGetChargerInputsConfig(void)
{
  LOG_DEBUG("[APP] Rcvd CMD GetChargerInputsConfig");
  return (s_ChgReqInputsSeq != s_ChgAppliedInputsSeq) ? s_ChgReqInputsConfig : s_ChgInputsReadout;
}

void app_OnCmdSetChargingConfig(uint8_t _config)
{
  LOG_WARNING("[APP] Rcvd CMD CmdSetChargingConfig: config=%u",
        (unsigned)_config);
  uint8_t seq = (uint8_t)(s_ChgReqChargingSeq + 1);
  AppEvent_t evt = { .type = APP_EVT_CHARGER_SET_CHARGING_CONFIG };
  evt.chargerConfig.config = _config;
  evt.chargerConfig.seq = seq;

  s_ChgReqChargingConfig = _config;
  s_ChgReqChargingSeq = seq;
  app_PostEvent(&evt);
}

uint8_t app_OnCmdGetChargingConfig(void)
{
  LOG_DEBUG("[APP] Rcvd CMD GetChargingConfig");
  return (s_ChgReqChargingSeq != s_ChgAppliedChargingSeq)
      ? s_ChgReqChargingConfig : s_ChgChargingReadout;
}

