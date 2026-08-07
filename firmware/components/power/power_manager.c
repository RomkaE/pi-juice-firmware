/*
 * power_manager.c
 *
 *  Created on: 2026
 *      Author: Roman Egoshin
 */

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#include "power_manager.h"
#include "charger_bq2416x.h"
#include "fuel_gauge_lc709203f.h"
#include "iosystem/analog.h"
#include "led/led.h"
#include "to_refactor/io_control.h"
#include "nv.h"
#include "board.h"
#include "utils/time_count.h"
#include "src/app.h"

// LOG:
#include "log/log.h"

#define PWR_TICK_MS               100
#define PWR_POWER_CYCLE_MS        100   // rail down time on a host reset
#define PWR_POLICY_PERIOD_MS      500   // wake triggers and watchdog

// The 5V check is meaningless while the rail is still coming up.
#define PWR_RAIL_SETTLE_MS        40

// Debounce in ADC datasets, not ms: the values refresh once per half ring (~64 ms).
#define PWR_PROTECTION_SAMPLES    3
#define PWR_5V_FAULT_MV           4500
#define PWR_5V_FAULT_AVDD_MV      3000

#define PWR_HOST_QUIET_MS         15000
#define PWR_TURNON_HOST_QUIET_MS  11000
#define PWR_TURNON_WAKE_QUIET_MS  12000
#define PWR_WAKE_REARM_MS         30000

#define PWR_VBAT_CUTOFF_DEFAULT_MV  3000
#define PWR_WAKEUP_ON_CHARGE_OFF    0xFFFF
#define PWR_WAKEUP_ON_CHARGE_MIN    5      // RSOC 0.5%, armed when the protection trips

// Retained: the host reads these back as event flags after a brown out.
static uint8_t s_FaultFlags __attribute__((section("no_init")));

static uint32_t s_BoostOnMs;
static uint32_t s_RestartAt __attribute__((section("no_init")));   // 0 = no pending restart
static uint32_t s_PolicyMs;
static uint32_t s_LastWakeupMs __attribute__((section("no_init")));

// Raw counts on purpose: VBAT_MV_TO_ADC assumes the nominal reference, which is what the cutoff
// path needs exactly when the supply is sagging. See analog.h.
static uint16_t s_VbatCutoffMv;
static uint16_t s_VbatCutoffAdc;

// Owned by the ANALOG task.
static uint8_t s_TripSamples;
static bool s_TripPosted;

static RunPinInstallationStatus_t s_RunPin = RUN_PIN_NOT_INSTALLED;
static uint8_t s_WakeupOnChargeConfig __attribute__((section("no_init")));
static uint16_t s_WakeupOnCharge __attribute__((section("no_init")));
static uint32_t s_DelayedPowerOffAt __attribute__((section("no_init")));
static uint16_t s_WatchdogConfig __attribute__((section("no_init")));
static uint32_t s_WatchdogPeriodMs __attribute__((section("no_init")));
static uint32_t s_WatchdogTimer __attribute__((section("no_init")));
static uint8_t s_RtcWakeupEvent __attribute__((section("no_init")));
static uint8_t s_IoWakeupEvent;

extern uint32_t lastHostCommandTimer;   // app.c, stamped on every host command

// No soft start sequencing here - the converter does its own, with current limiting.
static void RailSet(bool _on)
{
  bsp_Pwr5V_SetState(_on);

  if (_on)
  {
    s_BoostOnMs = HAL_GetTick();
    s_TripSamples = 0;
    s_TripPosted = false;
  }
}

static bool HasEnergy(void)
{
  return analog_GetVBattAvg() > s_VbatCutoffMv || charger_IsInputPresent();
}

static bool RailTurnOn(void)
{
  if (bsp_Pwr5V_GetState())
    return true;

  if (!HasEnergy())
  {
    LOG_WARNING("[PWR] turn on refused, vbat=%u cutoff=%u mV",
        (unsigned)analog_GetVBattAvg(), (unsigned)s_VbatCutoffMv);
    return false;
  }

  RailSet(true);   // bsp logs the transition
  return true;
}

static void RailTurnOff(void)
{
  RailSet(false);
  s_RestartAt = 0;
}

/*
 * Runs in the ANALOG task, once per fresh dataset - see the contract in analog.h. Reads and hands
 * over; the rail is cut by APP so an in-flight flash write can finish. Shedding the load goes
 * through the modules that own those pins, not through their registers.
 */
void analog_OnEvent_SamplesReady(void)
{
  if (!bsp_Pwr5V_GetState())
  {
    s_TripSamples = 0;
    return;
  }

  uint8_t trip = PWR_TRIP_NONE;

  // The 5V reading only means something once the rail has had time to come up.
  if (MS_TIME_COUNT(s_BoostOnMs) >= PWR_RAIL_SETTLE_MS
      && analog_Get5vPi() < PWR_5V_FAULT_MV && analog_GetAvdd() > PWR_5V_FAULT_AVDD_MV)
    trip = PWR_TRIP_5V_FAULT;
  else if (analog_GetRawBatt() < s_VbatCutoffAdc && !charger_IsInputPresent())
    trip = PWR_TRIP_VBAT_CUTOFF;

  if (trip == PWR_TRIP_NONE)
  {
    s_TripSamples = 0;
    return;
  }

  if (s_TripSamples < PWR_PROTECTION_SAMPLES)
    s_TripSamples++;

  if (s_TripSamples < PWR_PROTECTION_SAMPLES || s_TripPosted)
    return;

  s_TripPosted = true;   // one event per trip

  LOG_WARNING("[PWR] protection trip=%u v5=%u vbat_raw=%u", (unsigned)trip,
      (unsigned)analog_Get5vPi(), (unsigned)analog_GetRawBatt());

  led_Stop();
  IoControlShutdown();

  // TODO
  /*
  AppEvent_t evt = { .type = APP_EVT_POWER_PROTECTION };
  evt.data.powerTrip.trip = trip;
  app_PostEvent(&evt);
  */
}

static void CancelPendingWakeups(void)
{
  s_WakeupOnCharge = PWR_WAKEUP_ON_CHARGE_OFF;
  s_RtcWakeupEvent = 0;
  s_IoWakeupEvent = 0;
  s_DelayedPowerOffAt = 0;
}

/*
 * Two answers, not three: the old third branch woke the Pi by pulsing SCL and was reachable only
 * when an external 5V source held the rail. The board always owns the rail now.
 */
static bool HostRestart(void)
{
  s_LastWakeupMs = HAL_GetTick();

  if (bsp_Pwr5V_GetState() && s_RunPin == RUN_PIN_INSTALLED)
  {
    /* RUN instead of cycling the rail - the pin is not wired on this board, see HOST_RUN_PIN.
    HAL_GPIO_WritePin(HOST_RUN_PORT, HOST_RUN_PIN, GPIO_PIN_RESET);
    DelayUs(100);
    HAL_GPIO_WritePin(HOST_RUN_PORT, HOST_RUN_PIN, GPIO_PIN_SET);
    */
    return true;
  }

  if (bsp_Pwr5V_GetState())
  {
    RailSet(false);
    s_RestartAt = HAL_GetTick() + PWR_POWER_CYCLE_MS;
    if (s_RestartAt == 0)
      s_RestartAt++;   // 0 means "nothing pending"
    LOG_INFO("[PWR] power cycle, back up in %u ms", (unsigned)PWR_POWER_CYCLE_MS);
    return true;
  }

  return RailTurnOn();
}

static void OnWakeAccepted(void)
{
  CancelPendingWakeups();

  if (s_WatchdogConfig)   // config carries a restore flag: re-arm after a wake
  {
    s_WatchdogPeriodMs = s_WatchdogConfig * (uint32_t)60000;
    s_WatchdogTimer = MS_TIME_COUNT(lastHostCommandTimer) + s_WatchdogPeriodMs;
  }
}

// Wake triggers and the watchdog, at PWR_POLICY_PERIOD_MS.
static void PolicyTick(void)
{
  if (s_DelayedPowerOffAt && s_DelayedPowerOffAt <= HAL_GetTick())
  {
    s_DelayedPowerOffAt = 0;
    RailTurnOff();
  }

  if (MS_TIME_COUNT(s_PolicyMs) < PWR_POLICY_PERIOD_MS)
    return;

  s_PolicyMs = HAL_GetTick();

  // An unknown charge level must not pass the threshold and wake the host on a dead fuel gauge.
  uint16_t rsoc = fuel_gauge_GetRsoc();
  bool wakeOnCharge = rsoc != FUEL_GAUGE_RSOC_UNKNOWN && rsoc >= s_WakeupOnCharge
      && charger_IsInputPresent() && charger_IsBatteryPresent();

  bool wake = (wakeOnCharge || s_RtcWakeupEvent || s_IoWakeupEvent)
      && !s_DelayedPowerOffAt
      && MS_TIME_COUNT(lastHostCommandTimer) > PWR_HOST_QUIET_MS
      && MS_TIME_COUNT(s_LastWakeupMs) > PWR_WAKE_REARM_MS;

  if (s_WatchdogPeriodMs && MS_TIME_COUNT(lastHostCommandTimer) > s_WatchdogTimer)
  {
    LOG_WARNING("[PWR] host watchdog expired");
    s_FaultFlags |= PWR_FAULT_WATCHDOG_EXPIRED;
    s_WatchdogTimer += s_WatchdogPeriodMs;
    wake = true;
  }

  if (wake && !s_RestartAt && HostRestart())
    OnWakeAccepted();

  // Re-arm the charge wake once the source is gone, so plugging back in wakes the host.
  if (s_WakeupOnCharge == PWR_WAKEUP_ON_CHARGE_OFF && !charger_IsInputPresent()
      && (s_WakeupOnChargeConfig & 0x80))
  {
    s_WakeupOnCharge = (s_WakeupOnChargeConfig & 0x7F) <= 100
        ? (uint16_t)(s_WakeupOnChargeConfig & 0x7F) * 10 : PWR_WAKEUP_ON_CHARGE_OFF;
  }
}

void pwr_mngr_Init(bool _cold_start)
{
  if (_cold_start)
  {
    s_FaultFlags = 0;
    s_WakeupOnCharge = PWR_WAKEUP_ON_CHARGE_OFF;
    s_WakeupOnChargeConfig = 0x7F;
    s_DelayedPowerOffAt = 0;
    s_WatchdogPeriodMs = 0;
    s_WatchdogTimer = 0;
    s_RtcWakeupEvent = 0;
    s_IoWakeupEvent = 0;
    s_RestartAt = 0;
    s_LastWakeupMs = HAL_GetTick();

    if (nv_read_U8(NV_ADDR_WAKEUPONCHARGE_CONFIG, &s_WakeupOnChargeConfig) == NV_OK)
    {
      if (s_WakeupOnChargeConfig <= 100)
        s_WakeupOnChargeConfig |= 0x80;
    }

    if (s_WakeupOnChargeConfig & 0x80)
    {
      s_WakeupOnCharge = (s_WakeupOnChargeConfig & 0x7F) <= 100
          ? (uint16_t)(s_WakeupOnChargeConfig & 0x7F) * 10 : PWR_WAKEUP_ON_CHARGE_OFF;
    }

    if (nv_read_U8(NV_ADDR_WATCHDOG_CONFIGL, (uint8_t*)&s_WatchdogConfig) != NV_OK
     || nv_read_U8(NV_ADDR_WATCHDOG_CONFIGH, (uint8_t*)&s_WatchdogConfig + 1) != NV_OK)
    {
      s_WatchdogConfig = 0;
    }
  }

  uint8_t var8 = 0;
  if (nv_read_U8(NV_ADDR_RUN_PIN_CONFIG, &var8) == NV_OK && var8 <= RUN_PIN_INSTALLED)
    s_RunPin = var8;

  {
    BatteryProfile_T profile;
    bool have = battery_GetProfile(&profile);
    s_VbatCutoffMv = have ? (uint16_t)profile.cutoffVoltage * 20 : PWR_VBAT_CUTOFF_DEFAULT_MV;
  }
  s_VbatCutoffAdc = VBAT_MV_TO_ADC(s_VbatCutoffMv);

  s_PolicyMs = HAL_GetTick();
  s_TripSamples = 0;
  s_TripPosted = false;

  // The rail is not touched here: bsp_Pwr5V_Restore() already set it in main(), and forcing it
  // down would cut a host that survived the reset. Stamp the settle window instead.
  s_BoostOnMs = HAL_GetTick();

  LOG_INFO("[PWR] init: cutoff=%u mV (%u counts), run_pin=%u, rail=%u",
      (unsigned)s_VbatCutoffMv, (unsigned)s_VbatCutoffAdc, (unsigned)s_RunPin,
      (unsigned)bsp_Pwr5V_GetState());
}

uint32_t pwr_mngr_Tick(void)
{
  if (s_RestartAt && (int32_t)(HAL_GetTick() - s_RestartAt) >= 0)
  {
    s_RestartAt = 0;
    RailTurnOn();
  }

  PolicyTick();

  if (s_RestartAt)
  {
    int32_t remain = (int32_t)(s_RestartAt - HAL_GetTick());
    if (remain > 0 && remain < PWR_TICK_MS)
      return (uint32_t)remain;
  }

  return PWR_TICK_MS;
}

bool pwr_mngr_HostTurnOn(void)
{
  if (bsp_Pwr5V_GetState())
  {
    // Only worth doing when the host has been quiet long enough to be probably hung.
    if (MS_TIME_COUNT(s_LastWakeupMs) <= PWR_TURNON_WAKE_QUIET_MS
        || MS_TIME_COUNT(lastHostCommandTimer) <= PWR_TURNON_HOST_QUIET_MS)
      return false;
  }

  if (!HostRestart())
    return false;

  CancelPendingWakeups();
  return true;
}

bool pwr_mngr_HostTurnOff(void)
{
  if (!bsp_Pwr5V_GetState())
    return false;

  RailTurnOff();
  s_FaultFlags |= PWR_FAULT_POWER_OFF_BTN;
  return true;
}

bool pwr_mngr_HostReset(void)
{
  if (!HostRestart())
    return false;

  CancelPendingWakeups();
  return true;
}

void pwr_mngr_OnEvent_Protection(uint8_t _trip)
{
  LOG_ERROR("[PWR] protection cut, trip=%u", (unsigned)_trip);

  RailTurnOff();
  s_FaultFlags |= PWR_FAULT_FORCED_POWER_OFF;
  s_WakeupOnCharge = PWR_WAKEUP_ON_CHARGE_MIN;   // come back once there is any energy
}

void pwr_mngr_HostPollEvent(void)
{
  s_RtcWakeupEvent = 0;
  s_IoWakeupEvent = 0;
  s_WatchdogTimer = s_WatchdogPeriodMs;
}

void pwr_mngr_SetRtcWakeupEvent(void)
{
  s_RtcWakeupEvent = 1;
}

void pwr_mngr_SetIoWakeupEvent(void)
{
  s_IoWakeupEvent = 1;
}

void pwr_mngr_SetBatProfile(const BatteryProfile_T *_p_profile)
{
  s_VbatCutoffMv = _p_profile != NULL
      ? (uint16_t)_p_profile->cutoffVoltage * 20 : PWR_VBAT_CUTOFF_DEFAULT_MV;
  s_VbatCutoffAdc = VBAT_MV_TO_ADC(s_VbatCutoffMv);
}

// Derived, not stored: the charger already publishes everything this needs, ISR safe.
PowerSourceStatus_t pwr_mngr_GetInStatus(void)
{
  uint8_t inStat = charger_GetInStat();

  if (inStat == 0x03)
    return PWR_SOURCE_NOT_PRESENT;
  if (inStat == 0x01 || inStat == 0x02)
    return PWR_SOURCE_BAD;

  // TODO
  /*
  if (charger_IsDpmModeActive())
    return PWR_SOURCE_WEAK;
    */

  return PWR_SOURCE_NORMAL;
}

// The protocol keeps the field, but there is nothing left to detect - see the header.
PowerSourceStatus_t pwr_mngr_Get5vIoStatus(void)
{
  return PWR_SOURCE_NOT_PRESENT;
}

bool pwr_mngr_IsHostPowered(void)
{
  return bsp_Pwr5V_GetState();
}

uint8_t pwr_mngr_GetFaultFlags(void)
{
  return s_FaultFlags;
}

void pwr_mngr_KeepFaultFlags(uint8_t _mask)
{
  s_FaultFlags &= _mask;
}

void pwr_mngr_CmdSchedulePowerOff(uint8_t _delayCode)
{
  if (_delayCode <= 250)
  {
    s_DelayedPowerOffAt = HAL_GetTick() + _delayCode * 1024;
    if (s_DelayedPowerOffAt == 0)
      s_DelayedPowerOffAt++;   // 0 means "not active"
  }
  else if (_delayCode == 0xFF)
  {
    s_DelayedPowerOffAt = 0;
  }
}

uint8_t pwr_mngr_CmdGetPowerOffCounter(void)
{
  if (!s_DelayedPowerOffAt)
    return 0xFF;

  if (s_DelayedPowerOffAt > HAL_GetTick())
    return (s_DelayedPowerOffAt - HAL_GetTick()) >> 10;

  return 0;
}

void pwr_mngr_CmdSetRunPinConfig(uint8_t data[], uint8_t len)
{
  (void)len;
  if (data[0] > RUN_PIN_INSTALLED)
    return;

  nv_write_U8(NV_ADDR_RUN_PIN_CONFIG, data[0]);

  uint8_t var8 = 0;
  if (nv_read_U8(NV_ADDR_RUN_PIN_CONFIG, &var8) == NV_OK && var8 <= RUN_PIN_INSTALLED)
    s_RunPin = var8;
  else
    s_RunPin = RUN_PIN_NOT_INSTALLED;
}

void pwr_mngr_CmdGetRunPinConfig(uint8_t data[], uint16_t *len)
{
  data[0] = s_RunPin;
  *len = 1;
}

void pwr_mngr_CmdConfigureWatchdog(uint8_t data[], uint16_t len)
{
  if (len < 2)
    return;

  uint16_t cfg = ((uint16_t)data[1] << 8) | data[0];
  uint16_t d = cfg & 0x3FFF;
  d <<= ((cfg & 0x4000) >> 13);   // 4 minute resolution over the 16384-65536 range

  if (data[1] & 0x80)
  {
    s_WatchdogConfig = d;
    nv_write_U8(NV_ADDR_WATCHDOG_CONFIGL, s_WatchdogConfig);
    nv_write_U8(NV_ADDR_WATCHDOG_CONFIGH, s_WatchdogConfig >> 8);

    if (nv_read_U8(NV_ADDR_WATCHDOG_CONFIGL, (uint8_t*)&s_WatchdogConfig) != NV_OK
     || nv_read_U8(NV_ADDR_WATCHDOG_CONFIGH, (uint8_t*)&s_WatchdogConfig + 1) != NV_OK)
    {
      s_WatchdogConfig = 0;
    }

    if (s_WatchdogConfig == 0)
    {
      s_WatchdogPeriodMs = 0;
      s_WatchdogTimer = 0;
    }
  }
  else
  {
    s_WatchdogPeriodMs = d * (uint32_t)60000;
    s_WatchdogTimer = MS_TIME_COUNT(lastHostCommandTimer) + s_WatchdogPeriodMs;
  }
}

void pwr_mngr_CmdGetWatchdogConfiguration(uint8_t data[], uint16_t *len)
{
  if (s_WatchdogConfig)
  {
    uint16_t d = s_WatchdogConfig;
    if (d >= 0x4000)
      d = (d >> 2) | 0x4000;
    data[0] = d;
    data[1] = (d >> 8) | 0x80;
  }
  else
  {
    uint16_t d = s_WatchdogPeriodMs / 60000;
    if (d >= 0x4000)
      d = (d >> 2) | 0x4000;
    data[0] = d;
    data[1] = d >> 8;
  }

  *len = 2;
}

void pwr_mngr_CmdSetWakeupOnCharge(uint8_t data[], uint16_t len)
{
  (void)len;

  if (data[0] & 0x80)
  {
    s_WakeupOnChargeConfig = (data[0] & 0x7F) <= 100 ? data[0] : 0x7F;
    nv_write_U8(NV_ADDR_WAKEUPONCHARGE_CONFIG, s_WakeupOnChargeConfig);
    if (nv_read_U8(NV_ADDR_WAKEUPONCHARGE_CONFIG, &s_WakeupOnChargeConfig) != NV_OK)
      s_WakeupOnChargeConfig = 0x7F;

    if (s_WakeupOnChargeConfig == 0x7F)
      s_WakeupOnCharge = PWR_WAKEUP_ON_CHARGE_OFF;
  }
  else
  {
    s_WakeupOnCharge = (data[0] & 0x7F) <= 100
        ? (uint16_t)(data[0] & 0x7F) * 10 : PWR_WAKEUP_ON_CHARGE_OFF;
  }
}

void pwr_mngr_CmdGetWakeupOnCharge(uint8_t data[], uint16_t *len)
{
  if (s_WakeupOnChargeConfig & 0x80)
    data[0] = s_WakeupOnChargeConfig;
  else
    data[0] = s_WakeupOnCharge <= 1000 ? s_WakeupOnCharge / 10 : 0x7F;

  *len = 1;
}

// TODO - the VSYS switch has never been driven; both halves stay stubs.
void pwr_mngr_CmdSetVSysSwitchState(uint8_t _state)
{
  (void)_state;
}

uint8_t pwr_mngr_CmdGetVSysSwitchState(void)
{
  return 0;
}

// DC-DC is the only mode; accepted and ignored so the host does not see an error.
void pwr_mngr_CmdSetRegulatorConfig(uint8_t data[], uint8_t len)
{
  (void)data;
  (void)len;
}

void pwr_mngr_CmdGetRegulatorConfig(uint8_t data[], uint16_t *len)
{
  data[0] = PWR_REGULATOR_MODE_DCDC;
  *len = 1;
}
