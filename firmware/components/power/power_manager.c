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

// Debounce in ADC datasets, not ms: the values refresh once per half ring (~64 ms).
#define PWR_PROTECTION_SAMPLES    3
#define PWR_5V_FAULT_MV           4500
#define PWR_AVDD_FAULT_MV         3150

#define PWR_VBAT_CUTOFF_DEFAULT_MV  3000

// Retained: the host reads these back as event flags after a brown out.
static uint8_t s_StatusFlags __attribute__((section("no_init")));

/* The 5V check is meaningless while the rail is coming up, so APP disarms it for as long as that
 * takes - see the SM_APP_POWER_UP state. The cutoff check is not affected and stays armed. */
static bool s_5vCheckArmed = true;

// Raw counts on purpose: VBAT_MV_TO_ADC assumes the nominal reference, which is what the cutoff
// path needs exactly when the supply is sagging. See analog.h.
static uint16_t s_VbatCutoffMv;
static uint16_t s_VbatCutoffAdc;

// Owned by the ANALOG task.
//static uint8_t s_TripSamples;
//static bool s_TripPosted;

static RunPinInstallationStatus_t s_RunPin = RUN_PIN_NOT_INSTALLED;

void pwr_mngr_Arm5vCheck(bool _armed)
{
  s_5vCheckArmed = _armed;
}

//static bool HasEnergy(void)
//{
//  return analog_GetVBattAvg() > s_VbatCutoffMv || charger_IsInputPresent();
//}


/*
 * Runs in the ANALOG task, once per fresh dataset - see the contract in analog.h. Reads and hands
 * over; the rail is cut by APP so an in-flight flash write can finish. Shedding the load goes
 * through the modules that own those pins, not through their registers.
 */
void analog_SamplesReadyCallback(void)
{
  //TODO
  /*
  if (!bsp_Pwr5V_GetState())
    return;

  uint8_t trip = PWR_TRIP_NONE;

  return analog_Get5vPi() >= PWR_5V_FAULT_MV || analog_GetAvdd() <= PWR_5V_FAULT_AVDD_MV;


  if (s_5vCheckArmed && !pwr_mngr_Is5vRailGood())
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

  AppEvent_t evt = { .type = APP_EVT_POWER_PROTECTION };
  evt.powerTrip.trip = trip;
  app_PostEvent(&evt);
  */
}

void pwr_mngr_Init(bool _cold_start)
{
  if (_cold_start)
    s_StatusFlags = 0;

  uint8_t value = 0;
  uint8_t nv_res = nv_read_U8(NV_ADDR_RUN_PIN_CONFIG, &value);
  if (nv_res == NV_OK && value <= RUN_PIN_INSTALLED)
    s_RunPin = value;
  else
  {
    LOG_WARNING("[PWR] Set default config RUN_PIN_CONFIG");
    value = RUN_PIN_NOT_INSTALLED;
    pwr_mngr_CmdSetRunPinConfig(&value, 1);
  }

  {
    BatteryProfile_T profile;
    bool have = battery_GetProfile(&profile);
    s_VbatCutoffMv = have ? (uint16_t)profile.cutoffVoltage * 20 : PWR_VBAT_CUTOFF_DEFAULT_MV;
  }
  s_VbatCutoffAdc = VBAT_MV_TO_ADC(s_VbatCutoffMv);

  // The rail is not touched here: bsp_Pwr5V_Restore() already set it in main(), and forcing it
  // down would cut a host that survived the reset.

  LOG_INFO("[PWR] init: cutoff=%u mV (%u counts), run_pin=%u, rail=%u",
      (unsigned)s_VbatCutoffMv, (unsigned)s_VbatCutoffAdc, (unsigned)s_RunPin,
      (unsigned)bsp_Pwr5V_GetState());
}

bool pwr_mngr_HostOn(void)
{
  if (bsp_Pwr5V_GetState())
    return true;

//  if (!HasEnergy())
//  {
//    LOG_WARNING("[PWR] turn on refused, vbat=%u cutoff=%u mV",
//        (unsigned)analog_GetVBattAvg(), (unsigned)s_VbatCutoffMv);
//    return false;
//  }

  bsp_Pwr5V_SetState(true);
  return true;
}

bool pwr_mngr_HostOff(void)
{
  if (!bsp_Pwr5V_GetState())
    return false;

  bsp_Pwr5V_SetState(false);
  return true;
}

uint32_t pwr_mngr_HostRestart(void)
{
  if (!bsp_Pwr5V_GetState())
    return 0;

  bsp_Pwr5V_SetState(false);
  LOG_INFO("[PWR] power cycle, back up in %u ms", (unsigned)PWR_POWER_CYCLE_MS);
  return PWR_POWER_CYCLE_MS;
}

void pwr_mngr_SetBatProfile(const BatteryProfile_T *_p_profile)
{
  if (_p_profile != NULL)
    s_VbatCutoffMv =  (uint16_t)_p_profile->cutoffVoltage * 20;
  else
    s_VbatCutoffMv = PWR_VBAT_CUTOFF_DEFAULT_MV;
  s_VbatCutoffAdc = VBAT_MV_TO_ADC(s_VbatCutoffMv);
}

// Derived, not stored: the charger already publishes everything this needs, ISR safe.
PowerSourceStatus_t pwr_mngr_GetInStatus(void)
{
  ChargerInputStatus_t status = charger_GetInStatus();
  if (status == CHG_IN_UVLO)
    return PWR_SOURCE_NOT_PRESENT;
  if (status == CHG_IN_OVP || status == CHG_IN_WEAK)
    return PWR_SOURCE_BAD;
  if (charger_GetDpmStatus())
    return PWR_SOURCE_WEAK;

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

uint8_t pwr_mngr_GetStatusFlags(void)
{
  return s_StatusFlags;
}

void pwr_mngr_SetStatusFlags(uint8_t _flags)
{
  s_StatusFlags |= _flags;
}

void pwr_mngr_KeepStatusFlags(uint8_t _mask)
{
  s_StatusFlags &= _mask;
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
