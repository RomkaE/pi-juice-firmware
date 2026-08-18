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
#include "nv.h"
#include "board.h"
#include "utils/time_count.h"
#include "src/app.h"
#include "app-error/diag.h"

// LOG:
#include "log/log.h"

/*
 * Debounce in ADC datasets, not ms: the values refresh once per half ring (~64 ms).
 *
 * Three is also what makes the power-up safe. APP arms the 5V check shortly after raising the
 * 5V bus, and at most one dataset published after that can still average in the time before the
 * boost came up - the one whose window started earlier. Any value below 2 here would let that
 * single stale dataset trip the protection on every power-up.
 */
#define PWR_PROTECTION_SAMPLES    3
#define PWR_5V_FAULT_MV           4500
#define PWR_AVDD_FAULT_MV         3150

#define PWR_VBAT_CUTOFF_DEFAULT_MV  2800

// Retained: the host reads these back as event flags after a brown out.
static uint8_t s_StatusFlags __attribute__((section("no_init")));

/* The 5V check is meaningless while the bus is coming up, so APP disarms it for as long as that
 * takes - see the SM_APP_POWER_UP state. The cutoff check is not affected and stays armed. */
static bool s_5vCheckArmed = true;

/* From the battery profile, or PWR_VBAT_CUTOFF_DEFAULT_MV when there is none. Compared against
 * analog_GetVBatt() and only while AVDD reads sane, so a bad reference cannot fake a flat pack. */
static uint16_t s_VbatCutoffMv;

// Owned by the ANALOG task, reset by pwr_mngr_HostOn() when the 5V bus comes back.
static uint8_t s_TripSamples;
static bool s_TripPosted;

static RunPinInstallationStatus_t s_RunPin = RUN_PIN_NOT_INSTALLED;

void pwr_mngr_Arm5vCheck(bool _armed)
{
  s_5vCheckArmed = _armed;
}

/*
 * Runs in the ANALOG task, once per fresh dataset - see the contract in analog.h. Reads and hands
 * over; the 5V bus is cut by APP so an in-flight flash write can finish, and shedding the external
 * load is a state action, not something this callback does.
 */
void analog_SamplesReady_Callback(void)
{
  if (!bsp_Pwr5V_GetState())
    return;   // nothing to protect, and the readings below would be meaningless

  /* One dataset per fault reason, at debug level on purpose: these run before the debounce, and a
   * single stale dataset right after the 5V bus comes up is expected - see PWR_PROTECTION_SAMPLES.
   * The real trip is announced once, below. */
  uint8_t faults = 0;

  /* AVDD is the ADC reference, so once it sags the 5V figure means nothing. It does not confirm a
   * 5V bus fault then - it replaces it. */
  uint16_t avdd = analog_GetAvdd();
  if (avdd < PWR_AVDD_FAULT_MV)
  {
    LOG_DEBUG("[PWR] AVDD below limit: avdd=%umV", (unsigned)avdd);
    faults |= PWR_FAULT_AVDD;
  }
  else
  {
    if (s_5vCheckArmed)
    {
      uint16_t vbus = analog_Get5vPi();
      if (vbus < PWR_5V_FAULT_MV)
      {
        LOG_DEBUG("[PWR] 5V below limit: vbus=%umV", (unsigned)vbus);
        faults |= PWR_FAULT_5V_MASK;
      }
    }

    if (!charger_IsInputPresent())
    {
      uint16_t vbat = analog_GetVBatt();
      if (vbat < s_VbatCutoffMv)
      {
        LOG_DEBUG("[PWR] VBAT below cutoff: vbat=%umV cutoff=%umV",
              (unsigned)vbat, (unsigned)s_VbatCutoffMv);
        faults |= PWR_FAULT_VBAT_CUTOFF;
      }
    }
  }

  if (faults == 0)
  {
    s_TripSamples = 0;
    s_TripPosted = false;

    // The conditions are live: once a clean dataset arrives, stop reporting them as present.
    diag_Clear(DIAG_PWR_AVDD_SAG);
    diag_Clear(DIAG_PWR_5V_UNDER);
    diag_Clear(DIAG_PWR_VBAT_CUTOFF);
    return;
  }

  if (s_TripSamples < PWR_PROTECTION_SAMPLES)
    s_TripSamples++;

  if (s_TripSamples < PWR_PROTECTION_SAMPLES || s_TripPosted)
    return;

  s_TripPosted = true;   // one event per trip

  /* Reported once per trip, same as the event: the counters must measure trips, not datasets. */
  if (faults & PWR_FAULT_AVDD)
    diag_Set(DIAG_PWR_AVDD_SAG);
  if (faults & PWR_FAULT_5V_MASK)
    diag_Set(DIAG_PWR_5V_UNDER);
  if (faults & PWR_FAULT_VBAT_CUTOFF)
    diag_Set(DIAG_PWR_VBAT_CUTOFF);

  LOG_WARNING("[PWR] Protection: faults=0x%02X, avdd=%umV, vbat=%umV, 5V=%umV", (unsigned)faults,
      (unsigned)analog_GetAvdd(), (unsigned)analog_GetVBatt(), (unsigned)analog_Get5vPi());

  // Send event to app:
  AppEvent_t evt = { .type = APP_EVT_POWER_PROTECTION };
  evt.powerTrip.faults = faults;
  app_PostEvent(&evt);
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

  // The 5V bus is not touched here: bsp_Pwr5V_Restore() already set it in main(), and forcing it
  // down would cut a host that survived the reset.

  LOG_INFO("[PWR] init: cutoff=%umV, run_pin=%u, bus5v=%s",
      (unsigned)s_VbatCutoffMv, (unsigned)s_RunPin, bsp_Pwr5V_GetState() ? "ON" : "OFF");
}

bool pwr_mngr_HostOn(void)
{
  if (bsp_Pwr5V_GetState())
    return true;

  /* The protection callback bails out while the 5V bus is down, so it never gets to clear these
   * itself - without this a retry would never see a second trip. */
  s_TripSamples = 0;
  s_TripPosted = false;

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

void pwr_mngr_SetBatProfile(const BatteryProfile_T *_p_profile)
{
  if (_p_profile != NULL)
    s_VbatCutoffMv =  (uint16_t)_p_profile->cutoffVoltage * 20;
  else
    s_VbatCutoffMv = PWR_VBAT_CUTOFF_DEFAULT_MV;
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

// Not implemented now:
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
