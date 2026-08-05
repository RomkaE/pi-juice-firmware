/*
 * charger_bq2416x.c
 *
 *  Created on: 06.12.2016.
 *      Author: milan
 */

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#include "charger_bq2416x.h"
#include "bq24160_regs_map.h"
#include "driver/i2c/i2c_master.h"
#include "app-error/app_assert.h"
#include "app-error/app_error.h"
#include "src/app.h"

// FreeRTOS:
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

// LOG:
#include "log/log.h"

// Board policy, not host settings.
// Precedence: charger out -> boost -> charger USB-IN, so USB-IN precedence would let the pack
// charge itself. IN wins always; the device powers on with the same value.
#define BQ_PRECEDENCE_IMAGE     0     // SUPPLY_SEL stays clear, IN wins

// USB-IN is locked out unconditionally, for the same reason: the boost feeds that pin.
//   "During OTG lock, the USB input is ignored and DRV does not come up."
// It is NOT a permanent lock, and cannot be made one. From the same section:
//   "The watchdog timer must be reset while in USB_LOCK to maintain the USB lockout state."
// So the lock holds only while the round keeps kicking the timer. CHG_ST_LOST stops kicking on
// purpose, and ~30 s later the device enters DEFAULT mode and the input comes back.
#define BQ_OTG_LOCK_IMAGE       1     // OTG_LOCK set, USB-IN ignored

// Both fields still have to hold something while the input is ignored.
#define BQ_IUSB_LIMIT_IMAGE     CHG_IUSB_LIMIT_100MA
#define BQ_VIN_DPM_USB          6     // moot while USB-IN is locked out

// Inputs config byte, host register 0x5E (pijuice.py SetPowerInputsConfig). The board crosses the
// inputs over: host "5V GPIO" -> USB-IN pin, host "USB micro" -> IN pin.
// Bit 7 means "persist" and is consumed by APP, never seen here.
#define CHG_INCFG_PRECEDENCE    0x01  // refused, see charger_SanitizeInputsConfig()
#define CHG_INCFG_GPIO_IN_EN    0x02  // refused, see BQ_OTG_LOCK_IMAGE
#define CHG_INCFG_NO_BAT_TURNON 0x04
#define CHG_INCFG_IN_ILIM       0x08  // 0 = 1.5 A, 1 = 2.5 A
#define CHG_INCFG_IN_DPM_MASK   0x70  // 4.20 V + 0.08 V per step
#define CHG_INCFG_IN_DPM_SHIFT  4

// Charging config byte, host register 0x51 (pijuice.py SetChargingConfig).
#define CHG_CHGCFG_ENABLE       0x01

// One cadence: the whole register set is read and the watchdog (32 s) kicked on the same tick.
// CHG_INT posts into the same queue, so a status change is not waited out.
#define CHG_ACTIVE_PERIOD_MS    1000   // CHG_ST_ACTIVE: one read-and-reconcile round
#define CHG_INIT_RETRY_MS       1000   // CHG_ST_INIT: one bring-up attempt
#define CHG_RETRY_PERIOD_MS     30000  // CHG_ST_LOST: how often a written-off device is retried
#define CHG_POWER_ON_DELAY_MS   100    // the device needs this much after power on

#define CHG_INIT_ATTEMPTS       5      // bring-up tries before the device is declared absent
#define CHG_ERR_LIMIT           5      // consecutive failed rounds that drop ACTIVE to LOST

#define CHG_I2C_ERR_REINIT      10     // soft bus failures before the I2C master is kicked

#define CHG_TASK_STACK_WORDS    256   // deepest measured frame is 56 bytes, see the .su file
#define CHG_QUE_LEN             8

_Static_assert(pdMS_TO_TICKS(CHG_ACTIVE_PERIOD_MS) >= 1,
               "CHG_ACTIVE_PERIOD_MS must be at least one RTOS tick");

typedef enum
{
  CHG_ST_INIT = 0,   // configuring the bq2416x
  CHG_ST_ACTIVE,     // device answering: read, reconcile, publish
  CHG_ST_LOST,       // device unreachable, waiting to retry
  CHG_ST_COUNT       // also bounds an entry chain - see fsmDispatch()
} ChargerState_t;

typedef enum
{
  CHG_EV_ENTRY = 0,  // first event a state ever sees, delivered by fsmDispatch()
  CHG_EV_TICK,       // the state's own queue timeout expired
  CHG_EV_IRQ,        // CHG_INT edge, posted straight from the EXTI interrupt
  CHG_EV_CMD         // a parameter was set, see ChargerCmdType_t
} ChargerEventType_t;

// Everything taken from a battery profile: three register codes. Range-checked once, at the
// boundary, so the queue carries three bytes instead of a 32-byte profile.
typedef struct
{
  uint8_t chrg_voltage;   // register 3 code, 20 mV per step above 3.5 V
  uint8_t chrg_current;   // register 5 code, 75 mA per step above 550 mA
  uint8_t term_current;   // register 5 code, 50 mA per step above 50 mA
} BqBattProfile_t;

// What each command carries in ChargerCmd_t.arg - all one byte except the profile.
typedef enum
{
  CHG_CMD_SET_BAT_PROFILE = 1,  // arg.batProfile
  CHG_CMD_SET_INPUTS_CONFIG,    // arg.u8 - configuration byte
  CHG_CMD_SET_CHARGING_CONFIG,  // arg.u8 - configuration byte
  CHG_CMD_SET_THERMAL_STATE     // arg.u8 - BatteryThermalState_T
} ChargerCmdType_t;

// Payload of a CHG_EV_CMD event: which parameter, and its value.
typedef struct
{
  ChargerCmdType_t id;
  union
  {
    uint8_t u8;
    struct
    {
      bool valid;
      BqBattProfile_t profile;
    } batProfile;
  } arg;
} ChargerCmd_t;

typedef struct
{
  ChargerEventType_t type;   // says which member of data is live
  union
  {
    ChargerCmd_t cmd;  // CHG_EV_CMD. TICK and IRQ carry nothing at all
  } data;
} ChargerEvent_t;

static TickType_t s_StateTimeout;

// What the device holds, and what we want it to hold. raw[] for the sweep and the write verify,
// named fields everywhere else.
static BQ24160Regs_t s_DeviceRegs;
static BQ24160Regs_t s_TargetRegs;

static const uint8_t s_WritableMask[BQ_REG_COUNT] = BQ_WRITABLE_MASK_INIT;

// Bits that read the same no matter what is ever written to them, straight from the register
// tables. Not a checksum - a flip anywhere else still passes - but they catch the corruptions
// that actually happen on a wire: a byte that came back all zeroes or all ones.
//
//   reg 0 bit 7  TMR_RST  "Read: Always 0"
//   reg 2 bit 7  RESET    "Read: always get 1"
//   reg 4 bits 7:3        vendor code 010 and part number 00, fixed for a bq2416x at address 6Bh
//                         (bits 2:0 are the silicon revision and are left free)
//
// Only device invariants belong here. Bits this module owns - SUPPLY_SEL, OTG_LOCK, EN_NOBATOP -
// are deliberately absent: when one of those reads back wrong it is reconcile()'s business, not
// a corrupted read, and freezing our own policy into a validity check would be a trap.
static const uint8_t s_InvariantMask[BQ_REG_COUNT]  = { 0x80, 0, 0x80, 0, 0xF8, 0, 0, 0 };
static const uint8_t s_InvariantValue[BQ_REG_COUNT] = { 0x00, 0, 0x80, 0, 0x40, 0, 0, 0 };

static uint16_t s_I2cErrorCounter;   // bus level, published through charger_GetI2cErrorCount()

// What the getters answer with. One structure so that "what changed" is one comparison against
// one object. Byte-sized members: each is read whole from another task or from the I2C1 ISR.
// Enums held as uint8_t so one publishValue(uint8_t*) serves all nine - an enum here is 4 bytes.
typedef struct
{
  uint8_t status;           // ChargerStatus_T
  uint8_t fault;            // ChargerFaultStatus_T
  uint8_t in_stat;
  uint8_t usb_stat;
  uint8_t ts_fault;
  uint8_t dpm_active;       // 0 or 1
  uint8_t input_present;    // 0 or 1
  uint8_t battery_present;  // 0 or 1
} ChargerPublished_t;

static ChargerPublished_t s_Pub = {
    .status = CHG_NA,
    .fault = CHG_FAULT_UNKNOWN };

// Everything that decides what the device should hold. Written by cmdProcess() and read by
// buildTargetImage(), and by nothing else. The raw configuration bytes are not kept - APP holds
// those for the host read-back - and the profile is converted and copied in, never a live pointer
// into another task's memory.
typedef struct
{
  uint8_t no_battery_turnon;  // host 0x5E bit 2 - drives no register, only the getter
  uint8_t in_current_limit;   // host 0x5E bit 3: 0 = 1.5 A, 1 = 2.5 A
  uint8_t in_dpm;             // host 0x5E bits 6:4, 4.20 V + 80 mV per step
  uint8_t charging_enabled;   // host 0x51 bit 0
  uint8_t thermal_state;      // BatteryThermalState_T, aggregated by battery.c
  uint8_t profile_valid;
  BqBattProfile_t profile;
} ChargerConfig_t;

static ChargerConfig_t s_Cfg = { .thermal_state = BAT_TEMP_UNKNOWN };

static uint32_t s_ErrMask;  // CHARGER_ERR_* bits

static TaskHandle_t s_TaskHandle;
static StaticTask_t s_TaskTCB;
static StackType_t s_TaskStack[CHG_TASK_STACK_WORDS];

static QueueHandle_t s_QueHandle;
static StaticQueue_t s_Que;
static ChargerEvent_t s_QueBuf[CHG_QUE_LEN];

static bool postEvent(const ChargerEvent_t *_ev);

// Defined below the sync functions that need it.
static void publishValue(uint8_t *_p_dst, uint8_t _val, uint8_t _evt_type, const char *_name);

// Foreign profile -> the three codes, range-checked here and nowhere else.
// Currents saturate; battery.c's resistor profile really does reach 28 against a limit of 26.
// Voltage does not: capping it would charge an unknown pack to 4.76 V, so the profile is rejected
// and buildTargetImage() keeps charging disabled.
static bool profileConvert(const BatteryProfile_T *_src, BqBattProfile_t *_dst)
{
  if (_src->regulationVoltage > BQ_CHRG_VOLTAGE_MAX)
  {
    LOG_ERROR("[CHG] profile rejected: regulation voltage code %u, max %u",
        (unsigned )_src->regulationVoltage, (unsigned)BQ_CHRG_VOLTAGE_MAX);
    return false;
  }

  _dst->chrg_voltage = _src->regulationVoltage;
  _dst->chrg_current = (_src->chargeCurrent > BQ_CHRG_CURRENT_MAX) ?
          BQ_CHRG_CURRENT_MAX : _src->chargeCurrent;
  _dst->term_current = (_src->terminationCurr > BQ_TERM_CURRENT_MAX) ?
          BQ_TERM_CURRENT_MAX : _src->terminationCurr;
  return true;
}

static void countBusError(void)
{
  if (s_I2cErrorCounter < 0xFF)
    s_I2cErrorCounter++;

  // Many soft failures mean the bus is wedged. Kick the master once, then count from zero.
  if (s_I2cErrorCounter > CHG_I2C_ERR_REINIT)
  {
    LOG_WARNING("[CHG] I2C master re-init after %u errors", (unsigned )s_I2cErrorCounter);
    i2c_master_ReInit();
    s_I2cErrorCounter = 1;
  }
}

// One read, on purpose. Reading twice and comparing cannot tell "the device changed the value"
// from "the value came back corrupted", and registers 0, 1, 6 and 7 carry live status bits - so
// every input transition produced a false mismatch and cost a whole round. Both cases cost one
// round anyway: a bad status value is replaced a second later, a bad configuration value is
// caught by the write verify. It also halves the traffic on a bus shared with the fuel gauge.
//
// Does NOT clear the error streak on success: only a whole operation may end it, see regWrite().
static bool regReadRaw(uint8_t _reg)
{
  uint8_t v;

  if (i2c_master_ReadMem(BQ_I2C_ADDR, _reg, &v, 1) != I2C_OK)
  {
    countBusError();
    return false;
  }

  // One bad bit means the whole byte is untrustworthy - drop it rather than decode it. The
  // snapshot then has a hole, so regReadAll() ends the round and nothing is published from it.
  uint8_t mask = s_InvariantMask[_reg];
  if ((v & mask) != s_InvariantValue[_reg])
  {
    LOG_ERROR("[CHG] reg%u invalid: %02X, expected %02X within mask %02X",
        (unsigned)_reg, v, s_InvariantValue[_reg], mask);
    countBusError();
    return false;
  }

  s_DeviceRegs.raw[_reg] = v;
  return true;
}

// A read on its own is the whole operation, so its success ends the error streak.
static bool regRead(uint8_t _reg)
{
  if (!regReadRaw(_reg))
    return false;

  s_I2cErrorCounter = 0;
  return true;
}

// Writes and verifies over the writable bits only.
// Read-back is regReadRaw(), not regRead(): clearing the streak there would make a persistent
// read-back mismatch unable to ever reach CHG_I2C_ERR_REINIT.
static bool regWrite(uint8_t _reg)
{
  LOG_DEBUG("[CHG] reg%u write %02X -> %02X (mask %02X)", (unsigned)_reg,
      s_DeviceRegs.raw[_reg], s_TargetRegs.raw[_reg], s_WritableMask[_reg]);

  if (i2c_master_WriteMem(BQ_I2C_ADDR, _reg, &s_TargetRegs.raw[_reg], 1) != I2C_OK)
  {
    LOG_ERROR("[CHG] reg%u write failed", (unsigned)_reg);
    countBusError();
    return false;
  }

  if (!regReadRaw(_reg))
    return false;

  uint8_t mask = s_WritableMask[_reg];
  if ((s_DeviceRegs.raw[_reg] & mask) != (s_TargetRegs.raw[_reg] & mask))
  {
    LOG_ERROR("[CHG] reg%u verify failed: wrote %02X, read %02X (mask %02X)", (unsigned)_reg,
        s_TargetRegs.raw[_reg], s_DeviceRegs.raw[_reg], mask);
    countBusError();
    return false;
  }

  s_I2cErrorCounter = 0;
  return true;
}

// Writes only on a difference. The snapshot is refreshed in full at the top of every round, so
// no re-read is needed here.
static bool regSyncIfDiffers(uint8_t _reg)
{
  uint8_t mask = s_WritableMask[_reg];

  if ((s_TargetRegs.raw[_reg] & mask) == (s_DeviceRegs.raw[_reg] & mask))
    return true;

  return regWrite(_reg);
}

// Blanked so regSyncIfDiffers() cannot match stale content and skip a write. What the outside
// sees is publishUnknown()'s business.
static void cacheInvalidate(void)
{
  for (uint8_t r = 0; r < BQ_REG_COUNT; r++)
    s_DeviceRegs.raw[r] = 0;
}

static bool regReadAll(void)
{
  bool ok = true;

  for (uint8_t r = 0; r < BQ_REG_COUNT; r++)
  {
    if (!regRead(r))
      ok = false;
  }

  return ok;
}

// TMR_RST is set here and nowhere else, and this write cannot be verified - the bit reads 0.
// Once per round; the device allows 30 s.
static void kickWatchdog(void)
{
  BQ24160Reg_StatusControl_t kick = s_TargetRegs.reg.status_control;
  kick.wr.tmr_rst = 1;

  if (i2c_master_WriteMem(BQ_I2C_ADDR, BQ_REG_STATUS_CONTROL, &kick.raw, 1) != I2C_OK)
  {
    LOG_ERROR("[CHG] watchdog kick failed");
    countBusError();
  }
}

// The profile code, backed off by 140 mV above tWarm. WARM and HOT both sit above it.
// Zero without a profile - charging is disabled in that case anyway, see the CE bit below.
static uint8_t regulationVoltage(const ChargerConfig_t *_p_cfg)
{
  if (!_p_cfg->profile_valid)
    return 0;

  int16_t code = _p_cfg->profile.chrg_voltage;

  if (_p_cfg->thermal_state >= BAT_TEMP_WARM)
  {
    code -= (140 / 20);
    if (code < 0)
      code = 0;
  }

  return (uint8_t)code;
}

// Configuration -> the image the device should hold. Takes the configuration as an argument
// rather than reaching for s_Cfg, so what it depends on is in the signature: it writes
// s_TargetRegs and never touches the bus.
//
// Called when the configuration changes and at bring-up, not every round. The image is ours and
// lives in RAM, so nothing the device does to itself - a watchdog expiry, a DEFAULT mode entry -
// can disturb it. Noticing that the device has drifted away from it is reconcile()'s job.
static void buildTargetImage(const ChargerConfig_t *_p_cfg)
{
  // Register 0 - the input precedence is the only bit that is ours, and it is fixed.
  s_TargetRegs.reg.status_control.raw = 0;
  s_TargetRegs.reg.status_control.wr.supply_sel = BQ_PRECEDENCE_IMAGE;

  // Register 1 - USB-IN locked out, see BQ_OTG_LOCK_IMAGE. Bit 0 (no-battery operation) stays 0:
  // nothing drives it here or in V1.6, and no_battery_turnon is a different thing - it feeds the
  // 5V boost turn-on in power_management.c.
  // TODO(hw): whether bit 0 should be driven at all is a board question.
  s_TargetRegs.reg.supply_status.raw = 0;
  s_TargetRegs.reg.supply_status.rd_wr.otg_lock = BQ_OTG_LOCK_IMAGE;

  // Register 2 - charging needs a profile, the host's consent and a temperature we can charge at.
  bool allow = _p_cfg->profile_valid && _p_cfg->charging_enabled
      && _p_cfg->thermal_state != BAT_TEMP_COLD && _p_cfg->thermal_state != BAT_TEMP_HOT;

  s_TargetRegs.reg.control.raw = 0;
  s_TargetRegs.reg.control.rd_wr.iusb_limit = BQ_IUSB_LIMIT_IMAGE;
  s_TargetRegs.reg.control.rd_wr.en_stat = 1;
  s_TargetRegs.reg.control.rd_wr.te = 1;
  s_TargetRegs.reg.control.rd_wr.ce = allow ? 0 : 1;  // CE is inverted: 1 disables charging
  s_TargetRegs.reg.control.rd_wr.hz_mode = 0;         // never - it drops VSys and kills the MCU

  // Register 3 - dpdm_en stays clear: normal state, no forced D+/D- detection.
  s_TargetRegs.reg.bat_voltage.raw = 0;
  s_TargetRegs.reg.bat_voltage.rd_wr.iinlimit = _p_cfg->in_current_limit;
  s_TargetRegs.reg.bat_voltage.rd_wr.vbreg = regulationVoltage(_p_cfg);

  // Register 4 is read only - its writable mask is empty, so reconcile() skips it by itself.

  // Register 5 - both codes are in range by construction, see profileConvert().
  s_TargetRegs.reg.charge_current.raw = 0;
  if (_p_cfg->profile_valid)
  {
    s_TargetRegs.reg.charge_current.rd_wr.ichrg = _p_cfg->profile.chrg_current;
    s_TargetRegs.reg.charge_current.rd_wr.iterm = _p_cfg->profile.term_current;
  }

  // Register 6.
  s_TargetRegs.reg.vin_dpm.raw = 0;
  s_TargetRegs.reg.vin_dpm.rd_wr.vindpm_in = _p_cfg->in_dpm;
  s_TargetRegs.reg.vin_dpm.rd_wr.vindpm_usb = BQ_VIN_DPM_USB;

  // Register 7 - the device's safety timer is switched off, deliberately.
  //
  // It cannot do the job here. Its three settings are 27 min / 6 h / 9 h, none of which follows
  // from the pack, whereas the time a charge should take follows directly from capacity and
  // charge current. And the way it reports expiry is unusable for us: it disables charging,
  // resets the charge parameters to defaults and sets CE to 1, and clearing CE is what both
  // resumes charging and clears the fault - which reconcile() would do a second later.
  //
  // TODO: a charge timeout in firmware, computed from the profile capacity and the charge
  // current, latching on expiry until something explicit clears it.
  //
  // 2XTMR_EN goes with it: it only ever slowed the timer that is now off.
  s_TargetRegs.reg.safety_ntc.raw = 0;
  s_TargetRegs.reg.safety_ntc.rd_wr.tmr = BQ_TMR_OFF;
  s_TargetRegs.reg.safety_ntc.rd_wr.tmr2x_en = 0;
  s_TargetRegs.reg.safety_ntc.rd_wr.ts_en = 0;   // the thermistor is read by the fuel gauge

  // COLD and COOL both sit below tCool: halve the charge current.
  if (_p_cfg->profile_valid && _p_cfg->thermal_state != BAT_TEMP_UNKNOWN
      && _p_cfg->thermal_state <= BAT_TEMP_COOL)
    s_TargetRegs.reg.safety_ntc.rd_wr.low_chg = 1;

  LOG_INFO("[CHG] target image %02X %02X %02X %02X -- %02X %02X %02X, charging %s",
      s_TargetRegs.raw[0], s_TargetRegs.raw[1], s_TargetRegs.raw[2], s_TargetRegs.raw[3],
      s_TargetRegs.raw[5], s_TargetRegs.raw[6], s_TargetRegs.raw[7], allow ? "allowed" : "off");

  if (!allow)
    LOG_INFO("[CHG] charging off: profile=%u enabled=%u thermal=%u",
        (unsigned)_p_cfg->profile_valid, (unsigned)_p_cfg->charging_enabled,
        (unsigned)_p_cfg->thermal_state);
}

static void onAppEventLost(uint8_t _type)
{
  LOG_ERROR("[CHG] APP queue full, charger event %u dropped", _type);
  taskENTER_CRITICAL();
  s_ErrMask |= CHARGER_ERR_EVENT_LOST;
  taskEXIT_CRITICAL();
}

// The one place s_Pub is written and the one place a charger event is posted.
// Store before post, so a handler that calls a getter straight away reads the new value.
static void publishValue(uint8_t *_p_dst, uint8_t _val, uint8_t _evt_type, const char *_name)
{
  if (*_p_dst == _val)
    return;

  LOG_DEBUG("[CHG] %s %u -> %u", _name, (unsigned)*_p_dst, (unsigned)_val);
  *_p_dst = _val;

  AppEvent_t evt = { .type = _evt_type, .data.chargerValue = { _val } };
  if (!app_PostEvent(&evt))
    onAppEventLost(_evt_type);
}

// Same compare-store-post, older payload - these two already have consumers.
static void publishPresence(uint8_t *_p_dst, bool _present, uint8_t _evt_type, const char *_name)
{
  if (*_p_dst == (uint8_t)_present)
    return;

  LOG_INFO("[CHG] %s %u -> %u", _name, (unsigned)*_p_dst, (unsigned)_present);
  *_p_dst = (uint8_t)_present;

  AppEvent_t evt = { .type = _evt_type };
  if (_evt_type == APP_EVT_CHARGER_INPUT_PRESENCE)
    evt.data.chargerInput.present = _present;
  else
    evt.data.batteryPresence.present = _present;

  if (!app_PostEvent(&evt))
    onAppEventLost(_evt_type);
}

// Snapshot in, values out. Touches no static, posts no event.
// Each register is read into a local once, so no value mixes two moments of a sweep.
// Returns rather than fills: every member is set here, and the caller cannot forget that.
static ChargerPublished_t decodeSnapshot(void)
{
  BQ24160Reg_StatusControl_t status = s_DeviceRegs.reg.status_control;
  BQ24160Reg_SupplyStatus_t supply = s_DeviceRegs.reg.supply_status;
  ChargerPublished_t next;

  next.status = status.rd.status;
  next.fault = status.rd.fault;
  next.in_stat = supply.rd.in_status;
  next.usb_stat = supply.rd.usb_status;
  next.dpm_active = (s_DeviceRegs.reg.vin_dpm.rd.dpm_status != 0);
  next.ts_fault = s_DeviceRegs.reg.safety_ntc.rd.ts_fault;

  // Present when the status names a source: "ready" through "done", nothing outside that.
  next.input_present = (next.status > CHG_NO_VALID_SOURCE) && (next.status < CHG_NA);

  // Half the story - battery.c combines it with the pack voltage.
  next.battery_present = (supply.rd.bat_stat != BQ_BATSTAT_NOT_PRESENT);

  return next;
}

// Take the decoded values as the published ones, telling APP about each that moved.
static void publishChanges(const ChargerPublished_t *_p_next)
{
  publishValue(&s_Pub.status,      _p_next->status,      APP_EVT_CHARGER_STATUS,      "status");
  publishValue(&s_Pub.fault,       _p_next->fault,       APP_EVT_CHARGER_FAULT,       "fault");
  publishValue(&s_Pub.in_stat,     _p_next->in_stat,     APP_EVT_CHARGER_IN_STAT,     "instat");
  publishValue(&s_Pub.usb_stat,    _p_next->usb_stat,    APP_EVT_CHARGER_USB_STAT,    "usbstat");
  publishValue(&s_Pub.ts_fault,    _p_next->ts_fault,    APP_EVT_CHARGER_TS_FAULT,    "tsfault");
  publishValue(&s_Pub.dpm_active,  _p_next->dpm_active,  APP_EVT_CHARGER_DPM,         "dpm");

  publishPresence(&s_Pub.input_present, _p_next->input_present != 0,
      APP_EVT_CHARGER_INPUT_PRESENCE, "input");
  publishPresence(&s_Pub.battery_present, _p_next->battery_present != 0,
      APP_EVT_BATTERY_PRESENCE, "battery");
}

// Nothing is known about the device. Same publisher as a normal round, so losing it announces
// itself on every value rather than only on the two presence flags.
static void publishUnknown(void)
{
  ChargerPublished_t next = {
      .status = CHG_NA,
      .fault = CHG_FAULT_UNKNOWN };

  publishChanges(&next);
}

// Push every register the device is not already holding. Knows nothing about any setting - what
// to hold was decided by buildTargetImage(). Register 4 has an empty writable mask, so it falls
// out without a special case.
//
// Descending, so that every "how much" and "how high" register is in place before register 2
// carries the CE bit that lets charging start. A device fresh out of DEFAULT mode holds 3.6 V
// and 1 A, and this is what keeps that from being live for the length of two more writes.
//
// A failure does not stop the rest: one unreachable register is no reason to leave the others.
static bool reconcile(void)
{
  bool ok = true;

  for (uint8_t r = BQ_REG_COUNT; r-- > 0; )
  {
    if (!regSyncIfDiffers(r))
      ok = false;
  }

  return ok;
}

static bool chargerBringUp(void)
{
  // The only two unverified writes in the module - there is no snapshot to verify against yet.
  // Both images are assigned in full: building one from bits declared elsewhere is how the
  // documented "500 mA" silently became 800.
  s_TargetRegs.reg.supply_status.raw = 0;
  s_TargetRegs.reg.supply_status.rd_wr.otg_lock = BQ_OTG_LOCK_IMAGE;
  if (i2c_master_WriteMem(BQ_I2C_ADDR, BQ_REG_SUPPLY_STATUS,
      &s_TargetRegs.reg.supply_status.raw, 1) != I2C_OK)
  {
    countBusError();
    return false;
  }

  // Never select high impedance mode - it disables the VSys mosfet and cuts power to the MCU.
  s_TargetRegs.reg.control.raw = 0;
  s_TargetRegs.reg.control.rd_wr.iusb_limit = BQ_IUSB_LIMIT_IMAGE;
  s_TargetRegs.reg.control.rd_wr.en_stat = 1;
  s_TargetRegs.reg.control.rd_wr.te = 1;
  s_TargetRegs.reg.control.rd_wr.ce = 1;      // charging off until the first round says otherwise
  s_TargetRegs.reg.control.rd_wr.hz_mode = 0;
  if (i2c_master_WriteMem(BQ_I2C_ADDR, BQ_REG_CONTROL, &s_TargetRegs.reg.control.raw,
      1) != I2C_OK)
  {
    countBusError();
    return false;
  }

  // TODO - check
  TickType_t delay = pdMS_TO_TICKS(1);
  vTaskDelay(delay > 0 ? delay : 1);

  if (!regReadAll())
    return false;

  // Published here, not on the way into ACTIVE: a device that has just come back should not wait
  // out a round before it is announced.
  {
    ChargerPublished_t next = decodeSnapshot();
    publishChanges(&next);
  }

  BQ24160Reg_Vendor_t id = s_DeviceRegs.reg.vendor;
  LOG_INFO("[CHG] up: vendor=%u part=%u rev=%u, status=%u fault=%u instat=%u usbstat=%u",
      (unsigned)id.rd.vendor, (unsigned)id.rd.part, (unsigned)id.rd.revision,
      (unsigned)s_Pub.status, (unsigned)s_Pub.fault,
      (unsigned)s_Pub.in_stat, (unsigned)s_Pub.usb_stat);

  // Apply the configuration now rather than a second from now, and take a failure as "not ready":
  // a device that will not accept a write is not one to hand over to CHG_ST_ACTIVE.
  return reconcile();
}

// One round in four steps. A snapshot that could not be taken ends it: nothing is published, so
// the values keep standing until either the next round succeeds or CHG_ST_LOST blanks them.
static bool chargerRound(void)
{
  bool read_ok = regReadAll();  // 1. snapshot

  // Kicked whatever the read did. It is one write, it is the only thing keeping the device out
  // of DEFAULT mode, and a transient read failure is no reason to let a 30 s timer run down.
  // The image it sends is register 0's target, which is a constant - see buildTargetImage().
  kickWatchdog();

  if (!read_ok)
    return false;

  LOG_VERBOSE("[CHG] regs: %02X %02X %02X %02X %02X %02X %02X %02X",
      s_DeviceRegs.raw[0], s_DeviceRegs.raw[1], s_DeviceRegs.raw[2], s_DeviceRegs.raw[3],
      s_DeviceRegs.raw[4], s_DeviceRegs.raw[5], s_DeviceRegs.raw[6], s_DeviceRegs.raw[7]);

  ChargerPublished_t next = decodeSnapshot();   // 2. snapshot -> values, no side effects
  publishChanges(&next);                        // 3. compare, store, post

  return reconcile();           // 4. write back what differs
}

// CHG_INCFG_PRECEDENCE and CHG_INCFG_GPIO_IN_EN are deliberately not read - both are fixed by the
// board, see BQ_PRECEDENCE_IMAGE and BQ_OTG_LOCK_IMAGE.
static void applyInputsConfig(uint8_t _config)
{
  s_Cfg.no_battery_turnon = (_config & CHG_INCFG_NO_BAT_TURNON) != 0;
  s_Cfg.in_current_limit  = (_config & CHG_INCFG_IN_ILIM) != 0;
  s_Cfg.in_dpm            = (_config & CHG_INCFG_IN_DPM_MASK) >> CHG_INCFG_IN_DPM_SHIFT;

  LOG_INFO("[CHG] inputs cfg %02X: no_bat_turnon=%u in_ilim=%u in_dpm=%u",
      _config, (unsigned)s_Cfg.no_battery_turnon,
      (unsigned)s_Cfg.in_current_limit, (unsigned)s_Cfg.in_dpm);
}

static void applyChargingConfig(uint8_t _config)
{
  s_Cfg.charging_enabled = (_config & CHG_CHGCFG_ENABLE) != 0;

  LOG_INFO("[CHG] charging cfg %02X: enabled=%u", _config, (unsigned)s_Cfg.charging_enabled);
}

// Takes one command's value into the configuration and rebuilds the target image from it. That
// is the whole of "configuring": nothing here touches the bus, and getting the device to hold the
// new image is reconcile()'s business on the round that follows.
static void cmdProcess(const ChargerEvent_t *_ev)
{
  switch (_ev->data.cmd.id)
  {
    case CHG_CMD_SET_BAT_PROFILE:
      s_Cfg.profile_valid = _ev->data.cmd.arg.batProfile.valid;
      if (s_Cfg.profile_valid)
      {
        s_Cfg.profile = _ev->data.cmd.arg.batProfile.profile;
        LOG_INFO("[CHG] profile: vbreg=%u ichrg=%u iterm=%u", (unsigned)s_Cfg.profile.chrg_voltage,
            (unsigned)s_Cfg.profile.chrg_current, (unsigned)s_Cfg.profile.term_current);
      }
      else
        LOG_WARNING("[CHG] no usable profile, charging stays disabled");
    break;

    case CHG_CMD_SET_INPUTS_CONFIG:
      applyInputsConfig(_ev->data.cmd.arg.u8);
    break;

    case CHG_CMD_SET_CHARGING_CONFIG:
      applyChargingConfig(_ev->data.cmd.arg.u8);
    break;

    case CHG_CMD_SET_THERMAL_STATE:
      if (s_Cfg.thermal_state != _ev->data.cmd.arg.u8)
        LOG_INFO("[CHG] thermal state %u -> %u", (unsigned)s_Cfg.thermal_state,
            _ev->data.cmd.arg.u8);
      s_Cfg.thermal_state = _ev->data.cmd.arg.u8;
    break;

    default:
      LOG_WARNING("[CHG] unknown command %u", (unsigned )_ev->data.cmd.id);
    return;   // nothing was taken in, so the image cannot have changed
  }

  buildTargetImage(&s_Cfg);
}

// Every state lists every event it answers to; nothing hides behind a shared default.
// "break" stays put, "return" names the state to move to.
// CHG_EV_ENTRY is delivered by fsmDispatch() the moment the transition is made.
static ChargerState_t stInit(const ChargerEvent_t *_ev)
{
  static uint8_t init_attempts;

  switch (_ev->type)
  {
    case CHG_EV_ENTRY:
      LOG_INFO("[CHG] state INIT, %u attempts before giving up", (unsigned)CHG_INIT_ATTEMPTS);
      s_StateTimeout = 0; // first attempt at once, the tick below paces the retries after it
      init_attempts = 0;
    break;

    case CHG_EV_TICK:
      // Entry left the timeout at zero for an immediate first attempt; from here on it is paced.
      s_StateTimeout = pdMS_TO_TICKS(CHG_INIT_RETRY_MS);

      if (chargerBringUp())
        return CHG_ST_ACTIVE;

      LOG_WARNING("[CHG] bring-up attempt %u/%u failed, i2c errors=%u",
          (unsigned)(init_attempts + 1), (unsigned)CHG_INIT_ATTEMPTS, (unsigned)s_I2cErrorCounter);
      if (++init_attempts >= CHG_INIT_ATTEMPTS)
        return CHG_ST_LOST;
    break;

    case CHG_EV_IRQ:
    break;   // nothing published yet, the next attempt reads everything anyway

    case CHG_EV_CMD:
      // Only taken in: the bring-up already in progress reads it when it gets there.
      cmdProcess(_ev);
    break;

    default:
      LOG_WARNING("[CHG] unhandled event %u in INIT", (unsigned)_ev->type);
    break;
  }

  return CHG_ST_INIT;
}

static ChargerState_t stActive(const ChargerEvent_t *_ev)
{
  static uint8_t err_count;

  switch (_ev->type)
  {
    // The one event that does not end in a round - bring-up already read and published.
    // "return" here means "stay and skip the round below", not a transition.
    case CHG_EV_ENTRY:
      LOG_INFO("[CHG] state ACTIVE, round every %u ms", (unsigned)CHG_ACTIVE_PERIOD_MS);
      s_StateTimeout = pdMS_TO_TICKS(CHG_ACTIVE_PERIOD_MS);
      err_count = 0;
      return CHG_ST_ACTIVE;

    case CHG_EV_TICK:
    case CHG_EV_IRQ:
    break;   // nothing to take in, straight to the round below

    case CHG_EV_CMD:
      cmdProcess(_ev);
    break;

    default:
      LOG_WARNING("[CHG] unhandled event %u in ACTIVE", (unsigned )_ev->type);
      return CHG_ST_ACTIVE; // no reason to touch the bus over an event we do not know
  }

  // Everything else ends the same way: push whatever changed into the device now.
  if (chargerRound())
  {
    err_count = 0;
    return CHG_ST_ACTIVE;
  }

  if (++err_count >= CHG_ERR_LIMIT)
  {
    LOG_ERROR("[CHG] %u failed rounds in a row, i2c errors=%u",
        (unsigned)err_count, (unsigned)s_I2cErrorCounter);
    return CHG_ST_LOST;
  }

  LOG_WARNING("[CHG] round failed %u/%u", (unsigned)err_count, (unsigned)CHG_ERR_LIMIT);

  return CHG_ST_ACTIVE;
}

static ChargerState_t stLost(const ChargerEvent_t *_ev)
{
  switch (_ev->type)
  {
    case CHG_EV_ENTRY:
      s_StateTimeout = pdMS_TO_TICKS(CHG_RETRY_PERIOD_MS);

      // Stop answering from a snapshot we can no longer refresh, and tell APP now - this is the
      // moment the input and the pack stopped being visible.
      cacheInvalidate();
      publishUnknown();

      // No watchdog kick from here on: if we cannot talk to the device, the best it can do is
      // time out and fall back to its own defaults.
      LOG_ERROR("[CHG] state LOST, device unreachable (i2c errors=%u), retry in %u ms",
          (unsigned)s_I2cErrorCounter, (unsigned)CHG_RETRY_PERIOD_MS);
    break;

    case CHG_EV_TICK:
      return CHG_ST_INIT;   // periodic recovery attempt

    case CHG_EV_IRQ:
      // It raised its interrupt line, so it is alive - no reason to sit out the backoff.
      LOG_INFO("[CHG] interrupt from a device believed lost, retrying now");
      return CHG_ST_INIT;

    case CHG_EV_CMD:
      // Nothing reaches the device while it is written off - the recovery attempt applies it all.
      cmdProcess(_ev);
    break;

    default:
      LOG_WARNING("[CHG] unhandled event %u in LOST", (unsigned )_ev->type);
    break;
  }

  return CHG_ST_LOST;
}

// The only place that maps a state to its handler.
static ChargerState_t stateHandle(ChargerState_t _state, const ChargerEvent_t *_ev)
{
  switch (_state)
  {
    case CHG_ST_INIT:
      return stInit(_ev);
    case CHG_ST_ACTIVE:
      return stActive(_ev);
    case CHG_ST_LOST:
      return stLost(_ev);
    default:
      APP_ERROR(APP_ERR);
      return _state;
  }
}

// The only place that assigns the state. Handlers decide, this moves.
// An entry action may itself ask to move on, hence the loop; the guard catches a chain that
// does not settle.
static void fsmDispatch(const ChargerEvent_t *_ev)
{
  static ChargerState_t state = CHG_ST_INIT;

  ChargerState_t next = stateHandle(state, _ev);

  uint8_t guard = CHG_ST_COUNT;
  while (next != state)
  {
    if (guard-- == 0)
    {
      LOG_ERROR("[CHG] entry chain does not settle at state %u, event %u",
          (unsigned)next, (unsigned)_ev->type);
      APP_ERROR(APP_ERR);
    }

    state = next;
    static const ChargerEvent_t entryEv = { .type = CHG_EV_ENTRY };
    next = stateHandle(state, &entryEv);
  }
}

static bool postEvent(const ChargerEvent_t *_ev)
{
  if (s_QueHandle == NULL)
  {
    s_ErrMask |= CHARGER_ERR_NOT_READY;
    return false;
  }

  BaseType_t sent;

  if (xPortIsInsideInterrupt() != pdFALSE)
  {
    BaseType_t woken = pdFALSE;
    sent = xQueueSendFromISR(s_QueHandle, _ev, &woken);
    portYIELD_FROM_ISR(woken);

    if (sent != pdTRUE)
      s_ErrMask |= CHARGER_ERR_QUEUE_FULL;
  }
  else
  {
    sent = xQueueSend(s_QueHandle, _ev, 0);

    if (sent != pdTRUE)
    {
      LOG_ERROR("[CHG] event queue full, ev=%u dropped", _ev->type);
      taskENTER_CRITICAL();
      s_ErrMask |= CHARGER_ERR_QUEUE_FULL;
      taskEXIT_CRITICAL();
    }
  }

  return (sent == pdTRUE);
}

static void Task(void *parameters)
{
  (void)parameters;

  LOG_INFO("[CHG] task started, %u ms power-on delay", (unsigned)CHG_POWER_ON_DELAY_MS);

  vTaskDelay(pdMS_TO_TICKS(CHG_POWER_ON_DELAY_MS)); // the device needs time after power on

  // Entry event to start FSM:
  const ChargerEvent_t entryEv = { .type = CHG_EV_ENTRY };
  fsmDispatch(&entryEv);

  while (1)
  {
    ChargerEvent_t ev;
    if (xQueueReceive(s_QueHandle, &ev, s_StateTimeout) != pdTRUE)
      ev.type = CHG_EV_TICK;

    fsmDispatch(&ev);
  }
}

// Bitfield order is implementation-defined and getting it wrong is silent: fields declared in
// datasheet order land mirrored. Each named field is checked against its documented mask.
// TODO - remove
static void mapSelfTest(void)
{
  BQ24160Reg_StatusControl_t r0 = { .raw = 0 };
  r0.wr.tmr_rst = 1;                  ASSERT(r0.raw == BQ_WD_RESET_BIT);
  r0.raw = 0; r0.wr.supply_sel = 1;   ASSERT(r0.raw == BQ_PRECEDENCE_BIT);
  r0.raw = 0; r0.rd.status = 0x07;    ASSERT(r0.raw == BQ_STATUS_MASK);
  r0.raw = 0; r0.rd.fault = 0x07;     ASSERT(r0.raw == BQ_FAULT_MASK);

  BQ24160Reg_SupplyStatus_t r1 = { .raw = 0 };
  r1.rd_wr.en_nobatop = 1;  ASSERT(r1.raw == BQ_EN_NOBATOP_BIT);
  r1.raw = 0; r1.rd.bat_stat = 0x03;  ASSERT(r1.raw == BQ_BATSTAT_MASK);
  r1.raw = 0; r1.rd_wr.otg_lock = 1;    ASSERT(r1.raw == BQ_OTG_LOCK_BIT);
  r1.raw = 0; r1.rd.usb_status = 0x03;  ASSERT(r1.raw == BQ_USBSTAT_MASK);
  r1.raw = 0; r1.rd.in_status = 0x03;   ASSERT(r1.raw == BQ_INSTAT_MASK);

  BQ24160Reg_Control_t r2 = { .raw = 0 };
  r2.rd_wr.hz_mode = 1;       ASSERT(r2.raw == BQ_HIGH_IMPEDANCE_BIT);
  r2.raw = 0; r2.rd_wr.ce = 1;          ASSERT(r2.raw == BQ_CHG_DISABLE_BIT);
  r2.raw = 0; r2.rd_wr.te = 1;          ASSERT(r2.raw == BQ_TERM_ENABLE_BIT);
  r2.raw = 0; r2.rd_wr.en_stat = 1;     ASSERT(r2.raw == BQ_STAT_ENABLE_BIT);
  r2.raw = 0; r2.rd_wr.iusb_limit = 0x07;  ASSERT(r2.raw == BQ_IUSB_LIMIT_MASK);
  r2.raw = 0; r2.wr.reset = 1;       ASSERT(r2.raw == BQ_RESET_BIT);

  BQ24160Reg_BatVoltage_t r3 = { .raw = 0 };
  r3.rd_wr.dpdm_en = 1;     ASSERT(r3.raw == BQ_DPDM_EN_BIT);
  r3.raw = 0; r3.rd_wr.iinlimit = 1;    ASSERT(r3.raw == BQ_IINLIMIT_BIT);
  r3.raw = 0; r3.rd_wr.vbreg = 0x3F;    ASSERT(r3.raw == BQ_VBREG_MASK);

  BQ24160Reg_ChargeCurrent_t r5 = { .raw = 0 };
  r5.rd_wr.iterm = 0x07;    ASSERT(r5.raw == BQ_ITERM_MASK);
  r5.raw = 0; r5.rd_wr.ichrg = 0x1F;    ASSERT(r5.raw == BQ_ICHRG_MASK);

  BQ24160Reg_VinDpm_t r6 = { .raw = 0 };
  r6.rd_wr.vindpm_in = 0x07;   ASSERT(r6.raw == BQ_VINDPM_IN_MASK);
  r6.raw = 0; r6.rd_wr.vindpm_usb = 0x07;  ASSERT(r6.raw == BQ_VINDPM_USB_MASK);
  r6.raw = 0; r6.rd.dpm_status = 1;     ASSERT(r6.raw == BQ_DPM_STATUS_BIT);
  r6.raw = 0; r6.rd.minsys = 1;         ASSERT(r6.raw == BQ_MINSYS_STATUS_BIT);

  BQ24160Reg_SafetyNtc_t r7 = { .raw = 0 };
  r7.rd_wr.low_chg = 1;     ASSERT(r7.raw == BQ_HALF_CURRENT_BIT);
  r7.raw = 0; r7.rd.ts_fault = 0x03;  ASSERT(r7.raw == BQ_TS_FAULT_MASK);
  r7.raw = 0; r7.rd_wr.ts_en = 1;        ASSERT(r7.raw == BQ_TS_EN_BIT);
  r7.raw = 0; r7.rd_wr.tmr = 0x03;       ASSERT(r7.raw == BQ_TMR_MASK);
  r7.raw = 0; r7.rd_wr.tmr2x_en = 1;     ASSERT(r7.raw == BQ_2XTMR_EN_BIT);
}

uint8_t charger_SanitizeInputsConfig(uint8_t _config)
{
  return _config & ~(CHG_INCFG_PRECEDENCE | CHG_INCFG_GPIO_IN_EN);
}

void charger_Init(const BatteryProfile_T *_p_batt_profile,
                  uint8_t _in_cfg, uint8_t _chrg_cfg)
{
  LOG_INFO("[CHG] init: addr=%02X in_cfg=%02X chrg_cfg=%02X profile=%s", BQ_I2C_ADDR,
      _in_cfg, _chrg_cfg, (_p_batt_profile != NULL) ? "yes" : "none");

  mapSelfTest();

  // An argument, not charger_SetBatProfile(): the queue does not exist yet, so a posted event
  // would be dropped - which is what used to happen, leaving charging disabled.
  if (_p_batt_profile != NULL)
    s_Cfg.profile_valid = profileConvert(_p_batt_profile, &s_Cfg.profile);

  applyInputsConfig(_in_cfg);
  applyChargingConfig(_chrg_cfg);

  // The image has to exist before the task runs: bring-up writes it, and kickWatchdog() sends
  // register 0 out of it on every round.
  buildTargetImage(&s_Cfg);

  // Nothing has been read from the device yet, and the getters must not pretend otherwise.
  cacheInvalidate();
  publishUnknown();

  s_QueHandle = xQueueCreateStatic(sizeof(s_QueBuf) / sizeof(s_QueBuf[0]),
      sizeof(s_QueBuf[0]), (uint8_t*)s_QueBuf, &s_Que);
  ASSERT(s_QueHandle != NULL);

  s_TaskHandle = xTaskCreateStatic(Task, "CHG", sizeof(s_TaskStack) / sizeof(StackType_t),
      NULL, 6, s_TaskStack, &s_TaskTCB);
  ASSERT(s_TaskHandle != NULL);
}

void charger_NotifyFromISR(void)
{
  ChargerEvent_t ev = { .type = CHG_EV_IRQ };
  postEvent(&ev);
}

ChargerStatus_T charger_GetStatus(void)
{
  return (ChargerStatus_T)s_Pub.status;
}

bool charger_IsBatteryPresent(void)
{
  return s_Pub.battery_present != 0;
}

bool charger_IsInputPresent(void)
{
  return s_Pub.input_present != 0;
}

uint8_t charger_GetInStat(void)
{
  return s_Pub.in_stat;
}

uint8_t charger_GetUsbStat(void)
{
  return s_Pub.usb_stat;
}

bool charger_IsDpmModeActive(void)
{
  return s_Pub.dpm_active != 0;
}

uint8_t charger_GetTsFaultStatus(void)
{
  return s_Pub.ts_fault;
}

ChargerFaultStatus_T charger_GetFaultStatus(void)
{
  return (ChargerFaultStatus_T)s_Pub.fault;
}

uint8_t charger_GetI2cErrorCount(void)
{
  return s_I2cErrorCounter;
}

bool charger_IsNoBatteryTurnOnEnabled(void)
{
  return s_Cfg.no_battery_turnon != 0;
}

// Every setter but the profile carries a single byte.
static void postCmdU8(uint8_t _cmd, uint8_t _arg)
{
  ChargerEvent_t ev = { .type = CHG_EV_CMD };
  ev.data.cmd.id = _cmd;
  ev.data.cmd.arg.u8 = _arg;
  postEvent(&ev);
}

void charger_SetInputsConfig(uint8_t config)
{
  postCmdU8(CHG_CMD_SET_INPUTS_CONFIG, config);
}

void charger_SetChargingConfig(uint8_t config)
{
  postCmdU8(CHG_CMD_SET_CHARGING_CONFIG, config);
}

void charger_SetBatProfile(const BatteryProfile_T *batProfile)
{
  ChargerEvent_t ev = { .type = CHG_EV_CMD };
  ev.data.cmd.id = CHG_CMD_SET_BAT_PROFILE;

  // Converted in the caller's context, so the queue carries three codes. A profile that cannot
  // be honoured travels as "no profile", same as NULL.
  BqBattProfile_t profile = { 0 };
  ev.data.cmd.arg.batProfile.valid = (batProfile != NULL) && profileConvert(batProfile, &profile);
  ev.data.cmd.arg.batProfile.profile = profile;

  postEvent(&ev);
}

void charger_SetThermalState(BatteryThermalState_T state)
{
  postCmdU8(CHG_CMD_SET_THERMAL_STATE, (uint8_t)state);
}

uint32_t charger_GetErrMask(bool _clear)
{
  taskENTER_CRITICAL();
  uint32_t mask = s_ErrMask;
  if (_clear)
    s_ErrMask &= ~mask;
  taskEXIT_CRITICAL();
  return mask;
}

__attribute__((weak)) void charger_OnEvent_InputPresenceChanged(bool present)
{
  (void)present;
}

__attribute__((weak)) void charger_OnEvent_ValueChanged(uint8_t _type, uint8_t _value)
{
  (void)_type;
  (void)_value;
}
