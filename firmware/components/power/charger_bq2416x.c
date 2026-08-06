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
#include "utils/utils.h"
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
#define CHG_ACTIVE_PERIOD_MS    1000   // CHG_ST_ACTIVE: one read-and-sync round
#define CHG_INIT_RETRY_MS       1000   // CHG_ST_INIT: one bring-up attempt
#define CHG_RETRY_PERIOD_MS     30000  // CHG_ST_LOST: how often a written-off device is retried
#define CHG_POWER_ON_DELAY_MS   100    // the device needs this much after power on

#define CHG_INIT_ATTEMPTS       5      // bring-up tries before the device is declared absent
#define CHG_ERR_LIMIT           5      // consecutive failed rounds that drop ACTIVE to LOST

#define CHG_TASK_STACK_WORDS    256   // deepest measured frame is 56 bytes, see the .su file
#define CHG_QUE_LEN             8

_Static_assert(pdMS_TO_TICKS(CHG_ACTIVE_PERIOD_MS) >= 1,
               "CHG_ACTIVE_PERIOD_MS must be at least one RTOS tick");

typedef enum
{
  CHG_ST_INIT = 0,   // configuring the bq2416x
  CHG_ST_ACTIVE,     // device answering: read, publish, sync
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
// are deliberately absent: when one of those reads back wrong it is RegsMapSync()'s business, not
// a corrupted read, and freezing our own policy into a validity check would be a trap.
static const uint8_t s_InvariantMask[BQ_REG_COUNT]  = { 0x80, 0, 0x80, 0, 0xF8, 0, 0, 0 };
static const uint8_t s_InvariantValue[BQ_REG_COUNT] = { 0x00, 0, 0x80, 0, 0x40, 0, 0, 0 };

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

// Foreign profile -> the three codes, range-checked here and nowhere else.
// Currents saturate; battery.c's resistor profile really does reach 28 against a limit of 26.
// Voltage does not: capping it would charge an unknown pack to 4.76 V, so the profile is rejected
// and buildTargetImage() keeps charging disabled.
static bool ProfileConvert(const BatteryProfile_T *_src, BqBattProfile_t *_dst)
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

// The profile code, backed off by 140 mV above tWarm. WARM and HOT both sit above it.
// Zero without a profile - charging is disabled in that case anyway, see the CE bit below.
static uint8_t RegulationVoltage(const ChargerConfig_t *_p_cfg)
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

// The one place s_Pub is written and the one place a charger event is posted.
// Store before post, so a handler that calls a getter straight away reads the new value.
static void PublishValue(uint8_t *_p_dst, uint8_t _val, uint8_t _evt_type, const char *_name)
{
  if (*_p_dst == _val)
    return;

  LOG_INFO("[CHG] Publish <%s>: %u->%u", _name, (unsigned)*_p_dst, (unsigned)_val);
  *_p_dst = _val;

  AppEvent_t evt = { .type = _evt_type, .data.chargerValue = { _val } };
  app_PostEvent(&evt);
}

// Same compare-store-post, older payload - these two already have consumers.
static void PublishPresence(uint8_t *_p_dst, bool _present, uint8_t _evt_type, const char *_name)
{
  if (*_p_dst == (uint8_t)_present)
    return;

  LOG_INFO("[CHG] Publish presense <%s>: %u->%u", _name, (unsigned)*_p_dst, (unsigned)_present);
  *_p_dst = (uint8_t)_present;

  AppEvent_t evt = { .type = _evt_type };
  if (_evt_type == APP_EVT_CHARGER_INPUT_PRESENCE)
    evt.data.chargerInput.present = _present;
  else
    evt.data.batteryPresence.present = _present;

  app_PostEvent(&evt);
}

// Take the decoded values as the published ones, telling APP about each that moved.
static void PublishChanges(const ChargerPublished_t *_p_next)
{
  PublishValue(&s_Pub.status,      _p_next->status,      APP_EVT_CHARGER_STATUS,      "status");
  PublishValue(&s_Pub.fault,       _p_next->fault,       APP_EVT_CHARGER_FAULT,       "fault");
  PublishValue(&s_Pub.in_stat,     _p_next->in_stat,     APP_EVT_CHARGER_IN_STAT,     "instat");
  PublishValue(&s_Pub.usb_stat,    _p_next->usb_stat,    APP_EVT_CHARGER_USB_STAT,    "usbstat");
  PublishValue(&s_Pub.ts_fault,    _p_next->ts_fault,    APP_EVT_CHARGER_TS_FAULT,    "tsfault");
  PublishValue(&s_Pub.dpm_active,  _p_next->dpm_active,  APP_EVT_CHARGER_DPM,         "dpm");

  PublishPresence(&s_Pub.input_present, _p_next->input_present != 0,
      APP_EVT_CHARGER_INPUT_PRESENCE, "input");
  PublishPresence(&s_Pub.battery_present, _p_next->battery_present != 0,
      APP_EVT_BATTERY_PRESENCE, "battery");
}

// Nothing is known about the device. Same publisher as a normal round, so losing it announces
// itself on every value rather than only on the two presence flags.
static void PublishUnknown(void)
{
  ChargerPublished_t next = {
      .status = CHG_NA,
      .fault = CHG_FAULT_UNKNOWN };

  PublishChanges(&next);
}

// CHG_INCFG_PRECEDENCE and CHG_INCFG_GPIO_IN_EN are deliberately not read - both are fixed by the
// board, see BQ_PRECEDENCE_IMAGE and BQ_OTG_LOCK_IMAGE.
static void SetInputsConfig(uint8_t _config)
{
  s_Cfg.no_battery_turnon = (_config & CHG_INCFG_NO_BAT_TURNON) != 0;
  s_Cfg.in_current_limit  = (_config & CHG_INCFG_IN_ILIM) != 0;
  s_Cfg.in_dpm            = (_config & CHG_INCFG_IN_DPM_MASK) >> CHG_INCFG_IN_DPM_SHIFT;

  LOG_INFO("[CHG] Set inputs cfg: 0x%02X, no_bat_turnon=%u, in_ilim=%u, in_dpm=%u",
      _config, (unsigned)s_Cfg.no_battery_turnon,
      (unsigned)s_Cfg.in_current_limit, (unsigned)s_Cfg.in_dpm);
}

static void SetChargingConfig(uint8_t _config)
{
  s_Cfg.charging_enabled = (_config & CHG_CHGCFG_ENABLE) != 0;
  LOG_INFO("[CHG] Set charging cfg: 0x%02X, enabled=%u", _config, (unsigned)s_Cfg.charging_enabled);
}

// Snapshot in, values out. Touches no static, posts no event.
// Each register is read into a local once, so no value mixes two moments of a sweep.
// Returns rather than fills: every member is set here, and the caller cannot forget that.
static ChargerPublished_t DevRegsDecode(void)
{
  BQ24160Reg_StatusControl_t reg_status = s_DeviceRegs.reg.status_control;
  BQ24160Reg_SupplyStatus_t reg_supply = s_DeviceRegs.reg.supply_status;
  ChargerPublished_t decoded;

  decoded.status = reg_status.rd.status;
  decoded.fault = reg_status.rd.fault;
  decoded.in_stat = reg_supply.rd.in_status;
  decoded.usb_stat = reg_supply.rd.usb_status;
  decoded.dpm_active = (s_DeviceRegs.reg.vin_dpm.rd.dpm_status != 0);
  decoded.ts_fault = s_DeviceRegs.reg.safety_ntc.rd.ts_fault;

  // Present when the status names a source: "ready" through "done", nothing outside that.
  decoded.input_present = (decoded.status > CHG_NO_VALID_SOURCE) && (decoded.status < CHG_NA);

  // Half the story - battery.c combines it with the pack voltage.
  decoded.battery_present = (reg_supply.rd.bat_stat != BQ_BATSTAT_NOT_PRESENT);

  return decoded;
}

static void DevRegsInvalidate(void)
{
  for (uint8_t r = 0; r < BQ_REG_COUNT; r++)
    s_DeviceRegs.raw[r] = 0;
}

// Configuration -> the image the device should hold. Takes the configuration as an argument
// rather than reaching for s_Cfg, so what it depends on is in the signature: it writes
// s_TargetRegs and never touches the bus.
//
// Called at init and whenever the configuration changes, not every round. The image is ours and
// lives in RAM, so nothing the device does to itself - a watchdog expiry, a DEFAULT mode entry -
// can disturb it. Noticing that the device has drifted away from it is RegsMapSync()'s job.
static void TargetRegsBuild(const ChargerConfig_t *_p_cfg)
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
  s_TargetRegs.reg.bat_voltage.rd_wr.vbreg = RegulationVoltage(_p_cfg);

  // Register 4 is read only - its writable mask is empty, so RegsMapSync() skips it by itself.

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
  // resumes charging and clears the fault - which RegsMapSync() would do a second later.
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

  LOG_INFO("[CHG] target image: %02X %02X %02X %02X %02X %02X %02X, charging %s",
      s_TargetRegs.raw[0], s_TargetRegs.raw[1], s_TargetRegs.raw[2], s_TargetRegs.raw[3],
      s_TargetRegs.raw[5], s_TargetRegs.raw[6], s_TargetRegs.raw[7], allow ? "allowed" : "off");

  if (!allow)
    LOG_INFO("[CHG] charging off: profile=%u enabled=%u thermal=%u",
        (unsigned)_p_cfg->profile_valid, (unsigned)_p_cfg->charging_enabled,
        (unsigned)_p_cfg->thermal_state);
}

// One read, on purpose. Reading twice and comparing cannot tell "the device changed the value"
// from "the value came back corrupted", and registers 0, 1, 6 and 7 carry live status bits - so
// every input transition produced a false mismatch and cost a whole round. Both cases cost one
// round anyway: a bad status value is replaced a second later, a bad configuration value is
// caught by the write verify. It also halves the traffic on a bus shared with the fuel gauge.
static bool RegRead(uint8_t _reg)
{
  uint8_t value;

  if (i2c_master_ReadMem(BQ_I2C_ADDR, _reg, &value, 1) != I2C_OK)
  {
    return false;
  }

  // One bad bit means the whole byte is untrustworthy - drop it rather than decode it. The
  // snapshot then has a hole, so regReadAll() ends the round and nothing is published from it.
  uint8_t mask = s_InvariantMask[_reg];
  if ((value & mask) != s_InvariantValue[_reg])
  {
    LOG_ERROR("[CHG] REG[%u] invalid value: 0x%02X, expected 0x%02X within mask 0x%02X",
        (unsigned)_reg, value, s_InvariantValue[_reg], mask);
    return false;
  }

  s_DeviceRegs.raw[_reg] = value;
  return true;
}

// Writes and verifies over the writable bits only.
static bool RegWrite(uint8_t _reg)
{
  LOG_DEBUG("[CHG] REG[%u] write 0x%02X -> 0x%02X (mask 0x%02X)", (unsigned)_reg,
      s_DeviceRegs.raw[_reg], s_TargetRegs.raw[_reg], s_WritableMask[_reg]);

  if (i2c_master_WriteMem(BQ_I2C_ADDR, _reg, &s_TargetRegs.raw[_reg], 1) != I2C_OK)
  {
    LOG_ERROR("[CHG] REG[%u] write failed", (unsigned)_reg);
    return false;
  }

  if (!RegRead(_reg))
    return false;

  uint8_t mask = s_WritableMask[_reg];
  if ((s_DeviceRegs.raw[_reg] & mask) != (s_TargetRegs.raw[_reg] & mask))
  {
    LOG_ERROR("[CHG] REG[%u] verify failed: wrote 0x%02X, read 0x%02X (mask 0x%02X)", (unsigned)_reg,
        s_TargetRegs.raw[_reg], s_DeviceRegs.raw[_reg], mask);
    return false;
  }

  return true;
}

static bool RegsMapRead(void)
{
  bool res = true;

  for (uint8_t r = 0; r < BQ_REG_COUNT; r++)
  {
    if (!RegRead(r))
      res = false;
  }

  return res;
}

// WDT reset via read-modify-write:
static void ResetWatchdog(void)
{
  // Read:
  if (!RegRead(BQ_REG_STATUS_CONTROL))
  {
    LOG_ERROR("[CHG] watchdog not reset: register 0 unreadable");
    return;
  }

  // Modify:
  BQ24160Reg_StatusControl_t r0 = s_DeviceRegs.reg.status_control;
  r0.wr.tmr_rst = 1;

  // Write:
  int i2c_res = i2c_master_WriteMem(BQ_I2C_ADDR, BQ_REG_STATUS_CONTROL, &r0.raw, 1);
  if (i2c_res != I2C_OK)
    LOG_ERROR("[CHG] watchdog reset failed");
}

// Push every register the device is not already holding:
static bool RegsMapSync(void)
{
  bool res = true;
  for (uint8_t reg = 0; reg < BQ_REG_COUNT; reg++)
  {
    uint8_t mask = s_WritableMask[reg];
    if ((s_TargetRegs.raw[reg] & mask) == (s_DeviceRegs.raw[reg] & mask))
      continue;

    bool res_wr = RegWrite(reg);
    if (!res_wr)
    {
      res = false;
      break;
    }
  }

  return res;
}

// One standalone operation, then a round in four steps. A snapshot that could not be taken ends
// it: nothing is published, so the values keep standing until either the next round succeeds or
// CHG_ST_LOST blanks them.
static bool DevRound(void)
{
  // 0. Reset ICs watchdog timer:
  ResetWatchdog();

  // 1. Get snapshot:
  if (!RegsMapRead())
    return false;

  LOG_VERBOSE("[CHG] regs: %02X %02X %02X %02X %02X %02X %02X %02X",
      s_DeviceRegs.raw[0], s_DeviceRegs.raw[1], s_DeviceRegs.raw[2], s_DeviceRegs.raw[3],
      s_DeviceRegs.raw[4], s_DeviceRegs.raw[5], s_DeviceRegs.raw[6], s_DeviceRegs.raw[7]);

  // 2. Snapshot -> values, no side effects:
  ChargerPublished_t decoded = DevRegsDecode();

  // 3. Compare and publish:
  PublishChanges(&decoded);

  // 4. write back what differs
  return RegsMapSync();
}

// Takes one command's value into the configuration and rebuilds the target image from it. That
// is the whole of "configuring": nothing here touches the bus, and getting the device to hold the
// new image is RegsMapSync()'s business on the round that follows.
static void CmdProcess(const ChargerEvent_t *_ev)
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
      SetInputsConfig(_ev->data.cmd.arg.u8);
    break;

    case CHG_CMD_SET_CHARGING_CONFIG:
      SetChargingConfig(_ev->data.cmd.arg.u8);
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

  // Update target registers:
  TargetRegsBuild(&s_Cfg);
}

// Every state lists every event it answers to; nothing hides behind a shared default.
// "break" stays put, "return" names the state to move to.
// CHG_EV_ENTRY is delivered by fsmDispatch() the moment the transition is made.
static ChargerState_t state_Init(const ChargerEvent_t *_ev)
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

      // A round is all bring-up ever was: read, publish, write the target image back. The device
      // identity is checked inside RegRead() against s_InvariantMask[BQ_REG_VENDOR], so a round
      // that completes is itself the proof that a bq24160 is answering.
      if (DevRound())
      {
        LOG_INFO("[CHG] up: rev=%u", (unsigned)s_DeviceRegs.reg.vendor.rd.revision);
        return CHG_ST_ACTIVE;
      }

      LOG_WARNING("[CHG] bring-up attempt %u/%u failed",
          (unsigned)(init_attempts + 1), (unsigned)CHG_INIT_ATTEMPTS);
      if (++init_attempts >= CHG_INIT_ATTEMPTS)
        return CHG_ST_LOST;
    break;

    case CHG_EV_IRQ:
    break;   // the retry below reads and publishes everything anyway

    case CHG_EV_CMD:
      // Only taken in: the image is rebuilt here, the round that follows pushes it.
      CmdProcess(_ev);
    break;

    default:
      LOG_WARNING("[CHG] unhandled event %u in INIT", (unsigned)_ev->type);
    break;
  }

  return CHG_ST_INIT;
}

static ChargerState_t state_Active(const ChargerEvent_t *_ev)
{
  static uint8_t err_count;

  switch (_ev->type)
  {
    // The one event that does not end in a round - INIT's round already read and published.
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
      CmdProcess(_ev);
    break;

    default:
      LOG_WARNING("[CHG] unhandled event %u in ACTIVE", (unsigned )_ev->type);
      return CHG_ST_ACTIVE; // no reason to touch the bus over an event we do not know
  }

  // Everything else ends the same way: push whatever changed into the device now.
  if (DevRound())
  {
    err_count = 0;
    return CHG_ST_ACTIVE;
  }

  if (++err_count >= CHG_ERR_LIMIT)
  {
    LOG_ERROR("[CHG] %u failed rounds in a row", (unsigned)err_count);
    return CHG_ST_LOST;
  }

  LOG_WARNING("[CHG] round failed %u/%u", (unsigned)err_count, (unsigned)CHG_ERR_LIMIT);

  return CHG_ST_ACTIVE;
}

static ChargerState_t state_Lost(const ChargerEvent_t *_ev)
{
  switch (_ev->type)
  {
    case CHG_EV_ENTRY:
      s_StateTimeout = pdMS_TO_TICKS(CHG_RETRY_PERIOD_MS);

      // Stop answering from a snapshot we can no longer refresh, and tell APP now - this is the
      // moment the input and the pack stopped being visible.
      DevRegsInvalidate();
      PublishUnknown();

      // No watchdog kick from here on: if we cannot talk to the device, the best it can do is
      // time out and fall back to its own defaults.
      LOG_ERROR("[CHG] state LOST, device unreachable, retry in %u ms",
          (unsigned)CHG_RETRY_PERIOD_MS);
    break;

    case CHG_EV_TICK:
      return CHG_ST_INIT;   // periodic recovery attempt

    case CHG_EV_IRQ:
      // It raised its interrupt line, so it is alive - no reason to sit out the backoff.
      LOG_INFO("[CHG] interrupt from a device believed lost, retrying now");
      return CHG_ST_INIT;

    case CHG_EV_CMD:
      // Nothing reaches the device while it is written off - the recovery attempt applies it all.
      CmdProcess(_ev);
    break;

    default:
      LOG_WARNING("[CHG] unhandled event %u in LOST", (unsigned )_ev->type);
    break;
  }

  return CHG_ST_LOST;
}

// The only place that maps a state to its handler.
static ChargerState_t fsm_Handle(ChargerState_t _state, const ChargerEvent_t *_ev)
{
  switch (_state)
  {
    case CHG_ST_INIT:   return state_Init(_ev);
    case CHG_ST_ACTIVE: return state_Active(_ev);
    case CHG_ST_LOST:   return state_Lost(_ev);
    default:            APP_ERROR(APP_ERR); return _state;
  }
}

// The only place that assigns the state. Handlers decide, this moves.
// An entry action may itself ask to move on, hence the loop; the guard catches a chain that
// does not settle.
static void fsm_Dispatch(const ChargerEvent_t *_ev)
{
  static ChargerState_t state = CHG_ST_INIT;

  ChargerState_t next = fsm_Handle(state, _ev);

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
    next = fsm_Handle(state, &entryEv);
  }
}

static void Task(void *_parameters)
{
  (void)_parameters;

  LOG_INFO("[CHG] task started, %u ms power-on delay", (unsigned)CHG_POWER_ON_DELAY_MS);

  vTaskDelay(pdMS_TO_TICKS(CHG_POWER_ON_DELAY_MS)); // the device needs time after power on

  // Entry event to start FSM:
  const ChargerEvent_t entryEv = { .type = CHG_EV_ENTRY };
  fsm_Dispatch(&entryEv);

  while (1)
  {
    ChargerEvent_t ev;
    if (xQueueReceive(s_QueHandle, &ev, s_StateTimeout) != pdTRUE)
      ev.type = CHG_EV_TICK;

    fsm_Dispatch(&ev);
  }
}

static bool PostEvent(const ChargerEvent_t *_ev)
{
  bool sent = utils_PostEvent(s_QueHandle, _ev);
  if (!sent)
  {
    LOG_CRITICAL("[CHG] event queue full, ev=%u dropped", _ev->type);
    taskENTER_CRITICAL();
    s_ErrMask |= CHARGER_ERR_QUEUE_FULL;
    taskEXIT_CRITICAL();
  }

  return sent;
}

// Every setter but the profile carries a single byte.
static void PostCmd(uint8_t _cmd, uint8_t _arg)
{
  ChargerEvent_t ev = { .type = CHG_EV_CMD };
  ev.data.cmd.id = _cmd;
  ev.data.cmd.arg.u8 = _arg;
  PostEvent(&ev);
}

void charger_Init(const BatteryProfile_T *_p_batt_profile,
                  uint8_t _in_cfg, uint8_t _chrg_cfg)
{
  LOG_INFO("[CHG] init: addr=0x%02X in_cfg=0x%02X chrg_cfg=0x%02X profile=%s", BQ_I2C_ADDR,
      _in_cfg, _chrg_cfg, (_p_batt_profile != NULL) ? "yes" : "none");

  // An argument, not charger_SetBatProfile(): the queue does not exist yet, so a posted event
  // would be dropped - which is what used to happen, leaving charging disabled.
  if (_p_batt_profile != NULL)
    s_Cfg.profile_valid = ProfileConvert(_p_batt_profile, &s_Cfg.profile);

  SetInputsConfig(_in_cfg);
  SetChargingConfig(_chrg_cfg);

  // The image has to exist before the task runs - the first round writes it.
  TargetRegsBuild(&s_Cfg);

  // Nothing has been read from the device yet, and the getters must not pretend otherwise.
  DevRegsInvalidate();
  PublishUnknown();

  s_QueHandle = xQueueCreateStatic(sizeof(s_QueBuf) / sizeof(s_QueBuf[0]),
      sizeof(s_QueBuf[0]), (uint8_t*)s_QueBuf, &s_Que);
  ASSERT(s_QueHandle != NULL);

  s_TaskHandle = xTaskCreateStatic(Task, "CHG", sizeof(s_TaskStack) / sizeof(StackType_t),
      NULL, 6, s_TaskStack, &s_TaskTCB);
  ASSERT(s_TaskHandle != NULL);
}

uint8_t charger_SanitizeInputsConfig(uint8_t _config)
{
  return _config & ~(CHG_INCFG_PRECEDENCE | CHG_INCFG_GPIO_IN_EN);
}

void charger_NotifyFromISR(void)
{
  ChargerEvent_t ev = { .type = CHG_EV_IRQ };
  PostEvent(&ev);
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

bool charger_IsNoBatteryTurnOnEnabled(void)
{
  return s_Cfg.no_battery_turnon != 0;
}

void charger_SetInputsConfig(uint8_t _config)
{
  PostCmd(CHG_CMD_SET_INPUTS_CONFIG, _config);
}

void charger_SetChargingConfig(uint8_t _config)
{
  PostCmd(CHG_CMD_SET_CHARGING_CONFIG, _config);
}

void charger_SetBatProfile(const BatteryProfile_T *batProfile)
{
  ChargerEvent_t ev = { .type = CHG_EV_CMD };
  ev.data.cmd.id = CHG_CMD_SET_BAT_PROFILE;

  // Converted in the caller's context, so the queue carries three codes. A profile that cannot
  // be honoured travels as "no profile", same as NULL.
  BqBattProfile_t profile = { 0 };
  ev.data.cmd.arg.batProfile.valid = (batProfile != NULL) && ProfileConvert(batProfile, &profile);
  ev.data.cmd.arg.batProfile.profile = profile;

  PostEvent(&ev);
}

void charger_SetThermalState(BatteryThermalState_T state)
{
  PostCmd(CHG_CMD_SET_THERMAL_STATE, (uint8_t)state);
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
