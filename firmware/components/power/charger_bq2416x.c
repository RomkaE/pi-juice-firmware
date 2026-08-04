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
#include "driver/i2c/i2c_master.h"
#include "app-error/app_assert.h"
#include "src/app.h"
#include "utils/time_count.h"   // DelayUs() only - the MS_TIME_COUNT timers are gone

// FreeRTOS:
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

// LOG:
#include "log/log.h"

/*============================ DEVICE ========================================*/

#define BQ_I2C_ADDR             0xD6
#define BQ_REG_COUNT            8

#define BQ_REG_STATUS_CONTROL   0   // charger status, watchdog reset bit
#define BQ_REG_SUPPLY_STATUS    1   // IN/USB status, OTG lockout, BATSTAT
#define BQ_REG_CONTROL          2   // USB current limit, charge enable, termination
#define BQ_REG_BAT_VOLTAGE      3   // regulation voltage, IN current limit
#define BQ_REG_VENDOR           4   // read only
#define BQ_REG_CHARGE_CURRENT   5   // fast charge and termination current
#define BQ_REG_VIN_DPM          6
#define BQ_REG_SAFETY_NTC       7   // safety timer, thermal regulation, TS

#define BQ_WD_RESET_BIT         0x80  // register 0
#define BQ_OTG_LOCK_BIT         0x08  // register 1
#define BQ_BATSTAT_MASK         0x06  // register 1
#define BQ_BATSTAT_ABSENT       0x04
#define BQ_CHG_DISABLE_BIT      0x02  // register 2
#define BQ_HIGH_IMPEDANCE_BIT   0x01  // register 2 - never set, see chargerBringUp()
#define BQ_TERM_ENABLE_BIT      0x04  // register 2
#define BQ_HALF_CURRENT_BIT     0x01  // register 7

#define BQ_VIN_DPM_USB          6

/*============================ TUNING ========================================*/

/*
 * One cadence, not two. The old version woke every 20 ms to run the whole update pipeline and
 * read a single register every 90 ms, so a full register sweep took 720 ms and the cache was
 * always partly stale. Now the whole set is read once per tick and the device watchdog - which
 * expires after 32 s - is kicked on the same tick, so there is no second timer to keep in sync.
 *
 * CHG_INT still wakes the task immediately: the edge is posted into the same queue as everything
 * else, so a status change is not waited out.
 */
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

/*============================ STATE MACHINE =================================*/

/*
 *            bring-up ok                CHG_ERR_LIMIT failed rounds
 *   INIT ------------------> ACTIVE ------------------------------+
 *    ^ |                                                          |
 *    | | CHG_INIT_ATTEMPTS failed tries                           |
 *    | v                                                          v
 *    +- LOST <---------------------------------------------------+
 *       CHG_RETRY_PERIOD_MS, or a CHG_INT edge -> INIT
 *
 * Same shape as fuel_gauge_lc709203f.c: the task waits for an event and hands it over,
 * fsmDispatch() picks the handler for the current state, and the handler answers with the state
 * to be in next - it never switches state itself. s_State is assigned in exactly one place.
 *
 * There is no "no battery" state: the charger works, and must work, with no pack fitted - that is
 * how the board runs from an input source.
 */
typedef enum
{
  CHG_ST_INIT = 0,   // configuring the bq2416x
  CHG_ST_ACTIVE,     // device answering: read, reconcile, publish
  CHG_ST_LOST        // device unreachable, waiting to retry
} ChargerState_t;

typedef enum
{
  CHG_EV_TICK = 1,   // the state's own queue timeout expired
  CHG_EV_IRQ,        // CHG_INT edge, posted straight from the EXTI interrupt
  CHG_EV_CMD         // a parameter was set, see ChargerCmdType_t
} ChargerEventType_t;

/* What each command carries in ChargerCmd_t.arg - all one byte except the profile. */
typedef enum
{
  CHG_CMD_SET_BAT_PROFILE = 1,  // arg.batProfile
  CHG_CMD_SET_INPUTS_CONFIG,    // arg.u8 - configuration byte
  CHG_CMD_SET_CHARGING_CONFIG,  // arg.u8 - configuration byte
  CHG_CMD_SET_THERMAL_STATE,    // arg.u8 - BatteryThermalState_T
  CHG_CMD_SET_5V_IN_DETECTED,   // arg.u8 - boolean
  CHG_CMD_REEVAL_USB_LOCKOUT,   // carries nothing
  CHG_CMD_SET_USB_ILIM          // arg.u8 - ChargerUsbILimStep_T
} ChargerCmdType_t;

/* Payload of a CHG_EV_CMD event: which parameter, and its value. */
typedef struct
{
  ChargerCmdType_t id;
  union
  {
    uint8_t u8;
    struct
    {
      bool valid;
      BatteryProfile_T profile;
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

static ChargerState_t s_State = CHG_ST_INIT;
/*
 * Queue wait of the current state. Set on entry by fsmEnter(); a state may re-arm its own - that
 * is what INIT does to get its first attempt done immediately and pace only the retries.
 */
static TickType_t s_StateTimeout;
static uint8_t s_InitAttempts;   // CHG_ST_INIT
static uint8_t s_ErrCount;       // CHG_ST_ACTIVE, consecutive failed rounds

/*============================ REGISTER CACHE ================================*/

/*
 * Touched only by the CHG task - and now that really is true: the setters below queue an event
 * instead of writing registers in the caller's context.
 *
 * s_Regs is what the device holds, refreshed in full once per round. s_Regsw is what we want it
 * to hold. s_RegswMask marks the bits that are actually writable, and is used both to decide
 * whether a write is needed and to verify it afterwards.
 */
static uint8_t s_Regs[BQ_REG_COUNT]  = {0x00, 0x00, 0x8C, 0x14, 0x40, 0x32, 0x00, 0x98};
static uint8_t s_Regsw[BQ_REG_COUNT] = {0x08, 0x08, 0x1C, 0x02, 0x00, 0x00, 0x38, 0xC0};

/*
 * Register 1 was 0x0A here, which included BATSTAT bit 1 - a read-only status bit. Verifying a
 * write against it made every register 1 write report failure while the pack was in over-voltage.
 * 0x09 is what the datasheet lists as writable: OTG lockout and no-battery operation.
 */
static const uint8_t s_RegswMask[BQ_REG_COUNT] = {0x08, 0x09, 0x7F, 0xFF, 0x00, 0xFF, 0x3F, 0xE9};

static uint8_t s_I2cErrorCounter;   // bus level, published through charger_GetI2cErrorCount()

/*============================ PUBLISHED STATE ===============================*/

static ChargerStatus_T s_ChargerStatus = CHG_NA;
static ChargerUSBInLockoutStatus_T s_UsbInLockoutStatus = CHG_USB_IN_UNKNOWN;
static ChargerUsbInCurrentLimit_T s_UsbInCurrentLimit = CHG_IUSB_LIMIT_150MA;

static bool s_PowerSourcePresent;
static bool s_BatteryPresent;

/*============================ TOLD BY APP ===================================*/

static BatteryProfile_T s_BatProfile;
static bool s_BatProfileValid;
static BatteryThermalState_T s_ThermalState = BAT_TEMP_UNKNOWN;

/*
 * Nothing sets this while power_source.c is dormant, so USB-IN charging stays locked out. Same
 * behaviour as the old extern pwr5vInDetStatus, but now an explicit parameter rather than a
 * silent consequence of dead code.
 */
static bool s_5vInDetected;

static uint8_t s_ChargerInputsConfig;
static uint8_t s_UsbInEnabled;
static uint8_t s_NoBatteryTurnOn;
static uint8_t s_ChargerInputsPrecedence;
static uint8_t s_ChargerInLimit; // 0 - 1.5A, 1 - 2.5A
static uint8_t s_ChargerInDpm;
static uint8_t s_ChargingConfig;
static uint8_t s_ChargingEnabled;

/*
 * bq2416x register 1 bit 0 (no-battery operation). Nothing has ever assigned it, so it is a
 * constant zero - kept as a named field because s_NoBatteryTurnOn from the inputs configuration
 * looks like it was meant to drive it. Pre-existing, not changed here.
 */
static uint8_t s_NoBatteryOperationEnabled;

static uint32_t s_ErrMask;  // CHARGER_ERR_* bits

/*============================ RTOS OBJECTS ==================================*/

static TaskHandle_t s_TaskHandle;
static StaticTask_t s_TaskTCB;
static StackType_t s_TaskStack[CHG_TASK_STACK_WORDS];

static QueueHandle_t s_QueHandle;
static StaticQueue_t s_Que;
static ChargerEvent_t s_QueBuf[CHG_QUE_LEN];

static bool postEvent(const ChargerEvent_t *_ev);

/*============================ BUS ===========================================*/

static void countBusError(void)
{
  if (s_I2cErrorCounter < 0xFF)
    s_I2cErrorCounter++;

  /* Many soft failures - transport or a persistent read-back mismatch - mean the bus itself is
   * wedged. Kick the master once, then carry on counting from a clean slate. */
  if (s_I2cErrorCounter > CHG_I2C_ERR_REINIT) {
    LOG_WARNING("[CHG] I2C master re-init after %u errors", (unsigned)s_I2cErrorCounter);
    i2c_master_ReInit();
    s_I2cErrorCounter = 1;
  }
}

/* Reads one register twice and only accepts it when both reads agree. */
static bool regRead(uint8_t _reg)
{
  uint8_t v1, v2;

  if (i2c_master_ReadMem(BQ_I2C_ADDR, _reg, &v1, 1) != I2C_OK) {
    countBusError();
    return false;
  }

  v2 = ~v1;
  if (i2c_master_ReadMem(BQ_I2C_ADDR, _reg, &v2, 1) != I2C_OK) {
    countBusError();
    return false;
  }

  if (v1 != v2) {
    countBusError();   // the two reads disagree: noise on the line
    return false;
  }

  s_Regs[_reg] = v2;
  s_I2cErrorCounter = 0;
  return true;
}

/* Writes the desired image and verifies it came back, over the writable bits only. */
static bool regWrite(uint8_t _reg)
{
  if (i2c_master_WriteMem(BQ_I2C_ADDR, _reg, &s_Regsw[_reg], 1) != I2C_OK) {
    countBusError();
    return false;
  }

  if (!regRead(_reg))
    return false;

  uint8_t mask = s_RegswMask[_reg];
  if ((s_Regs[_reg] & mask) != (s_Regsw[_reg] & mask)) {
    countBusError();
    return false;
  }

  s_I2cErrorCounter = 0;
  return true;
}

/*
 * Writes only when the device does not already hold what we want. The cache is refreshed in full
 * at the top of every round, so this needs no re-read of its own - that is what the per-register
 * "dirty" flags used to be for.
 */
static bool regSyncIfDiffers(uint8_t _reg)
{
  uint8_t mask = s_RegswMask[_reg];

  if ((s_Regsw[_reg] & mask) == (s_Regs[_reg] & mask))
    return true;

  return regWrite(_reg);
}

static bool regReadAll(void)
{
  bool ok = true;

  for (uint8_t r = 0; r < BQ_REG_COUNT; r++) {
    if (!regRead(r))
      ok = false;
  }

  return ok;
}

/*
 * The reset bit must stay 0 in the write image, otherwise every other write to register 0 would
 * also reset the timer. Kicked once per round; the device allows 32 s.
 */
static void kickWatchdog(void)
{
  s_Regsw[BQ_REG_STATUS_CONTROL] = s_ChargerInputsPrecedence << 3;

  uint8_t v = s_Regsw[BQ_REG_STATUS_CONTROL] | BQ_WD_RESET_BIT;
  if (i2c_master_WriteMem(BQ_I2C_ADDR, BQ_REG_STATUS_CONTROL, &v, 1) != I2C_OK)
    countBusError();
}

/*============================ REGISTER RECONCILIATION =======================*/

static bool syncUsbLockout(void)
{
  s_Regsw[BQ_REG_SUPPLY_STATUS] = (s_NoBatteryOperationEnabled != 0);

  if (s_UsbInEnabled && s_5vInDetected
      && (s_Regs[BQ_REG_SUPPLY_STATUS] & BQ_BATSTAT_MASK) == 0x00)
    s_Regsw[BQ_REG_SUPPLY_STATUS] &= ~BQ_OTG_LOCK_BIT;
  else
    s_Regsw[BQ_REG_SUPPLY_STATUS] |= BQ_OTG_LOCK_BIT;

  s_UsbInLockoutStatus = CHG_USB_IN_UNKNOWN;

  if (!regSyncIfDiffers(BQ_REG_SUPPLY_STATUS))
    return false;

  s_UsbInLockoutStatus = (s_Regs[BQ_REG_SUPPLY_STATUS] & BQ_OTG_LOCK_BIT)
      ? CHG_USB_IN_LOCK : CHG_USB_IN_UNLOCK;
  return true;
}

static bool syncControl(void)
{
  // usb in current limit code, Enable STAT output, Enable charge current termination
  s_Regsw[BQ_REG_CONTROL] = ((s_UsbInCurrentLimit & 0x07) << 4) | 0x0C;

  bool allow = s_BatProfileValid && s_ChargingEnabled
            && s_ThermalState != BAT_TEMP_COLD && s_ThermalState != BAT_TEMP_HOT;

  if (allow) {
    s_Regsw[BQ_REG_CONTROL] &= ~BQ_CHG_DISABLE_BIT;
    s_Regsw[BQ_REG_CONTROL] |= BQ_TERM_ENABLE_BIT;
  } else {
    s_Regsw[BQ_REG_CONTROL] |= BQ_CHG_DISABLE_BIT;
  }
  s_Regsw[BQ_REG_CONTROL] &= ~BQ_HIGH_IMPEDANCE_BIT;

  return regSyncIfDiffers(BQ_REG_CONTROL);
}

static bool syncRegulationVoltage(void)
{
  // Bit 0 is not ours: keep whatever the image already carries.
  s_Regsw[BQ_REG_BAT_VOLTAGE] = (s_Regsw[BQ_REG_BAT_VOLTAGE] & 0x01) | (s_ChargerInLimit << 1);

  /* With no profile there is no voltage to ask for, and writing the register would set the
   * regulation voltage to its minimum - so leave the device alone. */
  if (!s_BatProfileValid)
    return true;

  int16_t regVolt = s_BatProfile.regulationVoltage;
  if (s_ThermalState >= BAT_TEMP_WARM) {
    // WARM and HOT both sit above tWarm: back the regulation voltage off by 140 mV
    regVolt -= (140 / 20);
    if (regVolt < 0)
      regVolt = 0;
  }
  s_Regsw[BQ_REG_BAT_VOLTAGE] |= regVolt << 2;

  return regSyncIfDiffers(BQ_REG_BAT_VOLTAGE);
}

static bool syncTempRegulation(void)
{
  // Timer slowed by 2x when in thermal regulation, 10-9 hour fast charge, TS function disabled
  s_Regsw[BQ_REG_SAFETY_NTC] = 0xC0;

  if (!s_BatProfileValid)
    return true;

  // COLD and COOL both sit below tCool: halve the charge current
  if (s_ThermalState != BAT_TEMP_UNKNOWN && s_ThermalState <= BAT_TEMP_COOL)
    s_Regsw[BQ_REG_SAFETY_NTC] |= BQ_HALF_CURRENT_BIT;

  return regSyncIfDiffers(BQ_REG_SAFETY_NTC);
}

static bool syncChargeCurrent(void)
{
  if (!s_BatProfileValid) {
    s_Regsw[BQ_REG_CHARGE_CURRENT] = 0;
    return true;
  }

  uint8_t current = s_BatProfile.chargeCurrent > 26 ? 26 : (s_BatProfile.chargeCurrent & 0x1F);
  s_Regsw[BQ_REG_CHARGE_CURRENT] = (current << 3) | (s_BatProfile.terminationCurr & 0x07);

  return regSyncIfDiffers(BQ_REG_CHARGE_CURRENT);
}

static bool syncVinDpm(void)
{
  s_Regsw[BQ_REG_VIN_DPM] = (uint8_t)s_ChargerInDpm | ((uint8_t)BQ_VIN_DPM_USB << 3);
  return regSyncIfDiffers(BQ_REG_VIN_DPM);
}

/*============================ PUBLISHING ====================================*/

static void onAppEventLost(uint8_t _type)
{
  LOG_ERROR("[CHG] APP event queue full, event %u dropped", _type);
  taskENTER_CRITICAL();
  s_ErrMask |= CHARGER_ERR_EVENT_LOST;
  taskEXIT_CRITICAL();
}

/*
 * Both edges go to APP and nowhere else. The battery one in particular is only half the story -
 * battery.c combines it with the pack voltage to decide whether a pack is really fitted.
 */
static void publishEdges(void)
{
  bool inputPresent = charger_IsInputPresent();
  if (s_PowerSourcePresent != inputPresent) {
    s_PowerSourcePresent = inputPresent;
    AppEvent_t evt = { .type = APP_EVT_CHARGER_INPUT_PRESENCE,
                       .data.chargerInput = { inputPresent } };
    if (!app_PostEvent(&evt))
      onAppEventLost(evt.type);
  }

  bool batPresent = charger_IsBatteryPresent();
  if (s_BatteryPresent != batPresent) {
    s_BatteryPresent = batPresent;
    AppEvent_t evt = { .type = APP_EVT_BATTERY_PRESENCE,
                       .data.batteryPresence = { batPresent } };
    if (!app_PostEvent(&evt))
      onAppEventLost(evt.type);
  }
}

/*============================ BRING-UP AND ROUND ============================*/

static bool chargerBringUp(void)
{
  s_Regsw[BQ_REG_SUPPLY_STATUS] |= BQ_OTG_LOCK_BIT;
  if (i2c_master_WriteMem(BQ_I2C_ADDR, BQ_REG_SUPPLY_STATUS,
                          &s_Regsw[BQ_REG_SUPPLY_STATUS], 1) != I2C_OK) {
    countBusError();
    return false;
  }

  /* NOTE: never select high impedance mode here - it disables the VSys mosfet and cuts power to
   * the MCU. Charging stays off until the first reconciliation round decides otherwise. */
  s_Regsw[BQ_REG_CONTROL] |= BQ_CHG_DISABLE_BIT;
  s_Regsw[BQ_REG_CONTROL] |= 0x20;   // USB limit 500 mA
  if (i2c_master_WriteMem(BQ_I2C_ADDR, BQ_REG_CONTROL,
                          &s_Regsw[BQ_REG_CONTROL], 1) != I2C_OK) {
    countBusError();
    return false;
  }

  DelayUs(500);

  if (!regReadAll())
    return false;

  if (!syncUsbLockout())
    return false;
  if (!syncTempRegulation())
    return false;

  s_ChargerStatus = (s_Regs[BQ_REG_STATUS_CONTROL] >> 4) & 0x07;
  return true;
}

/*
 * One round: take a fresh snapshot of the device, tell APP about anything that changed, keep the
 * watchdog happy, then push every desired value that the device is not already holding.
 *
 * A snapshot that could not be taken ends the round: writing to a device we cannot read back is
 * pointless, and it keeps a dead device from costing six failed write sequences per round.
 */
static bool chargerSync(void)
{
  if (!regReadAll())
    return false;

  s_ChargerStatus = (s_Regs[BQ_REG_STATUS_CONTROL] >> 4) & 0x07;
  publishEdges();

  kickWatchdog();

  bool ok = true;
  if (!syncUsbLockout())        ok = false;
  if (!syncControl())           ok = false;
  if (!syncRegulationVoltage()) ok = false;
  if (!syncTempRegulation())    ok = false;
  if (!syncChargeCurrent())     ok = false;
  if (!syncVinDpm())            ok = false;

  return ok;
}

/*============================ APPLIERS ======================================*/

/* Take a value in, nothing more. Whether it also means work is each state's own business. */

static void applyInputsConfig(uint8_t _config)
{
  s_ChargerInputsConfig = _config;
  s_ChargerInputsPrecedence = _config & 0x01;
  s_UsbInEnabled = (_config & 0x02) == 0x02;
  s_NoBatteryTurnOn = (_config & 0x04) == 0x04;
  s_ChargerInLimit = (_config & 0x08) == 0x08;
  s_ChargerInDpm = (_config >> 4) & 0x07;
}

static void applyChargingConfig(uint8_t _config)
{
  s_ChargingConfig = _config;
  s_ChargingEnabled = _config & 0x01;
}

static void applyUsbILim(uint8_t _step)
{
  switch ((ChargerUsbILimStep_T)_step)
  {
    case CHG_ILIM_STEP_UP:
      if (s_UsbInCurrentLimit < CHG_IUSB_LIMIT_1500MA)
        s_UsbInCurrentLimit++;
      break;
    case CHG_ILIM_STEP_DOWN:
      if (s_UsbInCurrentLimit > CHG_IUSB_LIMIT_150MA)
        s_UsbInCurrentLimit--;
      break;
    case CHG_ILIM_SET_MIN:
      s_UsbInCurrentLimit = CHG_IUSB_LIMIT_150MA;
      break;
    default:
      LOG_WARNING("[CHG] bad USB current limit step %u", _step);
      break;
  }
}

/*
 * Takes one command's value in, and that is all it does - no state is read, no transition is
 * decided. Every state calls this for CHG_EV_CMD and then says for itself what, if anything,
 * follows: in ACTIVE the round below pushes the new value into the device, in INIT and LOST it
 * simply waits for the next attempt.
 */
static void cmdProcess(const ChargerEvent_t *_ev)
{
  switch (_ev->data.cmd.id)
  {
    case CHG_CMD_SET_BAT_PROFILE:
      s_BatProfileValid = _ev->data.cmd.arg.batProfile.valid;
      if (s_BatProfileValid)
        s_BatProfile = _ev->data.cmd.arg.batProfile.profile;
      break;

    case CHG_CMD_SET_INPUTS_CONFIG:
      applyInputsConfig(_ev->data.cmd.arg.u8);
      break;

    case CHG_CMD_SET_CHARGING_CONFIG:
      applyChargingConfig(_ev->data.cmd.arg.u8);
      break;

    case CHG_CMD_SET_THERMAL_STATE:
      s_ThermalState = (BatteryThermalState_T)_ev->data.cmd.arg.u8;
      break;

    case CHG_CMD_SET_5V_IN_DETECTED:
      s_5vInDetected = (_ev->data.cmd.arg.u8 != 0);
      break;

    case CHG_CMD_REEVAL_USB_LOCKOUT:
      break;   // nothing to take in, the caller only wanted the round that follows

    case CHG_CMD_SET_USB_ILIM:
      applyUsbILim(_ev->data.cmd.arg.u8);
      break;

    default:
      LOG_WARNING("[CHG] unknown command %u", (unsigned)_ev->data.cmd.id);
      break;
  }
}

/*============================ STATES ========================================*/

/*
 * Every state below lists every event it answers to - reading one function tells you the whole
 * behaviour of that state, with nothing hidden behind a shared default handler. "break" means
 * stay where we are, "return" names the state to move to.
 */
static ChargerState_t stInit(const ChargerEvent_t *_ev)
{
  switch (_ev->type)
  {
    case CHG_EV_TICK:
      /* Entry left the timeout at zero so the first attempt happens at once. From here on the
       * attempts are paced. */
      s_StateTimeout = pdMS_TO_TICKS(CHG_INIT_RETRY_MS);

      if (chargerBringUp())
        return CHG_ST_ACTIVE;
      if (++s_InitAttempts >= CHG_INIT_ATTEMPTS)
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
  switch (_ev->type)
  {
    case CHG_EV_TICK:
    case CHG_EV_IRQ:
      break;   // nothing to take in, straight to the round below

    case CHG_EV_CMD:
      cmdProcess(_ev);
      break;

    default:
      LOG_WARNING("[CHG] unhandled event %u in ACTIVE", (unsigned)_ev->type);
      return CHG_ST_ACTIVE;   // no reason to touch the bus over an event we do not know
  }

  /*
   * Every event this state understands ends the same way, and that is the point of the state:
   * whatever just changed, push it into the device now rather than waiting for the next tick.
   */
  if (chargerSync()) {
    s_ErrCount = 0;
    return CHG_ST_ACTIVE;
  }

  if (++s_ErrCount >= CHG_ERR_LIMIT) {
    LOG_ERROR("[CHG] %u failed rounds in a row", (unsigned)s_ErrCount);
    return CHG_ST_LOST;
  }

  return CHG_ST_ACTIVE;
}

static ChargerState_t stLost(const ChargerEvent_t *_ev)
{
  switch (_ev->type)
  {
    case CHG_EV_TICK:
      return CHG_ST_INIT;   // periodic recovery attempt

    case CHG_EV_IRQ:
      /* The device raised its interrupt line, so it is alive again - no reason to sit out the
       * rest of the backoff. */
      LOG_INFO("[CHG] interrupt from a device believed lost");
      return CHG_ST_INIT;

    case CHG_EV_CMD:
      // Nothing reaches the device while it is written off - the recovery attempt applies it all.
      cmdProcess(_ev);
      break;

    default:
      LOG_WARNING("[CHG] unhandled event %u in LOST", (unsigned)_ev->type);
      break;
  }

  return CHG_ST_LOST;
}

/* Entry actions and the state's queue timeout. Called only by fsmDispatch(). */
static void fsmEnter(ChargerState_t _state)
{
  switch (_state)
  {
    case CHG_ST_INIT:
      s_StateTimeout = 0;   // first attempt at once, stInit() paces the retries after that
      s_InitAttempts = 0;
      LOG_INFO("[CHG] FSM state INIT");
      break;

    case CHG_ST_ACTIVE:
      s_StateTimeout = pdMS_TO_TICKS(CHG_ACTIVE_PERIOD_MS);
      s_ErrCount = 0;
      LOG_INFO("[CHG] FSM state ACTIVE");
      break;

    case CHG_ST_LOST:
      s_StateTimeout = pdMS_TO_TICKS(CHG_RETRY_PERIOD_MS);
      s_ChargerStatus = CHG_NA;
      s_UsbInLockoutStatus = CHG_USB_IN_UNKNOWN;
      /* No watchdog kick from here on. That is deliberate: if we cannot talk to the device, the
       * best thing it can do is time out and fall back to its own safe defaults. */
      LOG_WARNING("[CHG] device lost (i2c errors=%u), next attempt in %u ms",
                  (unsigned)s_I2cErrorCounter, (unsigned)CHG_RETRY_PERIOD_MS);
      break;

    default:
      ASSERT(0);
      break;
  }
}

/*
 * The whole transition, in one place: ask the current state what to do with the event, and if it
 * asked to move, assign the new state and run its entry actions. Handlers only decide - they
 * cannot switch state as a side effect and then carry on running in it, and nothing but this
 * function assigns s_State.
 */
static void fsmDispatch(const ChargerEvent_t *_ev)
{
  ChargerState_t next;

  switch (s_State)
  {
    case CHG_ST_INIT:   next = stInit(_ev);   break;
    case CHG_ST_ACTIVE: next = stActive(_ev); break;
    case CHG_ST_LOST:   next = stLost(_ev);   break;
    default:            ASSERT(0);            return;
  }

  if (next == s_State)
    return;

  s_State = next;
  fsmEnter(next);
}

/*============================ TASK ==========================================*/

/*
 * Waits for an event and hands it to the state machine. That is the whole task: no work here and
 * no "is it time yet" check anywhere - a wakeup either carries an event (from APP or from the
 * CHG_INT interrupt) or is the current state's timeout, dispatched as CHG_EV_TICK.
 */
static void Task(void *parameters)
{
  (void)parameters;

  LOG_INFO("[CHG] task started");

  vTaskDelay(pdMS_TO_TICKS(CHG_POWER_ON_DELAY_MS)); // the device needs time after power on

  /* The initial transition, the one fsmDispatch() cannot make: s_State already says INIT, this
   * runs its entry actions. */
  fsmEnter(CHG_ST_INIT);

  for (;;)
  {
    ChargerEvent_t ev;

    if (xQueueReceive(s_QueHandle, &ev, s_StateTimeout) != pdTRUE)
      ev.type = CHG_EV_TICK;

    fsmDispatch(&ev);
  }
}

/*============================ EVENT POSTING =================================*/

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

/*============================ PUBLIC API ====================================*/

void charger_Init(uint8_t _nvInputsConfig, uint8_t _nvChargingConfig)
{
  LOG_INFO("charger_Init...");

  applyInputsConfig(_nvInputsConfig);
  applyChargingConfig(_nvChargingConfig);

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
  return s_ChargerStatus;
}

bool charger_IsBatteryPresent(void)
{
  return !((s_Regs[BQ_REG_SUPPLY_STATUS] & BQ_BATSTAT_MASK) == BQ_BATSTAT_ABSENT);
}

bool charger_IsInputPresent(void)
{
  return (s_Regs[BQ_REG_STATUS_CONTROL] & 0x70)
      && ((s_Regs[BQ_REG_STATUS_CONTROL] & 0x70) < (6 * 16));
}

uint8_t charger_GetInStat(void)
{
  return (s_Regs[BQ_REG_SUPPLY_STATUS] >> 6) & 0x03;
}

uint8_t charger_GetUsbStat(void)
{
  return (s_Regs[BQ_REG_SUPPLY_STATUS] >> 4) & 0x03;
}

bool charger_IsDpmModeActive(void)
{
  return (s_Regs[BQ_REG_VIN_DPM] & 0x40) != 0;
}

ChargerUSBInLockoutStatus_T charger_GetUsbInLockoutStatus(void)
{
  return s_UsbInLockoutStatus;
}

uint8_t charger_GetTsFaultStatus(void)
{
  // NOTE (pre-existing, preserved on purpose): this is `s_Regs[7] & (0x06>>1)`, not
  // `(s_Regs[7]&0x06)>>1` - the original CHRGER_TS_FAULT_STATUS() macro has the same operator
  // precedence quirk. Not fixed here - this refactor is behaviour preserving.
  return s_Regs[BQ_REG_SAFETY_NTC] & 0x06 >> 1;
}

ChargerFaultStatus_T charger_GetFaultStatus(void)
{
  return (ChargerFaultStatus_T)(s_Regs[BQ_REG_STATUS_CONTROL] & 0x07);
}

uint8_t charger_GetI2cErrorCount(void)
{
  return s_I2cErrorCounter;
}

bool charger_IsNoBatteryTurnOnEnabled(void)
{
  return s_NoBatteryTurnOn != 0;
}

/* Every setter but the profile carries a single byte, so they all go out the same way. */
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
  ev.data.cmd.arg.batProfile.valid = (batProfile != NULL);
  if (batProfile != NULL)
    ev.data.cmd.arg.batProfile.profile = *batProfile;
  postEvent(&ev);
}

void charger_SetThermalState(BatteryThermalState_T state)
{
  postCmdU8(CHG_CMD_SET_THERMAL_STATE, (uint8_t)state);
}

void charger_Set5vInDetected(bool detected)
{
  postCmdU8(CHG_CMD_SET_5V_IN_DETECTED, detected ? 1 : 0);
}

void charger_SetUsbLockout(ChargerUSBInLockoutStatus_T status)
{
  /* The status argument was already ignored before this refactor: the lockout is derived from
   * the configuration and the 5V detection, so this only means "re-evaluate it now". */
  (void)status;
  postCmdU8(CHG_CMD_REEVAL_USB_LOCKOUT, 0);
}

void charger_SetUsbILim(ChargerUsbILimStep_T step)
{
  postCmdU8(CHG_CMD_SET_USB_ILIM, (uint8_t)step);
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
