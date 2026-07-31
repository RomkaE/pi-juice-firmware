/*
 * fuel_gauge_lc709203f.c
 *
 *  Created on: 06.12.2016.
 *      Author: milan
 */

#include <stdint.h>
#include <stdbool.h>

#include "fuel_gauge_lc709203f.h"
#include "iosystem/analog.h"
#include "driver/i2c/i2c_master.h"
#include "utils/crc8_atm.h"
#include "app-error/app_assert.h"
#include "app-error/app_error.h"

// FreeRTOS:
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

// LOG:
#include "log/log.h"

/*============================ DEVICE ========================================*/

#define LC_I2C_ADDR                 0x16    // NB: same number as LC_REG_STATUS_BIT, unrelated

#define LC_REG_THERMISTOR_B         0x06
#define LC_REG_CELL_TEMP            0x08
#define LC_REG_APA                  0x0B    // Adjustment Pack Application
#define LC_REG_APT                  0x0C    // Adjustment Pack Thermistor
#define LC_REG_ITE                  0x0F
#define LC_REG_IC_VERSION           0x11
#define LC_REG_CHANGE_PARAM         0x12
#define LC_REG_IC_POWER_MODE        0x15
#define LC_REG_STATUS_BIT           0x16

#define LC_POWER_MODE_OPERATIONAL   0x0001
#define LC_CHANGE_PARAM_DEFAULT     0x0001
#define LC_APA_DEFAULT              0x0036
#define LC_APT_DEFAULT              0x3000
#define LC_STATUS_TEMP_THERMISTOR   0x0001
#define LC_STATUS_TEMP_I2C          0x0000

#define LC_TEMP_MIN_VALID           0x09E4  // 253.2 K = -19.95 C, below this the NTC is not there
#define LC_TEMP_MAX_WRITE           0x0D04  // 333.2 K = +60 C, the highest the IC accepts

/*============================ TUNING ========================================*/

/*
 * There is no interrupt to work from: the LC709203F ALARM pin only has threshold sources (Alarm
 * Low RSOC / Alarm Low Cell Voltage), it never signals "a new measurement is ready", and on this
 * board its line is shared with PWR_5V_PG anyway. So each state carries its own queue timeout,
 * and the expiry is dispatched as an event like any other. The IC runs its own gauge algorithm
 * continuously - reading it more often than once a second buys nothing.
 */
#define FG_MEAS_PERIOD_MS       1000    // FG_ST_IC_ACTIVE: one measurement round
#define FG_INIT_RETRY_MS        1000    // FG_ST_IC_INIT: one bring-up attempt
#define FG_RETRY_PERIOD_MS      30000   // FG_ST_IC_LOST: how often a written-off IC is retried
#define FG_POWER_ON_DELAY_MS    100     // the IC needs this much after power on before it answers

#define FG_INIT_ATTEMPTS        5       // bring-up tries before the IC is declared absent
#define FG_ERR_LIMIT            5       // consecutive transport errors that drop ACTIVE to LOST

#define FG_TASK_STACK_WORDS     192
#define FG_QUE_LEN              4

#define FG_COUNT_SATURATING(c)	do { if ((c) < 0xFFFF) (c)++; } while (0)

_Static_assert(pdMS_TO_TICKS(FG_MEAS_PERIOD_MS) >= 1, "FG_MEAS_PERIOD_MS must be at least one RTOS tick");

/*============================ STATE MACHINE =================================*/

/*
 *    APP: battery present        bring-up ok
 *  NO_BATTERY ----------> IC_INIT ----------> IC_ACTIVE
 *       ^                    |                    |
 *       |                    | FG_INIT_ATTEMPTS   | FG_ERR_LIMIT transport errors
 *       |                    v                    v
 *       +---------------- IC_LOST <---------------+
 *   APP: battery absent      |  ^
 *   (from any state)         +--+ FG_RETRY_PERIOD_MS -> IC_INIT
 *
 * The state machine is the only thing that runs. The task waits for an event and hands it over;
 * fsmDispatch() picks the handler for the current state, and the handler answers with the state
 * to be in next - it never switches state itself. So s_State is assigned in exactly one place,
 * and a handler cannot flip the state halfway through and then keep running in the new one.
 *
 * Each state's whole behaviour is one function, and none of them asks "which state are we in":
 * rsocRead()/tempRefresh() are only reachable from IC_ACTIVE, icBringUp() only from IC_INIT.
 * Outside the dispatcher, s_State is read only by the telemetry getters - the documented exception.
 */
typedef enum
{
  FG_ST_NO_BATTERY = 0,  // no pack: the bus is not touched at all, the task sleeps indefinitely
  FG_ST_IC_INIT,         // pack present, configuring the LC709203F
  FG_ST_IC_ACTIVE,       // LC709203F answering - it is the source of RSOC and temperature
  FG_ST_IC_LOST          // LC709203F unreachable: RSOC unknown, MCU sensor for temperature
} FuelGaugeState_t;

/*
 * Two kinds of event. Battery presence is a thing that happened and every state answers it
 * differently, so it gets its own types. A parameter being set is one event with a parameter -
 * no state needs to intercept one setter and not another, so they collapse into a single case
 * per state and cmd_process() sorts out which is which.
 */
typedef enum
{
  FG_EV_TICK = 1,          // the state's own queue timeout expired
  FG_EV_BATTERY_PRESENT,   // from APP
  FG_EV_BATTERY_ABSENT,    // from APP
  FG_EV_CMD                // a parameter was set, see FuelGaugeCmdType_t
} FuelGaugeEventType_t;

/* What each command carries in FuelGaugeCmd_t.arg. */
typedef enum
{
  FG_CMD_SET_CONFIG = 1,   // arg.u8 - host register 0x93 byte
  FG_CMD_SET_BAT_PROFILE   // arg.batProfile
} FuelGaugeCmdType_t;

/* Payload of an FG_EV_CMD event: which parameter, and its value. */
typedef struct
{
  FuelGaugeCmdType_t id;
  union
  {
    uint8_t u8;
    struct
    {
      bool valid;
      BatteryProfile_T profile;
    } batProfile;
  } arg;
} FuelGaugeCmd_t;

typedef struct
{
  FuelGaugeEventType_t type;   // says which member of data is live
  union
  {
    FuelGaugeCmd_t cmd;  // FG_EV_CMD. TICK and the two presence events carry nothing
  } data;
} FuelGaugeEvent_t;

static FuelGaugeState_t s_State = FG_ST_NO_BATTERY;
/*
 * Queue wait of the current state. Set on entry by fsmEnter(); a state may re-arm its own - that
 * is what IC_INIT does to get its first attempt done immediately and pace only the retries.
 */
static TickType_t s_StateTimeout = portMAX_DELAY;
static uint8_t s_InitAttempts;                     // FG_ST_IC_INIT
static uint8_t s_ErrCount;                         // FG_ST_IC_ACTIVE

/*============================ PUBLISHED DATA ================================*/

static uint16_t s_BatteryRsoc = FUEL_GAUGE_RSOC_UNKNOWN;
static int8_t s_BatteryTemp = 25;
static int8_t s_NtcFaultFlag;

/*
 * Two temperature questions that used to be one tangle, kept apart on purpose.
 *
 * FgTempSource_t is where OUR reading comes from, and it follows the host configuration alone.
 * s_TempMode is what the IC uses for its own algorithm - it either measures the thermistor itself
 * or waits to be told a value over I2C. The two are related (reading the NTC needs the IC in
 * thermistor mode) but they are not the same thing, and merging them is what produced three
 * near-identical function names in the previous version.
 */
typedef enum
{
  FG_TSRC_NTC,     // the pack's thermistor, read through the IC
  FG_TSRC_MCU,     // the MCU's own sensor
  FG_TSRC_FIXED    // no sensor in use: a constant
} FgTempSource_t;

typedef enum
{
  FG_TMODE_UNKNOWN = 0,   // after a bring-up starts we cannot know what the IC is doing
  FG_TMODE_THERMISTOR,    // the IC measures the pack's NTC itself
  FG_TMODE_I2C            // the IC waits to be told a temperature
} FgTempMode_t;

static FgTempMode_t s_TempMode = FG_TMODE_UNKNOWN;

/* What one round of temperature work decided. Filled in by the readers, published by tempRefresh(). */
typedef struct
{
  int8_t celsius;   // the value to publish
  bool ntcFault;    // thermistor missing or open - goes to the host fault register
  bool feedIc;      // the IC did not measure this itself, so it has to be handed the value
} TempReading_t;

/*
 * Diagnostic counters, saturating, never cleared by the driver itself.
 *
 * A readWord() failure reports two things that used to be indistinguishable once they reached
 * the caller, which made it impossible to tell what the bus to the fuel gauge is actually doing:
 *
 *   - a HAL error (NACK on the address, timeout, BERR/ARLO) means the transfer did not complete
 *     - the problem is at bus or device level;
 *   - a CRC-8 mismatch means the transfer completed but a byte came back corrupted - the problem
 *     is noise on the data lines.
 *
 * The first points at pull-ups, routing or the device itself; the second at coupling from the
 * switching regulator, and is what the digital filter and the lowered SCL frequency on I2C2 are
 * meant to address. Only the first kind moves the state machine towards FG_ST_IC_LOST.
 */
static uint16_t s_HalErrorCount;
static uint16_t s_CrcErrorCount;

/*
 * log(R/R25) of a 10k NTC, fixed point, indexed by the profile's nominal resistance - used only
 * by ntcTempCelsius() to solve the B constant equation. Nothing to do with the removed SOC model.
 */
static const int16_t s_LogTbl[256]={-24562, -21803, -19743, -18097, -16728, -15554, -14528, -13616, -12795, -12050, -11366, -10735, -10149, -9602, -9090, -8607, -8151, -7720, -7310, -6919, -6547, -6190, -5848, -5520, -5205, -4901, -4608, -4326, -4052, -3788, -3532, -3283, -3042, -2808, -2580, -2358, -2142, -1932, -1727, -1527, -1332, -1141, -955, -773, -595, -420, -249, -82, 81, 242, 400, 554, 706, 855, 1002, 1145, 1287, 1426, 1562, 1697, 1829, 1959, 2087, 2213, 2338, 2460, 2581, 2700, 2817, 2932, 3046, 3158, 3269, 3378, 3486, 3593, 3698, 3802, 3904, 4005, 4105, 4204, 4302, 4398, 4494, 4588, 4681, 4773, 4864, 4954, 5043, 5132, 5219, 5305, 5391, 5475, 5559, 5642, 5724, 5805, 5885, 5965, 6044, 6122, 6199, 6276, 6352, 6427, 6501, 6575, 6648, 6721, 6793, 6864, 6935, 7005, 7074, 7143, 7212, 7279, 7347, 7413, 7479, 7545, 7610, 7675, 7739, 7802, 6487, 7188, 7834, 8432, 8990, 9513, 10004, 10467, 10905, 11322, 11718, 12096, 12457, 12803, 13135, 13454, 13761, 14057, 14343, 14619, 14886, 15144, 15395, 15638, 15875, 16104, 16328, 16545, 16757, 16964, 17165, 17362, 17554, 17741, 17925, 18104, 18280, 18451, 18620, 18785, 18947, 19105, 19261, 19413, 19563, 19710, 19855, 19997, 20137, 20274, 20409, 20542, 20673, 20802, 20928, 21053, 21176, 21297, 21416, 21534, 21650, 21764, 21877, 21988, 22098, 22207, 22313, 22419, 22523, 22626, 22728, 22828, 22927, 23025, 23122, 23217, 23312, 23406, 23498, 23589, 23680, 23769, 23858, 23945, 24032, 24117, 24202, 24286, 24369, 24451, 24533, 24613, 24693, 24772, 24851, 24928, 25005, 25081, 25157, 25231, 25305, 25379, 25452, 25524, 25595, 25666, 25736, 25806, 25875, 25944, 26011, 26079, 26146, 26212, 26278, 26343, 26408, 26472, 26536, 26599, 26661, 26724, 26785, 26847, 26908, 26968, 27028, 27088};

/*============================ CONFIGURATION =================================*/

static BatteryTempSenseConfig_T s_TempSensorConfig = BAT_TEMP_SENSE_CONFIG_AUTO_DETECT;

// Battery profile, copied in - never a live pointer into another task's memory:
static BatteryProfile_T s_BatProfile;
static bool s_BatProfileValid;

static uint32_t s_ErrMask;  // FUEL_GAUGE_ERR_* bits

/*============================ RTOS OBJECTS ==================================*/

static TaskHandle_t s_TaskHandle;
static StaticTask_t s_TaskTCB;
static StackType_t s_TaskStack[FG_TASK_STACK_WORDS];

static QueueHandle_t s_QueHandle;
static StaticQueue_t s_Que;
static FuelGaugeEvent_t s_QueBuf[FG_QUE_LEN];

/*============================ BUS ===========================================*/

/*
 * Returns 0 on success, +1 on a transport error (device absent or bus broken) and -1 on a CRC
 * mismatch. The sign matters: a CRC mismatch means the device answered, so it must not count
 * towards writing the device off.
 */
static int8_t readWord(uint8_t _reg, uint16_t *_word)
{
  uint8_t buf[6] = {LC_I2C_ADDR, _reg, LC_I2C_ADDR | 0x01, 0, 0, 0};

  if (i2c_master_ReadMem(LC_I2C_ADDR, _reg, buf + 3, 3) != I2C_OK) {
    FG_COUNT_SATURATING(s_HalErrorCount);
    return 1;
  }

  if (Crc8Block(0, buf, 5) != buf[5]) {
    FG_COUNT_SATURATING(s_CrcErrorCount);
    return -1;
  }

  *_word = (((uint16_t)buf[4]) << 8) | buf[3];
  return 0;
}

/* Returns 0 on success, 1 on a transport error. */
static int8_t writeWord(uint8_t _reg, uint16_t _word)
{
  uint8_t buf[5] = {LC_I2C_ADDR, _reg, (uint8_t)_word, (uint8_t)(_word >> 8), 0};
  buf[4] = Crc8Block(0, buf, 4);

  if (i2c_master_WriteMem(LC_I2C_ADDR, _reg, &buf[2], 3) != I2C_OK) {
    FG_COUNT_SATURATING(s_HalErrorCount);
    return 1;
  }
  return 0;
}

/*
 * analog_GetTempMCU() is declared uint16_t but the value behind it is an int16_t, so a below zero
 * reading comes back as a large unsigned number. Everything here wants it signed.
 */
static int16_t mcuTemp(void)
{
  return (int16_t)analog_GetTempMCU();
}

/*============================ BRING-UP ======================================*/

// Full device configuration:
static bool icSetTempMode(FgTempMode_t _mode);

static bool startingFlow(void)
{
  int8_t res;

  /* Whatever the IC was doing before, we are about to reconfigure it and it may have been power
   * cycled - so we no longer know its temperature mode. This is discarding knowledge, not setting
   * a mode; icSetTempMode() below does the setting, and needs this to not skip the write. */
  s_TempMode = FG_TMODE_UNKNOWN;

  // Read IC version:
  uint16_t icVersion;
  res = readWord(LC_REG_IC_VERSION, &icVersion);
  if (res != 0)
  {
    LOG_WARNING("[FG] no answer from the IC");
    return false;
  }
  LOG_INFO("[FG] IC version 0x%04X", icVersion);


  if (writeWord(LC_REG_IC_POWER_MODE, LC_POWER_MODE_OPERATIONAL) != 0) return false;
  if (writeWord(LC_REG_APA, LC_APA_DEFAULT) != 0) return false;
  if (writeWord(LC_REG_CHANGE_PARAM, LC_CHANGE_PARAM_DEFAULT) != 0) return false;
  if (writeWord(LC_REG_APT, LC_APT_DEFAULT) != 0) return false;

  if (s_BatProfileValid && s_BatProfile.ntcB != 0xFFFF) {
    if (writeWord(LC_REG_THERMISTOR_B, s_BatProfile.ntcB) != 0) return false;
  }

  if (!icSetTempMode(FG_TMODE_THERMISTOR)) return false;

  /* Prove it really answers. The value is deliberately dropped: this is a liveness probe, and
   * publishing is the first round's job a second from now - see tempRefresh()/rsocRead(). */
  uint16_t rsoc;
  if (readWord(LC_REG_ITE, &rsoc) != 0) return false;

  return true;
}

/*============================ TEMPERATURE ===================================*/

/*
 * Register 0x08 already holds a temperature, in 0.1 K - we do not derive it from a resistance.
 * The catch is that the IC converts its thermistor input assuming a 10 kOhm NTC at 25 C, with the
 * B constant we programmed into register 0x06. For a 10 kOhm pack, which every built-in profile
 * is, that reading is the answer and the first line below is the whole conversion.
 *
 * The rest is a correction for the one case the IC cannot know about: a custom profile whose
 * thermistor has a different nominal resistance. Written through host registers 0x86/0x87, so it
 * has to be supported, but nothing shipped exercises it. The IC's answer is then treated as a
 * measure of the resistance ratio and the Beta equation is re-solved for the real R25 -
 * s_LogTbl[] holds ln(R25/10k) in fixed point, hence the table lookup.
 */
static int16_t ntcTempCelsius(uint16_t _raw, uint16_t _ntcB, uint16_t _ntcOhms)
{
  if (_ntcOhms == 1000)   // 10 Ohm units, so 1000 = 10 kOhm = what the IC assumes
    return ((int16_t)_raw - 2732) / 10;

  /* Index into the log table by the NTC's nominal resistance. Inherited fixed point, left as is. */
  int32_t dr25 = _ntcOhms / 10;
  int32_t it = dr25 < 261 ? (dr25 - 4) >> 1 : ((dr25 + 2300) * 13) >> 8;
  it = it < 0 ? 0 : it;
  it = it > 255 ? 255 : it;

  // T = (B * T10k) / (B - T10k*log(k))
  int16_t tx10 = (_ntcB * (int32_t)_raw)
               / (_ntcB * 10 - (((int32_t)_raw * s_LogTbl[it]) >> 13));
  return tx10 - 273;
}

/* Puts the IC into one of its temperature modes and records what it actually took. */
static bool icSetTempMode(FgTempMode_t _mode)
{
  if (s_TempMode == _mode)
    return true;

  uint16_t word = (_mode == FG_TMODE_THERMISTOR)
      ? LC_STATUS_TEMP_THERMISTOR : LC_STATUS_TEMP_I2C;

  if (writeWord(LC_REG_STATUS_BIT, word) != 0)
    return false;

  s_TempMode = _mode;
  return true;
}

/*
 * Hands a temperature to the IC: it needs a cell temperature for its own algorithm, and when the
 * reading did not come from its own thermistor input, I2C mode is how it gets one.
 */
static bool icFeedTemp(int8_t _celsius)
{
  if (!icSetTempMode(FG_TMODE_I2C))
    return false;

  int32_t raw = (int32_t)_celsius * 10 + 2732;   // 0.1 K, clamped to what the IC accepts
  if (raw > LC_TEMP_MAX_WRITE) raw = LC_TEMP_MAX_WRITE;
  if (raw < LC_TEMP_MIN_VALID) raw = LC_TEMP_MIN_VALID;

  return (writeWord(LC_REG_CELL_TEMP, (uint16_t)raw) == 0);
}

/*
 * Reads the pack's thermistor through the IC and judges the result. Talks to the bus - it has to -
 * but publishes nothing: the verdict goes back to tempRefresh(), which is the only writer.
 */
static bool ntcRead(TempReading_t *_out)
{
  /* The IC has to be measuring the thermistor before we can read it. Getting it back there costs
   * a round, so this one is served from the MCU sensor - and the IC is about to measure for
   * itself, so it needs nothing fed to it. */
  if (s_TempMode != FG_TMODE_THERMISTOR) {
    _out->celsius = (int8_t)mcuTemp();
    _out->feedIc = false;
    return icSetTempMode(FG_TMODE_THERMISTOR);
  }

  uint16_t raw;
  int8_t rc = readWord(LC_REG_CELL_TEMP, &raw);
  if (rc != 0) {
    _out->celsius = s_BatteryTemp;   // nothing new to say, keep what is published
    _out->feedIc = false;
    return (rc < 0);                 // CRC mismatch is not the IC's fault, keep it alive
  }

  /* A compatible NTC cannot read below -20 C. Anything lower means an open or missing sensor,
   * and without a B constant in the profile we cannot convert the reading anyway. */
  if (raw <= LC_TEMP_MIN_VALID || !s_BatProfileValid || s_BatProfile.ntcB == 0xFFFF) {
    _out->celsius = (int8_t)mcuTemp();
    _out->ntcFault = true;
    return true;
  }

  int16_t ntcTemp = ntcTempCelsius(raw, s_BatProfile.ntcB, s_BatProfile.ntcResistance);
  int16_t boardTemp = mcuTemp();

  /* Stuck around 25 C while the board is clearly hot or cold: a fixed resistor is fitted where
   * the NTC should be. Not a fault - just not a thermistor. */
  if (ntcTemp >= 23 && ntcTemp <= 27 && (boardTemp < 15 || boardTemp > 45)) {
    _out->celsius = (int8_t)boardTemp;
    return true;
  }

  /* The reading is the pack's own, so the IC already has it - nothing to hand over. */
  _out->celsius = (int8_t)ntcTemp;
  _out->feedIc = false;
  return true;
}

/*
 * Where the battery temperature comes from. Decided by the host configuration alone - which of
 * the IC's modes that implies is not this one's business.
 */
static FgTempSource_t tempSource(void)
{
  switch (s_TempSensorConfig)
  {
    case BAT_TEMP_SENSE_CONFIG_NTC:
    case BAT_TEMP_SENSE_CONFIG_AUTO_DETECT: return FG_TSRC_NTC;
    case BAT_TEMP_SENSE_CONFIG_ON_BOARD:    return FG_TSRC_MCU;
    default:                                return FG_TSRC_FIXED;
  }
}

/*
 * One round's temperature work, and the only place s_BatteryTemp and s_NtcFaultFlag are written.
 * Everything above computes and answers; publishing happens here, once, where it can be seen.
 */
static bool tempRefresh(void)
{
  TempReading_t t = { .celsius = 25, .ntcFault = false, .feedIc = true };
  bool ok = true;

  switch (tempSource())
  {
    case FG_TSRC_NTC:
      ok = ntcRead(&t);
      break;

    case FG_TSRC_MCU:
      t.celsius = (int8_t)mcuTemp();
      break;

    case FG_TSRC_FIXED:
    default:
      break;   // the initialiser is the answer: no sensor in use, a constant
  }

  s_BatteryTemp = t.celsius;
  s_NtcFaultFlag = t.ntcFault ? 1 : 0;

  if (t.feedIc)
    ok = icFeedTemp(t.celsius) && ok;

  return ok;
}

/*============================ CHARGE ========================================*/

/* Reads the charge level out of the IC and publishes it. */
static bool rsocRead(void)
{
  uint16_t rsoc;
  int8_t rc = readWord(LC_REG_ITE, &rsoc);

  if (rc != 0)
    return (rc < 0);   // CRC mismatch: the IC is there, just keep the previous value

  s_BatteryRsoc = rsoc;
  return true;
}

/*============================ STATES ========================================*/

/* One place that knows the register 0x93 packing - also used by fuel_gauge_Init(). */
static void applyConfig(uint8_t _config)
{
  s_TempSensorConfig = (BatteryTempSenseConfig_T)(_config & FUEL_GAUGE_CONFIG_TEMP_SENSE_MASK);
}

/*
 * Takes one command's value in and reports one property of it: whether the IC holds a copy. It
 * decides nothing else - a state that is not talking to the IC ignores the answer, and the one
 * that is uses it to know a bring-up is due.
 */
static bool cmdProcess(const FuelGaugeEvent_t *_ev)
{
  switch (_ev->data.cmd.id)
  {
    case FG_CMD_SET_CONFIG:
      applyConfig(_ev->data.cmd.arg.u8);
      return false;   // ours alone: it picks which temperature source the next round uses

    case FG_CMD_SET_BAT_PROFILE:
      s_BatProfileValid = _ev->data.cmd.arg.batProfile.valid;
      if (s_BatProfileValid)
        s_BatProfile = _ev->data.cmd.arg.batProfile.profile;
      return true;    // the NTC B constant is written into the IC, so the IC has a stale copy

    default:
      LOG_WARNING("[FG] unknown command %u", (unsigned)_ev->data.cmd.id);
      return false;
  }
}

/*
 * Every state below lists every event it answers to - reading one function tells you the whole
 * behaviour of that state, with nothing hidden behind a shared default handler. "break" means
 * stay where we are, "return" names the state to move to.
 */
static FuelGaugeState_t stNoBattery(const FuelGaugeEvent_t *_ev)
{
  switch (_ev->type)
  {
    case FG_EV_BATTERY_PRESENT:
      return FG_ST_IC_INIT;

    case FG_EV_CMD:
      /* With no pack there is nobody to configure, so it does not matter whether the IC holds a
       * copy - icBringUp() reads everything when a pack turns up. */
      (void)cmdProcess(_ev);
      break;

    /* FG_EV_TICK cannot get here: this state waits on portMAX_DELAY. If it ever does, the
     * warning below is the one that says so. */
    default:
      LOG_WARNING("[FG] unhandled event %u in NO_BATTERY", (unsigned)_ev->type);
      break;
  }

  return FG_ST_NO_BATTERY;
}

static FuelGaugeState_t stIcInit(const FuelGaugeEvent_t *_ev)
{
  switch (_ev->type)
  {
    case FG_EV_TICK:
      /* Entry left the timeout at zero so the first attempt happens at once - there is no reason
       * to sit and wait before doing the thing we just decided to do. From here on the attempts
       * are paced: a pack that has just been plugged in may need a moment to settle. */
      s_StateTimeout = pdMS_TO_TICKS(FG_INIT_RETRY_MS);

      if (startingFlow())
        return FG_ST_IC_ACTIVE;
      if (++s_InitAttempts >= FG_INIT_ATTEMPTS)
        return FG_ST_IC_LOST;
      break;

    case FG_EV_BATTERY_ABSENT:
      return FG_ST_NO_BATTERY;

    case FG_EV_CMD:
      /* Already on the way to a bring-up, so a stale copy in the IC needs no transition of its
       * own: the next attempt, one second from now, writes whatever was just taken in. */
      (void)cmdProcess(_ev);
      break;

    default:
      LOG_WARNING("[FG] unhandled event %u in IC_INIT", (unsigned)_ev->type);
      break;
  }

  return FG_ST_IC_INIT;
}

static FuelGaugeState_t stIcActive(const FuelGaugeEvent_t *_ev)
{
  switch (_ev->type)
  {
    case FG_EV_TICK:
    {
      /* What a round in this state consists of, stated here rather than hidden behind one name.
       * Both always run - a temperature that could not be refreshed is no reason to skip the
       * charge reading - and either failing counts towards giving up on the IC. Two separate
       * bools on purpose: && would short-circuit and the order would have to be remembered. */
      bool rsocOk = rsocRead();
      bool tempOk = tempRefresh();

      if (rsocOk && tempOk) {
        s_ErrCount = 0;
        break;
      }
      if (++s_ErrCount >= FG_ERR_LIMIT) {
        LOG_ERROR("[FG] %u transport errors in a row", (unsigned)s_ErrCount);
        return FG_ST_IC_LOST;
      }
      break;
    }

    case FG_EV_BATTERY_ABSENT:
      return FG_ST_NO_BATTERY;

    case FG_EV_CMD:
      /* The only state that cares whether the IC holds a copy of what just changed: it is the
       * one talking to the IC, and the cheapest correct way to refresh the IC's copy - the NTC
       * B constant - is a full bring-up. Anything else is picked up by the next round. */
      if (cmdProcess(_ev))
        return FG_ST_IC_INIT;
      break;

    default:
      LOG_WARNING("[FG] unhandled event %u in IC_ACTIVE", (unsigned)_ev->type);
      break;
  }

  return FG_ST_IC_ACTIVE;
}

static FuelGaugeState_t stIcLost(const FuelGaugeEvent_t *_ev)
{
  switch (_ev->type)
  {
    case FG_EV_TICK:
      return FG_ST_IC_INIT;   // periodic recovery attempt

    case FG_EV_BATTERY_ABSENT:
      return FG_ST_NO_BATTERY;

    case FG_EV_CMD:
      // Nothing reaches the IC while it is written off - the recovery attempt applies it all.
      (void)cmdProcess(_ev);
      break;

    default:
      LOG_WARNING("[FG] unhandled event %u in IC_LOST", (unsigned)_ev->type);
      break;
  }

  return FG_ST_IC_LOST;
}

/* Entry actions and the state's queue timeout. Called only by fsmDispatch(). */
static void fsmEnter(FuelGaugeState_t _state)
{
  switch (_state)
  {
    case FG_ST_NO_BATTERY:
      s_StateTimeout = portMAX_DELAY;   // nothing to do until APP says otherwise
      s_BatteryRsoc = FUEL_GAUGE_RSOC_UNKNOWN;
      s_NtcFaultFlag = 0;
      LOG_INFO("[FG] FSM state NO_BATTERY");
      break;

    case FG_ST_IC_INIT:
      s_StateTimeout = 0;   // first attempt at once, stIcInit() paces the retries after that
      s_InitAttempts = 0;
      s_NtcFaultFlag = 0;   // nothing has been read yet, so there is nothing to report a fault on
      LOG_INFO("[FG] FSM state IC_INIT");
      break;

    case FG_ST_IC_ACTIVE:
      s_StateTimeout = pdMS_TO_TICKS(FG_MEAS_PERIOD_MS);
      s_ErrCount = 0;
      LOG_INFO("[FG] FSM state IC_ACTIVE");
      break;

    case FG_ST_IC_LOST:
      s_StateTimeout = pdMS_TO_TICKS(FG_RETRY_PERIOD_MS);
      s_BatteryRsoc = FUEL_GAUGE_RSOC_UNKNOWN;
      s_NtcFaultFlag = 0;
      /* The moment the split between the two counters is worth having: NACKs and timeouts point
       * at the bus or the device, CRC mismatches at noise on the data lines. */
      LOG_WARNING("[FG] IC lost (hal=%u crc=%u), next attempt in %u ms",
                  (unsigned)s_HalErrorCount, (unsigned)s_CrcErrorCount,
                  (unsigned)FG_RETRY_PERIOD_MS);
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
static void fsmDispatch(const FuelGaugeEvent_t *_ev)
{
  FuelGaugeState_t next;

  switch (s_State)
  {
    case FG_ST_NO_BATTERY: next = stNoBattery(_ev); break;
    case FG_ST_IC_INIT:    next = stIcInit(_ev);    break;
    case FG_ST_IC_ACTIVE:  next = stIcActive(_ev);  break;
    case FG_ST_IC_LOST:    next = stIcLost(_ev);    break;
    default:               APP_ERROR(APP_ERR);      return;
  }

  if (next != s_State)
  {
    s_State = next;
    fsmEnter(next);
  }
}

/*============================ TASK ==========================================*/

/*
 * Waits for an event and hands it to the state machine. That is the whole task: there is no work
 * here and no "is it time yet" check anywhere - a wakeup either carries an event from APP or is
 * the current state's timeout, which is dispatched as FG_EV_TICK.
 */
static void Task(void *parameters)
{
  (void)parameters;

  LOG_INFO("[FG] task started");

  // The IC needs time after power on:
  vTaskDelay(pdMS_TO_TICKS(FG_POWER_ON_DELAY_MS));

  // The initial FSM transition:
  fsmEnter(FG_ST_NO_BATTERY);   // TODO

  while(1)
  {
    FuelGaugeEvent_t ev;
    if (xQueueReceive(s_QueHandle, &ev, s_StateTimeout) != pdTRUE)
      ev.type = FG_EV_TICK;

    fsmDispatch(&ev);
  }
}

/*============================ EVENT POSTING =================================*/

static bool postEvent(const FuelGaugeEvent_t *_ev)
{
  if (s_QueHandle == NULL)
  {
    LOG_ERROR("[FG] event queue not ready");
    s_ErrMask |= FUEL_GAUGE_ERR_NOT_READY;
    return false;
  }

  BaseType_t sent;

  if (xPortIsInsideInterrupt() != pdFALSE)
  {
    BaseType_t woken = pdFALSE;
    sent = xQueueSendFromISR(s_QueHandle, _ev, &woken);
    portYIELD_FROM_ISR(woken);

    if (sent != pdTRUE)
      s_ErrMask |= FUEL_GAUGE_ERR_QUEUE_FULL;
  }
  else
  {
    sent = xQueueSend(s_QueHandle, _ev, 0);

    if (sent != pdTRUE)
    {
      LOG_ERROR("[FG] event queue full, ev=%u dropped", _ev->type);
      taskENTER_CRITICAL();
      s_ErrMask |= FUEL_GAUGE_ERR_QUEUE_FULL;
      taskEXIT_CRITICAL();
    }
  }

  return (sent == pdTRUE);
}

/*============================ PUBLIC API ====================================*/

void fuel_gauge_Init(uint8_t _cfg_mask)
{
  LOG_INFO("fuel_gauge_Init...");

  // Applied directly, the task does not exist yet so there is nobody to queue it for:
  applyConfig(_cfg_mask);

  s_QueHandle = xQueueCreateStatic(sizeof(s_QueBuf) / sizeof(s_QueBuf[0]),
                            sizeof(s_QueBuf[0]), (uint8_t*)s_QueBuf, &s_Que);
  ASSERT(s_QueHandle != NULL);

  s_TaskHandle = xTaskCreateStatic(Task, "LC709203F", sizeof(s_TaskStack) / sizeof(StackType_t),
                            NULL, 6, s_TaskStack, &s_TaskTCB);
  ASSERT(s_TaskHandle != NULL);
}

void fuel_gauge_SetConfig(uint8_t config)
{
  FuelGaugeEvent_t ev = { .type = FG_EV_CMD };
  ev.data.cmd.id = FG_CMD_SET_CONFIG;
  ev.data.cmd.arg.u8 = config;
  postEvent(&ev);
}

void fuel_gauge_SetBatProfile(const BatteryProfile_T *batProfile)
{
  FuelGaugeEvent_t ev = { .type = FG_EV_CMD };
  ev.data.cmd.id = FG_CMD_SET_BAT_PROFILE;
  ev.data.cmd.arg.batProfile.valid = (batProfile != NULL);
  if (batProfile != NULL)
    ev.data.cmd.arg.batProfile.profile = *batProfile;
  postEvent(&ev);
}

void fuel_gauge_SetBatteryPresent(bool present)
{
  FuelGaugeEvent_t ev = { .type = present ? FG_EV_BATTERY_PRESENT : FG_EV_BATTERY_ABSENT };
  postEvent(&ev);
}

/*
 * The getters below read s_State - the one documented exception to "only fsmDispatch() looks at
 * the state". They carry no logic: they just answer "unknown" instead of a stale number when the
 * IC is not the source.
 */
uint16_t fuel_gauge_GetRsoc(void)
{
  return s_BatteryRsoc;
}

int8_t fuel_gauge_GetTemp(void)
{
  return (s_State == FG_ST_IC_ACTIVE) ? s_BatteryTemp : (int8_t)mcuTemp();
}

BatteryTempSenseConfig_T fuel_gauge_GetTempSenseConfig(void)
{
  return s_TempSensorConfig;
}

bool fuel_gauge_IsIcFault(void)
{
  return (s_State == FG_ST_IC_LOST);
}

bool fuel_gauge_IsTempSenseFault(void)
{
  return s_NtcFaultFlag != 0;
}

bool fuel_gauge_IsConfigValid(uint8_t config)
{
  /* Only the temperature field is checked: the RSOC field's one non-default code selected the
   * removed software model, and a write carrying it is accepted and ignored rather than failed,
   * so that existing host software keeps working. */
  return (config & FUEL_GAUGE_CONFIG_TEMP_SENSE_MASK) < BAT_TEMP_SENSE_CONFIG_END;
}

uint8_t fuel_gauge_GetConfig(void)
{
  // The RSOC field always reads back as AUTO_DETECT, see fuel_gauge_IsConfigValid():
  return (uint8_t)(s_TempSensorConfig & FUEL_GAUGE_CONFIG_TEMP_SENSE_MASK);
}

uint32_t fuel_gauge_GetErrMask(bool _clear)
{
  taskENTER_CRITICAL();
  uint32_t mask = s_ErrMask;
  if (_clear)
    s_ErrMask &= ~mask;
  taskEXIT_CRITICAL();
  return mask;
}
