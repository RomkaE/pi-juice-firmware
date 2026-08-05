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
#include "utils/utils.h"
#include "app-error/app_assert.h"
#include "app-error/app_error.h"


// FreeRTOS:
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

// LOG:
#include "log/log.h"


#define LC_I2C_ADDR                 0x16    // NB: same number as LC_REG_STATUS_BIT, unrelated

#define LC_REG_BEFORE_RSOC          0x04
#define LC_REG_THERMISTOR_B         0x06
#define LC_REG_INITIAL_RSOC         0x07
#define LC_REG_CELL_TEMP            0x08
#define LC_REG_APA                  0x0B    // Adjustment Pack Application
#define LC_REG_APT                  0x0C    // Adjustment Pack Thermistor
#define LC_REG_ITE                  0x0F
#define LC_REG_IC_VERSION           0x11
#define LC_REG_CHANGE_PARAM         0x12
#define LC_REG_IC_POWER_MODE        0x15
#define LC_REG_STATUS_BIT           0x16
#define LC_REG_NUM_PARAM            0x1A

#define LC_POWER_MODE_OPERATIONAL   0x0001
#define LC_CHANGE_PARAM_DEFAULT     0x0001
#define LC_APA_DEFAULT              0x0036  // the 3000 mAh row of Table 7 - see apa_value()
#define LC_APT_DEFAULT              0x3000  // value from original PiJuice FW
#define LC_STATUS_TEMP_THERMISTOR   0x0001
#define LC_STATUS_TEMP_I2C          0x0000

#define LC_TEMP_MIN_VALID           0x09E4  // 253.2 K = -19.95 C, below this the NTC is not there
#define LC_TEMP_MAX_WRITE           0x0D04  // 333.2 K = +60 C, the highest the IC accepts

#define FG_MEAS_PERIOD_MS       1000    // FG_ST_IC_ACTIVE: one measurement round
#define FG_INIT_RETRY_MS        1000    // FG_ST_IC_INIT: one bring-up attempt
#define FG_RETRY_PERIOD_MS      10000   // FG_ST_IC_LOST: how often a written-off IC is retried
#define FG_POWER_ON_DELAY_MS    100     // the IC needs this much after power on before it answers
                                        // max Tinit 90ms - Table 4 in the LC709203F datasheet
#define FG_PARAM_SETTLE_MS      100     // settle time after a battery parameter write, register 0x12
                                        // Figure 18 in the LC709203F datasheet

#define FG_INIT_ATTEMPTS        5       // bring-up tries before the IC is declared absent
#define FG_ERR_LIMIT            5       // consecutive transport errors that drop ACTIVE to LOST
#define FG_STALE_LIMIT          3       // reads in a row without fresh data before a value goes unknown

/* Profile capacity not stated. BatteryProfile_T spells the same thing 0xFFFFFFFF - see the
 * PACK_CAPACITY_U16()/UNPACK_CAPACITY_U16() pair in battery.c. */
#define FG_CAPACITY_UNKNOWN     0xFFFF

#define FG_COUNT_SATURATING(c)  do { if ((c) < 0xFFFF) (c)++; } while (0)

#define LC_INIT_RSOC_REG        LC_REG_BEFORE_RSOC  // LC_REG_BEFORE_RSOC or LC_REG_INITIAL_RSOC

#define FG_TASK_STACK_WORDS     256
#define FG_QUE_LEN              4

_Static_assert( LC_INIT_RSOC_REG == LC_REG_BEFORE_RSOC || LC_INIT_RSOC_REG == LC_REG_INITIAL_RSOC,
                "LC_INIT_RSOC_REG must be LC_REG_BEFORE_RSOC or LC_REG_INITIAL_RSOC");
_Static_assert(pdMS_TO_TICKS(FG_MEAS_PERIOD_MS) >= 1, "FG_MEAS_PERIOD_MS must be at least one RTOS tick");

// Internal battery profile:
typedef struct
{
  uint16_t chrg_voltage;  // in mV (4200/4350)
  uint16_t capacity;      // in mAh
  struct
  {
    uint16_t resistance;  // in Ohm
    uint16_t b_const;
  } NTC;
} FgBattProfile_t;

typedef enum
{
  FG_READ_OK = 0,     // a fresh value was published
  FG_READ_NO_DATA,    // the IC answered, the value is not usable
  FG_READ_BUS_ERROR   // the IC did not answer
} FgReadResult_t;

// FSM:
typedef enum
{
  FG_ST_NO_BATTERY = 0,  // no pack: the bus is not touched at all, the task sleeps indefinitely
  FG_ST_IC_INIT,         // pack present, configuring the LC709203F
  FG_ST_IC_ACTIVE,       // LC709203F answering - it is the source of RSOC and temperature
  FG_ST_IC_LOST,         // LC709203F unreachable: RSOC unknown, MCU sensor for temperature
  FG_ST_COUNT            // also bounds an entry chain - see fsm_Dispatch()
} FuelGaugeState_t;

typedef enum
{
  FG_EV_ENTRY = 0,
  FG_EV_TICK,
  FG_EV_BATTERY_PRESENT,
  FG_EV_BATTERY_ABSENT,
  FG_EV_CMD
} FuelGaugeEventType_t;

typedef enum
{
  FG_CMD_SET_CONFIG = 1,
  FG_CMD_SET_BAT_PROFILE
} FuelGaugeCmdType_t;

typedef struct
{
  FuelGaugeEventType_t type;   // says which member of data is live
  union
  {
    struct
    {
      FuelGaugeCmdType_t id;
      union
      {
        uint8_t config;
        FgBattProfile_t battProfile;
      };
    } cmd;
  };
} FuelGaugeEvent_t;

/*
 * Queue wait of the current state. Each state sets its own while handling FG_EV_ENTRY, and may
 * re-arm it later - that is what IC_INIT does to get its first attempt done immediately and pace
 * only the retries after it.
 */
static TickType_t s_StateTimeout = portMAX_DELAY;   // in ticks

static uint16_t s_BattRsoc = FUEL_GAUGE_RSOC_UNKNOWN;
static int8_t s_BattTemp = FUEL_GAUGE_TEMP_UNKNOWN;

// Diagnostic/errors:
static uint32_t s_ErrMask;
static bool s_IcFault;

// Battery profile, copied in - never a live pointer into another task's memory:
static FgBattProfile_t s_BattProfile;
static bool s_BattProfileValid;


static TaskHandle_t s_TaskHandle;
static StaticTask_t s_TaskTCB;
static StackType_t s_TaskStack[FG_TASK_STACK_WORDS];

static QueueHandle_t s_QueHandle;
static StaticQueue_t s_Que;
static FuelGaugeEvent_t s_QueBuf[FG_QUE_LEN];

static int8_t readWord(uint8_t _reg, uint16_t *_word)
{
  uint8_t buf[6] = { LC_I2C_ADDR, _reg, LC_I2C_ADDR | 0x01, 0, 0, 0 };
  int i2c_res = i2c_master_ReadMem(LC_I2C_ADDR, _reg, buf + 3, 3);
  if (i2c_res != I2C_OK)
  {
    LOG_ERROR("[FG] readWord FAILED: reg=0x%02X, res=%d", _reg, i2c_res);
    return 1;
  }

  if (Crc8Block(0, buf, 5) != buf[5])
  {
    return -1;
  }

  *_word = (((uint16_t) buf[4]) << 8) | buf[3];
  return 0;
}

static int8_t writeWord(uint8_t _reg, uint16_t _word)
{
  uint8_t buf[5] = { LC_I2C_ADDR, _reg, (uint8_t) _word, (uint8_t) (_word >> 8), 0 };
  buf[4] = Crc8Block(0, buf, 4);
  int i2c_res = i2c_master_WriteMem(LC_I2C_ADDR, _reg, &buf[2], 3);
  if (i2c_res != I2C_OK)
  {
    LOG_ERROR("[FG] writeWord FAILED: reg=0x%02X, res=%d", _reg, i2c_res);
    return 1;
  }
  return 0;
}

static int8_t updateWord(const char *_name, uint8_t _reg, uint16_t _want, bool *_changed)
{
  uint16_t have;

  *_changed = false;      // nothing was written yet, and a failed read leaves it that way

  int8_t res = readWord(_reg, &have);
  if (res != 0)
    return res;

  *_changed = (have != _want);
  if (!*_changed)
    return 0;

  LOG_INFO("[FG] %s register stale: old=0x%04X, new=0x%04X", _name, (unsigned)have, (unsigned)_want);
  return writeWord(_reg, _want);
}

static void profileConvert(const BatteryProfile_T *_src, FgBattProfile_t *_dst)
{
  _dst->chrg_voltage = 3500 + (uint16_t)_src->regulationVoltage * 20;
  _dst->capacity = (_src->capacity >= FG_CAPACITY_UNKNOWN) ? FG_CAPACITY_UNKNOWN : (uint16_t)_src->capacity;
  /* Inside this module "not set" is zero, everywhere and for every field - that is what
   * fuel_gauge_SetBattProfile() writes for a missing profile and what read_Temp() tests against.
   * BatteryProfile_T spells it 0xFFFF instead (see battery.h), so it gets translated here rather
   * than leaking a second convention into the readers. */
  _dst->NTC.resistance = (_src->ntcResistance == 0xFFFF) ? 0 : _src->ntcResistance;
  _dst->NTC.b_const = (_src->ntcB == 0xFFFF) ? 0 : _src->ntcB;
}

/*
 * Adjustment Pack Application, register 0x0B. The IC uses it as a stand-in for the pack's internal
 * impedance and selects it purely by capacity - Table 7 in the LC709203F datasheet tabulates it at
 * nine capacities for the 3.7 V nominal / 4.2 V charge battery type, which is what every profile
 * here is.
 *
 * Between the listed points we interpolate rather than snap to the nearest row: the table only
 * carries round capacities and hardly any real pack sits on one - the profile table has 1000 mAh
 * for the PiJuice Zero but 1820 for BP7X - so snapping would throw away most of the resolution the
 * register has.
 *
 * Returns LC_APA_DEFAULT when a profile is in hand but states no capacity - the 3000 mAh row is the
 * middle of the range, which is as good as a guess gets. With no profile at all the register is not
 * written in the first place, see starting_flow().
 */
static uint16_t apa_value(uint16_t _capacity)
{
  static const struct
  {
    uint16_t mah;
    uint8_t apa;
  } tbl[] = { {  100, 0x08 }, {  200, 0x0B }, {  500, 0x10 },
              { 1000, 0x19 }, { 2000, 0x2D }, { 3000, 0x36 },
              { 4000, 0x3F }, { 5000, 0x43 }, { 6000, 0x49 } };
  const uint8_t count = sizeof(tbl) / sizeof(tbl[0]);

  if (_capacity == FG_CAPACITY_UNKNOWN || _capacity == 0)
    return LC_APA_DEFAULT;

  if (_capacity <= tbl[0].mah)
    return tbl[0].apa;

  for (uint8_t i = 1; i < count; i++)
  {
    if (_capacity > tbl[i].mah)
      continue;

    // Linear between the two bracketing rows, rounded to nearest:
    uint16_t span = tbl[i].mah - tbl[i - 1].mah;
    uint16_t off = _capacity - tbl[i - 1].mah;
    uint16_t rise = tbl[i].apa - tbl[i - 1].apa;
    return tbl[i - 1].apa + (uint16_t)(((uint32_t)rise * off + span / 2) / span);
  }

  return (uint16_t)tbl[count - 1].apa;   // above the table, the largest row is the closest it can get
}

static uint16_t change_parameter_value(uint16_t _chrg_voltage)
{
  // See table 8 in the LC709203F datasheet:
  return (_chrg_voltage > 4250) ? 0 : 1;
}

static bool starting_flow(void)
{
  // Persist across FG_INIT_ATTEMPTS: a parameter changed by a failed attempt
  // still requires RSOC recalibration. Clear only after re-initialisation
  // completes successfully:
  static bool s_ReinitRsoc;

  int8_t res;
  uint16_t value;

  // Read IC version:
  res = readWord(LC_REG_IC_VERSION, &value);
  if (res != 0)
    return false;
  LOG_INFO("[FG] IC version: 0x%04X", (unsigned)value);

  // Read Number of the parameter:
  res = readWord(LC_REG_NUM_PARAM, &value);
  if (res != 0)
    return false;
  LOG_INFO("[FG] LC_REG_NUM_PARAM: 0x%04X", (unsigned)value);

  // Operational mode:
  res = writeWord(LC_REG_IC_POWER_MODE, LC_POWER_MODE_OPERATIONAL);
  if (res != 0)
    return false;

  // Battery parameters:
  if (s_BattProfileValid)
  {
    // APA:
    bool apa_written = false;
    res = updateWord("APA", LC_REG_APA, apa_value(s_BattProfile.capacity), &apa_written);
    s_ReinitRsoc |= apa_written;
    if (res != 0)
      return false;

    // Set Battery profile:
    bool param_written = false;
    res = updateWord("PARAMETER", LC_REG_CHANGE_PARAM,
                     change_parameter_value(s_BattProfile.chrg_voltage), &param_written);
    s_ReinitRsoc |= param_written;
    if (res != 0)
      return false;

    // Need delay after write 0x12 command (Figure 18 in the LC709203F datasheet):
    if (param_written)
      vTaskDelay(pdMS_TO_TICKS(FG_PARAM_SETTLE_MS));
  }

  // Initial RSOC:
  if (s_ReinitRsoc)
  {
    LOG_INFO("[FG] battery parameters changed, re-initialising RSOC");

    // RSOC Initialization:
    res = writeWord(LC_INIT_RSOC_REG, 0xAA55);
    if (res != 0)
      return false;

    // Try read RSOC:
    res = readWord(LC_REG_ITE, &value);
    if (res != 0)
      return false;
    LOG_INFO("[FG] %s: rsoc=%u.%u%%", STRINGIFY(LC_INIT_RSOC_REG), (unsigned)(value / 10), (unsigned)(value % 10));

    s_ReinitRsoc = false;   // done - the only place it is cleared
  }

  // Thermistor:
  {
    // Set NTC mode:
    res = writeWord(LC_REG_STATUS_BIT, LC_STATUS_TEMP_THERMISTOR);
    if (res != 0)
      return false;

    /* NTC B constant. Zero is this module's "not stated" - see profileConvert() - and writing that
     * would poison the very register the IC derives 0x08 from. Left alone, the IC keeps its own
     * default and read_Temp() refuses the reading anyway. */
    if (s_BattProfileValid && s_BattProfile.NTC.b_const != 0)
    {
      res = writeWord(LC_REG_THERMISTOR_B, s_BattProfile.NTC.b_const);
      if (res != 0)
        return false;
    }

    // Adjustment Pack Thermistor:
    res = writeWord(LC_REG_APT, LC_APT_DEFAULT);
    if (res != 0)
      return false;
  }

  return true;
}

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
static int8_t ntcKelvin2Celsius(uint16_t _raw, uint16_t _ntcB, uint16_t _ntcOhms)
{
  static const int16_t s_LogTbl[256] = { -24562, -21803, -19743, -18097, -16728,
      -15554, -14528, -13616, -12795, -12050, -11366, -10735, -10149, -9602,
      -9090, -8607, -8151, -7720, -7310, -6919, -6547, -6190, -5848, -5520,
      -5205, -4901, -4608, -4326, -4052, -3788, -3532, -3283, -3042, -2808,
      -2580, -2358, -2142, -1932, -1727, -1527, -1332, -1141, -955, -773, -595,
      -420, -249, -82, 81, 242, 400, 554, 706, 855, 1002, 1145, 1287, 1426,
      1562, 1697, 1829, 1959, 2087, 2213, 2338, 2460, 2581, 2700, 2817, 2932,
      3046, 3158, 3269, 3378, 3486, 3593, 3698, 3802, 3904, 4005, 4105, 4204,
      4302, 4398, 4494, 4588, 4681, 4773, 4864, 4954, 5043, 5132, 5219, 5305,
      5391, 5475, 5559, 5642, 5724, 5805, 5885, 5965, 6044, 6122, 6199, 6276,
      6352, 6427, 6501, 6575, 6648, 6721, 6793, 6864, 6935, 7005, 7074, 7143,
      7212, 7279, 7347, 7413, 7479, 7545, 7610, 7675, 7739, 7802, 6487, 7188,
      7834, 8432, 8990, 9513, 10004, 10467, 10905, 11322, 11718, 12096, 12457,
      12803, 13135, 13454, 13761, 14057, 14343, 14619, 14886, 15144, 15395,
      15638, 15875, 16104, 16328, 16545, 16757, 16964, 17165, 17362, 17554,
      17741, 17925, 18104, 18280, 18451, 18620, 18785, 18947, 19105, 19261,
      19413, 19563, 19710, 19855, 19997, 20137, 20274, 20409, 20542, 20673,
      20802, 20928, 21053, 21176, 21297, 21416, 21534, 21650, 21764, 21877,
      21988, 22098, 22207, 22313, 22419, 22523, 22626, 22728, 22828, 22927,
      23025, 23122, 23217, 23312, 23406, 23498, 23589, 23680, 23769, 23858,
      23945, 24032, 24117, 24202, 24286, 24369, 24451, 24533, 24613, 24693,
      24772, 24851, 24928, 25005, 25081, 25157, 25231, 25305, 25379, 25452,
      25524, 25595, 25666, 25736, 25806, 25875, 25944, 26011, 26079, 26146,
      26212, 26278, 26343, 26408, 26472, 26536, 26599, 26661, 26724, 26785,
      26847, 26908, 26968, 27028, 27088 };

  if (_ntcOhms == 1000)   // 10 Ohm units, so 1000 = 10 kOhm = what the IC assumes
    return ((int16_t)_raw - 2732) / 10;

  /* Index into the log table by the NTC's nominal resistance. Inherited fixed point, left as is. */
  int32_t dr25 = _ntcOhms / 10;
  int32_t it = dr25 < 261 ? (dr25 - 4) >> 1 : ((dr25 + 2300) * 13) >> 8;
  it = it < 0 ? 0 : it;
  it = it > 255 ? 255 : it;

  /*
   * T = (B * T10k) / (B - T10k*log(k)).
   *
   * The factor of 10 that _raw carries cancels between the two sides, so the quotient comes out in
   * whole kelvin - a digit coarser than the branch above, which is why the two paths used to
   * disagree by a degree on the same sensor. Taking the remainder back out recovers the tenth
   * without widening the multiply, so both paths finish on the same 273.2 K subtraction.
   */
  int32_t num = (int32_t)_ntcB * _raw;
  int32_t den = (int32_t)_ntcB * 10 - (((int32_t)_raw * s_LogTbl[it]) >> 13);

  /* Only reachable with a B constant far too small to be a thermistor's - a corrupt custom
   * profile. Without this the division below would trap or return nonsense. */
  if (den <= 0)
    return FUEL_GAUGE_TEMP_UNKNOWN;

  int32_t kelvinX10 = (num / den) * 10 + ((num % den) * 10) / den;
  return (int8_t)((kelvinX10 - 2732) / 10);
}

/*
 * Both readers below only measure: they take the profile as an input and hand the answer back
 * through _out, and they never touch the published s_Batt* values. What a failed read does to the
 * published value is policy, and it lives in the publish_*() pair further down.
 *
 * They log per sample, at LOG_VERBOSE - this runs once a second, so anything louder would fill the
 * log for as long as a fault is present. The one-off warning belongs to the publisher, which is
 * what knows the value has actually gone unknown.
 */
static FgReadResult_t read_Temp(int8_t *_celsius)
{
  uint16_t temp_raw;
  int8_t rc = readWord(LC_REG_CELL_TEMP, &temp_raw);
  if (rc > 0)
    return FG_READ_BUS_ERROR;   // readWord() has already logged it

  if (rc < 0)
  {
    LOG_VERBOSE("[FG] <read_Temp> FAILED. Temperature CRC mismatch");
    return FG_READ_NO_DATA;
  }

  // Check profile:
  if (!s_BattProfileValid ||
      s_BattProfile.NTC.b_const == 0 ||
      s_BattProfile.NTC.resistance == 0)
  {
    LOG_VERBOSE("[FG] <read_Temp> battery profile carries no NTC data");
    return FG_READ_NO_DATA;
  }

  // A compatible NTC cannot read this low - an open sensor or none fitted at all:
  if (temp_raw <= LC_TEMP_MIN_VALID)
  {
    LOG_VERBOSE("[FG] <read_Temp> NTC reads %u (0.1 K), open or not fitted", (unsigned)temp_raw);
    return FG_READ_NO_DATA;
  }

  int8_t celsius = ntcKelvin2Celsius(temp_raw, s_BattProfile.NTC.b_const,
                                     s_BattProfile.NTC.resistance);
  if (celsius == FUEL_GAUGE_TEMP_UNKNOWN)
  {
    LOG_VERBOSE("[FG] <read_Temp> conversion failed, profile B=%u res=%u",
                (unsigned)s_BattProfile.NTC.b_const, (unsigned)s_BattProfile.NTC.resistance);
    return FG_READ_NO_DATA;
  }

  LOG_VERBOSE("[FG] temperature %d C (raw %u)", (int)celsius, (unsigned)temp_raw);
  *_celsius = celsius;
  return FG_READ_OK;
}

static FgReadResult_t read_RSOC(uint16_t *_rsoc)
{
  uint16_t rsoc;
  int8_t rc = readWord(LC_REG_ITE, &rsoc);
  if (rc > 0)
    return FG_READ_BUS_ERROR;   // readWord() has already logged it

  if (rc < 0)
  {
    LOG_VERBOSE("[FG] <read_RSOC> FAILED. RSOC CRC mismatch");
    return FG_READ_NO_DATA;
  }

  /* Register 0x0F is in 0.1 percent units, so 1000 is a full pack and anything above it is not a
   * charge level at all. The CRC should have caught that already - this is here so that nothing
   * outside the range can reach the thresholds in battery_UpdateChargeLed(). */
  if (rsoc > 1000)
  {
    LOG_VERBOSE("[FG] <read_RSOC> out of range: %u", (unsigned)rsoc);
    return FG_READ_NO_DATA;
  }

  LOG_VERBOSE("[FG] RSOC %u.%u", (unsigned)(rsoc / 10), (unsigned)(rsoc % 10));
  *_rsoc = rsoc;
  return FG_READ_OK;
}

static void publish_Temp(FgReadResult_t _res, int8_t _temp)
{
  static uint8_t stale_cnt;

  switch (_res)
  {
    case FG_READ_OK:
      stale_cnt = 0;
      if (s_BattTemp != _temp)
        LOG_INFO("[FG] Read batt temp: %d grad.", (int)_temp);
      s_BattTemp = _temp;
      break;

    case FG_READ_NO_DATA:
      if (stale_cnt < FG_STALE_LIMIT)
      {
        stale_cnt++;
        if (stale_cnt == FG_STALE_LIMIT)
        {
          LOG_WARNING("[FG] No usable temperature: currently UNKNOWN");
          s_BattTemp = FUEL_GAUGE_TEMP_UNKNOWN;
        }
      }
      break;

    default:
      break;
  }
}

static void publish_RSOC(FgReadResult_t _res, uint16_t _rsoc)
{
  static uint8_t stale_cnt;

  switch (_res)
  {
    case FG_READ_OK:
      stale_cnt = 0;
      if (s_BattRsoc != _rsoc)
        LOG_INFO("[FG] Read batt RSOC: %d.%d%%", (unsigned)(_rsoc / 10), (unsigned)(_rsoc % 10));
      s_BattRsoc = _rsoc;
      break;

    case FG_READ_NO_DATA:
      if (stale_cnt < FG_STALE_LIMIT)
      {
        stale_cnt++;
        if (stale_cnt == FG_STALE_LIMIT)
        {
          LOG_WARNING("[FG] No usable RSOC: currently UNKNOWN");
          s_BattRsoc = FUEL_GAUGE_RSOC_UNKNOWN;
        }
      }
      break;

    default:
      break;
  }
}

static bool cmdProcess(const FuelGaugeEvent_t *_ev)
{
  switch (_ev->cmd.id)
  {
    case FG_CMD_SET_CONFIG:
      // config not supported NOW. Used fixed DEFAULT configuration.
      return false;

    case FG_CMD_SET_BAT_PROFILE:
      if (_ev->cmd.battProfile.chrg_voltage != 0)
      {
        s_BattProfile = _ev->cmd.battProfile;
        s_BattProfileValid = true;
      }
      else
        s_BattProfileValid = false;
      return s_BattProfileValid;    // the battery profile is valid and changed, so the IC has a stale copy

    default:
      LOG_WARNING("[FG] unknown command %u", (unsigned)_ev->cmd.id);
      return false;
  }
}

/*
 * Every state below lists every event it answers to - reading one function tells you the whole
 * behaviour of that state, with nothing hidden behind a shared default handler. "break" means
 * stay where we are, "return" names the state to move to.
 *
 * FG_EV_ENTRY is the first event a state ever sees: fsm_Dispatch() delivers it the moment the
 * transition is made, so a state's queue timeout and its entry actions sit next to the rest of
 * its behaviour instead of in a table somewhere else.
 */
static FuelGaugeState_t state_NoBattery(const FuelGaugeEvent_t *_ev)
{
  switch (_ev->type)
  {
    case FG_EV_ENTRY:
      LOG_INFO("[FG] FSM state NO_BATTERY");
      s_StateTimeout = portMAX_DELAY;   // nothing to do until APP says otherwise
      s_BattRsoc = FUEL_GAUGE_RSOC_UNKNOWN;
      s_BattTemp = FUEL_GAUGE_TEMP_UNKNOWN;
      s_IcFault = false;    // no pack means no power to the IC - that is not the IC's fault
      break;

    case FG_EV_BATTERY_PRESENT:
      return FG_ST_IC_INIT;

    case FG_EV_CMD:
      /* With no pack there is nobody to configure, so it does not matter whether the IC holds a
       * copy - starting_flow() reads everything back when a pack turns up. */
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

static FuelGaugeState_t state_IcInit(const FuelGaugeEvent_t *_ev)
{
  static uint8_t init_attempts;
  FuelGaugeState_t next = FG_ST_IC_INIT;
  switch (_ev->type)
  {
    case FG_EV_ENTRY:
    {
      LOG_INFO("[FG] FSM state IC_INIT");
      s_StateTimeout = 0;   // first attempt at once, the tick below paces the retries after it
      s_IcFault = false;    // being tried again - not written off until the attempts run out
      init_attempts = 0;
    }
    break;

    case FG_EV_TICK:
    {
      s_StateTimeout = pdMS_TO_TICKS(FG_INIT_RETRY_MS);

      // IC initialization:
      bool res = starting_flow();
      if (res)
        next = FG_ST_IC_ACTIVE;
      else if (++init_attempts >= FG_INIT_ATTEMPTS)
        return FG_ST_IC_LOST;
    }
    break;

    case FG_EV_BATTERY_ABSENT:
    {
      next = FG_ST_NO_BATTERY;
    }
    break;

    case FG_EV_CMD:
    {
      if (cmdProcess(_ev))
        init_attempts = 0;
    }
    break;

    default:
      LOG_WARNING("[FG] unhandled event %u in IC_INIT", (unsigned)_ev->type);
    break;
  }

  return next;
}

static FuelGaugeState_t state_IcActive(const FuelGaugeEvent_t *_ev)
{
  static uint8_t err_count;
  FuelGaugeState_t next = FG_ST_IC_ACTIVE;
  switch (_ev->type)
  {
    case FG_EV_ENTRY:
      LOG_INFO("[FG] FSM state IC_ACTIVE");
      s_StateTimeout = pdMS_TO_TICKS(FG_MEAS_PERIOD_MS);
      s_IcFault = false;
      err_count = 0;
      break;

    case FG_EV_TICK:
    {
      uint16_t rsoc = FUEL_GAUGE_RSOC_UNKNOWN;
      int8_t temp = FUEL_GAUGE_TEMP_UNKNOWN;

      FgReadResult_t temp_res = read_Temp(&temp);
      FgReadResult_t rsoc_res = read_RSOC(&rsoc);

      publish_Temp(temp_res, temp);
      publish_RSOC(rsoc_res, rsoc);

      // Bus errors management:
      if (temp_res != FG_READ_BUS_ERROR && rsoc_res != FG_READ_BUS_ERROR)
      {
        err_count = 0;
      }
      else if (++err_count >= FG_ERR_LIMIT)
      {
        LOG_ERROR("[FG] %u bus errors in a row", (unsigned)err_count);
        next = FG_ST_IC_LOST;
      }
    }
    break;

    case FG_EV_BATTERY_ABSENT:
      return FG_ST_NO_BATTERY;

    case FG_EV_CMD:
      if (cmdProcess(_ev))
        next = FG_ST_IC_INIT;
    break;

    default:
      LOG_WARNING("[FG] unhandled event %u in IC_ACTIVE", (unsigned)_ev->type);
    break;
  }

  return next;
}

static FuelGaugeState_t state_IcLost(const FuelGaugeEvent_t *_ev)
{
  switch (_ev->type)
  {
    case FG_EV_ENTRY:
      /* The moment the split between the two counters is worth having: NACKs and timeouts point
       * at the bus or the device, CRC mismatches at noise on the data lines. */
      LOG_CRITICAL("[FG] IC lost, next attempt in %u ms", (unsigned)FG_RETRY_PERIOD_MS);
      s_StateTimeout = pdMS_TO_TICKS(FG_RETRY_PERIOD_MS);
      s_BattRsoc = FUEL_GAUGE_RSOC_UNKNOWN;
      s_BattTemp = FUEL_GAUGE_TEMP_UNKNOWN;
      s_IcFault = true;
      break;

    case FG_EV_TICK:
      return FG_ST_IC_INIT;   // periodic recovery attempt

    case FG_EV_BATTERY_ABSENT:
      return FG_ST_NO_BATTERY;

    case FG_EV_CMD:
      // Nothing reaches the IC while it is written off - the recovery attempt applies it all.
      cmdProcess(_ev);
      break;

    default:
      LOG_WARNING("[FG] unhandled event %u in IC_LOST", (unsigned)_ev->type);
      break;
  }

  return FG_ST_IC_LOST;
}

/* Hands an event to whichever state is current. The only place that maps a state to its handler. */
static FuelGaugeState_t stateHandle(FuelGaugeState_t _state, const FuelGaugeEvent_t *_ev)
{
  switch (_state)
  {
    case FG_ST_NO_BATTERY: return state_NoBattery(_ev);
    case FG_ST_IC_INIT:    return state_IcInit(_ev);
    case FG_ST_IC_ACTIVE:  return state_IcActive(_ev);
    case FG_ST_IC_LOST:    return state_IcLost(_ev);
    default:               APP_ERROR(APP_ERR); return _state;
  }
}

/*
 * The whole transition, in one place: ask the current state what to do with the event, and if it
 * asked to move, assign the new state and hand it FG_EV_ENTRY so it can arm itself. Handlers only
 * decide - they cannot switch state as a side effect and then carry on running in it, and nothing
 * but this function assigns s_State.
 */
static void fsm_Dispatch(const FuelGaugeEvent_t *_ev)
{
  static FuelGaugeState_t state = FG_ST_NO_BATTERY;

  // Call FSM:
  FuelGaugeState_t next = stateHandle(state, _ev);

  // State transition processing:
  uint8_t guard = FG_ST_COUNT;
  while (next != state)
  {
    if (guard-- == 0)
    {
      LOG_ERROR("[FG] entry chain does not settle, last state %u", (unsigned)next);
      APP_ERROR(APP_ERR);
    }

    state = next;
    static const FuelGaugeEvent_t entryEv = { .type = FG_EV_ENTRY };
    next = stateHandle(state, &entryEv);
  }
}

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

  // Entry event to start FSM:
  const FuelGaugeEvent_t entryEv = { .type = FG_EV_ENTRY };
  fsm_Dispatch(&entryEv);

  while(1)
  {
    FuelGaugeEvent_t ev;
    if (xQueueReceive(s_QueHandle, &ev, s_StateTimeout) != pdTRUE)
      ev.type = FG_EV_TICK;

    fsm_Dispatch(&ev);
  }
}

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
      LOG_CRITICAL("[FG] event queue full, ev=%u dropped", (unsigned)_ev->type);
      taskENTER_CRITICAL();
      s_ErrMask |= FUEL_GAUGE_ERR_QUEUE_FULL;
      taskEXIT_CRITICAL();
    }
  }

  return (sent == pdTRUE);
}

void fuel_gauge_Init(BatteryProfile_T *_p_batt_profile)
{
  LOG_INFO("fuel_gauge_Init...");

  if (_p_batt_profile)
  {
    profileConvert(_p_batt_profile, &s_BattProfile);
    s_BattProfileValid = true;
  }

  s_QueHandle = xQueueCreateStatic(sizeof(s_QueBuf) / sizeof(s_QueBuf[0]),
                            sizeof(s_QueBuf[0]), (uint8_t*)s_QueBuf, &s_Que);
  ASSERT(s_QueHandle != NULL);

  s_TaskHandle = xTaskCreateStatic(Task, "LC709203F", sizeof(s_TaskStack) / sizeof(StackType_t),
                            NULL, 6, s_TaskStack, &s_TaskTCB);
  ASSERT(s_TaskHandle != NULL);
}

void fuel_gauge_SetConfig(uint8_t _config)
{
  FuelGaugeEvent_t ev = { .type = FG_EV_CMD };
  ev.cmd.id = FG_CMD_SET_CONFIG;
  ev.cmd.config = _config;
  postEvent(&ev);
}

void fuel_gauge_SetBattProfile(const BatteryProfile_T *_p_batt_profile)
{
  FuelGaugeEvent_t ev = { .type = FG_EV_CMD };
  ev.cmd.id = FG_CMD_SET_BAT_PROFILE;
  /* A zeroed profile is how "no profile" travels: chrg_voltage cannot come out 0 for a real one,
   * because the conversion adds the 3.5 V offset. cmdProcess() tests exactly that. */
  FgBattProfile_t profile = { 0 };
  if (_p_batt_profile)
    profileConvert(_p_batt_profile, &profile);
  ev.cmd.battProfile = profile;

  postEvent(&ev);
}

void fuel_gauge_SetBatteryPresent(bool present)
{
  FuelGaugeEvent_t ev = { .type = present ? FG_EV_BATTERY_PRESENT : FG_EV_BATTERY_ABSENT };
  postEvent(&ev);
}

uint16_t fuel_gauge_GetRsoc(void)
{
  return s_BattRsoc;
}

int8_t fuel_gauge_GetTemp(void)
{
  return s_BattTemp;
}

BatteryTempSenseConfig_t fuel_gauge_GetTempSenseConfig(void)
{
  return BAT_TEMP_SENSE_CONFIG_NTC;
}

bool fuel_gauge_IsIcFault(void)
{
  return s_IcFault;
}

bool fuel_gauge_IsTempSenseFault(void)
{
  return s_BattTemp == FUEL_GAUGE_TEMP_UNKNOWN;
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
  return FUEL_GAUGE_CONFIG_DEFAULT;
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
