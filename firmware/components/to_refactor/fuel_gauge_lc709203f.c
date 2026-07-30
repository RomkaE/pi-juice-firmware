/*
 * fuel_gauge_lc709203f.c
 *
 *  Created on: 06.12.2016.
 *      Author: milan
 */

#include <stdint.h>
#include <stdbool.h>
#include "iosystem/analog.h"
#include <to_refactor/crc8_atm.h>
#include "fuel_gauge_lc709203f.h"
#include <to_refactor/time_count.h>
#include "stm32f0xx_hal.h"
#include "driver/i2c/i2c_master.h"
#include "charger_bq2416x.h"
#include "nv.h"
#include "app-error/app_assert.h"

// FreeRTOS:
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

// LOG:
#include "log/log.h"

#define FG_COUNT_SATURATING(c)	do { if ((c) < 0xFFFF) (c)++; } while (0)

/* Command types carried by the task queue. */
typedef enum
{
  FUEL_GAUGE_CMD_SET_BAT_PROFILE = 1,  // posted by APP on APP_EVT_BATTERY_PROFILE_CHANGED
  FUEL_GAUGE_CMD_SET_CONFIG            // host register 0x93 - fuel_gauge_CmdSetConfig()
} FuelGaugeCmdType_T;

typedef struct
{
  bool valid;
  BatteryProfile_T profile;
} FuelGaugeCmdBatProfile_T;

typedef struct
{
  uint8_t type;
  uint8_t seq;    // meaningful for CMD_SET_CONFIG only
  union
  {
    FuelGaugeCmdBatProfile_T batProfile;
    uint8_t config;
  } data;
} FuelGaugeCmd_T;

static uint16_t s_BatteryVoltage = 0xFFFF;
static uint16_t s_BatteryRsoc __attribute__((section("no_init")));
static uint16_t s_FuelGaugeTemp = 0;
static int8_t s_BatteryTemp = 25;
static volatile int16_t s_BatteryCurrent;

static uint32_t s_FuelGaugeTaskTimer;

static volatile int32_t s_DischargeRate = 0;
static uint32_t s_DischargeCount;
static uint32_t s_DischargeCountTemp;

static uint16_t s_PrevRsoc;
static uint8_t s_PrevBatPresent = 0;

static int8_t s_FuelGaugeI2cErrorCounter = 0;

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
 * meant to address.
 */
static uint16_t s_FuelGaugeHalErrorCount = 0;
static uint16_t s_FuelGaugeCrcErrorCount = 0;

static BatteryTempSenseConfig_T s_TempSensorConfig = BAT_TEMP_SENSE_CONFIG_AUTO_DETECT;
static RsocMeasurementConfig_T s_RsocMeasurementConfig = RSOC_MEASUREMENT_AUTO_DETECT;

static int8_t s_NtcFaultFlag __attribute__((section("no_init")));
static volatile uint8_t s_FuelGaugeTempMode = FUEL_GAUGE_TEMP_MODE_THERMISTOR;

static const int16_t s_LogTbl[256]={-24562, -21803, -19743, -18097, -16728, -15554, -14528, -13616, -12795, -12050, -11366, -10735, -10149, -9602, -9090, -8607, -8151, -7720, -7310, -6919, -6547, -6190, -5848, -5520, -5205, -4901, -4608, -4326, -4052, -3788, -3532, -3283, -3042, -2808, -2580, -2358, -2142, -1932, -1727, -1527, -1332, -1141, -955, -773, -595, -420, -249, -82, 81, 242, 400, 554, 706, 855, 1002, 1145, 1287, 1426, 1562, 1697, 1829, 1959, 2087, 2213, 2338, 2460, 2581, 2700, 2817, 2932, 3046, 3158, 3269, 3378, 3486, 3593, 3698, 3802, 3904, 4005, 4105, 4204, 4302, 4398, 4494, 4588, 4681, 4773, 4864, 4954, 5043, 5132, 5219, 5305, 5391, 5475, 5559, 5642, 5724, 5805, 5885, 5965, 6044, 6122, 6199, 6276, 6352, 6427, 6501, 6575, 6648, 6721, 6793, 6864, 6935, 7005, 7074, 7143, 7212, 7279, 7347, 7413, 7479, 7545, 7610, 7675, 7739, 7802, 6487, 7188, 7834, 8432, 8990, 9513, 10004, 10467, 10905, 11322, 11718, 12096, 12457, 12803, 13135, 13454, 13761, 14057, 14343, 14619, 14886, 15144, 15395, 15638, 15875, 16104, 16328, 16545, 16757, 16964, 17165, 17362, 17554, 17741, 17925, 18104, 18280, 18451, 18620, 18785, 18947, 19105, 19261, 19413, 19563, 19710, 19855, 19997, 20137, 20274, 20409, 20542, 20673, 20802, 20928, 21053, 21176, 21297, 21416, 21534, 21650, 21764, 21877, 21988, 22098, 22207, 22313, 22419, 22523, 22626, 22728, 22828, 22927, 23025, 23122, 23217, 23312, 23406, 23498, 23589, 23680, 23769, 23858, 23945, 24032, 24117, 24202, 24286, 24369, 24451, 24533, 24613, 24693, 24772, 24851, 24928, 25005, 25081, 25157, 25231, 25305, 25379, 25452, 25524, 25595, 25666, 25736, 25806, 25875, 25944, 26011, 26079, 26146, 26212, 26278, 26343, 26408, 26472, 26536, 26599, 26661, 26724, 26785, 26847, 26908, 26968, 27028, 27088};
static const int16_t s_OcvSocTableNormLipo[256]={4269,3804,3447,3141,2877,2641,2435,2247,2075,1923,1788,1669,1573,1497,1441,1395,1359,1331,1306,1285,1265,1248,1233,1219,1205,1193,1181,1169,1155,1138,1121,1101,1078,1056,1033,1012,993,973,954,937,919,901,884,867,851,833,818,802,788,773,758,747,734,721,711,699,687,676,665,654,642,632,622,609,600,589,578,568,557,547,536,526,517,507,499,489,481,472,462,454,446,438,430,420,412,404,396,388,378,370,362,354,345,338,328,320,310,302,293,284,277,268,260,250,241,232,223,213,204,195,185,175,163,155,144,135,123,113,103,92,82,69,59,47,36,23,12,0,-11,-25,-37,-49,-63,-76,-89,-103,-118,-132,-147,-161,-177,-192,-207,-224,-239,-256,-273,-291,-308,-327,-344,-363,-382,-401,-420,-441,-460,-481,-501,-523,-543,-565,-589,-610,-632,-656,-678,-702,-724,-748,-771,-796,-819,-842,-865,-889,-912,-936,-960,-984,-1008,-1031,-1056,-1079,-1102,-1127,-1150,-1175,-1199,-1224,-1247,-1273,-1297,-1323,-1347,-1371,-1396,-1421,-1446,-1471,-1499,-1525,-1551,-1577,-1604,-1632,-1658,-1685,-1712,-1740,-1768,-1796,-1824,-1851,-1880,-1909,-1938,-1967,-1995,-2024,-2053,-2082,-2112,-2142,-2173,-2203,-2234,-2264,-2296,-2327,-2358,-2388,-2420,-2452,-2484,-2515,-2548,-2580,-2614,-2646,-2679,-2712,-2747,-2780,-2814,-2850,-2884,-2920,-2956,-2993,-3031,-3069,-3110,-3154,-3200,-3256};
static const int16_t s_OcvSocTableNormLifepo4[256]={28097,26548,25030,23690,22426,21239,20117,19105,18094,17174,16307,15467,14648,13910,13247,12597,11996,11441,10945,10475,10040,9663,9293,8975,8686,8399,8155,7934,7715,7522,7342,7178,7007,6856,6700,6554,6403,6253,6109,5967,5814,5677,5539,5413,5285,5172,5054,4949,4832,4730,4629,4528,4428,4330,4227,4137,4045,3947,3859,3774,3695,3618,3532,3448,3371,3297,3221,3152,3076,3007,2946,2880,2803,2741,2676,2608,2535,2475,2401,2326,2259,2192,2124,2045,1977,1910,1829,1763,1690,1624,1554,1485,1417,1358,1293,1228,1171,1114,1058,992,941,881,829,774,728,684,634,585,544,502,461,418,389,353,322,286,255,229,197,172,145,122,101,76,51,33,12,0,-19,-39,-64,-78,-97,-107,-127,-145,-158,-171,-191,-209,-224,-233,-248,-259,-272,-290,-306,-323,-334,-352,-353,-373,-384,-401,-410,-426,-448,-461,-477,-494,-503,-521,-541,-552,-573,-586,-609,-628,-643,-668,-688,-710,-731,-753,-775,-798,-827,-850,-882,-913,-942,-979,-1006,-1042,-1080,-1113,-1149,-1185,-1225,-1264,-1303,-1339,-1379,-1424,-1471,-1517,-1546,-1593,-1628,-1665,-1703,-1745,-1788,-1823,-1861,-1887,-1926,-1958,-1989,-2023,-2060,-2089,-2115,-2140,-2169,-2192,-2220,-2237,-2261,-2284,-2311,-2334,-2343,-2368,-2396,-2410,-2428,-2447,-2464,-2481,-2497,-2519,-2536,-2556,-2578,-2595,-2616,-2635,-2662,-2684,-2705,-2753,-2765,-2795,-2836,-2870,-2908,-2961,-3014,-3082,-3165,-3277,-3432,-3694,-4197,-5466};
static uint16_t s_OcvSocTbl[256] __attribute__((section("no_init")));
static uint16_t s_RSocTbl[256] __attribute__((section("no_init")));
static uint8_t s_RSocTempCompensateTbl[256] __attribute__((section("no_init")));
static uint16_t s_C0 __attribute__((section("no_init")));
static int32_t s_Soc __attribute__((section("no_init")));

// Battery profile, copied in from battery_GetProfile()/fuel_gauge_CmdSetBatProfile() - never a
// live pointer into another task's memory.
static BatteryProfile_T s_BatProfile;
static bool s_BatProfileValid;

/* Request mirror for the host config register, same idiom as led.c/charger.c. */
static uint8_t s_ReqConfig;
static volatile uint8_t s_ReqConfigSeq;
static volatile uint8_t s_AppliedConfigSeq;

static uint32_t s_ErrMask;  // FUEL_GAUGE_ERR_* bits

// FreeRTOS task:
static TaskHandle_t s_TaskHandle;
static StaticTask_t s_TaskTCB;
static StackType_t s_TaskStack[512];   // TODO - remove magic number

// Command queue:
static QueueHandle_t s_QueHandle;
static StaticQueue_t s_Que;
static FuelGaugeCmd_T s_QueBuf[4];     // TODO - remove magic number

/*============================ TASK PRIVATE ==================================*/

static int8_t readWord(uint8_t cmd, uint16_t *word)
{
  uint8_t readData[10] = {0x16, cmd, 0x17, 0, 0, 0};

  int rc = i2c_master_ReadMem(0x16, cmd, readData+3, 3);
  if (rc != I2C_OK) {
    FG_COUNT_SATURATING(s_FuelGaugeHalErrorCount);
    /* Positive = transport error (device absent / bus). socEvaluateFuelGaugeIc() uses the sign
     * to tell this from a CRC mismatch (-1, device present). */
    return 1;
  }
  if (Crc8Block(0, readData, 5) == readData[5]) {
    *word = (((uint16_t)readData[4])<<8) | readData[3];
    return 0;
  } else {
    FG_COUNT_SATURATING(s_FuelGaugeCrcErrorCount);
    return -1;
  }
}

static int8_t writeWord(uint8_t cmd, uint16_t word)
{
  uint8_t writeData[5] = {0x16, cmd, word, word>>8, 0};
  writeData[4] = Crc8Block(0, writeData, 4);
  int rc = i2c_master_WriteMem(0x16, cmd, (uint8_t*)&writeData[2], 3);
  if (rc != I2C_OK) FG_COUNT_SATURATING(s_FuelGaugeHalErrorCount);
  return (rc == I2C_OK) ? 0 : 1;
}

static int32_t getSocFromOcv(uint16_t ocv)
{
  int32_t i;
  for (i = 0; i < 256; i++) {
    if (s_OcvSocTbl[i]>=ocv)
      return i<<23;
  }
  return ((int32_t)255)<<23;
}

static void dvInit(void)
{
  int16_t i;
  uint16_t ocv50 = 3800, ocv10 = 3649, ocv90 = 4077;
  static uint16_t r50=1.82*156, r10=1.82*160, r90=1.82*155; // BP7X
  int32_t ocvRef50 = 3791, ocvRef10 = 3652, ocvRef90 = 4070;
  const int16_t *ocvSocTableRef = s_OcvSocTableNormLipo;
  s_C0 = 1820;

  if (s_BatProfileValid) {
    if (s_BatProfile.chemistry==BAT_CHEMISTRY_LIFEPO4) {
      ocvSocTableRef = s_OcvSocTableNormLifepo4;
      ocvRef50 = 3243;
      ocvRef10 = 3111;
      ocvRef90 = 3283;
    } else {
      ocvSocTableRef = s_OcvSocTableNormLipo;
      ocvRef50 = 3791;
      ocvRef10 = 3652;
      ocvRef90 = 4070;
    }

    s_C0 = s_BatProfile.capacity;
    if (s_BatProfile.ocv50 != 0xFFFF) {
      ocv50 = s_BatProfile.ocv50;
    } else {
      ocv50 = ((uint16_t)s_BatProfile.regulationVoltage+175+s_BatProfile.cutoffVoltage)*10;
    }
    if (s_BatProfile.ocv10 != 0xFFFF) ocv10 = s_BatProfile.ocv10; else ocv10 = 0.96322*ocv50;
    if (s_BatProfile.ocv90 != 0xFFFF) ocv90 = s_BatProfile.ocv90; else ocv90 = 1.0735*ocv50;

    if (s_BatProfile.r50 != 0xFFFF) r50 = (int32_t)s_C0*s_BatProfile.r50/100000;
    if (s_BatProfile.r10 != 0xFFFF) r10 = (int32_t)s_C0*s_BatProfile.r10/100000; else r10 = r50;
    if (s_BatProfile.r90 != 0xFFFF) r90 = (int32_t)s_C0*s_BatProfile.r90/100000; else r90 = r50;
  }

  int32_t k1 = ((ocvRef90-ocvRef50)*(230-26)*(ocv50-ocv10))>>10;
  for (i = 0; i < 26; i++) {
    s_OcvSocTbl[i] = ocv50 - ((k1*ocvSocTableRef[i])>>16);
    s_RSocTbl[i] = 65535/(r50+(r50-r10)*(i-128.0)/(128-26));
  }

  int32_t OCVdSoc50 = (((int32_t)ocv50)<<13) / (230-26);
  k1 = ((ocvRef90-ocvRef50)*(ocv50-ocv10))>>4;
  int32_t k2 = (((ocvRef50-ocvRef10)*(ocv90-ocv50))>>4);
  for (i = 26; i < 128; i++) {
    int32_t d1= (OCVdSoc50 - ((k1*ocvSocTableRef[i])>>9))*(230.0-i);
    int32_t d2= (OCVdSoc50 - ((k2*ocvSocTableRef[i])>>9))*(i-26);
    s_OcvSocTbl[i] = (d2+d1)>>13;
    s_RSocTbl[i] = 65535/(r50+(r50-r10)*(i-128.0)/(128-26));
  }

  k2 = ((ocvRef50-ocvRef10)*(230-26)*(ocv90-ocv50))>>10;
  for (i = 128; i < 256; i++) {
    s_OcvSocTbl[i] = ocv50 - ((k2*ocvSocTableRef[i])>>16);
    s_RSocTbl[i] = 65535/(r50+(r50-r90)*(i-128.0)/(128-230));
  }

  for (i = -127; i < 129; i++) {
    s_RSocTempCompensateTbl[(uint8_t)i] = i < 21 ? (uint32_t)255 * 32 / (32 + 2*(20-i)) : 255; // 1 + 2*(20-batteryTemp)/(20-(-12)), krtemp ~ 3, temperature=i
  }

  s_BatteryCurrent = 0;
}

static int8_t icPreInit(void)
{
  // Set operational mode
  return writeWord(0x15, 0x0001); //3.7V
}

static int8_t icInit(void)
{
  int8_t succ;

  LOG_INFO("[FG] icInit...");
  uint16_t fgIcId;
  int8_t res = readWord(0x11, &fgIcId);
  if (res == 0)
  {
    LOG_INFO("[FG] IC Version: 0x%04X", fgIcId);
    // Set operational mode
    succ = writeWord(0x15, 0x0001); //3.7V
    if (succ != 0) return 1;

    // set APA
    succ = writeWord(0x0b, 0x36);
    if (succ != 0) return 2;

    // set change of the parameter
    succ = writeWord(0x12, 0x0001);
    if (succ != 0) return 3;

    // set APT
    succ = writeWord(0x0C, 0x3000);
    if (succ != 0) return 4;

    if (s_BatProfileValid && s_BatProfile.ntcB != 0xFFFF) {
      // Set NTC B constant
      succ = writeWord(0x06, s_BatProfile.ntcB);
      if (succ != 0) return 5;
    }

    // Set NTC mode
    succ = writeWord(0x16, 0x0001);
    if (succ != 0) return 6;
    s_FuelGaugeTempMode = FUEL_GAUGE_TEMP_MODE_THERMISTOR;

    succ = readWord(0x09, &s_BatteryVoltage);
    if (succ != 0) return 7;
    succ = readWord(0x0F, &s_BatteryRsoc);
    if (succ != 0) return 8;

    s_PrevRsoc = s_BatteryRsoc;
    s_DischargeCount = HAL_GetTick();
    s_DischargeCountTemp = s_DischargeCount;

    s_BatteryTemp = analog_GetTempMCU();

    LOG_INFO("[FG] icInit...DONE");
    return 0;
  } else {
    LOG_WARNING("[FG] Failed to read IC version");
    return -1;
  }
}

static void fuelGaugeInit(void)
{
  uint8_t config;
  if (nv_read_U8(NV_ADDR_FUEL_GAUGE_CONFIG, &config) == NV_OK) {
    s_TempSensorConfig = config&0x07;
    s_RsocMeasurementConfig = (config>>4)&0x03;
  }

  dvInit();

  uint16_t batVolt = analog_GetVBatt();
  if (batVolt > 2550) {
    if (s_Soc < 0 || s_Soc>2139095040)
      s_Soc = getSocFromOcv(batVolt);

    s_NtcFaultFlag = 0;
    if (icInit() != 0) {
      s_FuelGaugeI2cErrorCounter = -1;
    } else {
      s_FuelGaugeI2cErrorCounter = 0;
    }

  } else {
    s_BatteryRsoc = 0;
  }

  MS_TIME_COUNTER_INIT(s_FuelGaugeTaskTimer);
}

static void socEvaluateFuelGaugeIc(void)
{
  volatile int8_t succ;
  // read battery voltage
  succ = readWord(0x09, &s_BatteryVoltage);
  if (succ == 0) {
    s_FuelGaugeI2cErrorCounter = 0;
  } else if (succ > 0) {
    s_FuelGaugeI2cErrorCounter = s_FuelGaugeI2cErrorCounter < 127 ? s_FuelGaugeI2cErrorCounter + 1 : 127;
    return;
  } else {
    // CRC mismatch: the IC answered, so it is present - do not count it as absent, but
    // batteryVoltage was left untouched, so skip this cycle instead of deriving anything from
    // the stale value.
    return;
  }

  // read state of charge
  succ = readWord(0x0F, &s_BatteryRsoc);
  if (succ == 0) {
    s_FuelGaugeI2cErrorCounter = 0;
    s_Soc = ((int32_t)s_BatteryRsoc) << 21;
  } else if (succ > 0) {
    s_FuelGaugeI2cErrorCounter = s_FuelGaugeI2cErrorCounter < 127 ? s_FuelGaugeI2cErrorCounter + 1 : 127;
    return;
  } else {
    // CRC mismatch, see above - batteryRsoc still holds the previous value.
    return;
  }

  if (s_BatteryRsoc != s_PrevRsoc && (HAL_GetTick() - s_DischargeCount) > 500) {
    s_DischargeRate = ((int32_t)s_PrevRsoc - s_BatteryRsoc) * (int32_t)1843200 / (int32_t)(HAL_GetTick() - s_DischargeCount);
    s_BatteryCurrent = (s_DischargeRate * (s_BatProfileValid ? s_BatProfile.capacity : 10000)) >> 10;
    s_DischargeCount = HAL_GetTick();
    s_DischargeCountTemp = HAL_GetTick();
    s_PrevRsoc = s_BatteryRsoc;
  } else if ((HAL_GetTick() - s_DischargeCount) > 1843200) {
    s_DischargeRate = 0;
    s_DischargeCount = HAL_GetTick();
    s_DischargeCountTemp = HAL_GetTick();
  } else if ((HAL_GetTick() - s_DischargeCountTemp) > 50000) {
    s_DischargeCountTemp = HAL_GetTick();
  }
}

static void fuelGaugeTick(void)
{
  volatile int8_t succ;
  static uint8_t updateCnt;

  int32_t dt = MS_TIME_COUNT(s_FuelGaugeTaskTimer);
  if ( dt > 125 ) {

    MS_TIME_COUNTER_INIT(s_FuelGaugeTaskTimer);
    uint16_t batVolt = analog_GetVBattAvg();

    if (charger_IsBatteryPresent() && batVolt > 2550) {
      if (!s_PrevBatPresent) {
        s_PrevBatPresent = 1;
        if (s_RsocMeasurementConfig == RSOC_MEASUREMENT_DIRECT_DV) s_Soc = getSocFromOcv(batVolt);
        return;
      }
      updateCnt++;

      if (s_RsocMeasurementConfig == RSOC_MEASUREMENT_AUTO_DETECT && s_FuelGaugeI2cErrorCounter < 5 && s_FuelGaugeI2cErrorCounter > -5) {
        if ( s_FuelGaugeI2cErrorCounter >= 0 ) {
          // in case fuel gauge ic is present use it
          if (updateCnt&0x04) socEvaluateFuelGaugeIc();
        } else {
          // try to reinitialize
          if (icInit() != 0) {
            s_FuelGaugeI2cErrorCounter = s_FuelGaugeI2cErrorCounter > -127 ? s_FuelGaugeI2cErrorCounter - 1 : -127;
          } else {
            s_FuelGaugeI2cErrorCounter = 0;
          }
        }
      } else {
        s_BatteryVoltage = batVolt;
        int32_t ind = s_Soc>>23;
        int32_t dif= (int32_t)s_OcvSocTbl[ind]-s_BatteryVoltage;
        uint16_t rsoc = ((int32_t)s_RSocTbl[ind] * s_RSocTempCompensateTbl[(uint8_t)s_BatteryTemp]) >> 8;
        s_BatteryCurrent = (dif*(((uint32_t)s_C0*rsoc)>>6))>>10;
        int32_t dSoC = (((int32_t)dif*dt*(((int32_t)596*rsoc)>>8)))>>8;
        s_Soc -= dSoC;
        s_Soc = s_Soc<=2139095040?s_Soc:2139095040;
        s_Soc = (s_Soc>=0)?s_Soc:0;
        s_BatteryRsoc = (s_Soc>>7)*125>>21;
      }

      if (updateCnt&0x08)  {
        if ( s_TempSensorConfig == BAT_TEMP_SENSE_CONFIG_AUTO_DETECT || s_TempSensorConfig == BAT_TEMP_SENSE_CONFIG_NTC ) {
          if ( s_FuelGaugeI2cErrorCounter < 5 && s_FuelGaugeI2cErrorCounter >= 0 ) {
            // if left tries
            if (s_FuelGaugeTempMode == FUEL_GAUGE_TEMP_MODE_THERMISTOR) {
              // try to read battery temperature from fuel gauge ic
              succ = readWord(0x08, &s_FuelGaugeTemp);
              if (succ == 0) {
                s_FuelGaugeI2cErrorCounter = 0;
                // check if NTC measurement is valid, compatible NTC sensor should give temp reading above -20C
                if (s_FuelGaugeTemp <= 0x09E4 || !s_BatProfileValid || s_BatProfile.ntcB == 0xFFFF ) {
                  // in case of invalid measurement, use on board measurement and update fuel gauge
                  s_BatteryTemp = analog_GetTempMCU();
                  s_NtcFaultFlag = 1;
                  // Set I2C mode
                  if (writeWord(0x16, 0x0000) == 0) s_FuelGaugeTempMode = FUEL_GAUGE_TEMP_MODE_I2C;
                } else {
                  volatile int16_t ntcTemp;
                  if (s_BatProfile.ntcResistance == 1000) {
                    ntcTemp = ((int16_t)s_FuelGaugeTemp - 2732) / 10;
                  } else {
                    int32_t dr25 = s_BatProfile.ntcResistance / 10;
                    int32_t it = dr25<261 ? (dr25-4)>>1 : ((dr25+2300)*13)>>8;
                    it = it < 0 ? 0 : it;
                    it = it > 255 ? 255 : it;
                    volatile int16_t Tx10 = (s_BatProfile.ntcB * (int32_t)s_FuelGaugeTemp) / (s_BatProfile.ntcB*10 - (((int32_t)s_FuelGaugeTemp*s_LogTbl[it])>>13)); // T = (B * T10k) / (B - T10k*log(k))
                    ntcTemp = Tx10 - 273;
                  }
                  if ( ntcTemp>=23 && ntcTemp<=27 && (analog_GetTempMCU()<15 || analog_GetTempMCU() >45) ) {
                    // there can be fixed resistor instead of NTC, use mcu measurement instead
                    s_BatteryTemp = analog_GetTempMCU();
                    if (writeWord(0x16, 0x0000) == 0) s_FuelGaugeTempMode = FUEL_GAUGE_TEMP_MODE_I2C;
                  } else {
                    s_BatteryTemp = ntcTemp;
                  }
                  s_NtcFaultFlag = 0;
                }
              } else if (succ > 0) {
                // failed reading mean no fuel gauge ic on board
                s_FuelGaugeI2cErrorCounter = s_FuelGaugeI2cErrorCounter < 127 ? s_FuelGaugeI2cErrorCounter + 1 : 127;
              }
            } else {
              s_FuelGaugeTemp = analog_GetTempMCU() * 10 + 2732;
              s_FuelGaugeTemp = s_FuelGaugeTemp > 0x0D04 ? 0x0D04 : s_FuelGaugeTemp;
              s_FuelGaugeTemp = s_FuelGaugeTemp < 0x09E4 ? 0x09E4 : s_FuelGaugeTemp;
              succ = writeWord(0x08, s_FuelGaugeTemp);
              if (succ == 0) {
                s_FuelGaugeI2cErrorCounter = 0;
                // alternate to thermistor mode
                if (writeWord(0x16, 0x0001) == 0) s_FuelGaugeTempMode = FUEL_GAUGE_TEMP_MODE_THERMISTOR;
              } else if (succ > 0) {
                s_FuelGaugeI2cErrorCounter = s_FuelGaugeI2cErrorCounter < 127 ? s_FuelGaugeI2cErrorCounter + 1 : 127;
              }
              s_BatteryTemp = analog_GetTempMCU();
            }
          } else if ( s_FuelGaugeI2cErrorCounter > -5 && s_FuelGaugeI2cErrorCounter < 0 ) {
            // ic is not properly initialized, retry
            if (icInit() != 0) {
              s_FuelGaugeI2cErrorCounter = s_FuelGaugeI2cErrorCounter > -127 ? s_FuelGaugeI2cErrorCounter - 1 : -127;
            } else {
              s_FuelGaugeI2cErrorCounter = 0;
            }
          } else {
            // fuel gauge ic is not responsive or absent
            /*
             * Direct NTC measurement removed together with ADC channel 3 - battery temperature
             * is not measured through the ADC. Without the fuel gauge IC the MCU sensor is the
             * only source left, and there is no NTC left to report a fault about.
             */
            s_BatteryTemp = analog_GetTempMCU();
            s_NtcFaultFlag = 0;
          }
        } else  {
          if ( s_TempSensorConfig == BAT_TEMP_SENSE_CONFIG_ON_BOARD ) {
            s_BatteryTemp = analog_GetTempMCU();
          } else {
            s_BatteryTemp = 25;
          }
          if ( s_FuelGaugeI2cErrorCounter == 0 ) {
            s_FuelGaugeTemp = analog_GetTempMCU() * 10 + 2732;
            s_FuelGaugeTemp = s_FuelGaugeTemp > 0x0D04 ? 0x0D04 : s_FuelGaugeTemp;
            s_FuelGaugeTemp = s_FuelGaugeTemp < 0x09E4 ? 0x09E4 : s_FuelGaugeTemp;
            if (s_FuelGaugeTempMode == FUEL_GAUGE_TEMP_MODE_THERMISTOR) {
              // Set I2C mode
              if (writeWord(0x16, 0x0000) == 0) {
                s_FuelGaugeTempMode = FUEL_GAUGE_TEMP_MODE_I2C;
                s_FuelGaugeI2cErrorCounter = 0;
              } else if (succ > 0) {
                s_FuelGaugeI2cErrorCounter = s_FuelGaugeI2cErrorCounter < 127 ? s_FuelGaugeI2cErrorCounter + 1 : 127;
              }
            }
            if (s_FuelGaugeTempMode == FUEL_GAUGE_TEMP_MODE_I2C) {
              // write temperature data to fuel gauge
              if ( writeWord(0x08, s_FuelGaugeTemp) == 0) {
                s_FuelGaugeI2cErrorCounter = 0;
              } else if (succ > 0) {
                s_FuelGaugeI2cErrorCounter = s_FuelGaugeI2cErrorCounter < 127 ? s_FuelGaugeI2cErrorCounter + 1 : 127;
              }
            }
          }
        }
      }
    } else {
      s_PrevBatPresent = 0;
      s_FuelGaugeI2cErrorCounter = -1; // indicate that fuel gauge needs initialization after battery insertion
      s_BatteryRsoc = 0;
      s_BatteryVoltage = batVolt;
      s_BatteryTemp = analog_GetTempMCU();
    }
  }
}

static void applyBatProfile(void)
{
  if ( s_BatProfileValid && s_BatProfile.ntcB != 0xFFFF ) {
    if ( s_FuelGaugeI2cErrorCounter < 5 && s_FuelGaugeI2cErrorCounter >= 0 ) {
      // Set NTC B constant
      if (writeWord(0x06, s_BatProfile.ntcB)!= 0) {
        // declare as non initialized
        s_FuelGaugeI2cErrorCounter = -1;
      }
    }
  }

  dvInit();
}

static void applyConfig(uint8_t config)
{
  nv_write_U8(NV_ADDR_FUEL_GAUGE_CONFIG, config);

  uint8_t stored;
  if (nv_read_U8(NV_ADDR_FUEL_GAUGE_CONFIG, &stored) == NV_OK) {
    s_RsocMeasurementConfig = (stored>>4)&0x03;
    s_TempSensorConfig = stored&0x07;
  } else {
    s_TempSensorConfig = BAT_TEMP_SENSE_CONFIG_AUTO_DETECT;
    s_RsocMeasurementConfig = RSOC_MEASUREMENT_AUTO_DETECT;
  }
}

static void applyCmd(const FuelGaugeCmd_T *_pCmd)
{
  switch (_pCmd->type)
  {
    case FUEL_GAUGE_CMD_SET_BAT_PROFILE:
      s_BatProfileValid = _pCmd->data.batProfile.valid;
      if (s_BatProfileValid)
        s_BatProfile = _pCmd->data.batProfile.profile;
      applyBatProfile();
      break;

    case FUEL_GAUGE_CMD_SET_CONFIG:
      applyConfig(_pCmd->data.config);
      s_AppliedConfigSeq = _pCmd->seq;
      break;

    default:
      ASSERT(0);
      break;
  }
}

static void Task(void *parameters)
{
  (void)parameters;

  LOG_INFO("FUEL GAUGE task started");

  HAL_Delay(100); // after power-on, charger and fuel gauge require initialization time
  icPreInit();

  s_BatProfileValid = battery_GetProfile(&s_BatProfile);

  fuelGaugeInit();

  while (1)
  {
    /*
     * A queued command (profile change / host config write) must apply immediately, so it wakes
     * this wait early - but the heavy SOC processing below must not run more often than every
     * ~125 ms, since dt is not just a gate, it is a term in the coulomb-counting integration
     * (see fuelGaugeTick()). So this wait is a heartbeat, not the only pacing: fuelGaugeTick()
     * keeps its own internal MS_TIME_COUNT gate, exactly as the pre-task version did.
     */
    FuelGaugeCmd_T cmd;
    while (xQueueReceive(s_QueHandle, &cmd, pdMS_TO_TICKS(125)) == pdTRUE)
      applyCmd(&cmd);

    fuelGaugeTick();
  }
}

/*============================ COMMAND POSTING ===============================*/

static bool postCmd(const FuelGaugeCmd_T *_pCmd)
{
  if (s_QueHandle == NULL)
  {
    s_ErrMask |= FUEL_GAUGE_ERR_NOT_READY;
    return false;
  }

  BaseType_t sent;

  if (xPortIsInsideInterrupt() != pdFALSE)
  {
    BaseType_t woken = pdFALSE;
    sent = xQueueSendFromISR(s_QueHandle, _pCmd, &woken);
    portYIELD_FROM_ISR(woken);

    if (sent != pdTRUE)
      s_ErrMask |= FUEL_GAUGE_ERR_QUEUE_FULL;
  }
  else
  {
    sent = xQueueSend(s_QueHandle, _pCmd, 0);

    if (sent != pdTRUE)
    {
      LOG_ERROR("[FG] Command queue full, cmd=%u dropped", _pCmd->type);
      taskENTER_CRITICAL();
      s_ErrMask |= FUEL_GAUGE_ERR_QUEUE_FULL;
      taskEXIT_CRITICAL();
    }
  }

  return (sent == pdTRUE);
}

/*============================ PUBLIC API ====================================*/

void fuel_gauge_Init(void)
{
  LOG_INFO("fuel_gauge_Init...");

  s_QueHandle = xQueueCreateStatic(sizeof(s_QueBuf) / sizeof(s_QueBuf[0]),
                            sizeof(s_QueBuf[0]), (uint8_t*)s_QueBuf, &s_Que);
  ASSERT(s_QueHandle != NULL);

  s_TaskHandle = xTaskCreateStatic(Task, "FG", sizeof(s_TaskStack) / sizeof(StackType_t),
                            NULL, 6, s_TaskStack, &s_TaskTCB);
  ASSERT(s_TaskHandle != NULL);
}

uint16_t fuel_gauge_GetVoltage(void)
{
  return s_BatteryVoltage;
}

uint16_t fuel_gauge_GetRsoc(void)
{
  return s_BatteryRsoc;
}

int8_t fuel_gauge_GetTemp(void)
{
  return s_BatteryTemp;
}

int16_t fuel_gauge_GetCurrent(void)
{
  return s_BatteryCurrent;
}

BatteryTempSenseConfig_T fuel_gauge_GetTempSenseConfig(void)
{
  return s_TempSensorConfig;
}

bool fuel_gauge_IsIcFault(void)
{
  return s_RsocMeasurementConfig==RSOC_MEASUREMENT_AUTO_DETECT
      && (s_FuelGaugeI2cErrorCounter <= -5 || s_FuelGaugeI2cErrorCounter >= 5);
}

bool fuel_gauge_IsTempSenseFault(void)
{
  return s_NtcFaultFlag != 0;
}

void fuel_gauge_CmdSetBatProfile(const BatteryProfile_T *batProfile)
{
  FuelGaugeCmd_T cmd = { .type = FUEL_GAUGE_CMD_SET_BAT_PROFILE };
  cmd.data.batProfile.valid = (batProfile != NULL);
  if (batProfile != NULL)
    cmd.data.batProfile.profile = *batProfile;
  postCmd(&cmd);
}

int8_t fuel_gauge_CmdSetConfig(uint8_t *data, uint16_t len)
{
  (void)len;

  if ( (data[0]&0x07) >= BAT_TEMP_SENSE_CONFIG_END || ((data[0]&0x30)>>4) >= RSOC_MEASUREMENT_CONFIG_END )
    return 1;

  s_ReqConfig = data[0];
  FuelGaugeCmd_T cmd = { .type = FUEL_GAUGE_CMD_SET_CONFIG,
                        .seq = (uint8_t)(s_ReqConfigSeq + 1),
                        .data.config = data[0] };
  if (postCmd(&cmd))
    s_ReqConfigSeq = cmd.seq;

  return 0;
}

void fuel_gauge_GetConfig(uint8_t data[], uint16_t *len)
{
  data[0] = (s_ReqConfigSeq != s_AppliedConfigSeq)
      ? s_ReqConfig
      : (uint8_t)((s_TempSensorConfig&0x07) | ((s_RsocMeasurementConfig&0x03) << 4));
  *len = 1;
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
