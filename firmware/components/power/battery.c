/*
 * battery.c
 *
 *  Created on: 12.12.2016.
 *      Author: milan
 */

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "power/battery.h"
#include "power/charger_bq2416x.h"
#include "iosystem/analog.h"
#include <to_refactor/config_switch_resistor.h>
#include "fuel_gauge_lc709203f.h"
#include "nv.h"
#include "led.h"
#include "src/app.h"

// FreeRTOS:
#include "FreeRTOS.h"
#include "task.h"

// LOG:
#include "log/log.h"

#define BATTERY_PROFILES_COUNT()    ((sizeof(s_BatteryProfiles)/sizeof(BatteryProfile_T)))
#define PACK_CAPACITY_U16(c)        ((c==0xFFFFFFFF) ? 0xFFFF : (c >> ((c>=0x8000)*7)) | (c>=0x8000)*0x8000)
#define UNPACK_CAPACITY_U16(v)      ((v==0xFFFF) ? 0xFFFFFFFF : (uint32_t)(v&0x7FFF) << (((v&0x8000) >> 15)*7))

static const BatteryProfile_T s_BatteryProfiles[] = {
	{ 	// PiJuice Zero 1000mAh battery
		BAT_CHEMISTRY_LIPO,
		1000, // 1000mAh
		0x01, // 6250mA
		0x00, // 50mA
		0x22, // 4.18V
		150, // 3V
		3743, 3933, 4057,
		13500, 13300, 13300,
		0,
		2,
		49,
		65,
		3450,
		1000, // 10K
	},
	{ 	// BP7X battery
		BAT_CHEMISTRY_LIPO,
		1820, // 1820mAh
		0x05, // 925mA, ~0.5C
		0x00, // 50mA
		0x22, // 4.18V
		150, // 3V
		3649, 3800, 4077,
		20900, 20500, 20200,
		1,
		10,
		45,
		59,
		0x0D34,
		1000, // 10K
	},
	{ 	// SNN5843 battery
		BAT_CHEMISTRY_LIPO,
		2300, // 2300mAh
		0x08, // 1150mA, ~0.5C
		0x01, // 100mA
		0x22, // 4.18V
		150, // 3V
		3650, 3800, 4079,
		15300, 14900, 14820,
		1,
		10,
		45,
		59,
		0x0D34,
		1000, // 10K
	},
	{ 	// PiJuice 12000mAh battery
		BAT_CHEMISTRY_LIPO,
		12000, // 12000mAh
		0x1A, // 2500mA
		0x06, // 350mA
		0x22, // 4.18V
		150, // 3V
		3488, 3824, 4061,
		11200, 10800, 10800,
		0,
		2,
		49,
		65,
		3450,
		1000, // 10K
	},
	{ 	// PiJuice 5000mAh battery
		BAT_CHEMISTRY_LIPO,
		5000, // 5000mAh
		0x1A, // 2500mA
		0x04, // 250mA
		0x22, // 4.18V
		150, // 3V
		3506, 3870, 4056,
		11100, 10500, 10700,
		0,
		2,
		49,
		65,
		3450,
		1000, // 10K
	},
	{ 	// PiJuice BP7X 1600mAh battery
		BAT_CHEMISTRY_LIPO,
		1600, // 1600mAh
		0x05, // 925mA
		0x00, // 50mA
		0x22, // 4.18V
		150, // 3V
		3672, 3811, 4094,
		22200, 20800, 20500,
		0,
		2,
		50,
		70,
		0x0D34,
		1000, // 10K
	},
	{ 	// PiJuice SNN5843 1300mAh battery
		BAT_CHEMISTRY_LIPO,
		1300, // 1300mAh
		0x03, // 775mA
		0x00, // 50mA
		0x22, // 4.18V
		150, // 3V
		3675, 3818, 4105,
		15600, 15100, 15100,
		0,
		2,
		50,
		70,
		0x0D34,
		1000, // 10K
	},
	{ 	// PiJuice 1200mAh battery
		BAT_CHEMISTRY_LIPO,
		1200, // 1200mAh
		0x02, // 700mA
		0x00, // 50mA
		0x22, // 4.18V
		150, // 3V
		3514, 3859, 4045,
		19400, 17300, 16900,
		0,
		2,
		49,
		65,
		3450,
		1000, // 10K
	},
	{ 	// BP6X battery
		BAT_CHEMISTRY_LIPO,
		1400, // 1400mAh
		0x04, // 850mA, ~0.6C
		0x00, // 50mA
		0x22, // 4.18V
		150, // 3V
		3649, 3800, 4077,
		20670, 20319, 20215,
		1,
		10,
		45,
		59,
		0x0D34,
		1000, // 10K
	},
	{ 	// PiJuice 600mAh battery
		BAT_CHEMISTRY_LIPO,
		600, // 600mAh
		0x00, // 550mA
		0x00, // 50mA
		0x22, // 4.18V
		150, // 3V
		3659, 3816, 4087,
		37200, 22100, 22300,
		0,
		2,
		49,
		65,
		3450,
		1000, // 10K
	},
	{ 	// PiJuice 500mAh battery
		BAT_CHEMISTRY_LIPO,
		500, // 500mAh
		0x00, // 550mA
		0x00, // 50mA
		0x22, // 4.18V
		150, // 3V
		3659, 3914, 4060,
		16600, 15600, 15600,
		0,
		2,
		49,
		65,
		3450,
		1000, // 10K
	},
	{ 	// PiJuice 2500mAh battery
		BAT_CHEMISTRY_LIPO,
		2500, // 2500mAh
		0x0a, // 1300mA, ~0.5C
		0x01, // 100mA
		0x22, // 4.18V
		150, // 3V
		3650, 3800, 4079,
		15300, 14900, 14820,
		1,
		10,
		45,
		59,
		3450,
		1000, // 10K
	}
};

/*
 * The canonical custom profile: only ever populated by readEEprofileData()/readExtendedEEprofileData()
 * reading back from NV - never a direct copy of s_CustomProfileReq. This is a verify-after-write
 * pattern, same reasoning as led.c's applyConfig().
 */
static BatteryProfile_T s_CustomProfile = {
		// default battery
		BAT_CHEMISTRY_LIPO,
		1400, // 1400mAh
		0x04, // 850mA
		0x01, // 100mA
		0x22, // 4.18V
		150, // 3V
		3649, 3800, 4077,
		15900, 15630, 15550,
		1,
		10,
		45,
		60,
		0xFFFF,
		0xFFFF,
};

static bool s_Present;   // aggregated, see battery_UpdatePresence()
static BatteryThermalState_T s_ThermalState = BAT_TEMP_UNKNOWN;
static uint8_t s_BatProfileStatus = BATTERY_INVALID_PROFILE_ID;

// Currently selected profile, published as a value (never a pointer into another task's memory).
static BatteryProfile_T s_CurrentProfile;
static bool s_CurrentProfileValid;

/* "Write pending" mirror for host register 0x82/0x86 read-back, same idiom as led.c: a write is
 * applied asynchronously by the task, so a read arriving before it has must answer "busy". */
static volatile uint8_t s_ProfileReqSeq;
static volatile uint8_t s_ProfileAppliedSeq;

static uint32_t s_ErrMask;  // BATTERY_ERR_* bits

/*============================ PRIVATE ========================================*/

static void setCurrentProfile(const BatteryProfile_T *profile)
{
  taskENTER_CRITICAL();
  s_CurrentProfileValid = (profile != NULL);
  if (profile != NULL)
    s_CurrentProfile = *profile;
  taskEXIT_CRITICAL();
}

static int8_t readExtendedEEprofileData(void)
{
  uint8_t dataValid = 1;

  s_CustomProfile.chemistry = 0xFF;
  if (nv_read_U8(NV_ADDR_BAT_CHEMISTRY, (uint8_t*)&(s_CustomProfile.chemistry)) != NV_OK) dataValid = 0;

  s_CustomProfile.ocv10 = 0xFFFF;
  if (nv_read_U8(NV_ADDR_BAT_OCV10L, (uint8_t*)&(s_CustomProfile.ocv10)) != NV_OK) dataValid = 0;
  if (nv_read_U8(NV_ADDR_BAT_OCV10H, (uint8_t*)&(s_CustomProfile.ocv10)+1) != NV_OK) dataValid = 0;

  s_CustomProfile.ocv50 = 0xFFFF;
  if (nv_read_U8(NV_ADDR_BAT_OCV50L, (uint8_t*)&(s_CustomProfile.ocv50)) != NV_OK) dataValid = 0;
  if (nv_read_U8(NV_ADDR_BAT_OCV50H, (uint8_t*)&(s_CustomProfile.ocv50)+1) != NV_OK) dataValid = 0;

  s_CustomProfile.ocv90 = 0xFFFF;
  if (nv_read_U8(NV_ADDR_BAT_OCV90L, (uint8_t*)&(s_CustomProfile.ocv90)) != NV_OK) dataValid = 0;
  if (nv_read_U8(NV_ADDR_BAT_OCV90H, (uint8_t*)&(s_CustomProfile.ocv90)+1) != NV_OK) dataValid = 0;

  s_CustomProfile.r10 = 0xFFFF;
  if (nv_read_U8(NV_ADDR_BAT_R10L, (uint8_t*)&(s_CustomProfile.r10)) != NV_OK) dataValid = 0;
  if (nv_read_U8(NV_ADDR_BAT_R10H, (uint8_t*)&(s_CustomProfile.r10)+1) != NV_OK) dataValid = 0;

  s_CustomProfile.r50 = 0xFFFF;
  if (nv_read_U8(NV_ADDR_BAT_R50L, (uint8_t*)&(s_CustomProfile.r50)) != NV_OK) dataValid = 0;
  if (nv_read_U8(NV_ADDR_BAT_R50H, (uint8_t*)&(s_CustomProfile.r50)+1) != NV_OK) dataValid = 0;

  s_CustomProfile.r90 = 0xFFFF;
  if (nv_read_U8(NV_ADDR_BAT_R90L, (uint8_t*)&(s_CustomProfile.r90)) != NV_OK) dataValid = 0;
  if (nv_read_U8(NV_ADDR_BAT_R90H, (uint8_t*)&(s_CustomProfile.r90)+1) != NV_OK) dataValid = 0;

  return !dataValid; // return 0 if valid
}

static int8_t readEEprofileData(void)
{
  uint8_t dataValid = 1;
  uint8_t var8;
  uint16_t var16 = 0;

  /* Capacity is the one profile field that needs the full 16 bit cell, so it carries no
   * complement byte - all it can be checked for is presence. */
  if (nv_read_U16(NV_ADDR_BATT_CAPACITY, &var16) != NV_OK) dataValid = 0;
  s_CustomProfile.capacity = UNPACK_CAPACITY_U16(var16); // correction for large capacities over 32767

  if (nv_read_U8(NV_ADDR_CHARGE_CURRENT, &s_CustomProfile.chargeCurrent) != NV_OK) dataValid = 0;
  if (nv_read_U8(NV_ADDR_CHARGE_TERM_CURRENT, &s_CustomProfile.terminationCurr) != NV_OK) dataValid = 0;
  if (nv_read_U8(NV_ADDR_BATT_REG_VOLTAGE, &s_CustomProfile.regulationVoltage) != NV_OK) dataValid = 0;
  if (nv_read_U8(NV_ADDR_BATT_CUTOFF_VOLTAGE, &s_CustomProfile.cutoffVoltage) != NV_OK) dataValid = 0;

  /* The temperature points are int8_t, the NV layer deals in unsigned bytes. */
  if (nv_read_U8(NV_ADDR_BATT_TEMP_COLD, &var8) != NV_OK) dataValid = 0; else s_CustomProfile.tCold = (int8_t)var8;
  if (nv_read_U8(NV_ADDR_BATT_TEMP_COOL, &var8) != NV_OK) dataValid = 0; else s_CustomProfile.tCool = (int8_t)var8;
  if (nv_read_U8(NV_ADDR_BATT_TEMP_WARM, &var8) != NV_OK) dataValid = 0; else s_CustomProfile.tWarm = (int8_t)var8;
  if (nv_read_U8(NV_ADDR_BATT_TEMP_HOT, &var8) != NV_OK) dataValid = 0; else s_CustomProfile.tHot = (int8_t)var8;

  /* NTC constants are 16 bit as well, guarded by their own xor checksum variable. */
  if (nv_read_U16(NV_ADDR_BATT_NTC_B, &s_CustomProfile.ntcB) != NV_OK) dataValid = 0;
  if (nv_read_U16(NV_ADDR_BATT_NTC_RESISTANCE, &s_CustomProfile.ntcResistance) != NV_OK) dataValid = 0;
  if (nv_read_U16(NV_ADDR_BATT_NTC_CRC, &var16) != NV_OK) dataValid = 0;
  if (var16 != (s_CustomProfile.ntcB ^ s_CustomProfile.ntcResistance)) dataValid = 0;

  return !dataValid; // return 0 if valid
}

static void writeEEprofileData(const BatteryProfile_T *batProfile)
{
  uint16_t var = PACK_CAPACITY_U16(batProfile->capacity); // correction for large capacities over 32767
  nv_write_U16(NV_ADDR_BATT_CAPACITY, var);
  nv_write_U8(NV_ADDR_CHARGE_CURRENT, batProfile->chargeCurrent);
  nv_write_U8(NV_ADDR_CHARGE_TERM_CURRENT, batProfile->terminationCurr);
  nv_write_U8(NV_ADDR_BATT_REG_VOLTAGE, batProfile->regulationVoltage);
  nv_write_U8(NV_ADDR_BATT_CUTOFF_VOLTAGE, batProfile->cutoffVoltage);
  nv_write_U8(NV_ADDR_BATT_TEMP_COLD, batProfile->tCold);
  nv_write_U8(NV_ADDR_BATT_TEMP_COOL, batProfile->tCool);
  nv_write_U8(NV_ADDR_BATT_TEMP_WARM, batProfile->tWarm);
  nv_write_U8(NV_ADDR_BATT_TEMP_HOT, batProfile->tHot);
  nv_write_U16(NV_ADDR_BATT_NTC_B, batProfile->ntcB);
  nv_write_U16(NV_ADDR_BATT_NTC_RESISTANCE, batProfile->ntcResistance);
  nv_write_U16(NV_ADDR_BATT_NTC_CRC, batProfile->ntcB ^ batProfile->ntcResistance);
}

static void writeExtendedEEprofileData(const BatteryProfile_T *batProfile)
{
  nv_write_U8(NV_ADDR_BAT_CHEMISTRY, (uint8_t)(batProfile->chemistry));
  nv_write_U8(NV_ADDR_BAT_OCV10L, batProfile->ocv10);
  nv_write_U8(NV_ADDR_BAT_OCV10H, (batProfile->ocv10)>>8);
  nv_write_U8(NV_ADDR_BAT_OCV50L, batProfile->ocv50);
  nv_write_U8(NV_ADDR_BAT_OCV50H, (batProfile->ocv50)>>8);
  nv_write_U8(NV_ADDR_BAT_OCV90L, batProfile->ocv90);
  nv_write_U8(NV_ADDR_BAT_OCV90H, (batProfile->ocv90)>>8);
  nv_write_U8(NV_ADDR_BAT_R10L, batProfile->r10);
  nv_write_U8(NV_ADDR_BAT_R10H, (batProfile->r10)>>8);
  nv_write_U8(NV_ADDR_BAT_R50L, batProfile->r50);
  nv_write_U8(NV_ADDR_BAT_R50H, (batProfile->r50)>>8);
  nv_write_U8(NV_ADDR_BAT_R90L, batProfile->r90);
  nv_write_U8(NV_ADDR_BAT_R90H, (batProfile->r90)>>8);
}

/*
 * Selects a profile by id/status code - resistor/switch autodetect, one of the built-in
 * profiles, the stored custom profile, or an invalid/nonexistent marker. Behaviour preserving
 * translation of the pre-task BatInitProfile(): every `currentBatProfile = X` becomes
 * setCurrentProfile(X), so the publish is atomic w.r.t. battery_GetProfile() readers.
 */
static void initProfile(uint8_t initProfileId)
{
  if ( initProfileId == BATTERY_CUSTOM_PROFILE_ID ) {
    if (readEEprofileData() == 0) {
      readExtendedEEprofileData();
      setCurrentProfile(&s_CustomProfile);
      s_BatProfileStatus = BATTERY_CUSTOM_PROFILE_ID;
    } else {
      setCurrentProfile(NULL);
      s_BatProfileStatus = BATTERY_INVALID_CUSTOM_PROFILE_STATUS;
    }
  } else if (initProfileId == BATTERY_DEFAULT_PROFILE_ID) {
    // Make profile data based on dip switch or resistor configuration
    if ( switchConfigCode >= 0) {
      if ( switchConfigCode < BATTERY_PROFILES_COUNT() && resistorConfig1Code7 == -1 && resistorConfig2Code4 == -1 ){
        // Use switch coded profile id
        setCurrentProfile(&s_BatteryProfiles[switchConfigCode]);
        s_BatProfileStatus = BATTERY_CONFIG_SW_PROFILE_ID | switchConfigCode;
      } else if ( switchConfigCode == 1 && resistorConfig2Code4 >= 0 && resistorConfig2Code4 < BATTERY_PROFILES_COUNT() ){
        setCurrentProfile(&s_BatteryProfiles[resistorConfig2Code4]);
        s_BatProfileStatus = BATTERY_CONFIG_RES_PROFILE_ID | resistorConfig2Code4;
      } else if ( switchConfigCode == 1 && resistorConfig1Code7 >= 0 ){
        s_CustomProfile.chargeCurrent = ((resistorConfig1Code7&0x07) << 2); // offset 550mA
        s_CustomProfile.capacity = ((int16_t)s_CustomProfile.chargeCurrent * 75 + 550) * 2; // suppose charge current is 0.5 capacity
        s_CustomProfile.terminationCurr = (resistorConfig1Code7&0x04) | ((resistorConfig1Code7&0x08)>>2);
        s_CustomProfile.regulationVoltage = (resistorConfig1Code7>>4) * 5 + 5;
        s_CustomProfile.capacity = 0xFFFFFFFF; // undefined
        s_CustomProfile.cutoffVoltage = 150; // 3v
        s_CustomProfile.ntcB = 0x0D34;
        s_CustomProfile.ntcResistance = 1000;
        s_CustomProfile.tCold = 1;
        s_CustomProfile.tCool = 10;
        s_CustomProfile.tWarm = 45;
        s_CustomProfile.tHot = 60;
        s_CustomProfile.chemistry = 0xFF;
        s_CustomProfile.ocv10 = 0xFFFF;
        s_CustomProfile.ocv50 = 0xFFFF;
        s_CustomProfile.ocv90 = 0xFFFF;
        s_CustomProfile.r10 = 0xFFFF;
        s_CustomProfile.r50 = 0xFFFF;
        s_CustomProfile.r90 = 0xFFFF;

        s_BatProfileStatus = BATTERY_CONFIG_PROFILE_STATUS;
        setCurrentProfile(&s_CustomProfile);
      } else {
        setCurrentProfile(NULL);
        s_BatProfileStatus = BATTERY_CONFIG_INVALID_PROFILE_STATUS;
      }
    } else {
      setCurrentProfile(NULL);
      s_BatProfileStatus = BATTERY_CONFIG_INVALID_PROFILE_STATUS;
    }
  } else if (initProfileId < BATTERY_PROFILES_COUNT()) {
    s_BatProfileStatus = initProfileId;
    setCurrentProfile(&s_BatteryProfiles[s_BatProfileStatus]);
  } else if (initProfileId >= BATTERY_PROFILES_COUNT() && initProfileId < 15) {//32) {
    setCurrentProfile(NULL);
    s_BatProfileStatus = BATTERY_NONEXIST_PROFILE_ID | initProfileId; // non defined  profile
  } else {
    setCurrentProfile(NULL);
    s_BatProfileStatus = BATTERY_INVALID_PROFILE_ID;
  }
}

static void applySetProfile(uint8_t id)
{
  nv_write_U8(NV_ADDR_BATT_PROFILE, id);
  uint8_t storedId;
  if (nv_read_U8(NV_ADDR_BATT_PROFILE, &storedId) == NV_OK) {
    initProfile(storedId);
  } else {
    setCurrentProfile(NULL);
    s_BatProfileStatus = BATTERY_INVALID_PROFILE_ID;
  }
}

static void applyWriteCustomProfile(const BatteryProfile_T *req)
{
  writeEEprofileData(req);
  if (readEEprofileData() == 0) {
    s_BatProfileStatus = BATTERY_CUSTOM_PROFILE_ID;
    setCurrentProfile(&s_CustomProfile);
  } else {
    s_BatProfileStatus = BATTERY_INVALID_CUSTOM_PROFILE_STATUS;
    setCurrentProfile(NULL);
  }

  uint8_t storedId;
  if (nv_read_U8(NV_ADDR_BATT_PROFILE, &storedId) != NV_OK || storedId != BATTERY_CUSTOM_PROFILE_ID) {
    readExtendedEEprofileData();
    nv_write_U8(NV_ADDR_BATT_PROFILE, BATTERY_CUSTOM_PROFILE_ID);
    if (nv_read_U8(NV_ADDR_BATT_PROFILE, &storedId) == NV_OK
        && storedId == BATTERY_CUSTOM_PROFILE_ID) {
      if (s_CurrentProfileValid)
        s_BatProfileStatus = BATTERY_CUSTOM_PROFILE_ID;
      else
        s_BatProfileStatus = BATTERY_INVALID_CUSTOM_PROFILE_STATUS;
    } else {
      setCurrentProfile(NULL);
      s_BatProfileStatus = BATTERY_INVALID_PROFILE_ID;
    }
  }
}

static void applyWriteCustomExtendedProfile(const BatteryProfile_T *req)
{
  writeExtendedEEprofileData(req);
  readExtendedEEprofileData();
  if (s_BatProfileStatus == BATTERY_CUSTOM_PROFILE_ID) {
    setCurrentProfile(&s_CustomProfile);
  }
}

void battery_UpdateChargeLed(void)
{
  static uint8_t b = 0;

  uint16_t rsoc = fuel_gauge_GetRsoc();   // 0.1% units
  uint8_t r, g;
  if (rsoc == FUEL_GAUGE_RSOC_UNKNOWN) {
    /* No charge reading at all. Red alone is the "fault" colour here - showing green would
     * claim a full pack we have no evidence for. */
    r = led_GetParamR(LED_CHARGE_STATUS);
    g = 0;
  } else if (rsoc > 500) {
    r = 0;
    g = led_GetParamG(LED_CHARGE_STATUS);
  } else if (rsoc > 150) {
    r = led_GetParamR(LED_CHARGE_STATUS);
    g = led_GetParamG(LED_CHARGE_STATUS);
  } else {
    r = led_GetParamR(LED_CHARGE_STATUS);
    g = 0;
  }
  uint8_t paramB = led_GetParamB(LED_CHARGE_STATUS);

  BatteryStatus_T status = battery_GetStatus();
  if (status == BAT_STATUS_CHARGING_FROM_IN || status == BAT_STATUS_CHARGING_FROM_5V_IO) {
    b = b ? 0 : paramB;
  } else if (charger_GetStatus() == CHG_CHARGE_DONE) {
    b = paramB;
  } else {
    b = 0;
  }

  led_SetFuncRGB(LED_CHARGE_STATUS, r, g, b);
}

/*============================ PUBLIC API ====================================*/

void battery_Init(void)
{
  LOG_INFO("battery_Init...");

  uint8_t profileId;
  if (nv_read_U8(NV_ADDR_BATT_PROFILE, &profileId) != NV_OK) {
    LOG_WARNING("NV read battery profile: FAILED. Set default profile.");
    nv_write_U8(NV_ADDR_BATT_PROFILE, BATTERY_DEFAULT_PROFILE_ID);
  }

  if (nv_read_U8(NV_ADDR_BATT_PROFILE, &profileId) == NV_OK) {
    initProfile(profileId);
  }
}

void battery_ApplySetProfile(uint8_t id, uint8_t seq)
{
  applySetProfile(id);
  s_ProfileAppliedSeq = seq;
}

void battery_ApplyWriteCustomProfile(const BatteryProfile_T *req, uint8_t seq)
{
  applyWriteCustomProfile(req);
  s_ProfileAppliedSeq = seq;
}

void battery_ApplyWriteCustomExtendedProfile(const BatteryProfile_T *req)
{
  applyWriteCustomExtendedProfile(req);
}

bool battery_GetProfile(BatteryProfile_T *out)
{
  taskENTER_CRITICAL();
  bool valid = s_CurrentProfileValid;
  if (valid && out != NULL)
    *out = s_CurrentProfile;
  taskEXIT_CRITICAL();
  return valid;
}

bool battery_UpdatePresence(void)
{
  bool present = charger_IsBatteryPresent() && (analog_GetVBattAvg() > BATTERY_PRESENT_MV);

  if (present == s_Present)
    return false;

  s_Present = present;
  LOG_INFO("[BAT] battery %s", present ? "present" : "absent");
  return true;
}

bool battery_IsPresent(void)
{
  return s_Present;
}

/*
 * The thresholds nest as tCold < tCool < tWarm < tHot, so the tests run coldest-first and the
 * first match wins. That reproduces the charger's old overlapping predicates exactly: a reading
 * at or above tHot used to satisfy both "disable charging" and "above tWarm", which the ordinal
 * BAT_TEMP_HOT now carries on its own.
 */
static BatteryThermalState_T evaluateThermalState(void)
{
  if (!s_CurrentProfileValid)
    return BAT_TEMP_UNKNOWN;

  int8_t temp = fuel_gauge_GetTemp();

  /* "No reading", not a very cold pack - it is what the fuel gauge answers while the IC is not the
   * source. Without this it would satisfy the tCold test below and a lost IC would look like a
   * frozen battery, stopping the charge. */
  if (temp == FUEL_GAUGE_TEMP_UNKNOWN)
    return BAT_TEMP_UNKNOWN;

  if (temp <= s_CurrentProfile.tCold) return BAT_TEMP_COLD;
  if (temp <  s_CurrentProfile.tCool) return BAT_TEMP_COOL;
  if (temp >= s_CurrentProfile.tHot)  return BAT_TEMP_HOT;
  if (temp >  s_CurrentProfile.tWarm) return BAT_TEMP_WARM;

  return BAT_TEMP_NORMAL;
}

bool battery_UpdateThermalState(void)
{
  BatteryThermalState_T state = evaluateThermalState();

  if (state == s_ThermalState)
    return false;

  s_ThermalState = state;
  LOG_INFO("[BAT] thermal state %u", (unsigned)state);
  return true;
}

BatteryThermalState_T battery_GetThermalState(void)
{
  return s_ThermalState;
}

BatteryStatus_T battery_GetStatus(void)
{
  if (!s_Present)
    return BAT_STATUS_NOT_PRESENT;
  if (charger_GetStatus() == CHG_CHARGING_FROM_IN)
    return BAT_STATUS_CHARGING_FROM_IN;
  if (charger_GetStatus() == CHG_CHARGING_FROM_USB)
    return BAT_STATUS_CHARGING_FROM_5V_IO;
  return BAT_STATUS_NORMAL;
}

void battery_CmdSetProfile(uint8_t id)
{
  if (s_BatProfileStatus == id)
    return;

  uint8_t seq = (uint8_t)(s_ProfileReqSeq + 1);
  AppEvent_t evt = { .type = APP_EVT_BATTERY_SET_PROFILE };
  evt.data.batterySetProfile.id = id;
  evt.data.batterySetProfile.seq = seq;

  app_PostEvent(&evt);
  // TODO - check
  {
    s_BatProfileStatus = BATTERY_PROFILE_WRITE_BUSY_STATUS;
    s_ProfileReqSeq = seq;
  }
}

void battery_CmdWriteCustomProfile(uint8_t *data, uint16_t len)
{
  (void)len;

  uint8_t seq = (uint8_t)(s_ProfileReqSeq + 1);
  AppEvent_t evt = { .type = APP_EVT_BATTERY_WRITE_CUSTOM_PROFILE };
  evt.data.batteryCustomProfile.seq = seq;

  uint16_t var = (((uint16_t)data[1])<<8) | data[0];
  evt.data.batteryCustomProfile.profile.capacity = UNPACK_CAPACITY_U16(var); // correction for large capacities over 32767
  evt.data.batteryCustomProfile.profile.chargeCurrent = data[2];
  evt.data.batteryCustomProfile.profile.terminationCurr = data[3];
  evt.data.batteryCustomProfile.profile.regulationVoltage = data[4];
  evt.data.batteryCustomProfile.profile.cutoffVoltage = data[5];
  evt.data.batteryCustomProfile.profile.tCold = data[6];
  evt.data.batteryCustomProfile.profile.tCool = data[7];
  evt.data.batteryCustomProfile.profile.tWarm = data[8];
  evt.data.batteryCustomProfile.profile.tHot = data[9];
  evt.data.batteryCustomProfile.profile.ntcB = (((uint16_t)data[11])<<8) | data[10];
  evt.data.batteryCustomProfile.profile.ntcResistance = (((uint16_t)data[13])<<8) | data[12];

  app_PostEvent(&evt);
  // TODO - check
  {
    s_BatProfileStatus = BATTERY_PROFILE_WRITE_BUSY_STATUS;
    s_ProfileReqSeq = seq;
  }
}

void battery_CmdWriteCustomExtendedProfile(uint8_t *data, uint16_t len)
{
  (void)len;

  AppEvent_t evt = { .type = APP_EVT_BATTERY_WRITE_CUSTOM_EXTENDED_PROFILE };
  evt.data.batteryCustomExtProfile.profile.chemistry = data[0];
  evt.data.batteryCustomExtProfile.profile.ocv10 = *(uint16_t*)&data[1];
  evt.data.batteryCustomExtProfile.profile.ocv50 = *(uint16_t*)&data[3];
  evt.data.batteryCustomExtProfile.profile.ocv90 = *(uint16_t*)&data[5];
  evt.data.batteryCustomExtProfile.profile.r10 = *(uint16_t*)&data[7];
  evt.data.batteryCustomExtProfile.profile.r50 = *(uint16_t*)&data[9];
  evt.data.batteryCustomExtProfile.profile.r90 = *(uint16_t*)&data[11];

  app_PostEvent(&evt);
}

void battery_ReadCurrentProfile(uint8_t *data, uint16_t *len)
{
  BatteryProfile_T profile;
  bool valid = (s_ProfileReqSeq == s_ProfileAppliedSeq) && battery_GetProfile(&profile);

  if (!valid) {
    for (int i = 0; i < 14; i++) data[i] = 0;
  } else {
    uint16_t var = PACK_CAPACITY_U16(profile.capacity); // correction for large capacities over 32767
    data[0] = var;
    data[1] = var>>8;
    data[2] = profile.chargeCurrent;
    data[3] = profile.terminationCurr;
    data[4] = profile.regulationVoltage;
    data[5] = profile.cutoffVoltage;
    data[6] = profile.tCold;
    data[7] = profile.tCool;
    data[8] = profile.tWarm;
    data[9] = profile.tHot;
    data[10] = profile.ntcB;
    data[11] = profile.ntcB>>8;
    data[12] = profile.ntcResistance;
    data[13] = profile.ntcResistance>>8;
  }
  *len = 14;
}

void battery_ReadCurrentExtendedProfile(uint8_t *data, uint16_t *len)
{
  BatteryProfile_T profile;

  /*
   * NOTE (behaviour change from the pre-task version, on purpose): the old
   * BatteryReadCurrentExtendedProfile() dereferenced currentBatProfile with no NULL check, so a
   * host read while no profile was selected would crash. Zero-filling here instead matches what
   * battery_ReadCurrentProfile() already does for the same "no profile" case.
   */
  if (!battery_GetProfile(&profile)) {
    for (int i = 0; i < 17; i++) data[i] = 0;
    *len = 17;
    return;
  }

  data[0] = profile.chemistry;
  data[1] = profile.ocv10;
  data[2] = profile.ocv10>>8;
  data[3] = profile.ocv50;
  data[4] = profile.ocv50>>8;
  data[5] = profile.ocv90;
  data[6] = profile.ocv90>>8;
  data[7] = profile.r10;
  data[8] = profile.r10>>8;
  data[9] = profile.r50;
  data[10] = profile.r50>>8;
  data[11] = profile.r90;
  data[12] = profile.r90>>8;
  data[13] = 0xFF; // reserved for future use
  data[14] = 0xFF; // reserved for future use
  data[15] = 0xFF; // reserved for future use
  data[16] = 0xFF; // reserved for future use

  *len = 17;
}

void battery_ReadProfileStatus(uint8_t *data, uint16_t *len)
{
  data[0] = (s_ProfileReqSeq != s_ProfileAppliedSeq) ? BATTERY_PROFILE_WRITE_BUSY_STATUS : s_BatProfileStatus;
  *len = 1;
}

uint32_t battery_GetErrMask(bool _clear)
{
  taskENTER_CRITICAL();
  uint32_t mask = s_ErrMask;
  if (_clear)
    s_ErrMask &= ~mask;
  taskEXIT_CRITICAL();
  return mask;
}
