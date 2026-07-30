/*
 * battery.c
 *
 *  Created on: 12.12.2016.
 *      Author: milan
 */


#include <to_refactor/battery.h>
#include <to_refactor/config_switch_resistor.h>
#include <to_refactor/fuel_gauge_lc709203f.h>
#include <to_refactor/power_source.h>
#include <to_refactor/time_count.h>
#include "nv.h"
#include "charger_bq2416x.h"
#include "led.h"

#define BATTERY_PROFILES_COUNT() ((sizeof(batteryProfiles)/sizeof(BatteryProfile_T)))
#define PACK_CAPACITY_U16(c) 	((c==0xFFFFFFFF) ? 0xFFFF : (c >> ((c>=0x8000)*7)) | (c>=0x8000)*0x8000)
#define UNPACK_CAPACITY_U16(v) 	((v==0xFFFF) ? 0xFFFFFFFF : (uint32_t)(v&0x7FFF) << (((v&0x8000) >> 15)*7))

static int16_t setProfileReq = -1;

static int8_t writeCustomProfileReq = 0;
BatteryProfile_T customBatProfileReq;

BatteryStatus_T batteryStatus = BAT_STATUS_NOT_PRESENT;

const BatteryProfile_T batteryProfiles[] = {
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

BatteryProfile_T customBatProfile = {
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

// Resistor charge parameters config code
// code: v2 v1 v0 t1 c2 c1 c0
// v value: (code>>4 * 5 + 5) * 20mV + 3.5V, vcode = (code>>4) * 5 + 5
// c value: code&0x07 * 300mA + 550mA, ccode = code&0x07
// t value: c2t0 * 100mA + 50mA, tcode = (code&0x04>>1) | (code&0x08>>3)


//static BatteryProfile_T nvBatProfileParamAddr;

uint8_t batProfileStatus = BATTERY_INVALID_PROFILE_ID;

BatteryProfile_T const *currentBatProfile = NULL;

int8_t BatReadEEprofileData(void);
int8_t BatReadExtendedEEprofileData(void);
void BatWriteEEprofileData(BatteryProfile_T *batProfile);

void BatInitProfile(uint8_t initPofileId) {
	//uint16_t var;

	if ( initPofileId == BATTERY_CUSTOM_PROFILE_ID ) {
		if (BatReadEEprofileData() == 0) {
			BatReadExtendedEEprofileData();
			currentBatProfile = &customBatProfile;
			batProfileStatus = BATTERY_CUSTOM_PROFILE_ID;
		} else {
			currentBatProfile = NULL;
			batProfileStatus = BATTERY_INVALID_CUSTOM_PROFILE_STATUS;
		}
	} else if (initPofileId == BATTERY_DEFAULT_PROFILE_ID) {
		// Make profile data based on dip switch or resistor configuration
		if ( switchConfigCode >= 0) {
			if ( switchConfigCode < BATTERY_PROFILES_COUNT() && resistorConfig1Code7 == -1 && resistorConfig2Code4 == -1 ){
				// Use switch coded profile id
				currentBatProfile = &batteryProfiles[switchConfigCode];
				batProfileStatus = BATTERY_CONFIG_SW_PROFILE_ID | switchConfigCode;
			} else if ( switchConfigCode == 1 && resistorConfig2Code4 >= 0 && resistorConfig2Code4 < BATTERY_PROFILES_COUNT() ){
				currentBatProfile = &batteryProfiles[resistorConfig2Code4];
				batProfileStatus = BATTERY_CONFIG_RES_PROFILE_ID | resistorConfig2Code4;
			} else if ( switchConfigCode == 1 && resistorConfig1Code7 >= 0 ){
				customBatProfile.chargeCurrent = ((resistorConfig1Code7&0x07) << 2); // offset 550mA
				customBatProfile.capacity = ((int16_t)customBatProfile.chargeCurrent * 75 + 550) * 2; // suppose charge current is 0.5 capacity
				customBatProfile.terminationCurr = (resistorConfig1Code7&0x04) | ((resistorConfig1Code7&0x08)>>2);
				customBatProfile.regulationVoltage = (resistorConfig1Code7>>4) * 5 + 5;
				customBatProfile.capacity = 0xFFFFFFFF; // undefined
				customBatProfile.cutoffVoltage = 150; // 3v
				customBatProfile.ntcB = 0x0D34;
				customBatProfile.ntcResistance = 1000;
				customBatProfile.tCold = 1;
				customBatProfile.tCool = 10;
				customBatProfile.tWarm = 45;
				customBatProfile.tHot = 60;
				customBatProfile.chemistry=0xFF;
				customBatProfile.ocv10 = 0xFFFF;
				customBatProfile.ocv50 = 0xFFFF;
				customBatProfile.ocv90 = 0xFFFF;
				customBatProfile.r10 = 0xFFFF;
				customBatProfile.r50 = 0xFFFF;
				customBatProfile.r90 = 0xFFFF;

				batProfileStatus = BATTERY_CONFIG_PROFILE_STATUS;
				currentBatProfile = &customBatProfile;
			} else {
				currentBatProfile = NULL;
				batProfileStatus = BATTERY_CONFIG_INVALID_PROFILE_STATUS;
			}
		} else {
			currentBatProfile = NULL;
			batProfileStatus = BATTERY_CONFIG_INVALID_PROFILE_STATUS;
		}
	} else if (initPofileId < BATTERY_PROFILES_COUNT()) {
		batProfileStatus = initPofileId;
		currentBatProfile = &batteryProfiles[batProfileStatus];
	} else if (initPofileId >= BATTERY_PROFILES_COUNT() && initPofileId < 15) {//32) {
		currentBatProfile = NULL;
		batProfileStatus = BATTERY_NONEXIST_PROFILE_ID | initPofileId; // non defined  profile
	} else {
		currentBatProfile = NULL;
		batProfileStatus = BATTERY_INVALID_PROFILE_ID;
	}
}

void BatteryInit(void) {
	uint8_t profileId;

	if (nv_read_U8(NV_ADDR_BAT_PROFILE, &profileId) != NV_OK) {
		// Nothing usable stored yet - seed the "no profile selected" marker.
		nv_write_U8(NV_ADDR_BAT_PROFILE, BATTERY_DEFAULT_PROFILE_ID);
	}

	if (nv_read_U8(NV_ADDR_BAT_PROFILE, &profileId) == NV_OK) {
		BatInitProfile(profileId);
	}
}

int8_t BatReadExtendedEEprofileData(void) {
	uint8_t dataValid = 1;

	customBatProfile.chemistry=0xFF;
	if (nv_read_U8(NV_ADDR_BAT_CHEMISTRY, (uint8_t*)&(customBatProfile.chemistry)) != NV_OK) dataValid = 0;

	customBatProfile.ocv10=0xFFFF;
	if (nv_read_U8(NV_ADDR_BAT_OCV10L, (uint8_t*)&(customBatProfile.ocv10)) != NV_OK) dataValid = 0;
	if (nv_read_U8(NV_ADDR_BAT_OCV10H, (uint8_t*)&(customBatProfile.ocv10)+1) != NV_OK) dataValid = 0;

	customBatProfile.ocv50=0xFFFF;
	if (nv_read_U8(NV_ADDR_BAT_OCV50L, (uint8_t*)&(customBatProfile.ocv50)) != NV_OK) dataValid = 0;
	if (nv_read_U8(NV_ADDR_BAT_OCV50H, (uint8_t*)&(customBatProfile.ocv50)+1) != NV_OK) dataValid = 0;

	customBatProfile.ocv90=0xFFFF;
	if (nv_read_U8(NV_ADDR_BAT_OCV90L, (uint8_t*)&(customBatProfile.ocv90)) != NV_OK) dataValid = 0;
	if (nv_read_U8(NV_ADDR_BAT_OCV90H, (uint8_t*)&(customBatProfile.ocv90)+1) != NV_OK) dataValid = 0;

	customBatProfile.r10=0xFFFF;
	if (nv_read_U8(NV_ADDR_BAT_R10L, (uint8_t*)&(customBatProfile.r10)) != NV_OK) dataValid = 0;
	if (nv_read_U8(NV_ADDR_BAT_R10H, (uint8_t*)&(customBatProfile.r10)+1) != NV_OK) dataValid = 0;

	customBatProfile.r50=0xFFFF;
	if (nv_read_U8(NV_ADDR_BAT_R50L, (uint8_t*)&(customBatProfile.r50)) != NV_OK) dataValid = 0;
	if (nv_read_U8(NV_ADDR_BAT_R50H, (uint8_t*)&(customBatProfile.r50)+1) != NV_OK) dataValid = 0;

	customBatProfile.r90=0xFFFF;
	if (nv_read_U8(NV_ADDR_BAT_R90L, (uint8_t*)&(customBatProfile.r90)) != NV_OK) dataValid = 0;
	if (nv_read_U8(NV_ADDR_BAT_R90H, (uint8_t*)&(customBatProfile.r90)+1) != NV_OK) dataValid = 0;

	return !dataValid; // return 0 if valid
}

int8_t BatReadEEprofileData(void) {
	uint8_t dataValid = 1;
	uint8_t var8;
	uint16_t var16 = 0;

	/* Capacity is the one profile field that needs the full 16 bit cell, so it carries no
	 * complement byte - all it can be checked for is presence. */
	if (nv_read_U16(NV_ADDR_BAT_CAPACITY, &var16) != NV_OK) dataValid = 0;
	customBatProfile.capacity = UNPACK_CAPACITY_U16(var16); // correction for large capacities over 32767

	if (nv_read_U8(NV_ADDR_CHARGE_CURRENT, &customBatProfile.chargeCurrent) != NV_OK) dataValid = 0;
	if (nv_read_U8(NV_ADDR_CHARGE_TERM_CURRENT, &customBatProfile.terminationCurr) != NV_OK) dataValid = 0;
	if (nv_read_U8(NV_ADDR_BAT_REG_VOLTAGE, &customBatProfile.regulationVoltage) != NV_OK) dataValid = 0;
	if (nv_read_U8(NV_ADDR_BAT_CUTOFF_VOLTAGE, &customBatProfile.cutoffVoltage) != NV_OK) dataValid = 0;

	/* The temperature points are int8_t, the NV layer deals in unsigned bytes. */
	if (nv_read_U8(NV_ADDR_BAT_TEMP_COLD, &var8) != NV_OK) dataValid = 0; else customBatProfile.tCold = (int8_t)var8;
	if (nv_read_U8(NV_ADDR_BAT_TEMP_COOL, &var8) != NV_OK) dataValid = 0; else customBatProfile.tCool = (int8_t)var8;
	if (nv_read_U8(NV_ADDR_BAT_TEMP_WARM, &var8) != NV_OK) dataValid = 0; else customBatProfile.tWarm = (int8_t)var8;
	if (nv_read_U8(NV_ADDR_BAT_TEMP_HOT, &var8) != NV_OK) dataValid = 0; else customBatProfile.tHot = (int8_t)var8;

	/* NTC constants are 16 bit as well, guarded by their own xor checksum variable. */
	if (nv_read_U16(NV_ADDR_BAT_NTC_B, &customBatProfile.ntcB) != NV_OK) dataValid = 0;
	if (nv_read_U16(NV_ADDR_BAT_NTC_RESISTANCE, &customBatProfile.ntcResistance) != NV_OK) dataValid = 0;
	if (nv_read_U16(NV_ADDR_BAT_NTC_CRC, &var16) != NV_OK) dataValid = 0;
	if (var16 != (customBatProfile.ntcB ^ customBatProfile.ntcResistance)) dataValid = 0;

	return !dataValid; // return 0 if valid
}

void BatWriteEEprofileData(BatteryProfile_T *batProfile) {
	uint16_t var = PACK_CAPACITY_U16(batProfile->capacity); // correction for large capacities over 32767
	nv_write_U16(NV_ADDR_BAT_CAPACITY, var);
	nv_write_U8(NV_ADDR_CHARGE_CURRENT, batProfile->chargeCurrent);
	nv_write_U8(NV_ADDR_CHARGE_TERM_CURRENT, batProfile->terminationCurr);
	nv_write_U8(NV_ADDR_BAT_REG_VOLTAGE, batProfile->regulationVoltage);
	nv_write_U8(NV_ADDR_BAT_CUTOFF_VOLTAGE, batProfile->cutoffVoltage);
	nv_write_U8(NV_ADDR_BAT_TEMP_COLD, batProfile->tCold);
	nv_write_U8(NV_ADDR_BAT_TEMP_COOL, batProfile->tCool);
	nv_write_U8(NV_ADDR_BAT_TEMP_WARM, batProfile->tWarm);
	nv_write_U8(NV_ADDR_BAT_TEMP_HOT, batProfile->tHot);
	nv_write_U16(NV_ADDR_BAT_NTC_B, batProfile->ntcB);
	nv_write_U16(NV_ADDR_BAT_NTC_RESISTANCE, batProfile->ntcResistance);
	nv_write_U16(NV_ADDR_BAT_NTC_CRC, batProfile->ntcB ^ batProfile->ntcResistance);
}

void BatWriteExtendedEEprofileData(BatteryProfile_T *batProfile) {
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

static uint32_t chargeLedTaskMsCounter;

void BatteryTask(void) {
	static uint8_t b = 0;

	if (setProfileReq >= 0) {
		uint8_t id = setProfileReq;
		setProfileReq = -1;
		nv_write_U8(NV_ADDR_BAT_PROFILE, id);
		uint8_t storedId;
		if (nv_read_U8(NV_ADDR_BAT_PROFILE, &storedId) == NV_OK) {
			BatInitProfile(storedId);
		} else {
			currentBatProfile = NULL;
			batProfileStatus = BATTERY_INVALID_PROFILE_ID;
		}

		ChargerSetBatProfileReq(currentBatProfile);
		PowerSourceSetBatProfile(currentBatProfile);
		FuelGaugeSetBatProfile(currentBatProfile);
	}

	if (writeCustomProfileReq==1) {
		writeCustomProfileReq = 0;
		BatWriteEEprofileData(&customBatProfileReq);
		if (BatReadEEprofileData() == 0) {
			batProfileStatus = BATTERY_CUSTOM_PROFILE_ID;
			currentBatProfile = &customBatProfile;
		} else {
			batProfileStatus = BATTERY_INVALID_CUSTOM_PROFILE_STATUS;
			currentBatProfile = NULL;
		}

		uint8_t storedId;
		if (nv_read_U8(NV_ADDR_BAT_PROFILE, &storedId) != NV_OK || storedId != BATTERY_CUSTOM_PROFILE_ID) {
			BatReadExtendedEEprofileData();
			nv_write_U8(NV_ADDR_BAT_PROFILE, BATTERY_CUSTOM_PROFILE_ID);
			if (nv_read_U8(NV_ADDR_BAT_PROFILE, &storedId) == NV_OK
			    && storedId == BATTERY_CUSTOM_PROFILE_ID) {
				if (currentBatProfile != NULL)
					batProfileStatus = BATTERY_CUSTOM_PROFILE_ID;
				else
					batProfileStatus = BATTERY_INVALID_CUSTOM_PROFILE_STATUS;
			} else {
				currentBatProfile = NULL;
				batProfileStatus = BATTERY_INVALID_PROFILE_ID;
			}
		}

		ChargerSetBatProfileReq(currentBatProfile);
		PowerSourceSetBatProfile(currentBatProfile);
		FuelGaugeSetBatProfile(currentBatProfile);
	} else if (writeCustomProfileReq==2) {
		writeCustomProfileReq = 0;
		BatWriteExtendedEEprofileData(&customBatProfileReq);
		BatReadExtendedEEprofileData();
		if (batProfileStatus == BATTERY_CUSTOM_PROFILE_ID) {
			FuelGaugeSetBatProfile(currentBatProfile);
		}
	}

	if (!CHARGER_IS_BATTERY_PRESENT() || batteryVoltage < 2500) {
		batteryStatus = BAT_STATUS_NOT_PRESENT;
	} else if (chargerStatus == CHG_CHARGING_FROM_IN) {
		batteryStatus = BAT_STATUS_CHARGING_FROM_IN;
	} else if (chargerStatus == CHG_CHARGING_FROM_USB) {
		batteryStatus = BAT_STATUS_CHARGING_FROM_5V_IO;
	} else {
		batteryStatus = BAT_STATUS_NORMAL;
	}

	if (MS_TIME_COUNT(chargeLedTaskMsCounter) >= 900/*(state == STATE_LOWPOWER?2000:500)*/) {
		MS_TIME_COUNTER_INIT(chargeLedTaskMsCounter);

		uint8_t r,g;
		if (batteryRsoc > 500) {
			r = 0;
			g = led_GetParamG(LED_CHARGE_STATUS);
		} else if (batteryRsoc > 150) {
			r = led_GetParamR(LED_CHARGE_STATUS);
			g = led_GetParamG(LED_CHARGE_STATUS);
		} else {
			r = led_GetParamR(LED_CHARGE_STATUS);
			g = 0;
		}
		uint8_t paramB = led_GetParamB(LED_CHARGE_STATUS);

		if (batteryStatus == BAT_STATUS_CHARGING_FROM_IN || batteryStatus == BAT_STATUS_CHARGING_FROM_5V_IO) {//if (chargerStatus == CHG_CHARGING_FROM_IN || chargerStatus == CHG_CHARGING_FROM_USB) {
			b = b?0:paramB;//200 - r;
		} else if (chargerStatus == CHG_CHARGE_DONE) {
			b = paramB;
		} else {
			b = 0;
		}

		led_SetFuncRGB(LED_CHARGE_STATUS, r, g, b);
	}
}

int8_t BatterySetProfileReq(uint8_t id) {
	if (batProfileStatus != id) {
		batProfileStatus = BATTERY_PROFILE_WRITE_BUSY_STATUS;
		setProfileReq = id;
	}
	/*if (id < BATTERY_PROFILES_COUNT() || id == BATTERY_CUSTOM_PROFILE_ID || id == BATTERY_DEFAULT_PROFILE_ID) {
		setProfileReq = id;
		return 0;
	} else {
		return -1;
	}*/
	return 0;
}

int8_t BatteryWriteCustomProfileReq(uint8_t *data, uint16_t len) {
	batProfileStatus = BATTERY_PROFILE_WRITE_BUSY_STATUS;

	volatile uint16_t var = (((uint16_t)data[1])<<8) | data[0];
	customBatProfileReq.capacity = UNPACK_CAPACITY_U16(var); // correction for large capacities over 32767
	customBatProfileReq.chargeCurrent = data[2];
	customBatProfileReq.terminationCurr = data[3];
	customBatProfileReq.regulationVoltage = data[4];
	customBatProfileReq.cutoffVoltage = data[5];
	customBatProfileReq.tCold = data[6];
	customBatProfileReq.tCool = data[7];
	customBatProfileReq.tWarm = data[8];
	customBatProfileReq.tHot = data[9];
	customBatProfileReq.ntcB = (((uint16_t)data[11])<<8) | data[10];
	customBatProfileReq.ntcResistance = (((uint16_t)data[13])<<8) | data[12];

	writeCustomProfileReq = 1;
	return 0;
}

int8_t BatteryReadCurrentProfile(uint8_t *data, uint16_t *len) {
	if (writeCustomProfileReq || setProfileReq >= 0 || currentBatProfile == NULL) {
		int i = 0;
		for (i = 0; i < 14; i++) data[i] = 0;
	} else {
		volatile uint16_t var = PACK_CAPACITY_U16(currentBatProfile->capacity); // correction for large capacities over 32767
		data[0] = var; //currentBatProfile->capacity;
		data[1] = var>>8; //currentBatProfile->capacity>>8;
		data[2] = currentBatProfile->chargeCurrent;
		data[3] = currentBatProfile->terminationCurr;
		data[4] = currentBatProfile->regulationVoltage;
		data[5] = currentBatProfile->cutoffVoltage;
		data[6] = currentBatProfile->tCold;
		data[7] = currentBatProfile->tCool;
		data[8] = currentBatProfile->tWarm;
		data[9] = currentBatProfile->tHot;
		data[10] = currentBatProfile->ntcB;
		data[11] = currentBatProfile->ntcB>>8;
		data[12] = currentBatProfile->ntcResistance;
		data[13] = currentBatProfile->ntcResistance>>8;

	}
	*len = 14;
	return 0;
}

int8_t BatteryWriteCustomExtendedProfileReq(uint8_t data[], uint16_t len) {
	//batProfileStatus = BATTERY_PROFILE_WRITE_BUSY_STATUS;

	customBatProfileReq.chemistry = data[0];
	customBatProfileReq.ocv10 = *(uint16_t*)&data[1];
	customBatProfileReq.ocv50 = *(uint16_t*)&data[3];
	customBatProfileReq.ocv90 = *(uint16_t*)&data[5];
	customBatProfileReq.r10 = *(uint16_t*)&data[7];
	customBatProfileReq.r50 = *(uint16_t*)&data[9];
	customBatProfileReq.r90 = *(uint16_t*)&data[11];

	writeCustomProfileReq = 2;
	return 0;
}

int8_t BatteryReadCurrentExtendedProfile(uint8_t *data, uint16_t *len) {
	data[0] = currentBatProfile->chemistry;
	data[1] = currentBatProfile->ocv10;
	data[2] = currentBatProfile->ocv10>>8;
	data[3] = currentBatProfile->ocv50;
	data[4] = currentBatProfile->ocv50>>8;
	data[5] = currentBatProfile->ocv90;
	data[6] = currentBatProfile->ocv90>>8;
	data[7] = currentBatProfile->r10;
	data[8] = currentBatProfile->r10>>8;
	data[9] = currentBatProfile->r50;
	data[10] = currentBatProfile->r50>>8;
	data[11] = currentBatProfile->r90;
	data[12] = currentBatProfile->r90>>8;
	data[13] = 0xFF; // reserved for future use
	data[14] = 0xFF; // reserved for future use
	data[15] = 0xFF; // reserved for future use
	data[16] = 0xFF; // reserved for future use

	*len = 17;
	return 0;
}

const BatteryProfile_T *BatteryGetProfile(void) {
	/*if (batProfileStatus < BATTERY_PROFILES_COUNT) {
		return &(batteryProfiles[batProfileStatus]);
	} else {
		return &currentBatProfile;
	}*/
	return currentBatProfile;
}

int8_t BatteryReadProfileStatus(uint8_t *data, uint16_t *len) {
	data[0] = batProfileStatus;
	*len = 1;
	return 0;
}
