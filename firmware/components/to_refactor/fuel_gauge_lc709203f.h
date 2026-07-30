/*
 * fuel_gauge_lc709203f.h
 *
 *  Created on: 06.12.2016.
 *      Author: milan
 */

#ifndef FUEL_GAUGE_LC709203F_H_
#define FUEL_GAUGE_LC709203F_H_

#include <stdint.h>
#include <stdbool.h>

#include "to_refactor/battery.h"

#define FUEL_GAUGE_TEMP_MODE_THERMISTOR 	1
#define FUEL_GAUGE_TEMP_MODE_I2C		 	0

// Mask errors:
#define FUEL_GAUGE_ERR_QUEUE_FULL   (1UL << 0)  // command dropped, the task did not drain in time
#define FUEL_GAUGE_ERR_NOT_READY    (1UL << 1)  // command posted before fuel_gauge_Init() created the queue

typedef enum BatteryTempSenseConfig_T {
	BAT_TEMP_SENSE_CONFIG_NOT_USED = 0,
	BAT_TEMP_SENSE_CONFIG_NTC,
	BAT_TEMP_SENSE_CONFIG_ON_BOARD,
	BAT_TEMP_SENSE_CONFIG_AUTO_DETECT,
	BAT_TEMP_SENSE_CONFIG_END
} BatteryTempSenseConfig_T;

typedef enum RsocMeasurementConfig_T {
	RSOC_MEASUREMENT_AUTO_DETECT = 0,
	RSOC_MEASUREMENT_DIRECT_DV,
	RSOC_MEASUREMENT_CONFIG_END
} RsocMeasurementConfig_T;

/*
 * Creates the fuel gauge task and its queue, nothing else: I2C2 priming and NV access happen
 * inside the task body, so this must be called after i2c_master_Init() and nv_Init(), and after
 * battery_Init() (the task reads the current profile via battery_GetProfile() on startup).
 * Always does a full from-scratch init (SOC tables rebuilt, NTC fault flag cleared) - unlike
 * charger_Init(), it does not distinguish a POR from a warm reset.
 */
void fuel_gauge_Init(void);

// Published telemetry, safe to read from any task or from the I2C1 interrupt:
uint16_t fuel_gauge_GetVoltage(void);
uint16_t fuel_gauge_GetRsoc(void);
int8_t fuel_gauge_GetTemp(void);
int16_t fuel_gauge_GetCurrent(void);
BatteryTempSenseConfig_T fuel_gauge_GetTempSenseConfig(void);
bool fuel_gauge_IsIcFault(void);
bool fuel_gauge_IsTempSenseFault(void);

/* Posted by the APP task when battery.c selects a new profile (APP_EVT_BATTERY_PROFILE_CHANGED).
 * Copies the profile into the task's own state - never hands out a live pointer across tasks. */
void fuel_gauge_CmdSetBatProfile(const BatteryProfile_T *batProfile);

/* Host register 0x93 - fuel gauge configuration, persisted in the emulated EEPROM. */
int8_t fuel_gauge_CmdSetConfig(uint8_t *data, uint16_t len);
void fuel_gauge_GetConfig(uint8_t data[], uint16_t *len);

uint32_t fuel_gauge_GetErrMask(bool _clear);

#endif /* FUEL_GAUGE_LC709203F_H_ */
