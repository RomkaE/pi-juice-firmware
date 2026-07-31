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

#include "battery.h"    // BatteryProfile_T only - no behaviour dependency on battery.c

// Mask errors:
#define FUEL_GAUGE_ERR_QUEUE_FULL   (1UL << 0)  // event dropped, the task did not drain in time
#define FUEL_GAUGE_ERR_NOT_READY    (1UL << 1)  // event posted before fuel_gauge_Init() created the queue

/* Answer of fuel_gauge_GetRsoc() while the LC709203F is not the data source. Not a charge level -
 * callers must test for it before comparing against a threshold. */
#define FUEL_GAUGE_RSOC_UNKNOWN     0xFFFF

typedef enum BatteryTempSenseConfig_T {
	BAT_TEMP_SENSE_CONFIG_NOT_USED = 0,
	BAT_TEMP_SENSE_CONFIG_NTC,
	BAT_TEMP_SENSE_CONFIG_ON_BOARD,
	BAT_TEMP_SENSE_CONFIG_AUTO_DETECT,
	BAT_TEMP_SENSE_CONFIG_END
} BatteryTempSenseConfig_T;

/*
 * Host visible, but only AUTO_DETECT is implemented: DIRECT_DV selected a software SOC model that
 * has been removed. A write carrying it is accepted and ignored, and the field always reads back
 * as AUTO_DETECT.
 */
typedef enum RsocMeasurementConfig_T {
	RSOC_MEASUREMENT_AUTO_DETECT = 0,
	RSOC_MEASUREMENT_DIRECT_DV,
	RSOC_MEASUREMENT_CONFIG_END
} RsocMeasurementConfig_T;

/*
 * Configuration byte, host register 0x93 layout:
 *   bits 0-2 - BatteryTempSenseConfig_T
 *   bits 4-5 - RsocMeasurementConfig_T
 *
 * This module only applies the byte, it never stores it: persistence - and therefore the whole
 * NV_ADDR_FUEL_GAUGE_CONFIG round trip - belongs to the APP task, see app_FuelGaugeCmdSetConfig().
 */
#define FUEL_GAUGE_CONFIG_TEMP_SENSE_MASK   0x07
#define FUEL_GAUGE_CONFIG_RSOC_MASK         0x30
#define FUEL_GAUGE_CONFIG_RSOC_SHIFT        4
#define FUEL_GAUGE_CONFIG_DEFAULT \
    ((uint8_t)(BAT_TEMP_SENSE_CONFIG_AUTO_DETECT | (RSOC_MEASUREMENT_AUTO_DETECT << FUEL_GAUGE_CONFIG_RSOC_SHIFT)))

/*
 * This module knows two things: the LC709203F on I2C2, and the APP task. It never calls into
 * another subsystem, and nothing but APP calls the setters below - which is why it needs to be
 * told about the battery instead of asking the charger, and needs to be handed its configuration
 * instead of reading NV.
 *
 * Creates the task and its queue, nothing else - the device is brought up inside the task. Must
 * be called after i2c_master_Init(). config is the initial configuration byte; the battery
 * profile and the battery presence arrive afterwards through the setters.
 */
void fuel_gauge_Init(uint8_t config);

/*
 * Parameter setters, all called from the APP task. Each one only hands a value to the task, which
 * applies it at the next turn of its loop - no NV access, no blocking, no bus traffic here.
 */
void fuel_gauge_SetConfig(uint8_t config);
void fuel_gauge_SetBatProfile(const BatteryProfile_T *batProfile);

/* Told by APP (see battery.c, which aggregates the charger's BATSTAT and the ADC voltage).
 * The module does not second-guess it. */
void fuel_gauge_SetBatteryPresent(bool present);

// Published telemetry, safe to read from any task or from the I2C1 interrupt:
uint16_t fuel_gauge_GetRsoc(void);     // FUEL_GAUGE_RSOC_UNKNOWN when the IC is not the source
int8_t fuel_gauge_GetTemp(void);       // battery temperature, or the MCU sensor as a fallback
BatteryTempSenseConfig_T fuel_gauge_GetTempSenseConfig(void);
bool fuel_gauge_IsIcFault(void);
bool fuel_gauge_IsTempSenseFault(void);

/* Rejects reserved codes - the caller validates before it persists anything. */
bool fuel_gauge_IsConfigValid(uint8_t config);

/* Configuration currently in effect, packed back into the register 0x93 layout. */
uint8_t fuel_gauge_GetConfig(void);

uint32_t fuel_gauge_GetErrMask(bool _clear);

#endif /* FUEL_GAUGE_LC709203F_H_ */
