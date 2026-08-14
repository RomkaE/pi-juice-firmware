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
#include "battery.h"

// Mask errors:
#define FUEL_GAUGE_ERR_QUEUE_FULL   (1UL << 0)  // event dropped, the task did not drain in time
#define FUEL_GAUGE_ERR_NOT_READY    (1UL << 1)  // event posted before fuel_gauge_Init() created the queue

#define FUEL_GAUGE_RSOC_UNKNOWN     (UINT16_MAX)
#define FUEL_GAUGE_TEMP_UNKNOWN     (INT8_MIN)

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
    ((uint8_t)((RSOC_MEASUREMENT_AUTO_DETECT << FUEL_GAUGE_CONFIG_RSOC_SHIFT) | BAT_TEMP_SENSE_CONFIG_NTC))

// TODO - move to the app:
typedef enum
{
  BAT_TEMP_SENSE_CONFIG_NOT_USED = 0,
  BAT_TEMP_SENSE_CONFIG_NTC,
  BAT_TEMP_SENSE_CONFIG_ON_BOARD,
  BAT_TEMP_SENSE_CONFIG_AUTO_DETECT,
} BatteryTempSenseConfig_t;

// TODO - move to the app:
/* RSOC comes from the LC709203F itself - the software OCV model that the DIRECT_BY_MCU code used
 * to select is gone, so the IC is the only estimator left. */
typedef enum
{
  RSOC_MEASUREMENT_AUTO_DETECT = 0,
//  RSOC_MEASUREMENT_DIRECT_DV = 1,
} RsocMeasurementConfig_t;

void fuel_gauge_Init(BatteryProfile_T *_p_batt_profile);

void fuel_gauge_SetConfig(uint8_t config);

void fuel_gauge_SetBattProfile(const BatteryProfile_T *_p_batt_profile);

void fuel_gauge_SetBatteryPresent(bool present);

// Published telemetry, safe to read from any task or from the I2C1 interrupt:
uint16_t fuel_gauge_GetRsoc(void);     // FUEL_GAUGE_RSOC_UNKNOWN when the IC is not the source
int8_t fuel_gauge_GetTemp(void);       // battery temperature, or the MCU sensor as a fallback
BatteryTempSenseConfig_t fuel_gauge_GetTempSenseConfig(void);
bool fuel_gauge_IsIcFault(void);
bool fuel_gauge_IsTempSenseFault(void);

/* Rejects reserved codes - the caller validates before it persists anything. */
bool fuel_gauge_IsConfigValid(uint8_t config);

/* Configuration currently in effect, packed back into the register 0x93 layout. */
uint8_t fuel_gauge_GetConfig(void);

uint32_t fuel_gauge_GetErrMask(bool _clear);

void fuel_gauge_Temp_Callback(int8_t _temp);

void fuel_gauge_Rsoc_Callback(uint16_t _rsoc);

#endif /* FUEL_GAUGE_LC709203F_H_ */
