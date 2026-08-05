/*
 * charger_bq2416x.h
 *
 *  Created on: 06.12.2016.
 *      Author: milan
 */

#ifndef CHARGER_BQ2416X_H_
#define CHARGER_BQ2416X_H_

#include <stdint.h>
#include <stdbool.h>

#include "power/battery.h"

// Mask errors:
#define CHARGER_ERR_QUEUE_FULL   (1UL << 0)  // event dropped, the task did not drain in time
#define CHARGER_ERR_NOT_READY    (1UL << 1)  // event posted before charger_Init() created the queue
#define CHARGER_ERR_EVENT_LOST   (1UL << 2)  // presence event dropped, the APP event queue was full

typedef enum ChargerStatus_T {
	CHG_NO_VALID_SOURCE = 0,
	CHG_IN_READY,
	CHG_USB_READY,
	CHG_CHARGING_FROM_IN,
	CHG_CHARGING_FROM_USB,
	CHG_CHARGE_DONE,
	CHG_NA,
	CHG_FAULT
} ChargerStatus_T;

// Codes of the IUSB_LIMIT field. Only the minimum is ever written now - see BQ_IUSB_LIMIT_IMAGE.
typedef enum ChargerUsbInCurrentLimit_T {
	CHG_IUSB_LIMIT_100MA = 0,
	CHG_IUSB_LIMIT_150MA,
	CHG_IUSB_LIMIT_500MA,
	CHG_IUSB_LIMIT_800MA,
	CHG_IUSB_LIMIT_900MA,
	CHG_IUSB_LIMIT_1500MA,
} ChargerUsbInCurrentLimit_T;

typedef enum ChargerFaultStatus_T {
	CHG_FAULT_NORMAL = 0,
	CHG_FAULT_THERMAL_SHUTDOWN,
	CHG_FAULT_BATTERY_TEMPERATURE_FAULT,
	CHG_FAULT_WATCHDOG_TIMER_EXPIRED,
	CHG_FAULT_SAFETY_TIMER_EXPIRED,
	CHG_FAULT_IN_SUPPLY_FAULT,
	CHG_FAULT_USB_SUPPLY_FAULT,
	CHG_FAULT_BATTERY_FAULT,
	CHG_FAULT_UNKNOWN
} ChargerFaultStatus_T;

// Used when NV holds nothing usable: no turn-on without a battery, 2.5 A IN limit, VIN-DPM 4.2 V,
// charging enabled. The precedence and USB-IN bits are clear and stay clear.
#define CHARGER_INPUTS_CONFIG_DEFAULT    ((uint8_t)0x08)
#define CHARGER_CHARGING_CONFIG_DEFAULT  ((uint8_t)0x01)

void charger_Init(const BatteryProfile_T *_p_batt_profile,
                  uint8_t _in_cfg, uint8_t _chrg_cfg);

// Clears the bits this module refuses to take from the host, so the read-back matches what is
// actually in effect: the input precedence and the USB-IN enable, both fixed by the board.
// Apply wherever a configuration byte enters: a host write, and whatever NV hands back.
uint8_t charger_SanitizeInputsConfig(uint8_t _config);

// Posts the CHG_INT edge to the task. Safe from an interrupt only.
void charger_NotifyFromISR(void);

// Published status, safe to read from any task or from the I2C1 interrupt:
ChargerStatus_T charger_GetStatus(void);
bool charger_IsBatteryPresent(void);
bool charger_IsInputPresent(void);
uint8_t charger_GetInStat(void);
uint8_t charger_GetUsbStat(void);
bool charger_IsDpmModeActive(void);
uint8_t charger_GetTsFaultStatus(void);
ChargerFaultStatus_T charger_GetFaultStatus(void);
// Failures in a row, cleared by any successful bus operation - a streak, not a total.
uint8_t charger_GetI2cErrorCount(void);
bool charger_IsNoBatteryTurnOnEnabled(void);

// Setters, all called from the APP task. Each hands a value to the task and returns - no NV
// access, no blocking, no bus traffic in the caller's context. The register cache belongs to the
// CHG task, which is why even the lockout and current-limit calls go through the queue.
void charger_SetInputsConfig(uint8_t config);
void charger_SetChargingConfig(uint8_t config);
void charger_SetBatProfile(const BatteryProfile_T *batProfile);

// Battery temperature verdict, aggregated by battery.c from the profile and the live reading.
void charger_SetThermalState(BatteryThermalState_T state);

uint32_t charger_GetErrMask(bool _clear);

// Hook for the APP task, called on an edge of "is an input source present". Weak no-op.
void charger_OnEvent_InputPresenceChanged(bool present);

// The same for the rest of the published state. One function rather than seven because nothing
// consumes them yet: _type is the APP_EVT_CHARGER_* that fired, _value the new value.
// uint8_t, not AppEventType_t, so this header does not have to pull in app.h.
void charger_OnEvent_ValueChanged(uint8_t _type, uint8_t _value);

#endif /* CHARGER_BQ2416X_H_ */
