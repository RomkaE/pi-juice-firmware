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

#include "to_refactor/battery.h"

// Mask errors:
#define CHARGER_ERR_QUEUE_FULL   (1UL << 0)  // command dropped, the task did not drain in time
#define CHARGER_ERR_NOT_READY    (1UL << 1)  // command posted before charger_Init() created the queue
#define CHARGER_ERR_EVENT_LOST   (1UL << 2)  // input-presence event dropped, the APP event queue was full

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

typedef enum NTC_MonitorTemperature_T {
	COLD,
	COOL,
	NORMAL,
	WARM,
	HOT,
	UNKNOWN
} NTC_MonitorTemperature_T;

typedef enum ChargerUSBInLockoutStatus_T {
	CHG_USB_IN_UNKNOWN,
	CHG_USB_IN_LOCK,
	CHG_USB_IN_UNLOCK
} ChargerUSBInLockoutStatus_T;

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

/*
 * Creates the charger task and its queue, nothing else: register I/O and NV access happen
 * inside the task body, so this must be called after i2c_master_Init() and nv_Init(), and after
 * battery_Init() (the task reads the current profile via battery_GetProfile() on startup).
 */
void charger_Init(bool _reset);

/* Wakes the task from the CHG_INT EXTI edge. Safe from an interrupt only. */
void charger_NotifyFromISR(void);

// Published status, safe to read from any task or from the I2C1 interrupt:
ChargerStatus_T charger_GetStatus(void);
bool charger_IsBatteryPresent(void);
bool charger_IsInputPresent(void);
uint8_t charger_GetInStat(void);
uint8_t charger_GetUsbStat(void);
bool charger_IsDpmModeActive(void);
ChargerUSBInLockoutStatus_T charger_GetUsbInLockoutStatus(void);
uint8_t charger_GetTsFaultStatus(void);
ChargerFaultStatus_T charger_GetFaultStatus(void);
uint8_t charger_GetI2cErrorCount(void);
bool charger_IsNoBatteryTurnOnEnabled(void);

void charger_TriggerNtcMonitor(NTC_MonitorTemperature_T temp);

void charger_SetUsbLockout(ChargerUSBInLockoutStatus_T status);
void charger_UsbInCurrentLimitStepUp(void);
void charger_UsbInCurrentLimitSetMin(void);
void charger_UsbInCurrentLimitStepDown(void);

/* Host registers - config writes are queued to the task, reads answer from the applied/mirrored
 * value the same way led_CmdGetConfig() does. */
void charger_CmdWriteInputsConfig(uint8_t config);
uint8_t charger_ReadInputsConfig(void);
void charger_CmdWriteChargingConfig(uint8_t config);
uint8_t charger_ReadChargingConfig(void);

/* Posted by the APP task when battery.c selects a new profile (APP_EVT_BATTERY_PROFILE_CHANGED).
 * Copies the profile into the task's own state - never hands out a live pointer across tasks. */
void charger_CmdSetBatProfile(const BatteryProfile_T *batProfile);

uint32_t charger_GetErrMask(bool _clear);

/*
 * Hook for the APP task: called after the charger detects an edge on "is an input source
 * present". Weak no-op by default - same as button_OnEvent_PowerOn() - ready for real logic
 * without another refactor.
 */
void charger_OnEvent_InputPresenceChanged(bool present);

#endif /* CHARGER_BQ2416X_H_ */
