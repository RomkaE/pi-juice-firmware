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

#include "power/battery.h"   // BatteryProfile_T / BatteryThermalState_T - data types only

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
 * Used when NV holds nothing usable: 5V GPIO has precedence for charging, USB-IN enabled, no
 * turn-on without a battery, 2.5 A IN limit, VIN-DPM 4.2 V / charging enabled.
 */
#define CHARGER_INPUTS_CONFIG_DEFAULT    ((uint8_t)0x0B)
#define CHARGER_CHARGING_CONFIG_DEFAULT  ((uint8_t)0x01)

/* Which way charger_SetUsbILim() moves the USB input current limit. */
typedef enum ChargerUsbILimStep_T {
	CHG_ILIM_STEP_DOWN = 0,
	CHG_ILIM_STEP_UP,
	CHG_ILIM_SET_MIN
} ChargerUsbILimStep_T;

/*
 * This module knows two things: the bq2416x on I2C2, and the APP task. It never calls into
 * another subsystem - which is why it is told the battery temperature verdict instead of reading
 * the fuel gauge, told whether a 5V input was detected instead of reaching into power_source, and
 * handed its configuration instead of reading NV.
 *
 * Creates the task and its queue, nothing else - the device is brought up inside the task, so
 * this must be called after i2c_master_Init(). The two configuration bytes come from NV and are
 * used only when _reset is true (power-on): on a warm reset the module keeps whatever the host
 * last configured, which is why those settings live in the no_init section.
 */
void charger_Init(uint8_t _nvInputsConfig, uint8_t _nvChargingConfig);

/* Posts the CHG_INT edge to the task. Safe from an interrupt only. */
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

/*
 * Setters, all called from the APP task. Each one only hands a value to the task, which applies
 * it at the next turn of its loop - no NV access, no blocking, no bus traffic in the caller's
 * context. That last point is the reason charger_SetUsbLockout() and the current-limit calls are
 * here rather than doing the register write directly: the register cache belongs to the CHG task.
 */
void charger_SetInputsConfig(uint8_t config);
void charger_SetChargingConfig(uint8_t config);
void charger_SetBatProfile(const BatteryProfile_T *batProfile);

/* Battery temperature verdict, aggregated by battery.c from the profile and the live reading. */
void charger_SetThermalState(BatteryThermalState_T state);

/*
 * Whether a 5V input source has been detected. USB-IN charging stays locked out until this is
 * true. Defaults to false, and nothing sets it while power_source.c is dormant - so USB-IN
 * charging is currently disabled. Same behaviour as before, but now it is one explicit parameter
 * instead of a silent consequence of dead code.
 */
void charger_Set5vInDetected(bool detected);

/* Re-evaluate the USB-IN lockout now, without waiting for the next tick. */
void charger_SetUsbLockout(ChargerUSBInLockoutStatus_T status);

void charger_SetUsbILim(ChargerUsbILimStep_T step);

uint32_t charger_GetErrMask(bool _clear);

/*
 * Hook for the APP task: called after the charger detects an edge on "is an input source
 * present". Weak no-op by default - same as button_OnEvent_PowerOn() - ready for real logic
 * without another refactor.
 */
void charger_OnEvent_InputPresenceChanged(bool present);

#endif /* CHARGER_BQ2416X_H_ */
