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

typedef enum
{
	CHG_STATUS_NO_VALID_SOURCE = 0,
	CHG_STATUS_IN_READY,
	CHG_STATUS_USB_READY,
	CHG_STATUS_CHARGING_FROM_IN,
	CHG_STATUS_CHARGING_FROM_USB,
	CHG_STATUS_CHARGE_DONE,
	CHG_STATUS_NA,
	CHG_STATUS_FAULT
} ChargerStatus_t;

typedef enum
{
  CHG_FAULT_NORMAL = 0,
  CHG_FAULT_THERMAL_SHUTDOWN,
  CHG_FAULT_BATTERY_TEMPERATURE_FAULT,
  CHG_FAULT_WATCHDOG_TIMER_EXPIRED,
  CHG_FAULT_SAFETY_TIMER_EXPIRED,
  CHG_FAULT_IN_SUPPLY_FAULT,
  CHG_FAULT_USB_SUPPLY_FAULT,
  CHG_FAULT_BATTERY_FAULT,
  CHG_FAULT_UNKNOWN
} ChargerFaultStatus_t;

typedef enum
{
  CHG_IN_NORMAL = 0,
  CHG_IN_OVP,
  CHG_IN_WEAK,
  CHG_IN_UVLO
} ChargerInputStatus_t;

// Codes of the IUSB_LIMIT field. Only the minimum is ever written now - see BQ_IUSB_LIMIT_IMAGE.
typedef enum ChargerUsbInCurrentLimit_T {
	CHG_IUSB_LIMIT_100MA = 0,
	CHG_IUSB_LIMIT_150MA,
	CHG_IUSB_LIMIT_500MA,
	CHG_IUSB_LIMIT_800MA,
	CHG_IUSB_LIMIT_900MA,
	CHG_IUSB_LIMIT_1500MA,
} ChargerUsbInCurrentLimit_T;

typedef struct
{
  ChargerStatus_t status;
  ChargerFaultStatus_t fault;
  ChargerInputStatus_t in_stat;
  bool batt_present;
  bool dpm_stat;
//  uint8_t ts_fault;
//  uint8_t usb_stat;
//  uint8_t input_present;    // 0 or 1
} ChargerSnapshot_t;

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
ChargerStatus_t charger_GetStatus(void);
bool charger_IsBatteryPresent(void);
bool charger_IsInputPresent(void);
bool charger_GetDpmStatus(void);
ChargerInputStatus_t charger_GetInStatus(void);
ChargerFaultStatus_t charger_GetFaultStatus(void);
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

// Which fields of the snapshot changed - the argument of charger_SnapshotChangedCallback().
#define CHG_CHANGED_STATUS        0x01
#define CHG_CHANGED_FAULT         0x02
#define CHG_CHANGED_IN_STATUS     0x04
#define CHG_CHANGED_BATT_PRESENT  0x08
#define CHG_CHANGED_DPM_STATUS    0x10

/* The mask is what the publisher already computes to log the transitions, so the consumer does not
 * have to keep a second mirror of the snapshot just to rediscover it. */
void charger_SnapshotChanged_Callback(const ChargerSnapshot_t *_p_snapshot, uint8_t _changed);

#endif /* CHARGER_BQ2416X_H_ */
