/*
 * power_manager.h
 *
 * Replaces power_source.c + power_management.c. Dropped along the way: detection of an external
 * 5V source on the Pi header (the probe read a 64 ms ADC average, so it never saw its own
 * stimulus), and the selectable regulator mode - DC-DC is hardwired, PA11 stays low.
 *
 *  Created on: 2026
 *      Author: Roman Egoshin
 */

#ifndef POWER_MANAGER_H_
#define POWER_MANAGER_H_

#include <stdint.h>
#include <stdbool.h>

#include "power/battery.h"

// Register 0x40 bits 4-5 (IN) and 6-7 (5V IO).
typedef enum
{
	PWR_SOURCE_NOT_PRESENT = 0,
	PWR_SOURCE_BAD,
	PWR_SOURCE_WEAK,
	PWR_SOURCE_NORMAL
} PowerSourceStatus_t;

typedef enum
{
	RUN_PIN_NOT_INSTALLED = 0,
	RUN_PIN_INSTALLED,
} RunPinInstallationStatus_t;

// Register 0x45 bits 0-3. Bits 5 and 6 belong to battery.c and the charger.
#define PWR_STATUS_POWER_OFF_BTN     0x01
#define PWR_STATUS_FORCED_POWER_OFF  0x02
#define PWR_STATUS_FORCED_VSYS_OFF   0x04
#define PWR_STATUS_HOST_WDT_EXPIRED  0x08

#define PWR_REGULATOR_MODE_DCDC     2

// Carried by APP_EVT_POWER_PROTECTION:
#define PWR_FAULT_5V_MASK           0x01
#define PWR_FAULT_AVDD              0x02
#define PWR_FAULT_VBAT_CUTOFF       0x04

void pwr_mngr_Init(bool _cold_start);

bool pwr_mngr_HostOn(void);

bool pwr_mngr_HostOff(void);

void pwr_mngr_SetBatProfile(const BatteryProfile_T *_p_profile);

void pwr_mngr_Arm5vCheck(bool _armed);

// True while the rail reads good, or while AVDD is too low for the reading to mean anything.
bool pwr_mngr_Is5vRailGood(void);

// Published state, safe to read from any task or from the I2C1 interrupt.
PowerSourceStatus_t pwr_mngr_GetInStatus(void);
PowerSourceStatus_t pwr_mngr_Get5vIoStatus(void);
bool pwr_mngr_IsHostPowered(void);
uint8_t pwr_mngr_GetStatusFlags(void);
void pwr_mngr_SetStatusFlags(uint8_t _flags);   // OR-ed in by whoever knows the cause
void pwr_mngr_KeepStatusFlags(uint8_t _mask);   // host write to 0x45: keep only the masked bits

// Host commands, called from command_server in the I2C1 interrupt.
/* Register 0x5F, kept for host compatibility: the RUN pin is not wired on this board, so the
 * stored value is handed back and nothing else. */
void pwr_mngr_CmdSetRunPinConfig(uint8_t data[], uint8_t len);
void pwr_mngr_CmdGetRunPinConfig(uint8_t data[], uint16_t *len);
void pwr_mngr_CmdSetVSysSwitchState(uint8_t _state);
uint8_t pwr_mngr_CmdGetVSysSwitchState(void);

// Register 0x96, kept for host compatibility: writes ignored, reads answer DC-DC.
void pwr_mngr_CmdSetRegulatorConfig(uint8_t data[], uint8_t len);
void pwr_mngr_CmdGetRegulatorConfig(uint8_t data[], uint16_t *len);

#endif /* POWER_MANAGER_H_ */
