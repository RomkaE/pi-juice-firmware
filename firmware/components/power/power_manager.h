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

// Carried by APP_EVT_POWER_PROTECTION.
typedef enum
{
	PWR_TRIP_NONE = 0,
	PWR_TRIP_VBAT_CUTOFF,   // pack under cutoff, no input source
	PWR_TRIP_5V_FAULT       // boost collapsed or 5V regulator power bad
} PowerTrip_t;

// Register 0x45 bits 0-3. Bits 5 and 6 belong to battery.c and the charger.
#define PWR_FAULT_POWER_OFF_BTN     0x01
#define PWR_FAULT_FORCED_POWER_OFF  0x02
#define PWR_FAULT_FORCED_VSYS_OFF   0x04
#define PWR_FAULT_WATCHDOG_EXPIRED  0x08

#define PWR_REGULATOR_MODE_DCDC     2

// The FSM runs in the APP task - see the contract in src/app.h. Moved by events and pwr_mngr_Tick().
void pwr_mngr_Init(bool _cold_start);

// Runs what is due, returns ms until it wants the next call. Deadlines inside are absolute, so a
// late call moves on later, never wrongly - APP can block for tens of ms in a flash write.
uint32_t pwr_mngr_Tick(void);

// Host power requests, from the APP task. Each answers whether it did anything.
bool pwr_mngr_HostTurnOn(void);
bool pwr_mngr_HostTurnOff(void);
bool pwr_mngr_HostReset(void);

void pwr_mngr_OnEvent_Protection(uint8_t _trip);

void pwr_mngr_HostPollEvent(void);
void pwr_mngr_SetRtcWakeupEvent(void);
void pwr_mngr_SetIoWakeupEvent(void);
void pwr_mngr_SetBatProfile(const BatteryProfile_T *_p_profile);

// Published state, safe to read from any task or from the I2C1 interrupt.
PowerSourceStatus_t pwr_mngr_GetInStatus(void);
PowerSourceStatus_t pwr_mngr_Get5vIoStatus(void);
bool pwr_mngr_IsHostPowered(void);
uint8_t pwr_mngr_GetFaultFlags(void);
void pwr_mngr_KeepFaultFlags(uint8_t _mask);   // host write to 0x45: keep only the masked bits

// Host commands, called from command_server in the I2C1 interrupt.
void pwr_mngr_CmdSchedulePowerOff(uint8_t _delayCode);
uint8_t pwr_mngr_CmdGetPowerOffCounter(void);
void pwr_mngr_CmdSetRunPinConfig(uint8_t data[], uint8_t len);
void pwr_mngr_CmdGetRunPinConfig(uint8_t data[], uint16_t *len);
void pwr_mngr_CmdConfigureWatchdog(uint8_t data[], uint16_t len);
void pwr_mngr_CmdGetWatchdogConfiguration(uint8_t data[], uint16_t *len);
void pwr_mngr_CmdSetWakeupOnCharge(uint8_t data[], uint16_t len);
void pwr_mngr_CmdGetWakeupOnCharge(uint8_t data[], uint16_t *len);
void pwr_mngr_CmdSetVSysSwitchState(uint8_t _state);
uint8_t pwr_mngr_CmdGetVSysSwitchState(void);

// Register 0x96, kept for host compatibility: writes ignored, reads answer DC-DC.
void pwr_mngr_CmdSetRegulatorConfig(uint8_t data[], uint8_t len);
void pwr_mngr_CmdGetRegulatorConfig(uint8_t data[], uint16_t *len);

#endif /* POWER_MANAGER_H_ */
