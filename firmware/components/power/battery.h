/*
 * battery.h
 *
 *  Created on: 12.12.2016.
 *      Author: milan
 */

#ifndef BATTERY_H_
#define BATTERY_H_

#include <stdint.h>
#include <stdbool.h>

// bit 0-3: 0 - 0xE profile id, 0xF - custom profile; bit 4-5: source, 0-host or stored,1-dip switch, 2-resistor; bit 6: validity, 0-valid,1-invalid
// special value 0xFF: default profile id, defined by dip switch or resistor if present
#define BATTERY_DEFAULT_PROFILE_ID      0xFF
#define BATTERY_NONEXIST_PROFILE_ID     ((0x0<<4)|(0x1<<6)) //0xE0 // ored with stored profile id
#define BATTERY_CUSTOM_PROFILE_ID       ((uint8_t)0x0F)
#define BATTERY_CONFIG_RES_PROFILE_ID   ((0x2<<4)|(0x0<<6)) //0x40 // profile id is selected with resistor, ored with id
#define BATTERY_CONFIG_SW_PROFILE_ID    ((0x1<<4)|(0x0<<6)) //0x20 // profile id is selected with dip switch, ored with id
#define BATTERY_INVALID_PROFILE_ID      ((0x0<<4)|(0x1<<6)) //0xEF // stored profile id in eeprom is invalid

#define BATTERY_CONFIG_PROFILE_STATUS         ((0x2<<4)|(0x0<<6)|0x0F)//0x60 // current profile data are configured with resistor
#define BATTERY_CONFIG_INVALID_PROFILE_STATUS ((0x2<<4)|(0x1<<6)|0x0F)//0x6F //dip switch/resistor configuration is invalid
#define BATTERY_INVALID_CUSTOM_PROFILE_STATUS ((0x0<<4)|(0x1<<6)|0x0F)//0x8F

#define BATTERY_PROFILE_WRITE_BUSY_STATUS 0xF0 // profile id/data write not completed

/* Charge-status LED refresh period - the APP task calls battery_UpdateChargeLed() on this cadence. */
#define BATTERY_CHARGE_LED_PERIOD_MS  900

typedef enum BatteryChemistry_T {
	BAT_CHEMISTRY_LIPO = 0,
	BAT_CHEMISTRY_LIFEPO4
} BatteryChemistry_T;

typedef struct
{
	BatteryChemistry_T chemistry;
	uint32_t capacity; // mAh
	uint8_t chargeCurrent; // bq24160 register 5 bit code, 75mA resolution, 550mA offset
	uint8_t terminationCurr; // bq24160 register 3 bit code, 50mA resolution, 50mA offset
	uint8_t regulationVoltage; // bq24160 register 6 bit code, 20mV resolution, 3.5V offset
	uint8_t cutoffVoltage; // discharge cut-off voltage, 20mV resolution
	uint16_t ocv10, ocv50, ocv90;
	uint16_t r10, r50, r90;
	int8_t tCold; // cold temperature point
	int8_t tCool;
	int8_t tWarm;
	int8_t tHot;
	uint16_t ntcB; // B constant
	uint16_t ntcResistance; // 10ohm unit, 0xFFFF undefined
} BatteryProfile_T;

/*
 * Thermal verdict, aggregated here from the profile's four thresholds and whatever temperature
 * source is alive. The charger acts on this instead of reading a raw temperature and the profile
 * itself - see charger_SetThermalState().
 *
 * The order matters and callers may rely on it: HOT implies "above tWarm" and COLD implies
 * "below tCool", because that is how the thresholds nest. UNKNOWN means "no usable reading" and
 * must be treated exactly like NORMAL - the same thing the charger did when the temperature
 * sensor was configured as not used.
 */
typedef enum BatteryThermalState_T {
	BAT_TEMP_UNKNOWN = 0,   // no profile, or the temperature sensor is not in use
	BAT_TEMP_COLD,          // <= tCold
	BAT_TEMP_COOL,          // <  tCool
	BAT_TEMP_NORMAL,
	BAT_TEMP_WARM,          // >  tWarm
	BAT_TEMP_HOT            // >= tHot
} BatteryThermalState_T;

typedef enum BatteryStatus_T {
	BAT_STATUS_NORMAL = 0,
	BAT_STATUS_CHARGING_FROM_IN,
	BAT_STATUS_CHARGING_FROM_5V_IO,
	BAT_STATUS_NOT_PRESENT
} BatteryStatus_T;

/* Below this the pack is treated as absent whatever the charger's own detector says. */
#define BATTERY_PRESENT_MV  2550

/*
 * Battery has no hardware of its own - unlike led/button/analog/charger/fuel_gauge, it never
 * talks to a peripheral - so unlike those, it is not a FreeRTOS task. The three battery_Cmd*()
 * entry points below are still interrupt-safe (they only parse the request and hand it to the
 * APP task), but battery_Init() and the battery_Apply*() functions do real NV access and must
 * only ever run in a task context - battery_Init() from APP's own startup, battery_Apply*() only
 * from app_ProcessEvent().
 *
 * Loads the current profile from NV, synchronously. Must be called after nv_Init(), and before
 * fuel_gauge_Init()/charger_Init() - both read the initial profile via battery_GetProfile() on
 * their own startup.
 */
void battery_Init(void);

/*
 * Copies out the currently selected profile. Returns false (out untouched) when no profile is
 * selected - callers must treat that the same way a NULL currentBatProfile used to be treated.
 * out may be NULL to just query whether a profile is selected.
 */
bool battery_GetProfile(BatteryProfile_T *out);

BatteryStatus_T battery_GetStatus(void);

/*
 * Battery presence, aggregated here from two independent sources that know nothing about each
 * other: the charger's own BATSTAT detector and the pack voltage measured by the ADC. This is
 * what battery.c is for - the modules produce readings, the aggregation and the decision live in
 * one place, in APP task context.
 *
 * battery_UpdatePresence() re-evaluates and returns true when the answer changed, so the caller
 * knows it has something to announce. APP calls it on a charger BATSTAT edge and on its own
 * periodic tick, so a slow voltage collapse is noticed as well as a pack being pulled.
 */
bool battery_UpdatePresence(void);
bool battery_IsPresent(void);

/*
 * Same shape for the thermal verdict: the profile's thresholds live here, the temperature comes
 * from whichever source is alive, and the result is what the charger is told. Update() returns
 * true when the verdict changed, so APP knows it has something to announce.
 *
 * No hysteresis, exactly as before this was factored out: a temperature sitting on a threshold
 * can flap at the APP tick rate. Known, deliberately unchanged.
 */
bool battery_UpdateThermalState(void);
BatteryThermalState_T battery_GetThermalState(void);

/*
 * Host register 0x82 - selects one of the built-in profiles, BATTERY_CUSTOM_PROFILE_ID or
 * BATTERY_DEFAULT_PROFILE_ID. Callable from the I2C1 interrupt: only parses the request and
 * posts it to the APP task (app_PostEvent()), which is where the NV write actually happens -
 * same reasoning as led.c keeping flash access out of the interrupt.
 */
void battery_CmdSetProfile(uint8_t id);

/* Host registers 0x86/0x87 - custom profile data (basic / extended fields). Same interrupt
 * safety as battery_CmdSetProfile() above. */
void battery_CmdWriteCustomProfile(uint8_t *data, uint16_t len);
void battery_CmdWriteCustomExtendedProfile(uint8_t *data, uint16_t len);

void battery_ReadCurrentProfile(uint8_t *data, uint16_t *len);
void battery_ReadCurrentExtendedProfile(uint8_t *data, uint16_t *len);
void battery_ReadProfileStatus(uint8_t *data, uint16_t *len);

/*
 * Applies a profile command queued via the three battery_Cmd*() functions above - called only
 * from app_ProcessEvent() in APP task context, never directly. This is where the NV write and
 * the currently-selected-profile publish actually happen.
 */
void battery_ApplySetProfile(uint8_t id, uint8_t seq);
void battery_ApplyWriteCustomProfile(const BatteryProfile_T *req, uint8_t seq);
void battery_ApplyWriteCustomExtendedProfile(const BatteryProfile_T *req);

/* Charge-status LED colour, driven by RSOC and charger state - called every ~900 ms from the
 * APP task's main loop. */
void battery_UpdateChargeLed(void);

uint32_t battery_GetErrMask(bool _clear);

#endif /* BATTERY_H_ */
