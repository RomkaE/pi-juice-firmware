/*
 * app.h
 *
 *  Created on: Jul 20, 2026
 *      Author: Roman Egoshin
 */

#ifndef SRC_APP_H_
#define SRC_APP_H_

#include <stdint.h>
#include <stdbool.h>

#include "power/battery.h"

/*
 * System wide event queue.
 *
 * Subsystem tasks detect, the APP task decides. A component task posts what happened and goes
 * straight back to its own job; the APP task drains the queue and runs the functional logic in
 * one place and one context. That context matters: the handlers drive the 5V boost, the RUN
 * pin and the charger, and those sequences must not interleave with each other.
 *
 * New producers add a type here and a case to the dispatcher in app.c.
 */
typedef enum
{
  APP_EVT_BUTTON = 1,                          // a button event that has a configured function
  APP_EVT_BUTTON_RESET_CONFIG,                 // both power buttons held down: reset the configuration
  APP_EVT_BATTERY_SET_PROFILE,                 // host register 0x82 - battery_CmdSetProfile()
  APP_EVT_BATTERY_WRITE_CUSTOM_PROFILE,        // host register 0x86 - battery_CmdWriteCustomProfile()
  APP_EVT_BATTERY_WRITE_CUSTOM_EXTENDED_PROFILE, // host register 0x87 - battery_CmdWriteCustomExtendedProfile()
  APP_EVT_CHARGER_INPUT_PRESENCE,              // charger.c detected an edge on "is an input source present"
  APP_EVT_BATTERY_PRESENCE,                    // charger.c detected an edge on the bq2416x BATSTAT
  APP_EVT_FUEL_GAUGE_SET_CONFIG,               // host register 0x93 - app_FuelGaugeCmdSetConfig()
  APP_EVT_CHARGER_SET_INPUTS_CONFIG,           // app_ChargerCmdWriteInputsConfig()
  APP_EVT_CHARGER_SET_CHARGING_CONFIG,         // app_ChargerCmdWriteChargingConfig()
} AppEventType_t;

typedef struct
{
  uint8_t func;    // ButtonFunction_T the button is configured with
  uint8_t index;   // button index in the host facing numbering
  uint8_t event;   // ButtonEvent_T that fired
} AppEventButton_t;

typedef struct
{
  uint8_t id;
  uint8_t seq;   // see battery.c's "write pending" mirror
} AppEventBatterySetProfile_t;

typedef struct
{
  BatteryProfile_T profile; // only the basic fields are meaningful
  uint8_t seq;              // see battery.c's "write pending" mirror
} AppEventBatteryCustomProfile_t;

typedef struct
{
  BatteryProfile_T profile; // only the extended (chemistry/ocv/r) fields are meaningful
} AppEventBatteryCustomExtProfile_t;

typedef struct
{
  bool present;
} AppEventChargerInput_t;

/* Raw BATSTAT edge from the charger - battery.c turns it into "is a pack present". */
typedef struct
{
  bool present;
} AppEventBatteryPresence_t;

typedef struct
{
  uint8_t config;  // register 0x93 layout, see fuel_gauge_lc709203f.h
  uint8_t seq;     // see the "write pending" mirror in app.c
} AppEventFuelGaugeConfig_t;

typedef struct
{
  uint8_t config;  // bit 7 means "persist", see app.c
  uint8_t seq;
} AppEventChargerConfig_t;

typedef struct
{
  uint8_t type;    // AppEventType_T
  union
  {
    AppEventButton_t button;
    AppEventBatterySetProfile_t batterySetProfile;
    AppEventBatteryCustomProfile_t batteryCustomProfile;
    AppEventBatteryCustomExtProfile_t batteryCustomExtProfile;
    AppEventChargerInput_t chargerInput;
    AppEventBatteryPresence_t batteryPresence;
    AppEventFuelGaugeConfig_t fuelGaugeConfig;
    AppEventChargerConfig_t chargerConfig;
  } data;
} AppEvent_t;

void app_Init(void);

/*
 * Hands an event to the APP task. Safe from an interrupt and from a task, never blocks.
 * Returns false when the queue is full, which the caller is expected to count as a fault -
 * the event is lost, not retried.
 */
bool app_PostEvent(const AppEvent_t *_pEvent);

/*
 * Host register 0x93 - fuel gauge configuration.
 *
 * The fuel gauge module itself is a plain driver: it applies a configuration byte and never
 * persists one. Storage policy lives here instead, because the NV write has to happen in a task
 * context - CmdSetConfig() runs under the I2C1 interrupt - and the APP task is the one that owns
 * that kind of decision. So: the command is parsed and posted here, the APP task writes it to
 * NV, reads it back and only then hands the stored value to fuel_gauge_SetConfig().
 *
 * Returns non-zero when the byte is rejected outright, so the host sees the write fail.
 */
int8_t app_FuelGaugeCmdSetConfig(uint8_t *data, uint16_t len);
void app_FuelGaugeReadConfig(uint8_t data[], uint16_t *len);

/*
 * Host registers for the charger's two configuration bytes, same split as above: the charger
 * applies, the APP task persists. Bit 7 of the value means "store it"; a read answers with the
 * request until it has been applied, then with the NV-verified byte.
 */
void app_ChargerCmdWriteInputsConfig(uint8_t config);
uint8_t app_ChargerReadInputsConfig(void);
void app_ChargerCmdWriteChargingConfig(uint8_t config);
uint8_t app_ChargerReadChargingConfig(void);

#endif /* SRC_APP_H_ */
