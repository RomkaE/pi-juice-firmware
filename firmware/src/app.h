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
#include "iosystem/button.h"

typedef enum
{
  APP_EVT_SM_ENTRY = 0,
  APP_EVT_BUTTON,                                 // a button event that has a configured function
  APP_EVT_BUTTON_RESET_CONFIG,                    // both power buttons held down: reset the configuration
  APP_EVT_BATTERY_SET_PROFILE,                    // host register 0x82 - battery_CmdSetProfile()
  APP_EVT_BATTERY_WRITE_CUSTOM_PROFILE,           // host register 0x86 - battery_CmdWriteCustomProfile()
  APP_EVT_BATTERY_WRITE_CUSTOM_EXTENDED_PROFILE,  // host register 0x87 - battery_CmdWriteCustomExtendedProfile()
  APP_EVT_CHARGER_INPUT_PRESENCE,                 // charger.c detected an edge on "is an input source present"
  APP_EVT_CHRGR_BATT_PRESENCE,                    // charger.c detected an edge on the bq2416x BATSTAT
  APP_EVT_CMD_FUEL_GAUGE_SET_CONFIG,              // host register 0x93 - app_FuelGaugeCmdSetConfig()
  APP_EVT_CHARGER_SET_INPUTS_CONFIG,              // app_ChargerCmdWriteInputsConfig()
  APP_EVT_CHARGER_SET_CHARGING_CONFIG,            // app_ChargerCmdWriteChargingConfig()
  APP_EVT_POWER_PROTECTION,                       // undervoltage or 5V fault, from the ANALOG task
} AppEventType_t;

typedef struct
{
//  uint8_t index;  // button index in the host facing numbering
  ButtonFunction_T func;
//  ButtonEvent_T event;
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

/* Payload of the six APP_EVT_CHARGER_* value events - the new value, nothing else. */
typedef struct
{
  uint8_t value;
} AppEventChargerValue_t;

typedef struct
{
  AppEventType_t type;
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
    AppEventChargerValue_t chargerValue;
//    PowerTrip_t powerTrip;
  };
} AppEvent_t;

void app_Init(void);

void app_PostEvent(const AppEvent_t *_p_event);

int8_t app_OnCmdSetFuelGaugeConfig(uint8_t *data, uint16_t len);

void app_OnCmdGetFuelGaugeConfig(uint8_t data[], uint16_t *len);

void app_ChargerCmdWriteInputsConfig(uint8_t config);
uint8_t app_ChargerReadInputsConfig(void);
void app_ChargerCmdWriteChargingConfig(uint8_t config);
uint8_t app_ChargerReadChargingConfig(void);

#endif /* SRC_APP_H_ */
