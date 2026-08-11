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

#include "iosystem/button.h"
#include "power/battery.h"
#include "power/charger_bq2416x.h"
#include "power/power_manager.h"
#include "power/fuel_gauge_lc709203f.h"

typedef enum
{
  APP_EVT_SM_ENTRY = 0,
  APP_EVT_BUTTON,                                 // a button event that has a configured function
  APP_EVT_BUTTON_RESET_CONFIG,                    // both power buttons held down: reset the configuration
  APP_EVT_CHRGR_SNAPSHOT,

  APP_EVT_CMD_SCHEDULE_POWER_OFF,                 // host register 0x62 - app_OnCmdSchedulePowerOff()
//  APP_EVT_POWER_POLICY,                           // see the power policy timer in app.c
//  APP_EVT_RAIL_ON,                                // the power cycle down time has elapsed

  APP_EVT_TIMER_POWER_UP,
  APP_EVT_TIMER_POWER_OFF,

  APP_EVT_CMD_BATT_SET_PROFILE,                    // host register 0x82 - battery_CmdSetProfile()
  APP_EVT_CMD_BATT_WRITE_CUSTOM_PROFILE,           // host register 0x86 - battery_CmdWriteCustomProfile()
  APP_EVT_CMD_BATT_WRITE_CUSTOM_EXTENDED_PROFILE,  // host register 0x87 - battery_CmdWriteCustomExtendedProfile()

//  APP_EVT_CHARGER_INPUT_PRESENCE,                 // charger.c detected an edge on "is an input source present"
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

/* Which protection cut the rail - see PowerTrip_t. */
typedef struct
{
  uint8_t trip;
} AppEventPowerTrip_t;

/* Register 0x62 as the host wrote it: seconds until the cut, or 0xFF to call it off. */
typedef struct
{
  uint8_t delay_sec;
} AppEventPowerOff_t;

typedef struct
{
  AppEventType_t type;
  union
  {
    AppEventButton_t button;
    AppEventBatterySetProfile_t batterySetProfile;
    AppEventBatteryCustomProfile_t batteryCustomProfile;
    AppEventBatteryCustomExtProfile_t batteryCustomExtProfile;

    ChargerSnapshot_t chrgr_snapshot;
    AppEventChargerInput_t chargerInput;
    AppEventBatteryPresence_t batteryPresence;
    AppEventFuelGaugeConfig_t fuelGaugeConfig;
    AppEventChargerConfig_t chargerConfig;
    AppEventChargerValue_t chargerValue;
    AppEventPowerTrip_t powerTrip;
    AppEventPowerOff_t powerOff;
  };
} AppEvent_t;

void app_Init(void);

void app_PostEvent(const AppEvent_t *_p_event);

void app_SetRtcWakeupEvent(void);
void app_SetIoWakeupEvent(void);

void app_OnCmdSetFuelGaugeConfig(uint8_t *_data, uint16_t _len);
void app_OnCmdGetFuelGaugeConfig(uint8_t _data[], uint16_t *_p_len);
void app_OnCmdSchedulePowerOff(uint8_t _delay_code);
uint8_t app_OnCmdGetPowerOffCounter(void);
void app_OnCmdSetHostWDTConfig(uint8_t _data[], uint16_t _len);
void app_OnCmdGetHostWDTConfig(uint8_t _data[], uint16_t *_p_len);
void app_OnCmdSetWakeupOnCharge(uint8_t _data[], uint16_t _len);
void app_OnCmdGetWakeupOnCharge(uint8_t _data[], uint16_t *_p_len);
void app_OnCmdSetChargerInputsConfig(uint8_t _in_config);
uint8_t app_OnCmdGetChargerInputsConfig(void);
void app_OnCmdSetChargingConfig(uint8_t _config);
uint8_t app_OnCmdGetChargingConfig(void);

#endif /* SRC_APP_H_ */
