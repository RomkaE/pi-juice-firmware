/*
 * power_management.h
 *
 *  Created on: 04.04.2017.
 *      Author: milan
 */

#ifndef POWER_MANAGEMENT_H_
#define POWER_MANAGEMENT_H_

#include <stdint.h>
#include <stdbool.h>

typedef enum RunPinInstallationStatus_T {
	RUN_PIN_NOT_INSTALLED = 0,
	RUN_PIN_INSTALLED,
} RunPinInstallationStatus_T;

extern RunPinInstallationStatus_T runPinInstallationStatus;
extern uint8_t watchdogExpiredFlag;
extern uint8_t rtcWakeupEventFlag;
extern uint8_t ioWakeupEvent;

void PowerManagementInit(bool _reset);
void PowerManagementTask(void);

/*
 * Host power requests, called only from the APP task - that is where button events land, see
 * buttonEventCbs[] in app.c.
 *
 * Whether a request is appropriate right now is decided here, not by the caller: it depends on
 * whether the 5V boost is on, whether an external 5V source is present and how long the host has
 * been quiet - all of it this module's own knowledge.
 *
 * Each answers whether it actually did anything.
 */
bool power_HostTurnOn(void);
bool power_HostTurnOff(void);
bool power_HostReset(void);
void RunPinInstallationStatusSetConfigCmd(uint8_t data[], uint8_t len);
void RunPinInstallationStatusGetConfigCmd(uint8_t data[], uint16_t *len);
void PowerMngmtSchedulePowerOff(uint8_t dalayCode);
uint8_t PowerMngmtGetPowerOffCounter(void);
void PowerMngmtConfigureWatchdogCmd(uint8_t data[], uint16_t len);
void PowerMngmtGetWatchdogConfigurationCmd(uint8_t data[], uint16_t *len);
void PowerMngmtHostPollEvent(void);
//int8_t WakeUpHost(void);

void PowerMngmtSetWakeupOnChargeCmd(uint8_t data[], uint16_t len);
void PowerMngmtGetWakeupOnChargeCmd(uint8_t data[], uint16_t *len);

#endif /* POWER_MANAGEMENT_H_ */
