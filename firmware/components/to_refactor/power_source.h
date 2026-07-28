/*
 * power_source.h
 *
 *  Created on: 06.12.2016.
 *      Author: milan
 */

#ifndef POWER_SOURCE_H_
#define POWER_SOURCE_H_

#include <to_refactor/battery.h>
#include <to_refactor/time_count.h>   /* PWR_SOURCE_NEED_POLL() below expands MS_TIME_COUNT() */
#include "board.h"

#define PWR_5V_IN_DETECTION_STATUS_UNKNOWN		0
#define PWR_5V_IN_DETECTION_STATUS_NOT_PRESENT	1
#define PWR_5V_IN_DETECTION_STATUS_PRESENT	2
#define PWR_5V_TURN_ON_TIMEOUT	40

#define PWR_5V_BOOST_EN_STATUS()	 	(HAL_GPIO_ReadPin(PWR_5V_BOOST_EN_PORT, PWR_5V_BOOST_EN_PIN) == GPIO_PIN_SET)
#define PWR_SOURCE_NEED_POLL() 	(pwr5vInDetStatus == PWR_5V_IN_DETECTION_STATUS_PRESENT || MS_TIME_COUNT(pwr5vOnTimeout) <= PWR_5V_TURN_ON_TIMEOUT)
#define PWR_VSYS_OUTPUT_EN_STATUS()	 	(HAL_GPIO_ReadPin(PWR_VSYS_EN_PORT, PWR_VSYS_EN_PIN) == GPIO_PIN_RESET)

#define REGULATOR_5V_SWITCHING_STATUS_SUCCESS		0
#define REGULATOR_5V_SWITCHING_STATUS_NO_ENERGY		1

extern uint32_t pwr5vOnTimeout;

typedef enum PowerSourceStatus_T {
	PWR_SOURCE_NOT_PRESENT = 0,
	PWR_SOURCE_BAD,
	PWR_SOURCE_WEAK,
	PWR_SOURCE_NORMAL
} PowerSourceStatus_T;

typedef enum PowerRegulatorConfig_T {
	PWR_REGULATOR_MODE_PWR_DET = 0,
	PWR_REGULATOR_MODE_LDO,
	PWR_REGULATOR_MODE_DCDC,
	PWR_REGULATOR_MODE_END
} PowerRegulatorConfig_T;

extern PowerSourceStatus_T powerInStatus;
extern PowerSourceStatus_T power5vIoStatus;
extern uint8_t pwr5vInDetStatus;
extern uint8_t delayedPowerOff;
extern uint8_t forcedPowerOffFlag;
extern uint8_t forcedVSysOutputOffFlag;

void PowerSourceInit(void);
void PowerSourceTask(void);
void PowerSource5vIoDetectionTask(void);
void PowerSourceExitLowPower(void);
void PowerSourceEnterLowPower(void);
void PowerSourceSetBatProfile(const BatteryProfile_T* batProfile);
void PowerSourceSetVSysSwitchState(uint8_t state);
uint8_t PowerSourceGetVSysSwitchState();
int8_t Turn5vBoost(uint8_t onOff);
void Power5VSetModeLDO(void);
void SetPowerRegulatorConfigCmd(uint8_t data[], uint8_t len);
void GetPowerRegulatorConfigCmd(uint8_t data[], uint16_t *len);

#endif /* POWER_SOURCE_H_ */
