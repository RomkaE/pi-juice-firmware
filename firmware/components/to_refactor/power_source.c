/*
 * power_source.c
 *
 *  Created on: 06.12.2016.
 *      Author: milan
 */

#include "iosystem/analog.h"
#include <to_refactor/fuel_gauge_lc709203f.h>
#include <to_refactor/power_source.h>
#include <to_refactor/time_count.h>
#include "charger_bq2416x.h"
#include "stm32f0xx_hal.h"
#include "nv.h"

#define PWR_5V_IO_DET_ADC_THRESHOLD		2950
#define VBAT_TURNOFF_ADC_THRESHOLD		0 // mV unit
#define PWR_5V_DET_LDO_EN_STATUS()		(HAL_GPIO_ReadPin(PWR_5V_LDO_EN_PORT, PWR_5V_LDO_EN_PIN) == GPIO_PIN_SET)
#define PWR_SOURCE_PRESENT()			    (powerInStatus==PWR_SOURCE_NORMAL || powerInStatus==PWR_SOURCE_WEAK || power5vIoStatus==PWR_SOURCE_NORMAL || power5vIoStatus==PWR_SOURCE_WEAK)
#define PWR_SOURCE_5VREG_IS_PWR_BAD() ((analog_GetHwRev() == HARD_REV_2_3_AND_ABOVE) && (HAL_GPIO_ReadPin(PWR_5V_PG_PORT, PWR_5V_PG_PIN) == GPIO_PIN_RESET))


uint8_t forcedPowerOffFlag __attribute__((section("no_init")));
uint8_t forcedVSysOutputOffFlag __attribute__((section("no_init")));

extern uint8_t resetStatus;
extern uint16_t wakeupOnCharge;

uint8_t pwr5vInDetStatus = PWR_5V_IN_DETECTION_STATUS_UNKNOWN;
static uint16_t vbatPwrOffTresh;

/*
 * Same cutoff as vbatPwrOffTresh, pre-converted to raw ADC counts for the code paths that
 * compare an uncorrected GetSample() reading. VBAT_MV_TO_ADC lives in analog.h, next to the
 * divider constants it is built from.
 *
 * This used to live in analogWDGConfig.LowThreshold - the analog watchdog was configured
 * with it but its interrupt did nothing, so the threshold register doubled as storage.
 */
static uint16_t vbatPwrOffTreshAdc;

uint8_t pwr5VChgLoadMaximumReached = 0;
uint32_t pwr5vPresentCounter;

//static uint32_t delayedInitTimeCount;

static uint32_t pwr5vDetTimeCount;

uint32_t pwr5vOnTimeout __attribute__((section("no_init")));

static uint32_t forcedPowerOffCounter __attribute__((section("no_init")));

//volatile uint8_t vsysSwitchLimit __attribute__((section("no_init")));

PowerSourceStatus_T powerInStatus = PWR_SOURCE_NOT_PRESENT;
PowerSourceStatus_T power5vIoStatus = PWR_SOURCE_NOT_PRESENT;

PowerRegulatorConfig_T powerRegulatorConfig = PWR_REGULATOR_MODE_PWR_DET;

// power ldo enable
__STATIC_INLINE void PWR_5V_DET_LDO_ENABLE(uint8_t enabled) {
	if  (powerRegulatorConfig == PWR_REGULATOR_MODE_DCDC) {
		HAL_GPIO_WritePin(PWR_5V_LDO_EN_PORT, PWR_5V_LDO_EN_PIN, GPIO_PIN_RESET);
	} else if (powerRegulatorConfig == PWR_REGULATOR_MODE_LDO) {
		if (PWR_5V_BOOST_EN_STATUS())
			HAL_GPIO_WritePin(PWR_5V_LDO_EN_PORT, PWR_5V_LDO_EN_PIN, GPIO_PIN_SET);
		else
			HAL_GPIO_WritePin(PWR_5V_LDO_EN_PORT, PWR_5V_LDO_EN_PIN, GPIO_PIN_RESET);
	} else {
		HAL_GPIO_WritePin(PWR_5V_LDO_EN_PORT, PWR_5V_LDO_EN_PIN, enabled?GPIO_PIN_SET:GPIO_PIN_RESET);
	}
}

void Power5VSetModeLDO(void) {
	PWR_5V_DET_LDO_ENABLE(1);
}

__STATIC_INLINE int Retry5VTurnOn() {
	int n = 5;
	while(n--) {
		DelayUs(200);
		// Check battery voltage
		volatile int16_t batVolt = analog_GetVBatt();
		if ( (!PWR_SOURCE_PRESENT()) && batVolt < vbatPwrOffTresh) {
			HAL_GPIO_WritePin(PWR_5V_BOOST_EN_PORT, PWR_5V_BOOST_EN_PIN, GPIO_PIN_RESET);
			forcedPowerOffFlag = 1;
			return REGULATOR_5V_SWITCHING_STATUS_NO_ENERGY;
		}
	}

	HAL_GPIO_WritePin(PWR_5V_BOOST_EN_PORT, PWR_5V_BOOST_EN_PIN, GPIO_PIN_RESET);
	DelayUs(5);
	HAL_GPIO_WritePin(PWR_5V_BOOST_EN_PORT, PWR_5V_BOOST_EN_PIN, GPIO_PIN_SET);

	return REGULATOR_5V_SWITCHING_STATUS_SUCCESS;
}

int8_t Turn5vBoost(uint8_t onOff) {
	if (onOff) {

	  // Already ENABLED:
		if (PWR_5V_BOOST_EN_STATUS()) return 0;

		int status;
		if ( batteryVoltage > vbatPwrOffTresh || PWR_SOURCE_PRESENT()/*chargerStatus != CHG_NO_VALID_SOURCE*/) {
			PWR_5V_DET_LDO_ENABLE(0);
			DelayUs(5);
			HAL_GPIO_WritePin(PWR_5V_BOOST_EN_PORT, PWR_5V_BOOST_EN_PIN, GPIO_PIN_SET);

			// Retry turn on in case of large capacitive load
			status = Retry5VTurnOn();
			if (status == REGULATOR_5V_SWITCHING_STATUS_SUCCESS) {
				status = Retry5VTurnOn();
			}
			if (status == REGULATOR_5V_SWITCHING_STATUS_SUCCESS) {
				MS_TIME_COUNTER_INIT(pwr5vOnTimeout);
			}
		} else {
			status = REGULATOR_5V_SWITCHING_STATUS_NO_ENERGY;
		}
		return status;
	} else {
		PWR_5V_DET_LDO_ENABLE(0);
		HAL_GPIO_WritePin(PWR_5V_BOOST_EN_PORT, PWR_5V_BOOST_EN_PIN, GPIO_PIN_RESET);
		MS_TIME_COUNTER_INIT(pwr5vDetTimeCount);

		return REGULATOR_5V_SWITCHING_STATUS_SUCCESS;
	}
}

/*
__STATIC_INLINE void TurnVSysOutput(uint8_t onOff){
	if (onOff){
		HAL_GPIO_WritePin(PWR_VSYS_EN_PORT, PWR_VSYS_EN_PIN, GPIO_PIN_RESET);
	} else {
		HAL_GPIO_WritePin(PWR_VSYS_EN_PORT, PWR_VSYS_EN_PIN, GPIO_PIN_SET);
	}
}
*/

void PowerSourceInit(bool _reset)
{
	// initialize global variables after power-up
	if (_reset)
	{
		forcedPowerOffFlag = 0;
		forcedVSysOutputOffFlag = 0;
		forcedPowerOffCounter = 0;
		pwr5vOnTimeout = (uint32_t)0-1;
	}

	MS_TIME_COUNTER_INIT(pwr5vPresentCounter);
	MS_TIME_COUNTER_INIT(pwr5vDetTimeCount);

	uint8_t temp = 0;
	if (nv_read_U8(NV_ADDR_POWER_REGULATOR_CONFIG, &temp) == NV_OK
	    && temp < PWR_REGULATOR_MODE_END) {
		powerRegulatorConfig = temp;
	}

	vbatPwrOffTresh = currentBatProfile!=NULL ? (uint16_t)(currentBatProfile->cutoffVoltage)*20+VBAT_TURNOFF_ADC_THRESHOLD : (uint16_t)3000+VBAT_TURNOFF_ADC_THRESHOLD;
	vbatPwrOffTreshAdc = VBAT_MV_TO_ADC(vbatPwrOffTresh);

	DelayUs(100);
	volatile uint16_t batVolt = analog_GetVBatt();

	// maintain regulator state before reset
	// TODO - WTF! REMOVE! Pin could have not valid state!!!!
	/*
	if ( HAL_GPIO_ReadPin(PWR_5V_BOOST_EN_PORT, PWR_5V_BOOST_EN_PIN) == GPIO_PIN_SET ) {
		// if there is mcu power-on, but reg was on, it can be power lost fault condition, check sources

		if (executionState == EXECUTION_STATE_POR_RESET && batVolt < vbatPwrOffTresh && CHARGER_INSTAT()) {
			forcedPowerOffFlag = 1;
			Turn5vBoost(0);
			wakeupOnCharge = 5; // schedule wake up when there is enough energy
		} else {
			HAL_GPIO_WritePin(PWR_5V_BOOST_EN_PORT, PWR_5V_BOOST_EN_PIN, GPIO_PIN_SET);
			MS_TIME_COUNTER_INIT(pwr5vOnTimeout);
		}
	} else {
		PWR_5V_DET_LDO_ENABLE(0);
		Turn5vBoost(0);
	}
	*/

	/*
	if ( executionState == EXECUTION_STATE_COLD_START || executionState == EXECUTION_STATE_POR_RESET ) {
		// after power-up state is 500mA limit
		HAL_GPIO_WritePin(PWR_VSYS_ILIM_PORT, PWR_VSYS_ILIM_PIN, GPIO_PIN_SET);
		vsysSwitchLimit = 5;
	} else {
		if (vsysSwitchLimit == 21 )
			HAL_GPIO_WritePin(PWR_VSYS_ILIM_PORT, PWR_VSYS_ILIM_PIN,  GPIO_PIN_RESET); // 2.1A limit value, is valid only after reset
		else
			HAL_GPIO_WritePin(PWR_VSYS_ILIM_PORT, PWR_VSYS_ILIM_PIN, GPIO_PIN_SET);
	}
	*/

	/*
	if (HAL_GPIO_ReadPin(PWR_VSYS_EN_PORT, PWR_VSYS_EN_PIN) == GPIO_PIN_RESET && executionState != EXECUTION_STATE_COLD_START ) { // after power-up vsys switch should be disabled
		if ( executionState == EXECUTION_STATE_POR_RESET && batVolt < vbatPwrOffTresh && CHARGER_INSTAT() ) {
			TurnVSysOutput(0);
			forcedVSysOutputOffFlag = 1;
		} else {
			TurnVSysOutput(1);
		}
	} else {
		TurnVSysOutput(0);
	}
	*/

	/*
	GPIO_InitTypeDef GPIO_InitStruct;
	GPIO_InitStruct.Pin = PWR_5V_BOOST_EN_PIN|PWR_VSYS_EN_PIN;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(PWR_5V_BOOST_EN_PORT, &GPIO_InitStruct);
	*/

}

/*__STATIC_INLINE*/ void CheckMinimumPower(int16_t volt5) {
	if ( PWR_5V_BOOST_EN_STATUS() ) {

		if ((volt5 < 2000 && analog_GetAvdd() > 2500) || PWR_SOURCE_5VREG_IS_PWR_BAD())
		{
			//5V DCDC is in fault overcurrent state, turn it off to prevent draining battery
			forcedPowerOffFlag = 1;
			Turn5vBoost(0);
			return;
		}

		uint16_t samt = analog_GetVBatt();
		// if no sources connected, turn off 5V regulator and system switch when battery voltage drops below minimum
		if ( samt < vbatPwrOffTreshAdc && pwr5vInDetStatus != PWR_5V_IN_DETECTION_STATUS_PRESENT && CHARGER_INSTAT()) {
			/*if ( PWR_VSYS_OUTPUT_EN_STATUS() ) {
				TurnVSysOutput(0);
				forcedVSysOutputOffFlag = 1;
				MS_TIME_COUNTER_INIT(forcedPowerOffCounter); // leave 2 ms for switch to react
			}
			else*/
			  if ( MS_TIME_COUNT(forcedPowerOffCounter) >= 2) {
				PWR_5V_DET_LDO_ENABLE(0);
				forcedPowerOffFlag = 1;
				Turn5vBoost(0);
				wakeupOnCharge = 5; // schedule wake up when power is applied
			}
		}
	} else {
	  /*
		int16_t batVolt = analog_GetVBatt();
		if ( (!PWR_SOURCE_PRESENT() ) && batVolt < vbatPwrOffTresh && PWR_VSYS_OUTPUT_EN_STATUS()) {
			TurnVSysOutput(0);
			forcedVSysOutputOffFlag = 1;
		}*/
	}
}

void PowerSource5vIoDetectionTask(void)
{
  if ( MS_TIME_COUNT(pwr5vOnTimeout) < PWR_5V_TURN_ON_TIMEOUT)
  {
    int16_t batVolt;
    if ((!PWR_SOURCE_PRESENT()) && MS_TIME_COUNT(pwr5vOnTimeout) > 0)
    {
      batVolt = analog_GetVBatt();
      if (batVolt < vbatPwrOffTresh)
      {
        forcedPowerOffFlag = 1;
        Turn5vBoost(0);
      }
    }

    // Wait for 5V to become stable after turn on timeout
    return;
  }


	int16_t volt5 = analog_Get5vPi();

	CheckMinimumPower(volt5);

	if ( PWR_5V_BOOST_EN_STATUS() ) {

		if (PWR_5V_DET_LDO_EN_STATUS()) {
			uint16_t samp = analog_GetRawPWR();
			if ( samp < PWR_5V_IO_DET_ADC_THRESHOLD) {
				if (pwr5vInDetStatus != PWR_5V_IN_DETECTION_STATUS_NOT_PRESENT) {
					MS_TIME_COUNTER_INIT(pwr5vPresentCounter);
					ChargerUsbInCurrentLimitStepDown();
				}
				pwr5vInDetStatus = PWR_5V_IN_DETECTION_STATUS_NOT_PRESENT;
				if (pwr5VChgLoadMaximumReached > 1) pwr5VChgLoadMaximumReached --;
				ChargerSetUSBLockout(CHG_USB_IN_LOCK);
				PWR_5V_DET_LDO_ENABLE(0);
			}
		} else if (pwr5vInDetStatus != PWR_5V_IN_DETECTION_STATUS_NOT_PRESENT) {
			pwr5vInDetStatus = PWR_5V_IN_DETECTION_STATUS_UNKNOWN;
			if (pwr5VChgLoadMaximumReached > 1) pwr5VChgLoadMaximumReached --;
			ChargerSetUSBLockout(CHG_USB_IN_LOCK);
			ChargerUsbInCurrentLimitStepDown();
			MS_TIME_COUNTER_INIT(pwr5vPresentCounter);
		}

		if ( pwr5vInDetStatus != PWR_5V_IN_DETECTION_STATUS_PRESENT && MS_TIME_COUNT(pwr5vDetTimeCount) > 95 ) {
			MS_TIME_COUNTER_INIT(pwr5vDetTimeCount);

			if (!PWR_5V_DET_LDO_EN_STATUS()) {
				PWR_5V_DET_LDO_ENABLE(1);
				//Delay(100);
			}

			// find out if PMOS goes to cutoff or active state
			volatile uint8_t i = 200, fetCutoffCount = 0, fetActiveCount = 0;
			//volatile uint16_t activeSamples[6];
			volatile uint32_t sam = 0;
			while ( i-- && fetActiveCount < 3) {
				DelayUs(120);//HAL_Delay(2);//
			    sam = analog_GetRawPWR();
			    if (sam >= PWR_5V_IO_DET_ADC_THRESHOLD) fetCutoffCount ++;
			    else fetCutoffCount = 0;
			    if (sam < PWR_5V_IO_DET_ADC_THRESHOLD)	{
			    	fetActiveCount ++;
			    }
			    else fetActiveCount = 0;
			    CheckMinimumPower(volt5);
			    //activeSamples[i] = sam;
			}
			if ( fetCutoffCount >= 200 ) {//if ( sam >= PWR_5V_IO_DET_ADC_THRESHOLD ) {
				// turn on usb in if pmos is cutoff
				pwr5vInDetStatus = PWR_5V_IN_DETECTION_STATUS_PRESENT;
				ChargerSetUSBLockout(CHG_USB_IN_UNLOCK);
				//pwr5vIoLoadCurrent = 0;
				MS_TIME_COUNTER_INIT(pwr5vPresentCounter);
			} else {
				if (fetActiveCount >= 3) {
					pwr5vInDetStatus = PWR_5V_IN_DETECTION_STATUS_NOT_PRESENT;
				}
				PWR_5V_DET_LDO_ENABLE(0);
			}
		} /*else {
			pwr5vIoLoadCurrent = 0;
		}*/
	} else {
		if (volt5 < 4800) {
			if (pwr5vInDetStatus != PWR_5V_IN_DETECTION_STATUS_NOT_PRESENT) {
				MS_TIME_COUNTER_INIT(pwr5vPresentCounter);
				ChargerUsbInCurrentLimitStepDown();
			}
			pwr5vInDetStatus = PWR_5V_IN_DETECTION_STATUS_NOT_PRESENT;
			ChargerSetUSBLockout(CHG_USB_IN_LOCK);
			if (pwr5VChgLoadMaximumReached > 1) pwr5VChgLoadMaximumReached --;
		} else if (pwr5vInDetStatus != PWR_5V_IN_DETECTION_STATUS_PRESENT && MS_TIME_COUNT(pwr5vDetTimeCount) > 500) {
		  HAL_GPIO_WritePin(PWR_5V_BOOST_EN_PORT, PWR_5V_BOOST_EN_PIN, GPIO_PIN_SET);
			PWR_5V_DET_LDO_ENABLE(1);
			pwr5vInDetStatus = PWR_5V_IN_DETECTION_STATUS_PRESENT;
			ChargerSetUSBLockout(CHG_USB_IN_UNLOCK); // turn on charger in
			MS_TIME_COUNTER_INIT(pwr5vPresentCounter);
			//wakeupOnCharge = 0xFFFF;
		}
	}

	if ( MS_TIME_COUNT(pwr5vPresentCounter) > 800 ) {
		MS_TIME_COUNTER_INIT(pwr5vPresentCounter);
		if (pwr5vInDetStatus == PWR_5V_IN_DETECTION_STATUS_PRESENT ) {
			if (pwr5VChgLoadMaximumReached > 1) {
				ChargerUsbInCurrentLimitStepUp();
				pwr5VChgLoadMaximumReached = 2;
			} else if (pwr5VChgLoadMaximumReached == 0) {
				pwr5VChgLoadMaximumReached = 3;
			}
		} else if (pwr5vInDetStatus == PWR_5V_IN_DETECTION_STATUS_NOT_PRESENT) {
			// this means input is disconnected, and flag can be cleared
			pwr5VChgLoadMaximumReached = 0;
			ChargerUsbInCurrentLimitSetMin();
		}
	}
}

void PowerSourceTask(void) {

	uint8_t pwrStat = CHARGER_INSTAT();
	if (pwrStat == 0x03) {
		powerInStatus = PWR_SOURCE_NOT_PRESENT;
	} else if (pwrStat == 0x01 || pwrStat == 0x02) {
		powerInStatus = PWR_SOURCE_BAD;
	} else if (CHARGER_IS_DPM_MODE_ACTIVE()) {
		powerInStatus = PWR_SOURCE_WEAK;
	} else {
		powerInStatus = PWR_SOURCE_NORMAL;
	}

	if (usbInLockoutStatus == CHG_USB_IN_UNLOCK) {
		pwrStat = CHARGER_USBSTAT();
		if (/*!CHARGER_IS_USBIN_LOCKED() ||*/ pwrStat == 0x03) {
			power5vIoStatus = PWR_SOURCE_NOT_PRESENT;
		} else if ( pwrStat == 0x01 || pwrStat == 0x02) {
			power5vIoStatus = PWR_SOURCE_BAD;
		} else if (/*CHARGER_IS_DPM_MODE_ACTIVE() &&*/ pwr5VChgLoadMaximumReached == 1 ) {
			power5vIoStatus = PWR_SOURCE_WEAK;
		} else {
			power5vIoStatus = PWR_SOURCE_NORMAL;
		}
	} else if (pwr5vInDetStatus != PWR_5V_IN_DETECTION_STATUS_PRESENT) {
		power5vIoStatus = PWR_SOURCE_NOT_PRESENT;
	} else {
		power5vIoStatus = PWR_SOURCE_NORMAL;
	}
}

void PowerSourceSetBatProfile(const BatteryProfile_T* batProfile) {
	vbatPwrOffTresh =currentBatProfile!=NULL ? (uint16_t)(currentBatProfile->cutoffVoltage)*20+VBAT_TURNOFF_ADC_THRESHOLD : (uint16_t)3000+VBAT_TURNOFF_ADC_THRESHOLD;
	vbatPwrOffTreshAdc = VBAT_MV_TO_ADC(vbatPwrOffTresh);
}

void PowerSourceSetVSysSwitchState(uint8_t state) {
  // TODO
  /*
	if (state == 5) {
		HAL_GPIO_WritePin(PWR_VSYS_ILIM_PORT, PWR_VSYS_ILIM_PIN, GPIO_PIN_SET);
    if (analog_GetRawBatt() > vbatPwrOffTreshAdc || PWR_SOURCE_PRESENT())
			TurnVSysOutput(1);
		vsysSwitchLimit = state;
	} else if (state == 21) {
		HAL_GPIO_WritePin(PWR_VSYS_ILIM_PORT, PWR_VSYS_ILIM_PIN, GPIO_PIN_RESET);
		if (analog_GetRawBatt() > vbatPwrOffTreshAdc || PWR_SOURCE_PRESENT())
			TurnVSysOutput(1);
		vsysSwitchLimit = state;
	} else {
		TurnVSysOutput(0);
	}
	forcedVSysOutputOffFlag = 0;
	*/
}

uint8_t PowerSourceGetVSysSwitchState() {
  // TODO
//	if (PWR_VSYS_OUTPUT_EN_STATUS()) {
//		return vsysSwitchLimit;
//		/*if (HAL_GPIO_ReadPin(PWR_VSYS_ILIM_PORT, PWR_VSYS_ILIM_PIN) == GPIO_PIN_SET) {
//			return 5;
//		} else {
//			return 21;
//		}*/
//	} else {
//		return 0;
//	}
  return 0;
}

void SetPowerRegulatorConfigCmd(uint8_t data[], uint8_t len) {
	if (data[0] >= PWR_REGULATOR_MODE_END) return;
	nv_write_U8(NV_ADDR_POWER_REGULATOR_CONFIG, data[0]);

	uint8_t temp = 0;
	if (nv_read_U8(NV_ADDR_POWER_REGULATOR_CONFIG, &temp) == NV_OK
	    && temp < PWR_REGULATOR_MODE_END) {
		powerRegulatorConfig = temp;
	}
}

void GetPowerRegulatorConfigCmd(uint8_t data[], uint16_t *len) {
	data[0] = powerRegulatorConfig;
	*len = 1;
}
