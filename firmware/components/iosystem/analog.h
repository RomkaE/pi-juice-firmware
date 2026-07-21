/*
 * analog.h
 *
 *  Created on: 11.12.2016.
 *      Author: milan
 */

#ifndef ANALOG_H_
#define ANALOG_H_

#include "stdint.h"
#include "stm32f0xx_hal.h"

#define ADC_SCAN_CHANNELS		8
#define ADC_BUFFER_LENGTH		((uint16_t)512*ADC_SCAN_CHANNELS)
#define ADC_CS1_CHN	        0	// PA0, 5V rail sense (/2 divider), labelled CS1 in schematic
#define ADC_CS2_CHN	        1	// PA1, current-sense amp output on >=2.3 / shunt tap on <2.3
#define ADC_VBAT_SENS_CHN	  2
#define ADC_NTC_CHN	        3
#define POW_DET_SENS_CHN	  4
#define ADC_IO1_CHN	        5
#define ADC_TEMP_SENS_CHN	  6
#define ADC_VREF_BUFF_CHN	  7

// Board hardware revision, detected at runtime from the CS1/CS2 analog signals.
#define HARD_REV_BELOW_2_3	0
#define HARD_REV_2_3_AND_ABOVE	1 // introduced current sensor amp NCS213
#define HARD_REV_UNKNOWN	0xFF
#define ADC_CONT_MODE_NORMAL	0
#define ADC_CONT_MODE_LOW_VOLTAGE 	1 // In this mode one channel in scan group is internal reference
#define ADC_GET_BUFFER_SAMPLE(i)	(analogIn[(i)])

// Written into the first/last cell before starting DMA; AnalogSamplesReady() reports
// ready once DMA has overwritten both. Must be outside the 12-bit ADC range and must
// match the width of analogIn[] elements.
#define ADC_SAMPLE_SENTINEL		((uint16_t)0xFFFF)

//#define ANALOG_IS_SAMPLES_VALID()	 (HAL_IS_BIT_SET(hadc.Instance->CR, ADC_CR_ADSTART) && (analogBufferTicks > (HAL_GetTick()+100) ))

extern int32_t mcuTemperature;

extern ADC_HandleTypeDef hadc;
extern uint16_t analogIn[ADC_BUFFER_LENGTH];

/*
 * Analog supply voltage in mV, derived from the internal reference. Refreshed by
 * AnalogTask() from the freshest VREFINT sample and by AnalogPowerIsGood() after the 5V
 * regulator settles. Kept behind an accessor so analog.c owns the only writable copy.
 */
uint16_t AnalogGetAvdd(void);

/*
 * Detected board revision, one of HARD_REV_*. Probed by AnalogTask() from the CS1/CS2
 * signals once the 5V rail is up, so it stays HARD_REV_UNKNOWN until the first probe.
 */
uint8_t GetHardwareRev(void);

//extern uint32_t analogBufferTicks;

void AnalogInit(void);
void AnalogTask(void);
int16_t Get5vIoVoltage();
void AnalogStop(void);
void AnalogStart(void);
void AnalogPowerIsGood(void);
//void AnalogSetAdcMode(uint8_t mode);
uint8_t AnalogSamplesReady();
uint16_t GetSampleVoltage(uint8_t channel);
uint16_t GetAverageBatteryVoltage(uint8_t channel);
void GetAdcSignals02(uint32_t pos, uint8_t* buf);
void GetAdcSignals12(uint32_t pos, uint8_t* buf);

uint16_t GetSample(uint8_t channel);

#endif /* ANALOG_H_ */
