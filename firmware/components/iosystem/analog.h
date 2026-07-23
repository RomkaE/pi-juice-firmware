/*
 * analog.h
 *
 *  Created on: 11.12.2016.
 *      Author: milan
 */

#ifndef ANALOG_H_
#define ANALOG_H_

#include <stdint.h>


#include "stm32f0xx_hal.h"

/*
 * The DMA ring holds ADC_FRAMES complete scans, one uint16_t per channel per frame, so
 * index = frame * ADC_SCAN_CHANNELS + channel. A frame takes ~108 us (6 channels x
 * (239.5 + 12.5) cycles at HSI14), hence the ring spans ~6.9 ms - several frames' worth of
 * history for the 20 ms task loop, in well under 1 KB of the F030CC's 32 KB of RAM.
 * Never index it directly: use GetSample()/GetSampleSum(), which locate the freshest frame
 * from the DMA counter.
 *
 * Note ADC_SCAN_CHANNELS is no longer a power of two - nothing may reintroduce the old
 * "multiply by 32768/ADC_SCAN_CHANNELS and shift" trick for dividing by it.
 */
#define ADC_SCAN_CHANNELS		6
#define ADC_FRAMES          64
#define ADC_BUFFER_LENGTH		((uint16_t)ADC_FRAMES * ADC_SCAN_CHANNELS)

//
#define ADC_CS1_CHN_IDX         0
#define ADC_CS2_CHN_IDX	        1
#define ADC_VBAT_CHN_IDX	      2
#define ADC_POW_DET_CHN_IDX	    3
#define ADC_TEMP_INT_CHN_IDX	  4
#define ADC_VREF_INT_CHN_IDX	  5

// Board hardware revision, detected at runtime from the CS1/CS2 analog signals.
#define HARD_REV_BELOW_2_3	0
#define HARD_REV_2_3_AND_ABOVE	1 // introduced current sensor amp NCS213
#define HARD_REV_UNKNOWN	0xFF
#define ADC_CONT_MODE_NORMAL	0
#define ADC_CONT_MODE_LOW_VOLTAGE 	1 // In this mode one channel in scan group is internal reference

/*
 * Battery sense divider on PA2: Vbat = Vpin * VBAT_DIVIDER_NUM / VBAT_DIVIDER_DEN.
 * Single definition for every VBAT conversion in the firmware - before this there were four
 * hand-rolled variants (11/8, 1374/1000 twice, 4535/32768) that agreed to within 0.2%.
 */
#define VBAT_DIVIDER_NUM		1374
#define VBAT_DIVIDER_DEN		1000
#define ADC_VREF_NOMINAL_MV		3300

/*
 * Battery voltage in mV -> raw ADC count, for code that compares GetSample() readings
 * directly instead of converting them.
 *
 * Deliberately assumes the nominal reference rather than AnalogGetAvdd(): this is used on
 * the undervoltage cutoff path, which has to stay meaningful exactly when the supply is
 * sagging and the measured AVDD is least trustworthy. The cost is that the cutoff tracks
 * the real AVDD only as well as the 3.3 V rail holds.
 *
 * The evaluation order keeps every intermediate under 2^32 for mV up to ~1 MV.
 */
#define VBAT_MV_TO_ADC(mV)	((uint16_t)((uint32_t)(mV) * 4096U / VBAT_DIVIDER_NUM \
					* VBAT_DIVIDER_DEN / ADC_VREF_NOMINAL_MV))

// Written into the first/last cell before starting DMA; AnalogSamplesReady() reports
// ready once DMA has overwritten both. Must be outside the 12-bit ADC range and must
// match the width of analogIn[] elements.
#define ADC_SAMPLE_SENTINEL		((uint16_t)0xFFFF)

//#define ANALOG_IS_SAMPLES_VALID()	 (HAL_IS_BIT_SET(hadc.Instance->CR, ADC_CR_ADSTART) && (analogBufferTicks > (HAL_GetTick()+100) ))

extern int32_t mcuTemperature;

// TODO - remove
extern ADC_HandleTypeDef hadc;
extern uint16_t analogIn[ADC_BUFFER_LENGTH];

void AnalogInit(void);

void AnalogTask(void);

void AnalogStop(void);

void AnalogStart(void);

/*
 * Analog supply voltage in mV, derived from the internal reference. Refreshed by
 * AnalogTask() from the freshest VREFINT sample and by AnalogPowerIsGood() after the 5V
 * regulator settles. Kept behind an accessor so analog.c owns the only writable copy.
 */
uint16_t analog_GetAvdd(void);

/*
 * Detected board revision, one of HARD_REV_*. Probed by AnalogTask() from the CS1/CS2
 * signals once the 5V rail is up, so it stays HARD_REV_UNKNOWN until the first probe.
 */
uint8_t analog_GetHardwareRev(void);

int16_t Get5vIoVoltage();

void AnalogPowerIsGood(void);
//void AnalogSetAdcMode(uint8_t mode);
uint8_t AnalogSamplesReady();
/*
 * Battery voltage in mV. GetBatteryVoltage() uses the freshest frame and is what the
 * protection paths want; GetAverageBatteryVoltage() averages VBAT_AVERAGE_FRAMES frames for
 * the fuel gauge. Both apply the same divider and the same measured AVDD.
 */
uint16_t GetBatteryVoltage(void);
uint16_t GetAverageBatteryVoltage(void);

/* Freshest complete sample of one channel. */
uint16_t GetSample(uint8_t channel);

/*
 * Sum (not average) of one channel over the last `frames` frames, freshest first. Returning
 * the sum lets each caller keep its own scaling arithmetic unchanged. `frames` must not
 * exceed ADC_FRAMES; it is the caller's averaging window, expressed in frames rather than
 * in buffer offsets, so it stays correct if the ring is resized.
 */
uint32_t GetSampleSum(uint8_t channel, uint16_t frames);

#endif /* ANALOG_H_ */
