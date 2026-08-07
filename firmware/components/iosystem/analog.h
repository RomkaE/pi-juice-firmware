/*
 * analog.h
 *
 *  Created on: 11.12.2016.
 *      Author: milan
 */

#ifndef ANALOG_H_
#define ANALOG_H_

#include <stdint.h>
#include <stdbool.h>

/*
 * Battery sense divider on PA2: Vbat = Vpin * VBAT_DIVIDER_NUM / VBAT_DIVIDER_DEN.
 * Single definition for every VBAT conversion in the firmware - before this there were four
 * hand-rolled variants (11/8, 1374/1000 twice, 4535/32768) that agreed to within 0.2%.
 */
#define VBAT_DIVIDER_NUM      1374
#define VBAT_DIVIDER_DEN      1000
#define ADC_VREF_NOMINAL_MV   3300

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
#define VBAT_MV_TO_ADC(mV)  ((uint16_t)((uint32_t)(mV) * 4096U / VBAT_DIVIDER_NUM \
          * VBAT_DIVIDER_DEN / ADC_VREF_NOMINAL_MV))

// Mask errors:
#define ANALOG_ERR_NO_STREAM      (1UL << 0)  // no DMA events within the watchdog timeout
#define ANALOG_ERR_PROC_OVERRUN   (1UL << 1)  // processing task fell behind, half-buffers dropped
#define ANALOG_ERR_HW_OVERRUN     (1UL << 2)  // ADC hardware OVR: a conversion was overwritten in DR

void analog_Init(void);

uint16_t analog_GetTempMCU(void);

uint16_t analog_GetAvdd(void);

uint16_t analog_GetRawBatt(void);

uint16_t analog_GetVBatt(void);

uint16_t analog_GetVBattAvg(void);

uint16_t analog_Get5vPi(void);

uint16_t analog_GetRawPWR(void);

uint32_t analog_GetErrMask(bool _clear);

/*
 * Weak no-op, called once per half ring (~64 ms) after every value above is updated.
 * Runs in the ANALOG task, which must drain the next DMA half in time: do not block, do not take
 * a lock another task can hold across a flash write. Read and post an event, nothing more.
 */
void analog_OnEvent_SamplesReady(void);

#endif /* ANALOG_H_ */
