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
#include "config.h"

/*
 * Battery sense divider on PA2: Vbat = Vpin * VBAT_DIVIDER_NUM / VBAT_DIVIDER_DEN.
 * Single definition for every VBAT conversion in the firmware - before this there were four
 * hand-rolled variants (11/8, 1374/1000 twice, 4535/32768) that agreed to within 0.2%.
 */
#define VBAT_DIVIDER_NUM      1374
#define VBAT_DIVIDER_DEN      1000
#define ADC_VREF_NOMINAL_MV   3300

void analog_Init(void);

#if ANALOG_TEMP_MCU_ENABLED
uint16_t analog_GetTempMCU(void);
#endif

uint16_t analog_GetAvdd(void);

uint16_t analog_GetVBatt(void);

uint16_t analog_Get5vPi(void);

/*
 * Weak no-op, called once per half ring (~64 ms) after every value above is updated.
 * Runs in the ANALOG task, which must drain the next DMA half in time: do not block, do not take
 * a lock another task can hold across a flash write. Read and post an event, nothing more.
 */
void analog_SamplesReady_Callback(void);

#endif /* ANALOG_H_ */
