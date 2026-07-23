/*
 * analog.c
 *
 *  Created on: 11.12.2016.
 *      Author: milan
 */

#include <stdint.h>
#include "analog.h"
#include <to_refactor/config_switch_resistor.h>
#include <to_refactor/time_count.h>
#include "stm32f0xx_hal.h"
#include "nv.h"
#include "app-error/app_error.h"

// ST HAL:
#include "stm32f0xx_hal.h"
#include "stm32f0xx_ll_adc.h"

// LOG:
#include "log/log.h"

/* mcuTemperature sensor calibration value address */
#define TEMP30_CAL_ADDR           TEMPSENSOR_CAL1_ADDR

/*
 * Timeout for a polled conversion sequence, which needs ~108 us (6 channels x
 * (239.5 + 12.5) cycles at HSI14).
 *
 * The value is not critical: HAL_GetTick() is overridden onto the FreeRTOS tick (main.c)
 * and configTICK_RATE_HZ is 100, so it advances in 10 ms steps - every HAL timeout is
 * quantised to 10..20 ms regardless of what is asked for. What matters here is that the
 * return value is checked at all, so a failed conversion cannot be scaled into s_AVDD.
 */
#define ADC_SEQUENCE_TIMEOUT_MS		5

static uint8_t s_HwRev = HARD_REV_UNKNOWN;

static uint16_t s_AVDD;

static uint32_t tempCalcCounter;

// TODO - static
uint16_t analogIn[ADC_BUFFER_LENGTH];

// TODO - static
int32_t mcuTemperature = 25; // will contain the mcuTemperature in degree Celsius

static void updateAVDD(uint32_t _raw_vrefint)
{
  static uint32_t rcnt_raw_vrefint;
  if (rcnt_raw_vrefint != _raw_vrefint)
  {
    rcnt_raw_vrefint = _raw_vrefint;
    s_AVDD = ((uint32_t)VREFINT_CAL_VREF * (uint32_t)*VREFINT_CAL_ADDR) / _raw_vrefint;
    LOG_INFO("Update AVDD: vref_cal = %u, raw_vref=%u, avdd=%u", *VREFINT_CAL_ADDR, _raw_vrefint, s_AVDD);
  }
}

/*
 * One-shot board-revision probe. PA1 (ADC_CS2_CHN_IDX) carries the NCS213 current-sense amp
 * output on >=2.3 hardware (idles near 1570 counts while PA0/5V-sense sits ~3100) and a
 * plain shunt tap that tracks PA0 within a few counts on <2.3. A single comparison with a
 * wide margin (3x the largest shunt drop, 3x below the NCS idle offset) separates the two.
 * Needs the 5V rail up so both taps are energised; retries every task tick until then.
 */
static void DetectHardwareRev(void)
{
  if (s_HwRev != HARD_REV_UNKNOWN)
    return;
  if (!AnalogSamplesReady() || Get5vIoVoltage() <= 4500)
    return;
  int32_t d = (int32_t) GetSample(ADC_CS1_CHN_IDX)
      - (int32_t) GetSample(ADC_CS2_CHN_IDX);
  s_HwRev = (d > 500) ? HARD_REV_2_3_AND_ABOVE : HARD_REV_BELOW_2_3;
}

/*
 * Index of the freshest complete sample of `channel` in the DMA ring.
 *
 * The DMA counter runs down, so ADC_BUFFER_LENGTH - counter - 1 is the last cell written.
 * Truncating that to a frame boundary gives the frame in progress; if the requested channel
 * has not been converted in it yet, step back one frame.
 */
static int32_t FreshSampleIndex(uint8_t channel) {
	int32_t last = ADC_BUFFER_LENGTH - (int32_t)__HAL_DMA_GET_COUNTER(hadc.DMA_Handle) - 1;
	int32_t ind = (last / ADC_SCAN_CHANNELS) * ADC_SCAN_CHANNELS + channel;
	if (ind > last) ind -= ADC_SCAN_CHANNELS; // channel not written in the current frame yet
	if (ind < 0) ind += ADC_BUFFER_LENGTH;    // wrapped past the start of the ring
	return ind;
}

/*
 * GetSampleVoltage() lived here: a ratiometric channel-to-mV conversion that took VREFINT
 * from the same frame instead of the AnalogTask-refreshed AVDD. Its last caller was the IO1
 * analog read; removed with it. Worth restoring if a measurement ever needs to be immune to
 * AVDD moving between frames.
 */

/* Frames averaged by GetAverageBatteryVoltage(). Power of two - see the shift below. */
#define VBAT_AVERAGE_FRAMES	8
#define VBAT_AVERAGE_SHIFT	15	// 12 (ADC bits) + 3 (log2 VBAT_AVERAGE_FRAMES)

/* mV at the divider tap -> mV at the battery. */
#define VBAT_FROM_PIN_MV(v)	((uint16_t)((v) * VBAT_DIVIDER_NUM / VBAT_DIVIDER_DEN))

uint16_t GetBatteryVoltage(void) {
	uint32_t pinMv = ((uint32_t)GetSample(ADC_VBAT_CHN_IDX) * analog_GetAvdd()) >> 12;
	return VBAT_FROM_PIN_MV(pinMv);
}

uint16_t GetAverageBatteryVoltage(void) {
	uint32_t sum = GetSampleSum(ADC_VBAT_CHN_IDX, VBAT_AVERAGE_FRAMES);
	uint32_t pinMv = (sum * analog_GetAvdd()) >> VBAT_AVERAGE_SHIFT;
	return VBAT_FROM_PIN_MV(pinMv);
}

int16_t Get5vIoVoltage() {
	int16_t adcAvg = GetSampleSum(ADC_CS1_CHN_IDX, 4) >> 2;
	return (s_AVDD > 3200 && s_AVDD < 3400) ? (adcAvg * s_AVDD) >> 11 : (adcAvg * 3300) >> 11;//adcAvg * s_AVDD / 4096 * 2;
}

uint16_t GetSample(uint8_t channel) {
	return analogIn[FreshSampleIndex(channel)];
}

uint32_t GetSampleSum(uint8_t channel, uint16_t frames) {
	int32_t ind = FreshSampleIndex(channel);
	uint32_t sum = 0;
	while (frames--) {
		sum += analogIn[ind];
		ind -= ADC_SCAN_CHANNELS;
		if (ind < 0) ind += ADC_BUFFER_LENGTH;
	}
	return sum;
}

uint8_t AnalogSamplesReady() {
	return analogIn[0] != ADC_SAMPLE_SENTINEL && analogIn[ADC_BUFFER_LENGTH-1] != ADC_SAMPLE_SENTINEL;
}

/*
 * The scan group, and with it the layout of every frame in analogIn[].
 *
 *  buffer idx | ADC ch | pin  | signal
 *  -----------+--------+------+-------------------------------------------------
 *      0      |   0    | PA0  | 5V rail sense through a /2 divider (CS1)
 *      1      |   1    | PA1  | current-sense amp output (>=2.3) / shunt tap (<2.3) (CS2)
 *      2      |   2    | PA2  | VBAT
 *      3      |   4    | PA4  | POW_DET_SEN
 *      4      |  16    |  -   | MCU temperature
 *      5      |  17    |  -   | VREFINT
 *
 * CH3 (PA3, battery NTC) and CH7 (PA7, IO1) used to be in this group and were dropped - see
 * the notes at their former ConfigChannel calls below.
 *
 * STM32F0 has no configurable ranks: HAL_ADC_ConfigChannel only sets or clears a bit in
 * CHSELR, and the sequence order comes from SCANDIR. With ADC_SCAN_DIRECTION_FORWARD the
 * scan runs in ascending channel number no matter in which order the calls below are made,
 * so the buffer index of a signal is its position in the sorted channel set - which is what
 * the ADC_*_CHN_IDX constants in analog.h spell out.
 *
 * The delicate part is CH5 (config resistor): it is enabled for a single measurement at the
 * top of this function and removed again with ADC_RANK_NONE. If that removal ever gets
 * lost, indices 5/6/7 shift by one with no other symptom - hence the assert at the end.
 *
 * SamplingTime is not per channel either: on F0 there is a single SMPR shared by the whole
 * group, so the last assignment wins and speeding up one channel speeds up all of them.
 */
void AnalogInit(void)
{
  ADC_ChannelConfTypeDef ch_cfg;
  ch_cfg.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;  // The ADC doesn't have separate timing for different channels

  // Read PCB SWITCH configuration:
  {
    ch_cfg.Channel = ADC_CHANNEL_5;
    ch_cfg.Rank = ADC_RANK_CHANNEL_NUMBER;
    if (HAL_ADC_ConfigChannel(&hadc, &ch_cfg) != HAL_OK)
    {
      APP_ERROR(APP_HAL_ERROR);
    }

    // TODO - meas only one channel?! Incorrect value?
    uint32_t resistorConfigAdc;
    HAL_ADC_Start(&hadc);
    if (HAL_ADC_PollForConversion(&hadc, ADC_SEQUENCE_TIMEOUT_MS) != HAL_OK)
    {
      APP_ERROR(APP_HAL_ERROR);
    }
    resistorConfigAdc = HAL_ADC_GetValue(&hadc);
    HAL_ADC_Stop(&hadc);
    SwitchResCongigInit(resistorConfigAdc);

    // Disable ACD_IN5:
    ch_cfg.Channel = ADC_CHANNEL_5;
    ch_cfg.Rank = ADC_RANK_NONE;
    if (HAL_ADC_ConfigChannel(&hadc, &ch_cfg) != HAL_OK)
    {
      APP_ERROR(APP_HAL_ERROR);
    }
  }

  // VREF_INT channel:
  {
    ch_cfg.Channel = ADC_CHANNEL_17;
    ch_cfg.Rank = ADC_RANK_CHANNEL_NUMBER;
    if (HAL_ADC_ConfigChannel(&hadc, &ch_cfg) != HAL_OK)
    {
      APP_ERROR(APP_HAL_ERROR);
    }

    // Read initial VREF_INT value:
    // TODO - meas only one channel?! Incorrect value?
    HAL_ADC_Start(&hadc);
    if (HAL_ADC_PollForConversion(&hadc, ADC_SEQUENCE_TIMEOUT_MS) != HAL_OK)
    {
      APP_ERROR(APP_HAL_ERROR); // s_AVDD below would divide by a stale reading
    }
    uint32_t raw_vref_int = HAL_ADC_GetValue(&hadc);
    HAL_ADC_Stop(&hadc);

    // Calc corrected AVDD:
    updateAVDD(raw_vref_int);
  }

  // CS1:
  ch_cfg.Channel = ADC_CHANNEL_0;
  if (HAL_ADC_ConfigChannel(&hadc, &ch_cfg) != HAL_OK)
  {
    APP_ERROR(APP_HAL_ERROR);
  }

  // CS2:
  ch_cfg.Channel = ADC_CHANNEL_1;
  if (HAL_ADC_ConfigChannel(&hadc, &ch_cfg) != HAL_OK)
  {
    APP_ERROR(APP_HAL_ERROR);
  }

  // VBATЖ
  ch_cfg.Channel = ADC_CHANNEL_2;
  if (HAL_ADC_ConfigChannel(&hadc, &ch_cfg) != HAL_OK)
  {
    APP_ERROR(APP_HAL_ERROR);
  }

  /*
   * CH3 (PA3, battery NTC) is not scanned: battery temperature is not going to be measured
   * through the ADC. The pin stays in analog mode, the thermistor path in the fuel gauge
   * driver falls back to the MCU temperature sensor.
   */

  // POW_DET_SEN:
  ch_cfg.Channel = ADC_CHANNEL_4;
  if (HAL_ADC_ConfigChannel(&hadc, &ch_cfg) != HAL_OK)
  {
    APP_ERROR(APP_HAL_ERROR);
  }

  // Int temperature:
  ch_cfg.Channel = ADC_CHANNEL_16;
  if (HAL_ADC_ConfigChannel(&hadc, &ch_cfg) != HAL_OK)
  {
    APP_ERROR(APP_HAL_ERROR);
  }

  /*
   * CHSELR now defines both the scan order and the frame layout: one converted channel per
   * set bit, ascending. Any mismatch with ADC_SCAN_CHANNELS silently shifts every
   * ADC_*_CHN_IDX above the offending channel.
   */
  ASSERT(__builtin_popcount(hadc.Instance->CHSELR) == ADC_SCAN_CHANNELS);

  // make bufer data invalid:
  analogIn[0] = ADC_SAMPLE_SENTINEL;
  analogIn[ADC_BUFFER_LENGTH - 1] = ADC_SAMPLE_SENTINEL;

  // Start conversion in DMA mode:
  if (HAL_ADC_Start_DMA(&hadc, (uint32_t*) analogIn, ADC_BUFFER_LENGTH) != HAL_OK)
  {
    APP_ERROR(APP_HAL_ERROR);
  }

  // TODO?:
  MS_TIME_COUNTER_INIT(tempCalcCounter);
}

void AnalogTask(void)
{

	if (MS_TIME_COUNT(tempCalcCounter) > 2000)
	{
		int32_t vtemp = (((uint32_t)GetSample(ADC_TEMP_INT_CHN_IDX)) * s_AVDD * 10) >> 12;
		int32_t v30 = (((uint32_t) * TEMP30_CAL_ADDR ) * TEMPSENSOR_CAL_VREFANALOG) >> 12;
		mcuTemperature = (v30 - vtemp) / 43 + 30; //avg_slope = 4.3
		//mcuTemperature = ((((int32_t)*TEMP30_CAL_ADDR - analogIn[7]) * 767) >> 12) + 30;
		MS_TIME_COUNTER_INIT(tempCalcCounter);
	}

	// Update AVDD in runtime:
	uint16_t vRefSample = GetSample(ADC_VREF_INT_CHN_IDX);
	// zero before DMA has filled the ring; the macro divides by it
	if (vRefSample != 0)
		updateAVDD(vRefSample);

	// TODO - change to one-shot call during init
	DetectHardwareRev();
}

void AnalogStop(void)
{
  if (HAL_IS_BIT_SET(hadc.Instance->CR, ADC_CR_ADSTART))
  {
    HAL_ADC_Stop_DMA(&hadc);
  }
}

void AnalogStart(void) {
  if (!HAL_IS_BIT_SET(hadc.Instance->CR, ADC_CR_ADSTART))
  {
    // make bufer data invalid
    analogIn[0] = ADC_SAMPLE_SENTINEL;
    analogIn[ADC_BUFFER_LENGTH - 1] = ADC_SAMPLE_SENTINEL;
    if (HAL_ADC_Start_DMA(&hadc, (uint32_t*) analogIn, ADC_BUFFER_LENGTH)
        != HAL_OK)
    {
      APP_ERROR(APP_HAL_ERROR);
    }
  }
}

uint16_t analog_GetAvdd(void)
{
  return s_AVDD;
}

uint8_t analog_GetHardwareRev(void)
{
  return s_HwRev;
}

/*
 * Refresh s_AVDD right after the 5V regulator has settled, without waiting for the ring to
 * refill.
 *
 * The single conversion below yields VREFINT and not some other channel only because
 * EOCSelection is ADC_EOC_SEQ_CONV: the poll waits for the end of the whole sequence, and
 * CH17 is the highest channel number, hence the last one converted with SCANDIR forward.
 * Adding any channel above 17 to the group would break this silently.
 *
 * The commented-out reconfiguration below would narrow the group to CH17 for the duration.
 * It is deliberately left off: this runs on the 5V turn-on path, where extra ADC stop/start
 * cycles cost more than the sequence they would save.
 */
void AnalogPowerIsGood(void)
{
	if (HAL_IS_BIT_SET(hadc.Instance->CR, ADC_CR_ADSTART))
	{
		HAL_ADC_Stop_DMA(&hadc);
	}

	// TODO - meas only one channel?! Incorrect value?
	HAL_ADC_Start(&hadc);
	if (HAL_ADC_PollForConversion(&hadc, ADC_SEQUENCE_TIMEOUT_MS) == HAL_OK)
	{
	  uint32_t raw_vref_int = HAL_ADC_GetValue(&hadc);
	  updateAVDD(raw_vref_int);
	}
	HAL_ADC_Stop(&hadc);


	// make bufer data invalid
	analogIn[0] = ADC_SAMPLE_SENTINEL;
	analogIn[ADC_BUFFER_LENGTH-1] = ADC_SAMPLE_SENTINEL;
	if (HAL_ADC_Start_DMA(&hadc, (uint32_t*)analogIn, ADC_BUFFER_LENGTH) != HAL_OK)
	{
		APP_ERROR(APP_HAL_ERROR);
	}
}

/*void AnalogSetAdcMode(uint8_t mode) {
	if (mode == ADC_CONT_MODE_NORMAL) {

		uint8_t stopped = 0;
		if (HAL_IS_BIT_SET(hadc.Instance->CR, ADC_CR_ADSTART))  {
			HAL_ADC_Stop_DMA(&hadc);
			stopped = 1;
		}

		// Substitute internal reference voltage with channel 1
		sConfig.Channel = ADC_CHANNEL_17;
		sConfig.Rank = ADC_RANK_NONE;
		if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
		{
		APP_ERROR(APP_HAL_ERROR);
		}

		sConfig.Channel = ADC_CHANNEL_1;
		sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
		sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
		if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
		{
		APP_ERROR(APP_HAL_ERROR);
		}

		// make buffer data invalid
		analogIn[0] = ADC_SAMPLE_SENTINEL;
		analogIn[ADC_BUFFER_LENGTH-1] = ADC_SAMPLE_SENTINEL;

		if (stopped) {
			if (HAL_ADC_Start_DMA(&hadc, (uint32_t*)analogIn, ADC_BUFFER_LENGTH) != HAL_OK)
			{
				APP_ERROR(APP_HAL_ERROR);
			}
		}
	} else if (mode == ADC_CONT_MODE_LOW_VOLTAGE) {

		uint8_t stopped = 0;
		if (HAL_IS_BIT_SET(hadc.Instance->CR, ADC_CR_ADSTART))  {
			HAL_ADC_Stop_DMA(&hadc);
			stopped = 1;
		}

		// Substitute channel 1 with internal reference voltage channel
		sConfig.Channel = ADC_CHANNEL_1;
		sConfig.Rank = ADC_RANK_NONE;
		if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
		{
		APP_ERROR(APP_HAL_ERROR);
		}

		sConfig.Channel = ADC_CHANNEL_17; // Vref
		sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
		sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;;
		if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
		{
		APP_ERROR(APP_HAL_ERROR);
		}

		// make buffer data invalid
		analogIn[0] = ADC_SAMPLE_SENTINEL;
		analogIn[ADC_BUFFER_LENGTH-1] = ADC_SAMPLE_SENTINEL;

		if (stopped) {
			if (HAL_ADC_Start_DMA(&hadc, (uint32_t*)analogIn, ADC_BUFFER_LENGTH) != HAL_OK)
			{
				APP_ERROR(APP_HAL_ERROR);
			}
		}
	}
}*/
