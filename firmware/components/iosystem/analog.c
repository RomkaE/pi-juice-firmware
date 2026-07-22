/*
 * analog.c
 *
 *  Created on: 11.12.2016.
 *      Author: milan
 */

#include "analog.h"
#include <to_refactor/config_switch_resistor.h>
#include <to_refactor/time_count.h>
#include "stm32f0xx_hal.h"
#include "nv.h"
#include "app-error/app_error.h"

/* mcuTemperature sensor calibration value address */
#define TEMP30_CAL_ADDR ((uint16_t*) ((uint32_t) 0x1FFFF7B8))
#define VREFINT_CAL_ADDR ((uint16_t*) ((uint32_t) 0x1FFFF7BA))

#define ANALOG_ADC_GET_AVDD(a)		 (((uint32_t)*VREFINT_CAL_ADDR ) * 3300 / (a)) // refVolt = vRefAdc * aVdd / 4096 //volatile uint32_t refVolt = ((uint32_t)*VREFINT_CAL_ADDR ) * 3300 / 4096;

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

extern ADC_HandleTypeDef hadc;
ADC_ChannelConfTypeDef sConfig;

static uint32_t tempCalcCounter;

volatile uint32_t vRefAdc;

uint16_t analogIn[ADC_BUFFER_LENGTH];

uint16_t AnalogGetAvdd(void) {
	return s_AVDD;
}

uint8_t GetHardwareRev(void) {
	return s_HwRev;
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

//int16_t testBuf[512] __attribute__((section("no_init")));
//volatile uint16_t testInd = 0;
#if defined LOGGING
/*
 * These two walk the DMA ring by hand to dump a waveform. They were written against a
 * 512-frame ring (~74 ms of history); the ring is now ADC_FRAMES (~9.2 ms), so they would
 * still compile and still return data, just over an 8x shorter window - silently different
 * traces. Logging is not maintained; bringing it back should use a dedicated log buffer
 * over the two or three channels it actually needs, not the ADC ring.
 */
#error "LOGGING needs a dedicated log buffer - the ADC ring is now ADC_FRAMES frames (~9.2 ms), not 512"

// Function used for logging of 5V GPIO and battery voltage signals
// Signals are compressed to one byte per sample
void GetAdcSignals02(uint32_t pos, uint8_t* buf) {
	//volatile int32_t p = pos/ADC_SCAN_CHANNELS-1;
    int32_t ind = (((ADC_BUFFER_LENGTH - (int32_t)pos - 1) * (32768/ADC_SCAN_CHANNELS)) >> 15) * ADC_SCAN_CHANNELS;// + channel; //p*ADC_SCAN_CHANNELS+channel;//

    if (ind > (ADC_BUFFER_LENGTH - (int32_t)pos - 1)) ind -= ADC_SCAN_CHANNELS; // check if calculated channel sample is fresh
    if (ind < 0) ind += ADC_BUFFER_LENGTH;
    int i;
    for(i=0; i<10; i++) {
    	if (ind > ADC_BUFFER_LENGTH) ind -= ADC_BUFFER_LENGTH; // check if calculated channel sample is fresh
		//int16_t indRef = ind + ADC_VREF_BUFF_CHN;// - channel;
		buf[i] = analogIn[ind+2]>>3;//(analogIn[ind] * ((uint32_t)*VREFINT_CAL_ADDR ) * 412 / analogIn[indRef]) >>  9;
		buf[i+10] = analogIn[ind]>>4;
		//testBuf[i] = (analogIn[ind+2]*8* 4535 / analogIn[indRef] * ((uint32_t)*VREFINT_CAL_ADDR )) >> 15 ;//((analogIn[ind]&0xFFFF) * ((uint32_t)*VREFINT_CAL_ADDR ) * 412 / analogIn[indRef]) >>  8;
		ind += ADC_SCAN_CHANNELS*2;
		/*if ((analogIn[ind]>>16) == 0xF00F) {
			testInd = i;
		}*/
    }
    return;
}

// Function used for logging of 5V GPIO, GPIO current and battery voltage signals
// Signals are compressed to one byte per sample
void GetAdcSignals12(uint32_t pos, uint8_t* buf) {
    int32_t ind = (((ADC_BUFFER_LENGTH - (int32_t)pos - 1) * (32768/ADC_SCAN_CHANNELS)) >> 15) * ADC_SCAN_CHANNELS;// + channel; //p*ADC_SCAN_CHANNELS+channel;//

    if (ind > (ADC_BUFFER_LENGTH - (int32_t)pos - 1)) ind -= ADC_SCAN_CHANNELS; // check if calculated channel sample is fresh

    ind += (int)3*1*ADC_SCAN_CHANNELS; // get back 8 samples in order to read signal history, every fourth sample copied
    //if (ind < 0) ind += ADC_BUFFER_LENGTH;
    if (ind > ADC_BUFFER_LENGTH) ind -= ADC_BUFFER_LENGTH;
    //int ch0Ind = ind+ADC_SCAN_CHANNELS*8;
    //if (ch0Ind > ADC_BUFFER_LENGTH) ch0Ind -= ADC_BUFFER_LENGTH;
    buf[0] = analogIn[ind]>>4; // only one sample of 5V GPIO
    //buf++;
    int i;
    if (GetHardwareRev()==0) {
		for(i=1; i<9; i++) {
			 if (ind < 0) ind += ADC_BUFFER_LENGTH;
			buf[i] = (analogIn[ind+2]&0x0800) ? analogIn[ind+2]>>3 : 0;//(analogIn[ind] * ((uint32_t)*VREFINT_CAL_ADDR ) * 412 / analogIn[indRef]) >>  9;
			int dif = (int)(analogIn[ind]) - analogIn[ind+1];
			buf[i+8] = dif&0x7f;// > 0 ?(dif<128?dif:127) : 0;
			ind -= ADC_SCAN_CHANNELS*1;
		}
	} else {
		for(i=1; i<9; i++) {
			if (ind < 0) ind += ADC_BUFFER_LENGTH;
			buf[i] = (analogIn[ind+2]&0x0800) ? analogIn[ind+2]>>3 : 0;//(analogIn[ind] * ((uint32_t)*VREFINT_CAL_ADDR ) * 412 / analogIn[indRef]) >>  9;
			int dif = ((int)1574-(analogIn[ind+1])) >> 4;
			buf[i+8] = (dif > 0) && (analogIn[ind]>1200) ? (dif<128?dif:127)|0x80 : 0;
			//testBuf[i] = (analogIn[ind]);
			ind -= ADC_SCAN_CHANNELS*1;
		}
	}
    //volatile uint32_t p = pos;
    /*ind = (((ADC_BUFFER_LENGTH - (int32_t)pos - 1) * (32768/ADC_SCAN_CHANNELS)) >> 15) * ADC_SCAN_CHANNELS;
    if (ind > (ADC_BUFFER_LENGTH - (int32_t)pos - 1)) ind -= ADC_SCAN_CHANNELS;
    if (ind < 0) ind += ADC_BUFFER_LENGTH;
    for(i=0; i<512; i++) {testBuf[i] = (analogIn[ind]); ind +=8;}*/
    return;
}

//void SetMarker(uint8_t channel){
    /*int32_t pos =  __HAL_DMA_GET_COUNTER(hadc.DMA_Handle);
    int32_t ind = (((ADC_BUFFER_LENGTH - pos - 1) * (32768/ADC_SCAN_CHANNELS)) >> 15) * ADC_SCAN_CHANNELS + channel;
	if (ind > (ADC_BUFFER_LENGTH - pos - 1)) ind -= ADC_SCAN_CHANNELS; // check if calculated channel sample is fresh
	if (ind < 0) ind += ADC_BUFFER_LENGTH;
	analogIn[ind] |= 0xF00F0000;

	ind -= ADC_SCAN_CHANNELS;
	if (ind < 0) ind += ADC_BUFFER_LENGTH;
	analogIn[ind] |= 0xF00F0000;

	ind += ADC_SCAN_CHANNELS*2;
	if (ind > ADC_BUFFER_LENGTH) ind -= ADC_BUFFER_LENGTH;
	analogIn[ind] |= 0xF00F0000;*/
	//int i;
	//for(i=0; i<ADC_BUFFER_LENGTH; i+=ADC_SCAN_CHANNELS) analogIn[i] |= 0xF00F0000;
//}
#endif

/* Frames averaged by GetAverageBatteryVoltage(). Power of two - see the shift below. */
#define VBAT_AVERAGE_FRAMES	8
#define VBAT_AVERAGE_SHIFT	15	// 12 (ADC bits) + 3 (log2 VBAT_AVERAGE_FRAMES)

/* mV at the divider tap -> mV at the battery. */
#define VBAT_FROM_PIN_MV(v)	((uint16_t)((v) * VBAT_DIVIDER_NUM / VBAT_DIVIDER_DEN))

uint16_t GetBatteryVoltage(void) {
	uint32_t pinMv = ((uint32_t)GetSample(ADC_VBAT_SENS_CHN_IDX) * AnalogGetAvdd()) >> 12;
	return VBAT_FROM_PIN_MV(pinMv);
}

uint16_t GetAverageBatteryVoltage(void) {
	uint32_t sum = GetSampleSum(ADC_VBAT_SENS_CHN_IDX, VBAT_AVERAGE_FRAMES);
	uint32_t pinMv = (sum * AnalogGetAvdd()) >> VBAT_AVERAGE_SHIFT;
	return VBAT_FROM_PIN_MV(pinMv);
}

int32_t mcuTemperature = 25; // will contain the mcuTemperature in degree Celsius

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
  // TODO - some sort of clever check and initialization of a resistor soldered onto the board:
  {
    sConfig.Channel = ADC_CHANNEL_5;
    sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
    sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
    if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
    {
      APP_ERROR(APP_HAL_ERROR);
    }

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
    sConfig.Channel = ADC_CHANNEL_5; // CHG_CUR
    sConfig.Rank = ADC_RANK_NONE;
    if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
    {
      APP_ERROR(APP_HAL_ERROR);
    }
  }


  {
  sConfig.Channel = ADC_CHANNEL_17; // Vref
  sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
  sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5; //ADC_SAMPLETIME_28CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    APP_ERROR(APP_HAL_ERROR);
  }

  HAL_ADC_Start(&hadc);
  if (HAL_ADC_PollForConversion(&hadc, ADC_SEQUENCE_TIMEOUT_MS) != HAL_OK)
  {
    APP_ERROR(APP_HAL_ERROR); // s_AVDD below would divide by a stale reading
  }
  vRefAdc = HAL_ADC_GetValue(&hadc);
  HAL_ADC_Stop(&hadc);
  /*sConfig.Channel = ADC_CHANNEL_17; //
   sConfig.Rank = ADC_RANK_NONE;
   if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
   {
   APP_ERROR(APP_HAL_ERROR);
   }*/

  s_AVDD = ANALOG_ADC_GET_AVDD(vRefAdc);
  }

  sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
  sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5; //ADC_SAMPLETIME_28CYCLES_5;
  /**Configure for the selected ADC regular channel to be converted.
   */
  sConfig.Channel = ADC_CHANNEL_0; // CS1
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    APP_ERROR(APP_HAL_ERROR);
  }

  /**Configure for the selected ADC regular channel to be converted.
   */
  sConfig.Channel = ADC_CHANNEL_1; // CS2
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    APP_ERROR(APP_HAL_ERROR);
  }

  /**Configure for the selected ADC regular channel to be converted.
   */
  sConfig.Channel = ADC_CHANNEL_2;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    APP_ERROR(APP_HAL_ERROR);
  }

  /*
   * CH3 (PA3, battery NTC) is not scanned: battery temperature is not going to be measured
   * through the ADC. The pin stays in analog mode, the thermistor path in the fuel gauge
   * driver falls back to the MCU temperature sensor.
   */

  /**Configure for the selected ADC regular channel to be converted.
   */
  sConfig.Channel = ADC_CHANNEL_4; // POW_DET_SEN
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    APP_ERROR(APP_HAL_ERROR);
  }

  /*sConfig.Channel = ADC_CHANNEL_6;
   if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
   {
   APP_ERROR(APP_HAL_ERROR);
   }*/

  /*
   * TODO: CH7 (PA7, IO1) is not scanned - the analog-input mode of the user IO pins was
   * removed. Restoring it means putting the channel back here and reinstating the read in
   * IoRead(); note that IO2 (PA8) has no ADC channel at all on this part.
   */

  sConfig.Channel = ADC_CHANNEL_16; // temperature
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    APP_ERROR(APP_HAL_ERROR);
  }

  /*
   * CHSELR now defines both the scan order and the frame layout: one converted channel per
   * set bit, ascending. Any mismatch with ADC_SCAN_CHANNELS silently shifts every
   * ADC_*_CHN_IDX above the offending channel.
   */
  assert_param(__builtin_popcount(hadc.Instance->CHSELR) == ADC_SCAN_CHANNELS);

  // make bufer data invalid
  analogIn[0] = ADC_SAMPLE_SENTINEL;
  analogIn[ADC_BUFFER_LENGTH - 1] = ADC_SAMPLE_SENTINEL;

  // Start conversion in DMA mode
  if (HAL_ADC_Start_DMA(&hadc, (uint32_t*) analogIn, ADC_BUFFER_LENGTH)
      != HAL_OK)
  {
    APP_ERROR(APP_HAL_ERROR);
  }

  MS_TIME_COUNTER_INIT(tempCalcCounter);
}

/*
 * One-shot board-revision probe. PA1 (ADC_CS2_CHN_IDX) carries the NCS213 current-sense amp
 * output on >=2.3 hardware (idles near 1570 counts while PA0/5V-sense sits ~3100) and a
 * plain shunt tap that tracks PA0 within a few counts on <2.3. A single comparison with a
 * wide margin (3x the largest shunt drop, 3x below the NCS idle offset) separates the two.
 * Needs the 5V rail up so both taps are energised; retries every task tick until then.
 */
static void DetectHardwareRev(void) {
	if (s_HwRev != HARD_REV_UNKNOWN) return;
	if (!AnalogSamplesReady() || Get5vIoVoltage() <= 4500) return;
	int32_t d = (int32_t)GetSample(ADC_CS1_CHN_IDX) - (int32_t)GetSample(ADC_CS2_CHN_IDX);
	s_HwRev = (d > 500) ? HARD_REV_2_3_AND_ABOVE : HARD_REV_BELOW_2_3;
}

void AnalogTask(void) {

	if (MS_TIME_COUNT(tempCalcCounter) > 2000) {
		int32_t vtemp = (((uint32_t)GetSample(ADC_TEMP_SENS_CHN_IDX)) * s_AVDD * 10) >> 12;
		volatile int32_t v30 = (((uint32_t)*TEMP30_CAL_ADDR ) * 33000) >> 12;
		mcuTemperature = (v30 - vtemp) / 43 + 30; //avg_slope = 4.3
		//mcuTemperature = ((((int32_t)*TEMP30_CAL_ADDR - analogIn[7]) * 767) >> 12) + 30;
		MS_TIME_COUNTER_INIT(tempCalcCounter);
	}
	uint16_t vRefSample = GetSample(ADC_VREF_BUFF_CHN_IDX);
	if (vRefSample != 0) { // zero before DMA has filled the ring; the macro divides by it
		s_AVDD = ANALOG_ADC_GET_AVDD(vRefSample);
	}

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
void AnalogPowerIsGood(void) {
	// get avdd
	if (HAL_IS_BIT_SET(hadc.Instance->CR, ADC_CR_ADSTART)) {
		HAL_ADC_Stop_DMA(&hadc);
	}

	/*sConfig.Channel = ADC_CHANNEL_16; //
	sConfig.Rank = ADC_RANK_NONE;
	if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
	{
		APP_ERROR(APP_HAL_ERROR);
	}*/

	/*sConfig.Channel = ADC_CHANNEL_17; // Vref
	sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
	sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;//ADC_SAMPLETIME_28CYCLES_5;
	if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
	{
		APP_ERROR(APP_HAL_ERROR);
	}*/

	/*
	 * ADC_SEQUENCE_TIMEOUT_MS, not 1: the sequence itself needs ~144 us, but the HAL timeout
	 * is counted in whole 1 ms ticks, so a value of 1 expires anywhere between 0 and 1 ms
	 * depending on where inside the current tick the call lands.
	 */
	HAL_ADC_Start(&hadc);
	if (HAL_ADC_PollForConversion(&hadc, ADC_SEQUENCE_TIMEOUT_MS) == HAL_OK) {
		vRefAdc = HAL_ADC_GetValue(&hadc);
	} // on timeout keep the previous vRefAdc rather than scaling by a stale/zero reading
	HAL_ADC_Stop(&hadc);
	/*sConfig.Channel = ADC_CHANNEL_17; // remove adc ref voltage channel
	sConfig.Rank = ADC_RANK_NONE;
	if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
	{
		APP_ERROR(APP_HAL_ERROR);
	}*/

	/*sConfig.Channel = ADC_CHANNEL_16; // return to temperature channel
	sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
	if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
	{
		APP_ERROR(APP_HAL_ERROR);
	}*/

	s_AVDD = ANALOG_ADC_GET_AVDD(vRefAdc);

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
