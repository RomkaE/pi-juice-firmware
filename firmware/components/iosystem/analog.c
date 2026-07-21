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

//#define ANALOG_GET_VDG_AVG()	(4790 - (((analogIn[3] + analogIn[(uint16_t)ADC_SCAN_CHANNELS*256+3] + analogIn[(uint16_t)ADC_SCAN_CHANNELS*512+3] + analogIn[(uint16_t)ADC_SCAN_CHANNELS*768+3]) * aVdd) >> 13))


static uint16_t aVdd;

extern ADC_HandleTypeDef hadc;
ADC_ChannelConfTypeDef sConfig;

static uint32_t tempCalcCounter;

volatile uint32_t vRefAdc;

uint16_t analogIn[ADC_BUFFER_LENGTH];

uint16_t AnalogGetAvdd(void) {
	return aVdd;
}

static uint8_t hardwareRev = HARD_REV_UNKNOWN;

uint8_t GetHardwareRev(void) {
	return hardwareRev;
}

uint16_t GetSampleVoltage(uint8_t channel) {
    int32_t pos =  __HAL_DMA_GET_COUNTER(hadc.DMA_Handle);
    int32_t ind = (((ADC_BUFFER_LENGTH - pos - 1) * (32768/ADC_SCAN_CHANNELS)) >> 15) * ADC_SCAN_CHANNELS + channel;
	if (ind > (ADC_BUFFER_LENGTH - pos - 1)) ind -= ADC_SCAN_CHANNELS; // check if calculated channel sample is fresh
	if (ind < 0) ind += ADC_BUFFER_LENGTH;
	int16_t indRef = ind + ADC_VREF_BUFF_CHN - channel;
	return (analogIn[ind] * ((uint32_t)*VREFINT_CAL_ADDR ) * 412 / analogIn[indRef]) >>  9;
}

//int16_t testBuf[512] __attribute__((section("no_init")));
//volatile uint16_t testInd = 0;
#if defined LOGGING
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

uint16_t GetAverageBatteryVoltage(uint8_t channel) {
	uint16_t i;
	uint32_t sum = 0;
	for (i = channel; i < ADC_BUFFER_LENGTH; i+=(ADC_BUFFER_LENGTH/8)) {
		sum += analogIn[i];
	}
	return (sum* 4535 / analogIn[ADC_VREF_BUFF_CHN] * ((uint32_t)*VREFINT_CAL_ADDR )) >> 15 ;//(sum*2267) >> 14;
	//return (sum * ((uint32_t)*VREFINT_CAL_ADDR ) * 412 / analogIn[ADC_VREF_BUFF_CHN]) >>  8;
}

int32_t mcuTemperature = 25; // will contain the mcuTemperature in degree Celsius

int16_t Get5vIoVoltage() {
	int16_t adcAvg = (analogIn[0] + /*analogIn[1] +*/ analogIn[ADC_BUFFER_LENGTH/4] /*+ analogIn[ADC_BUFFER_LENGTH/4+1]*/ + analogIn[ADC_BUFFER_LENGTH/2] /*+ analogIn[ADC_BUFFER_LENGTH/2+1]*/ + analogIn[ADC_BUFFER_LENGTH*3/4] /*+ analogIn[ADC_BUFFER_LENGTH*3/4+1]*/) >>2;//>> 3;
	return (aVdd > 3200 && aVdd < 3400) ? (adcAvg * aVdd) >> 11 : (adcAvg * 3300) >> 11;//adcAvg * aVdd / 4096 * 2;
}

uint16_t GetSample(uint8_t channel) {
  int32_t pos =  __HAL_DMA_GET_COUNTER(hadc.DMA_Handle);
  int32_t ind = (((ADC_BUFFER_LENGTH - pos - 1) * (32768/ADC_SCAN_CHANNELS)) >> 15) * ADC_SCAN_CHANNELS + channel;
	if (ind > (ADC_BUFFER_LENGTH - pos - 1)) ind -= ADC_SCAN_CHANNELS; // check if calculated channel sample is fresh
	if (ind < 0) ind += ADC_BUFFER_LENGTH;
	return analogIn[ind];
}

uint8_t AnalogSamplesReady() {
	return analogIn[0] != ADC_SAMPLE_SENTINEL && analogIn[ADC_BUFFER_LENGTH-1] != ADC_SAMPLE_SENTINEL;
}

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
    HAL_ADC_PollForConversion(&hadc, 1);
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
  HAL_ADC_PollForConversion(&hadc, 1);
  vRefAdc = HAL_ADC_GetValue(&hadc);
  HAL_ADC_Stop(&hadc);
  /*sConfig.Channel = ADC_CHANNEL_17; //
   sConfig.Rank = ADC_RANK_NONE;
   if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
   {
   APP_ERROR(APP_HAL_ERROR);
   }*/

  aVdd = ANALOG_ADC_GET_AVDD(vRefAdc);
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

  /**Configure for the selected ADC regular channel to be converted.
   */
  sConfig.Channel = ADC_CHANNEL_3; // NTC
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    APP_ERROR(APP_HAL_ERROR);
  }

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

  sConfig.Channel = ADC_CHANNEL_7; // IO1
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    APP_ERROR(APP_HAL_ERROR);
  }

  sConfig.Channel = ADC_CHANNEL_16; // temperature
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    APP_ERROR(APP_HAL_ERROR);
  }

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
 * One-shot board-revision probe. PA1 (ADC_CS2_CHN) carries the NCS213 current-sense amp
 * output on >=2.3 hardware (idles near 1570 counts while PA0/5V-sense sits ~3100) and a
 * plain shunt tap that tracks PA0 within a few counts on <2.3. A single comparison with a
 * wide margin (3x the largest shunt drop, 3x below the NCS idle offset) separates the two.
 * Needs the 5V rail up so both taps are energised; retries every task tick until then.
 */
static void DetectHardwareRev(void) {
	if (hardwareRev != HARD_REV_UNKNOWN) return;
	if (!AnalogSamplesReady() || Get5vIoVoltage() <= 4500) return;
	int32_t d = (int32_t)GetSample(ADC_CS1_CHN) - (int32_t)GetSample(ADC_CS2_CHN);
	hardwareRev = (d > 500) ? HARD_REV_2_3_AND_ABOVE : HARD_REV_BELOW_2_3;
}

void AnalogTask(void) {

	if (MS_TIME_COUNT(tempCalcCounter) > 2000) {
		int32_t vtemp = (((uint32_t)analogIn[ADC_TEMP_SENS_CHN]) * aVdd * 10) >> 12;
		volatile int32_t v30 = (((uint32_t)*TEMP30_CAL_ADDR ) * 33000) >> 12;
		mcuTemperature = (v30 - vtemp) / 43 + 30; //avg_slope = 4.3
		//mcuTemperature = ((((int32_t)*TEMP30_CAL_ADDR - analogIn[7]) * 767) >> 12) + 30;
		MS_TIME_COUNTER_INIT(tempCalcCounter);
	}
	aVdd = ANALOG_ADC_GET_AVDD(GetSample(ADC_VREF_BUFF_CHN));

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

	HAL_ADC_Start(&hadc);
	HAL_ADC_PollForConversion(&hadc, 1);
	vRefAdc = HAL_ADC_GetValue(&hadc);
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

	aVdd = ANALOG_ADC_GET_AVDD(vRefAdc);

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
