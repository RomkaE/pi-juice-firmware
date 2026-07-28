/*
 * led.c
 *
 *  Created on: 06.12.2016.
 *      Author: milan
 */

#include <to_refactor/time_count.h>
#include "led.h"
#include "stm32f0xx_hal.h"
#include "nv.h"
#include "main.h"


extern TIM_HandleTypeDef htim3;
//GPIO_InitTypeDef GPIO_InitStruct;

typedef struct
{
	LedFunction_T func;
	uint8_t paramR;
	uint8_t paramG;
	uint8_t paramB;

	uint8_t r;
	uint8_t g;
	uint8_t b;

	uint8_t blinkRepeat;

	uint8_t blinkR1;
	uint8_t blinkG1;
	uint8_t blinkB1;
	uint16_t blinkPeriod1;

	uint8_t blinkR2;
	uint8_t blinkG2;
	uint8_t blinkB2;
	uint16_t blinkPeriod2;

	uint16_t blinkCount;
	uint32_t blinkTimer;

} Led_T;

static Led_T s_LED = { LED_CHARGE_STATUS, 60, 60, 100};

static const uint16_t pwm_table[] = {
     65535,    65508,    65479,    65451,    65422,    65394,    65365,    65337,
     65308,    65280,    65251,    65223,    65195,    65166,    65138,    65109,
     65081,    65052,    65024,    64995,    64967,    64938,    64909,    64878,
     64847,    64815,    64781,    64747,    64711,    64675,    64637,    64599,
     64559,    64518,    64476,    64433,    64389,    64344,    64297,    64249,
     64200,    64150,    64099,    64046,    63992,    63937,    63880,    63822,
     63763,    63702,    63640,    63577,    63512,    63446,    63379,    63310,
     63239,    63167,    63094,    63019,    62943,    62865,    62785,    62704,
     62621,    62537,    62451,    62364,    62275,    62184,    62092,    61998,
     61902,    61804,    61705,    61604,    61501,    61397,    61290,    61182,
     61072,    60961,    60847,    60732,    60614,    60495,    60374,    60251,
     60126,    59999,    59870,    59739,    59606,    59471,    59334,    59195,
     59053,    58910,    58765,    58618,    58468,    58316,    58163,    58007,
     57848,    57688,    57525,    57361,    57194,    57024,    56853,    56679,
     56503,    56324,    56143,    55960,    55774,    55586,    55396,    55203,
     55008,    54810,    54610,    54408,    54203,    53995,    53785,    53572,
     53357,    53140,    52919,    52696,    52471,    52243,    52012,    51778,
     51542,    51304,    51062,    50818,    50571,    50321,    50069,    49813,
     49555,    49295,    49031,    48764,    48495,    48223,    47948,    47670,
     47389,    47105,    46818,    46529,    46236,    45940,    45641,    45340,
     45035,    44727,    44416,    44102,    43785,    43465,    43142,    42815,
     42486,    42153,    41817,    41478,    41135,    40790,    40441,    40089,
     39733,    39375,    39013,    38647,    38279,    37907,    37531,    37153,
     36770,    36385,    35996,    35603,    35207,    34808,    34405,    33999,
     33589,    33175,    32758,    32338,    31913,    31486,    31054,    30619,
     30181,    29738,    29292,    28843,    28389,    27932,    27471,    27007,
     26539,    26066,    25590,    25111,    24627,    24140,    23649,    23153,
     22654,    22152,    21645,    21134,    20619,    20101,    19578,    19051,
     18521,    17986,    17447,    16905,    16358,    15807,    15252,    14693,
     14129,    13562,    12990,    12415,    11835,    11251,    10662,    10070,
     9473,    8872,    8266,    7657,    7043,    6424,    5802,    5175,
     4543,    3908,    3267,    2623,    1974,    1320,    662,    0
 };

void LedInit(void)
{
  // Start LED_D1 red (TIM3_CH1)
  if (HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1) != HAL_OK)
  {
    // PWM generation Error
    Error_Handler();
  }

  // Start LED_D1 green (TIM3_CH2)
  if (HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2) != HAL_OK)
  {
    // PWM generation Error
    Error_Handler();
  }

  // Start LED_D1 blue (TIM3_CH3)
  if (HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3) != HAL_OK)
  {
    // PWM generation Error
    Error_Handler();
  }

	uint16_t var = 0;
	EE_ReadVariable(NV_LED_D1_FUNC, &var);
	if (((~var)&0xFF) == (var>>8)) {
		s_LED.func = var&0xFF;
	}
	EE_ReadVariable(NV_LED_D1_PARAM_R, &var);
	if (((~var)&0xFF) == (var>>8)) {
		s_LED.paramR = var&0xFF;
	}
	EE_ReadVariable(NV_LED_D1_PARAM_G, &var);
	if (((~var)&0xFF) == (var>>8)) {
		s_LED.paramG = var&0xFF;
	}
	EE_ReadVariable(NV_LED_D1_PARAM_B, &var);
	if (((~var)&0xFF) == (var>>8)) {
		s_LED.paramB = var&0xFF;
	}
	if (s_LED.func == LED_USER_LED) {
		LedSetRGB(LED_D1, s_LED.paramR, s_LED.paramG, s_LED.paramB);
	}else {
		LedSetRGB(LED_D1, 0, 0, 0);
	}
}

uint8_t LedGetParamR(uint8_t func) {
	if (s_LED.func == func)
		return s_LED.paramR;
	return 0;
}
uint8_t LedGetParamG(uint8_t func) {
	if (s_LED.func == func)
		return s_LED.paramG;
	return 0;
}
uint8_t LedGetParamB(uint8_t func) {
	if (s_LED.func == func)
		return s_LED.paramB;
	return 0;
}

static void ProcessBlink(void) {
	if (s_LED.blinkCount) {
		if ( (s_LED.blinkCount & 0x1) ) {
			// odd count for off color
			if (MS_TIME_COUNT(s_LED.blinkTimer) >= s_LED.blinkPeriod2) {
				MS_TIME_COUNTER_INIT(s_LED.blinkTimer);
				s_LED.blinkCount --;
				if (s_LED.blinkCount == 0) {
					if (s_LED.blinkRepeat == 0xFF) {
						s_LED.blinkCount = 256;
					} else {
						LedSetRGB(LED_D1, s_LED.r, s_LED.g, s_LED.b);
					}
				} else
					LedSetRGB(LED_D1, s_LED.blinkR1, s_LED.blinkG1, s_LED.blinkB1);
			}
		} else {
			// even count for on color
			if (MS_TIME_COUNT(s_LED.blinkTimer) >= s_LED.blinkPeriod1) {
				LedSetRGB(LED_D1, s_LED.blinkR2, s_LED.blinkG2, s_LED.blinkB2);
				MS_TIME_COUNTER_INIT(s_LED.blinkTimer);
				s_LED.blinkCount --;
			}
		}
	}
}

void LedTask(void) {
	ProcessBlink();
}
void LedSetRGB(uint8_t led, uint8_t r, uint8_t g, uint8_t b) {
  if (led == LED_D1)
  {
    TIM3->CCR1 = 65535 - pwm_table[r];
    TIM3->CCR2 = 65535 - pwm_table[g];
    TIM3->CCR3 = 65535 - pwm_table[b];
  }
  else
  {
    // TODO - error logs
  }
}

void LedFunctionSetRGB(LedFunction_T func, uint8_t r, uint8_t g, uint8_t b) {
	if (s_LED.func == func) {
		s_LED.r = r;
		s_LED.g = g;
		s_LED.b = b;
		s_LED.blinkRepeat = 0;
		s_LED.blinkCount = 0;
		LedSetRGB(LED_D1, r, g, b);
	}
}

void LedStop(void) {
	if ((htim3.Instance->CR1 & TIM_CR1_CEN) == TIM_CR1_CEN) {
		HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
		HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_2);
		HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_3);
	}
}

void LedStart(void) {
	if ((htim3.Instance->CR1 & TIM_CR1_CEN) == 0)
		LedInit();
}

void LedSetConfiguarion(uint8_t led, uint8_t data[], uint8_t len) {
	uint16_t var = 0;
	if (led == LED_D1) {
		NvWriteVariableU8(NV_LED_D1_FUNC, data[0]);
		NvWriteVariableU8(NV_LED_D1_PARAM_R, data[1]);
		NvWriteVariableU8(NV_LED_D1_PARAM_G, data[2]);
		NvWriteVariableU8(NV_LED_D1_PARAM_B, data[3]);

		EE_ReadVariable(NV_LED_D1_FUNC, &var);
		if (((~var)&0xFF) == (var>>8)) {
			s_LED.func = var&0xFF;
		}
		EE_ReadVariable(NV_LED_D1_PARAM_R, &var);
		if (((~var)&0xFF) == (var>>8)) {
			s_LED.paramR = var&0xFF;
		}
		EE_ReadVariable(NV_LED_D1_PARAM_G, &var);
		if (((~var)&0xFF) == (var>>8)) {
			s_LED.paramG = var&0xFF;
		}
		EE_ReadVariable(NV_LED_D1_PARAM_B, &var);
		if (((~var)&0xFF) == (var>>8)) {
			s_LED.paramB = var&0xFF;
		}
		if (s_LED.func == LED_USER_LED) {
			LedSetRGB(LED_D1, s_LED.paramR, s_LED.paramG, s_LED.paramB);
		}else {
			LedSetRGB(LED_D1, 0, 0, 0);
		}
	}
	else
	{
	  // TODO - error logs
	}
}

/*
 * Only D1 is populated, so the second LED's registers (0x103, 0x105, 0x107) answer with
 * zeros of the expected length. Returning without touching *len is not an option: the
 * command server checksums whatever length the caller pre-set - 1 byte, see i2c_slave.c -
 * so the host would get a stale byte carrying a valid checksum. Same approach as
 * CmdServerReadRetiredU16() takes for the retired current registers.
 */
static void LedReadUnused(uint8_t data[], uint16_t *len, uint16_t n) {
  for (uint16_t i = 0; i < n; i++) data[i] = 0;
  *len = n;
}

void LedGetConfiguarion(uint8_t led, uint8_t data[], uint16_t *len) {
	if (led > LED_D1) { LedReadUnused(data, len, 4); return; }
	data[0] = s_LED.func;
	data[1] = s_LED.paramR;
	data[2] = s_LED.paramG;
	data[3] = s_LED.paramB;
	*len = 4;
}

void LedCmdSetState(uint8_t led, uint8_t data[], uint8_t len) {
	if (led > LED_D1 || s_LED.func != LED_USER_LED) return;
	s_LED.r = data[0];
	s_LED.g = data[1];
	s_LED.b = data[2];
	LedSetRGB(LED_D1, data[0], data[1], data[2]);
}

void LedCmdGetState(uint8_t led, uint8_t data[], uint16_t *len) {
	if (led > LED_D1) { LedReadUnused(data, len, 3); return; }
	data[0] = s_LED.r;
	data[1] = s_LED.g;
	data[2] = s_LED.b;
	*len = 3;
}

void LedCmdSetBlink(uint8_t led, uint8_t data[], uint8_t len) {
	if (led > LED_D1 || s_LED.func == LED_NOT_USED) return;

	s_LED.blinkRepeat = data[0];
	s_LED.blinkCount = data[0];
	s_LED.blinkCount <<= 1; // *=2
	s_LED.blinkR1 = data[1];
	s_LED.blinkG1 = data[2];
	s_LED.blinkB1 = data[3];
	s_LED.blinkPeriod1 = data[4];
	s_LED.blinkPeriod1 *= 10;
	s_LED.blinkR2 = data[5];
	s_LED.blinkG2 = data[6];
	s_LED.blinkB2 = data[7];
	s_LED.blinkPeriod2 = data[8];
	s_LED.blinkPeriod2 *= 10;


	if (s_LED.blinkRepeat) {
		LedSetRGB(LED_D1, s_LED.blinkR1, s_LED.blinkG1, s_LED.blinkB1);
		MS_TIME_COUNTER_INIT(s_LED.blinkTimer);
	} else {
		LedSetRGB(LED_D1, s_LED.r, s_LED.g, s_LED.b);
	}

}

void LedCmdGetBlink(uint8_t led, uint8_t data[], uint16_t *len) {
	if (led > LED_D1) { LedReadUnused(data, len, 9); return; }

	data[0] = s_LED.blinkRepeat < 255 ? s_LED.blinkCount >> 1 : 255;
	data[1] = s_LED.blinkR1;
	data[2] = s_LED.blinkG1;
	data[3] = s_LED.blinkB1;
	data[4] = s_LED.blinkPeriod1 / 10;
	data[5] = s_LED.blinkR2;
	data[6] = s_LED.blinkG2;
	data[7] = s_LED.blinkB2;
	data[8] = s_LED.blinkPeriod2 / 10;
	*len = 9;
}
