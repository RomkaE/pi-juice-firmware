/*
 * io_control.c
 *
 *  Created on: 23.10.2017.
 *      Author: milan
 */

#include "iosystem/analog.h"
#include "stm32f0xx_hal.h"
#include "nv.h"
#include "main.h"
#include "board.h"

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim14;
TIM_HandleTypeDef *htim;
static GPIO_InitTypeDef gpioInitStruct;
static GPIO_TypeDef *ioPort;   // port of the IO selected by the last pin dispatch
TIM_OC_InitTypeDef sConfigOC;

uint8_t ioConfig[2] __attribute__((section("no_init")));
uint16_t ioParam1[2] __attribute__((section("no_init")));
uint16_t ioParam2[2] __attribute__((section("no_init")));
uint16_t pwmLevel[2] __attribute__((section("no_init")));


/* TIM1 init function */
void MX_TIM1_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig;

  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 7;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 665;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 100;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }

  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }

  // TODO - fix
  extern void HAL_TIM_MspPostInit(TIM_HandleTypeDef* htim);
  HAL_TIM_MspPostInit(&htim1);

  // Start channel 1
  /*if (HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1) != HAL_OK)
  {
    // PWM Generation Error
    Error_Handler();
  }*/

}

/* TIM14 init function */
void MX_TIM14_Init(void)
{

  TIM_OC_InitTypeDef sConfigOC;

  htim14.Instance = TIM14;
  htim14.Init.Prescaler = 7;
  htim14.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim14.Init.Period = 100;
  htim14.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_Base_Init(&htim14) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_TIM_PWM_Init(&htim14) != HAL_OK)
  {
    Error_Handler();
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 50;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim14, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }

  // TODO - fix
  extern void HAL_TIM_MspPostInit(TIM_HandleTypeDef* htim);
  HAL_TIM_MspPostInit(&htim14);

  // Start channel 1
  /*if (HAL_TIM_PWM_Start(&htim14, TIM_CHANNEL_1) != HAL_OK)
  {
    // PWM Generation Error
    Error_Handler();
  }*/

}

void IoConfigure(uint8_t pin) {
	if (pin == 1) {
		gpioInitStruct.Pin = EXT_IO1_PIN;
		gpioInitStruct.Alternate = EXT_IO1_GPIO_AF;
		ioPort = EXT_IO1_PORT;
		htim = &htim14;
	} else if (pin == 2) {
		gpioInitStruct.Pin = EXT_IO2_PIN;
		gpioInitStruct.Alternate = EXT_IO2_GPIO_AF;
		ioPort = EXT_IO2_PORT;
		htim = &htim1;
	} else {
		return;
	}

	if (htim->Instance->BDTR&(TIM_BDTR_MOE)) HAL_TIM_PWM_Stop(htim, TIM_CHANNEL_1);

	switch (ioConfig[pin-1]&0x30) {
	case 0x10: gpioInitStruct.Pull = GPIO_PULLDOWN; break;
	case 0x20: gpioInitStruct.Pull = GPIO_PULLUP; break;
	default: gpioInitStruct.Pull = GPIO_NOPULL; break;
	}

	switch (ioConfig[pin-1]&0x0F) {
	case 0: case 1:
		/*
		 * 0 = not used, 1 = analog input. TODO: mode 1 no longer measures anything (see
		 * IoRead) - both are kept as plain analog mode, which is the right idle state for
		 * an unused pin. Still accepted over I2C so stored configurations keep loading.
		 */
		//HAL_TIM_PWM_Stop(htim, TIM_CHANNEL_1);
		gpioInitStruct.Mode = GPIO_MODE_ANALOG;
	    HAL_GPIO_Init(ioPort, &gpioInitStruct);
	    break;
	case 2:
		// digital input
		//HAL_TIM_PWM_Stop(htim, TIM_CHANNEL_1);
		if ((pin==2) && (ioParam1[pin-1]&0x03) == 1) {
			gpioInitStruct.Mode = GPIO_MODE_IT_FALLING; // on IO1 enable wakeup on falling edge
		} else if ((pin==2) && (ioParam1[pin-1]&0x03) == 2) {
			gpioInitStruct.Mode = GPIO_MODE_IT_RISING; // on IO1 enable wakeup on rising edge
		} else {
			gpioInitStruct.Mode = GPIO_MODE_INPUT;
		}
		HAL_GPIO_Init(ioPort, &gpioInitStruct);
		break;
	case 3:
		// digital output, push-pull
		//HAL_TIM_PWM_Stop(htim, TIM_CHANNEL_1);
		gpioInitStruct.Mode = GPIO_MODE_OUTPUT_PP;
		gpioInitStruct.Speed = GPIO_SPEED_FREQ_LOW;
		HAL_GPIO_WritePin(ioPort, gpioInitStruct.Pin, ioParam1[pin-1]&0x01 ? GPIO_PIN_SET : GPIO_PIN_RESET);
		HAL_GPIO_Init(ioPort, &gpioInitStruct);
		break;
	case 4:
		// digital output, open drain
		gpioInitStruct.Mode = GPIO_MODE_OUTPUT_OD;
		gpioInitStruct.Speed = GPIO_SPEED_FREQ_LOW;
		HAL_GPIO_WritePin(ioPort, gpioInitStruct.Pin, ioParam1[pin-1]&0x01 ? GPIO_PIN_SET : GPIO_PIN_RESET);
		HAL_GPIO_Init(ioPort, &gpioInitStruct);
		break;
	case 5:
		// pus = arr*psc/8, arr = 65535, pus = 8192*psc, psc = pus/8192, arr = pus*8/psc
		// pwm output
		htim->Instance->PSC = ioParam1[pin-1] < 4096 ? 0 : (((uint32_t)ioParam1[pin-1])/4096);
		htim->Instance->ARR = ((uint32_t)ioParam1[pin-1]+1)*16/(htim->Instance->PSC+1)-1;
		//HAL_TIM_Base_Init(&htim1);
		htim->Instance->CCR1 = ioParam2[pin-1] == 65535 ? 65535 : (uint32_t)htim->Instance->ARR*ioParam2[pin-1]/65534;
		pwmLevel[pin-1] = ioParam2[pin-1];
		gpioInitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
		// push-pull
		gpioInitStruct.Mode = GPIO_MODE_AF_PP;
		HAL_GPIO_Init(ioPort, &gpioInitStruct);
		//gpioInitStruct.Pin = EXT_IO1_PIN;
		//gpioInitStruct.Alternate = EXT_IO2_GPIO_AF;
		//HAL_GPIO_Init(ioPort, &gpioInitStruct);
		HAL_TIM_PWM_Start(htim, TIM_CHANNEL_1); // Start channel 1
		//HAL_TIMEx_PWMN_Start(htim, TIM_CHANNEL_1);
		break;
	case 6:
		// pwm output
		htim->Instance->PSC = ioParam1[pin-1] < 4096 ? 0 : (((uint32_t)ioParam1[pin-1])/4096);
		htim->Instance->ARR = ((uint32_t)ioParam1[pin-1]+1)*16/(htim->Instance->PSC+1)-1;
		//HAL_TIM_Base_Init(&htim1);
		htim->Instance->CCR1 = ioParam2[pin-1] == 65535 ? 65535 : (uint32_t)htim->Instance->ARR*ioParam2[pin-1]/65534;
		pwmLevel[pin-1] = ioParam2[pin-1];
		gpioInitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
		// push-pull
		gpioInitStruct.Mode = GPIO_MODE_AF_OD;
		HAL_GPIO_Init(ioPort, &gpioInitStruct);
		//gpioInitStruct.Pin = EXT_IO1_PIN;
		//gpioInitStruct.Alternate = EXT_IO2_GPIO_AF;
		//HAL_GPIO_Init(ioPort, &gpioInitStruct);
		HAL_TIM_PWM_Start(htim, TIM_CHANNEL_1); // Start channel 1
		break;
	default:
		//HAL_TIM_PWM_Stop(htim, TIM_CHANNEL_1);
		gpioInitStruct.Mode = GPIO_MODE_ANALOG;
	    HAL_GPIO_Init(ioPort, &gpioInitStruct);
	}
}

void IoNvReadConfig(uint8_t pin) {
	if (nv_read_U8(NV_ADDR_IO_CONFIG1+(pin-1)*3, &ioConfig[pin-1]) != NV_OK) {
		ioConfig[pin-1] = 0x80; // in case of nv write error, set to non configured
		return;
	}
	/* The two parameters are full 16 bit values, so they carry no complement byte. A failed
	 * read leaves them at whatever survived in no_init RAM, as before. */
	(void)nv_read_U16(NV_ADDR_IO_CONFIG1_PARAM1+(pin-1)*3, &ioParam1[pin-1]);
	(void)nv_read_U16(NV_ADDR_IO_CONFIG1_PARAM2+(pin-1)*3, &ioParam2[pin-1]);
}

void IoControlInit() {
  IoNvReadConfig(1);
	IoNvReadConfig(2);
	IoConfigure(1);
	IoConfigure(2);
}

void IoSetConfiguarion(uint8_t pin, uint8_t data[], uint8_t len) {

	ioConfig[pin-1] = data[0];
	ioParam1[pin-1] = data[2];
	ioParam1[pin-1] <<= 8;
	ioParam1[pin-1] |= data[1];
	ioParam2[pin-1] = data[4];
	ioParam2[pin-1] <<= 8;
	ioParam2[pin-1] |= data[3];

	if (data[0]&0x80) {
		nv_write_U8(NV_ADDR_IO_CONFIG1+(pin-1)*3, data[0]);
		nv_write_U16(NV_ADDR_IO_CONFIG1_PARAM1+(pin-1)*3, ioParam1[pin-1]);
		nv_write_U16(NV_ADDR_IO_CONFIG1_PARAM2+(pin-1)*3, ioParam2[pin-1]);

		IoNvReadConfig(pin);
	}

	IoConfigure(pin);
}

void IoGetConfiguarion(uint8_t pin, uint8_t data[], uint16_t *len) {
	data[0] = ioConfig[pin-1];
	data[1] = ioParam1[pin-1] & 0xFF;
	data[2] = (ioParam1[pin-1] >> 8) & 0xFF;
	data[3] = ioParam2[pin-1] & 0xFF;
	data[4] = (ioParam2[pin-1] >> 8) & 0xFF;
	*len = 5;
}

void IoWrite(uint8_t pin, uint8_t data[], uint8_t len)
{
	uint16_t val;
	if (pin == 1) {
		gpioInitStruct.Pin = EXT_IO1_PIN;
		ioPort = EXT_IO1_PORT;
		htim = &htim14;
	} else if (pin == 2) {
		gpioInitStruct.Pin = EXT_IO2_PIN;
		ioPort = EXT_IO2_PORT;
		htim = &htim1;
	} else {
		return;
	}

	switch (ioConfig[pin-1]&0x0F) {
	case 3: case 4:
		HAL_GPIO_WritePin(ioPort, gpioInitStruct.Pin, data[1]&0x01 ? GPIO_PIN_SET : GPIO_PIN_RESET);
		break;
	case 5: case 6:
		val = data[1];
		val <<= 8;
		val |= data[0];
		htim->Instance->CCR1 = val == 65535 ? 65535 : (uint32_t)htim->Instance->ARR*val/65534;
		pwmLevel[pin-1] = val;
	default:
		break;
	}
}

void IoRead(uint8_t pin, uint8_t data[], uint16_t *len)
{
	if (pin == 1) {
		gpioInitStruct.Pin = EXT_IO1_PIN;
		ioPort = EXT_IO1_PORT;
		htim = &htim14;
	} else if (pin == 2) {
		gpioInitStruct.Pin = EXT_IO2_PIN;
		ioPort = EXT_IO2_PORT;
		htim = &htim1;
	} else {
		return;
	}

	switch (ioConfig[pin-1]&0x0F) {
		case 1:
			/*
			 * TODO: analog input on IO1/IO2 removed - PA7 is no longer in the ADC scan group
			 * and PA8 never had a channel. Mode 1 now reads back 0.
			 */
			data[0] = 0;
			data[1] = 0;
			break;
		case 2:
			data[0] = HAL_GPIO_ReadPin(ioPort, gpioInitStruct.Pin);
			break;
		case 3: case 4:
			data[0] = HAL_GPIO_ReadPin(ioPort, gpioInitStruct.Pin);
			data[1] = (ioPort->ODR & (uint32_t)gpioInitStruct.Pin) == (uint32_t)gpioInitStruct.Pin;
			break;
		case 5: case 6:
			//val = htim->Instance->CCR1 == 65535 ? 65535 : (uint32_t)htim->Instance->CCR1 * 65535 / htim->Instance->ARR;
			data[0] = pwmLevel[pin-1]&0xFF;
			data[1] = (pwmLevel[pin-1] >> 8) & 0xFF;
			break;
		default:
			break;
	}
	*len = 2;
}
