/*
 * button.c
 *
 *  Created on: 28.03.2017.
 *      Author: milan
 */

#include <to_refactor/button.h>
#include <to_refactor/time_count.h>
#include "stm32f0xx_hal.h"
#include "board.h"
#include "nv.h"


typedef struct
{
	ButtonFunction_T pressFunc;
	uint16_t pressConfig; // reserved
	ButtonFunction_T releaseFunc;
	uint16_t releaseConfig; // reserved
	ButtonFunction_T singlePressFunc;
	uint16_t singlePressTime; // 0- reserved, 1 - 255  time
	ButtonFunction_T doublePressFunc;
	uint16_t doublePressTime; // 0- reserved, 1 - 255  time
	ButtonFunction_T longPressFunc1;
	uint16_t longPressTime1;
	ButtonFunction_T longPressFunc2;
	uint16_t longPressTime2;
	GPIO_PinState state; // current press state, GPIO_PIN_RESET for release, GPIO_PIN_SET for press
	ButtonEvent_T event;
	ButtonEvent_T tempEvent;
	uint32_t pressTimer;
	uint8_t staticLongPressEvent;
} Button_T;

Button_T buttons[3] = {
	{ // sw1
		BUTTON_EVENT_NO_FUNC,
		0,
		BUTTON_EVENT_NO_FUNC,
		0,
		BUTTON_EVENT_FUNC_POWER_ON,
		800,
		BUTTON_EVENT_NO_FUNC,
		0,
		BUTTON_EVENT_FUNC_SYS_EVENT|1,
		10000,
		BUTTON_EVENT_FUNC_POWER_OFF,
		20000
	},
	{ // sw2
		BUTTON_EVENT_NO_FUNC,
		0,
		BUTTON_EVENT_NO_FUNC,
		0,
		BUTTON_EVENT_FUNC_USER_EVENT|1,
		400,
		BUTTON_EVENT_FUNC_USER_EVENT|2,
		600,
		BUTTON_EVENT_NO_FUNC,
		0,
		BUTTON_EVENT_NO_FUNC,
		0
	},
	{ // sw3
		BUTTON_EVENT_FUNC_USER_EVENT|3,
		0,
		BUTTON_EVENT_FUNC_USER_EVENT|4,
		0,
		BUTTON_EVENT_NO_FUNC,
		0,
		BUTTON_EVENT_NO_FUNC,
		0,
		BUTTON_EVENT_NO_FUNC,
		0,
		BUTTON_EVENT_NO_FUNC,
		0
	}
};

ButtonEventCb_T buttonEventCbs[BUTTON_EVENT_FUNC_NUMBER] = {
	NULL, // BUTTON_EVENT_NO_FUNC
	PowerOnButtonEventCb, // BUTTON_EVENT_FUNC_POWER_ON
	PowerOffButtonEventCb, // BUTTON_EVENT_FUNC_POWER_OFF
	ButtonEventFuncPowerResetCb, // BUTTON_EVENT_FUNC_POWER_RESET
};

static int8_t writebuttonConfigData = -1;
Button_T buttonConfigData;

static ButtonFunction_T GetFuncOfEvent(uint8_t b) {
	switch (buttons[b].event) {
	case BUTTON_EVENT_PRESS:
		return buttons[b].pressFunc;
	case BUTTON_EVENT_RELEASE:
		return buttons[b].releaseFunc;
	case BUTTON_EVENT_SINGLE_PRESS:
		return buttons[b].singlePressFunc;
	case BUTTON_EVENT_DOUBLE_PRESS:
		return buttons[b].doublePressFunc;
	case BUTTON_EVENT_LONG_PRESS1:
		return buttons[b].longPressFunc1;
	case BUTTON_EVENT_LONG_PRESS2:
		return buttons[b].longPressFunc2;
	default:
		return BUTTON_EVENT_NO_FUNC;
	}
}

/*__STATIC_INLINE*/ void ProcessButton( uint8_t b, GPIO_PinState pinState ) {
	volatile ButtonEvent_T oldEv = buttons[b].event;

	if ( pinState != buttons[b].state ) {
		if ( pinState == GPIO_PIN_SET ) {
			if (MS_TIME_COUNT(buttons[b].pressTimer) > 30000) {
				// 30 seconds event timeout, remove it
				buttons[b].tempEvent = 0;
				buttons[b].event = 0;
				oldEv = 0;
			}
			if ( buttons[b].doublePressTime && buttons[b].doublePressFunc!=BUTTON_EVENT_NO_FUNC && MS_TIME_COUNT(buttons[b].pressTimer)  < buttons[b].doublePressTime ) {
				buttons[b].tempEvent = BUTTON_EVENT_DOUBLE_PRESS;
				buttons[b].event = BUTTON_EVENT_DOUBLE_PRESS;
			} else if ( buttons[b].pressFunc != BUTTON_EVENT_NO_FUNC ) {
				buttons[b].tempEvent = BUTTON_EVENT_PRESS;
				buttons[b].event = BUTTON_EVENT_PRESS;
			}
			MS_TIME_COUNTER_INIT(buttons[b].pressTimer);
		} else {
			// if release
			if (buttons[b].tempEvent != BUTTON_EVENT_DOUBLE_PRESS) {
				if ( buttons[b].singlePressTime && buttons[b].singlePressFunc!=BUTTON_EVENT_NO_FUNC && MS_TIME_COUNT(buttons[b].pressTimer) < buttons[b].singlePressTime ) {
					buttons[b].tempEvent = BUTTON_EVENT_SINGLE_PRESS;
					if ( buttons[b].doublePressFunc == BUTTON_EVENT_NO_FUNC ) buttons[b].event = BUTTON_EVENT_SINGLE_PRESS;
				} else if ( buttons[b].releaseFunc != BUTTON_EVENT_NO_FUNC ) {
					buttons[b].tempEvent = BUTTON_EVENT_RELEASE;
					if ( buttons[b].doublePressFunc == BUTTON_EVENT_NO_FUNC ) buttons[b].event = BUTTON_EVENT_RELEASE;
				}
			}
			buttons[b].staticLongPressEvent = 0;
		}
		buttons[b].state = pinState;
	} else if ( pinState == GPIO_PIN_SET ) {
		uint32_t timePased = MS_TIME_COUNT(buttons[b].pressTimer);
		if ( buttons[b].longPressFunc2 != BUTTON_EVENT_NO_FUNC && timePased > buttons[b].longPressTime2 ) {
			if ( buttons[b].tempEvent != BUTTON_EVENT_LONG_PRESS2 ) {
				buttons[b].tempEvent = BUTTON_EVENT_LONG_PRESS2;
				buttons[b].event = BUTTON_EVENT_LONG_PRESS2;
			}
		} else if ( buttons[b].longPressFunc1 != BUTTON_EVENT_NO_FUNC && timePased > buttons[b].longPressTime1 ) {
			if ( buttons[b].tempEvent != BUTTON_EVENT_LONG_PRESS1 ) {
				buttons[b].tempEvent = BUTTON_EVENT_LONG_PRESS1;
				buttons[b].event = BUTTON_EVENT_LONG_PRESS1;
			}
		}
		if (timePased > BUTTON_STATIC_LONG_PRESS_TIME) {
			buttons[b].staticLongPressEvent = 1;
		}
	} else if ( buttons[b].tempEvent && pinState == GPIO_PIN_RESET && buttons[b].doublePressFunc!=BUTTON_EVENT_NO_FUNC  && MS_TIME_COUNT(buttons[b].pressTimer)  > buttons[b].doublePressTime ) {
		// generate single press event only if there was no double press
		if ( buttons[b].tempEvent == BUTTON_EVENT_SINGLE_PRESS ) {
			buttons[b].event = BUTTON_EVENT_SINGLE_PRESS;
		} else if ( buttons[b].tempEvent == BUTTON_EVENT_RELEASE ) {
			buttons[b].event = BUTTON_EVENT_RELEASE;
		}
		buttons[b].tempEvent = 0;
	}

	if ( buttons[b].event  > oldEv ) {
	    volatile ButtonFunction_T func = GetFuncOfEvent(b);
		if ( func < BUTTON_EVENT_FUNC_NUMBER && buttonEventCbs[func] != NULL ) {
			buttonEventCbs[func](b, buttons[b].event);
		}
	}
}

/* NV offsets of the ten configuration bytes within one button's block, relative to its
 * BUTTON_PRESS_FUNC_SWx entry. The gaps are the reserved *_CONFIG slots of the press and
 * release events, which are never stored. Shared by the read and the write path below. */
static const uint8_t buttonNvOffsets[10] = { 0, 2, 4, 5, 6, 7, 8, 9, 10, 11 };

static uint8_t ButtonReadConfigurationNv(uint8_t b) {
	uint8_t nvOffset = b * (NV_ADDR_BUTTON_PRESS_FUNC_SW2 - NV_ADDR_BUTTON_PRESS_FUNC_SW1) + NV_ADDR_BUTTON_PRESS_FUNC_SW1;
	uint8_t val[10];

	/* Nothing is applied unless the whole block reads back valid, so bail out on the first
	 * failure - the caller discards buttonConfigData on a non zero result anyway. */
	for (uint8_t i = 0; i < 10; i++) {
		if (nv_read_U8(nvOffset + buttonNvOffsets[i], &val[i]) != NV_OK)
			return 1;
	}

	buttonConfigData.pressFunc       = val[0];
	buttonConfigData.releaseFunc     = val[1];
	buttonConfigData.singlePressFunc = val[2];
	buttonConfigData.singlePressTime = val[3];
	buttonConfigData.doublePressFunc = val[4];
	buttonConfigData.doublePressTime = val[5];
	buttonConfigData.longPressFunc1  = val[6];
	buttonConfigData.longPressTime1  = val[7];
	buttonConfigData.longPressFunc2  = val[8];
	buttonConfigData.longPressTime2  = val[9];

	return 0;
}

static void ButtonSetConfigData(uint8_t b) {
	buttons[b].pressFunc = buttonConfigData.pressFunc;
	buttons[b].releaseFunc = buttonConfigData.releaseFunc;
	buttons[b].singlePressFunc = buttonConfigData.singlePressFunc;
	buttons[b].singlePressTime = buttonConfigData.singlePressTime * 100;
	buttons[b].doublePressFunc = buttonConfigData.doublePressFunc;
	buttons[b].doublePressTime = buttonConfigData.doublePressTime * 100;
	buttons[b].longPressFunc1 = buttonConfigData.longPressFunc1;
	buttons[b].longPressTime1 = buttonConfigData.longPressTime1 * 100;
	buttons[b].longPressFunc2 = buttonConfigData.longPressFunc2;
	buttons[b].longPressTime2 = buttonConfigData.longPressTime2 * 100;
}

void ButtonInit(void) {
	if ( ButtonReadConfigurationNv(0) == 0 ) {
		ButtonSetConfigData(0);
	}

	if ( ButtonReadConfigurationNv(1) == 0 ) {
		ButtonSetConfigData(1);
	}

	if ( ButtonReadConfigurationNv(2) == 0 ) {
		ButtonSetConfigData(2);
	}
}

int8_t IsButtonActive(void) {
	//return buttons[0].tempEvent || buttons[1].tempEvent || buttons[2].tempEvent;
	return MS_TIME_COUNT(buttons[0].pressTimer) < 2000 || MS_TIME_COUNT(buttons[1].pressTimer) < 2000 || MS_TIME_COUNT(buttons[2].pressTimer) < 2000
			|| buttons[0].state || buttons[1].state || buttons[2].state;
}

void ButtonTask(void) {

	uint8_t oldDualLongPressStatus = buttons[0].staticLongPressEvent && buttons[1].staticLongPressEvent;

	/*
	 * TODO(hw): index 0 is SW1 for the host - command 0x110 maps straight to
	 * ButtonSetConfiguarion(0, ...) - and buttons[0] carries the POWER_ON/POWER_OFF
	 * defaults. But index 0 is read from SW2, and SW1, the only button actually
	 * populated, lands on index 1 whose defaults are USER_EVENT only.
	 *
	 * So either the schematic labels are swapped relative to this mapping, or the
	 * populated button genuinely cannot power the board on or off. Needs a hardware
	 * check before touching - swapping the two lines below changes behaviour.
	 */
	ProcessButton(0, HAL_GPIO_ReadPin(BTN_SW2_PORT, BTN_SW2_PIN)); // host-facing sw1

	ProcessButton(1, HAL_GPIO_ReadPin(BTN_SW1_PORT, BTN_SW1_PIN)); // host-facing sw2

	ProcessButton(2, HAL_GPIO_ReadPin(BTN_SW3_PORT, BTN_SW3_PIN));  // host-facing sw3

	if ((buttons[0].staticLongPressEvent && buttons[1].staticLongPressEvent) > oldDualLongPressStatus) ButtonDualLongPressEventCb();

	if (writebuttonConfigData >= 0) {
		uint8_t nvOffset = writebuttonConfigData * (NV_ADDR_BUTTON_PRESS_FUNC_SW2 - NV_ADDR_BUTTON_PRESS_FUNC_SW1) + NV_ADDR_BUTTON_PRESS_FUNC_SW1;
		const uint8_t val[10] = {
			buttonConfigData.pressFunc,       buttonConfigData.releaseFunc,
			buttonConfigData.singlePressFunc, buttonConfigData.singlePressTime,
			buttonConfigData.doublePressFunc, buttonConfigData.doublePressTime,
			buttonConfigData.longPressFunc1,  buttonConfigData.longPressTime1,
			buttonConfigData.longPressFunc2,  buttonConfigData.longPressTime2
		};
		for (uint8_t i = 0; i < 10; i++)
			nv_write_U8(nvOffset + buttonNvOffsets[i], val[i]);

		if ( ButtonReadConfigurationNv(writebuttonConfigData) == 0 ) {
			ButtonSetConfigData(writebuttonConfigData);
		}
		writebuttonConfigData = -1;
	}
}

ButtonEvent_T GetButtonEvent(uint8_t b) {
	ButtonEvent_T event = buttons[b].event;
	//buttons[b].event = 0;
	return event;
}

void ButtonRemoveEvent(uint8_t b) {
	buttons[b].event = 0;
}

uint8_t IsButtonEvent(void) {
	return buttons[0].event || buttons[1].event || buttons[2].event;
}

void ButtonSetConfiguarion(uint8_t b, uint8_t data[], uint8_t len) {
	if (b > 3) return;
	buttonConfigData.pressFunc = data[0];
	/*if ( buttonConfigData.func >= BUTTON_FUNC_NUMBER  ) return;
	if ( buttonConfigData.func != BUTTON_FUNC_POWER_BUTTON && buttons[b].func == BUTTON_FUNC_POWER_BUTTON ) {
		// ensure this will not override existing power button, if only one defined
		int i, pwrBtnCnt = 0;
		for (i=0;i<3;i++) {
			if (buttons[i].func == BUTTON_FUNC_POWER_BUTTON) pwrBtnCnt ++;
		}
		if (pwrBtnCnt<2) return;
	}*/
	buttonConfigData.pressConfig = data[1];
	buttonConfigData.releaseFunc = data[2];
	buttonConfigData.releaseConfig = data[3];
	buttonConfigData.singlePressFunc = data[4];
	buttonConfigData.singlePressTime = data[5];
	buttonConfigData.doublePressFunc = data[6];
	buttonConfigData.doublePressTime = data[7];
	buttonConfigData.longPressFunc1 = data[8];
	buttonConfigData.longPressTime1 = data[9];
	buttonConfigData.longPressFunc2 = data[10];
	buttonConfigData.longPressTime2 = data[11];
	writebuttonConfigData = b;
}

void ButtonGetConfiguarion(uint8_t b, uint8_t data[], uint16_t *len) {
	data[0] = buttons[b].pressFunc;
	data[1] = buttons[b].pressConfig;
	data[2] = buttons[b].releaseFunc;
	data[3] = buttons[b].releaseConfig;
	data[4] = buttons[b].singlePressFunc;
	data[5] = buttons[b].singlePressTime / 100;
	data[6] = buttons[b].doublePressFunc;
	data[7] = buttons[b].doublePressTime / 100;
	data[8] = buttons[b].longPressFunc1;
	data[9] = buttons[b].longPressTime1 / 100;
	data[10] = buttons[b].longPressFunc2;
	data[11] = buttons[b].longPressTime2 / 100;
	*len = 12;
}

__weak void PowerOnButtonEventCb(uint8_t b, ButtonEvent_T event) {
	UNUSED(b);
	UNUSED(event);
}

__weak void PowerOffButtonEventCb(uint8_t b, ButtonEvent_T event) {
	UNUSED(b);
	UNUSED(event);
}

__weak void ButtonEventFuncPowerResetCb(uint8_t b, ButtonEvent_T event) {
	UNUSED(b);
	UNUSED(event);
}

__weak void ButtonDualLongPressEventCb(void) {

}

