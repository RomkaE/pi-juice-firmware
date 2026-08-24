/*
 * button.c
 *
 *  Created on: 28.03.2017.
 *      Author: milan
 */

#include <stdint.h>
#include <stdbool.h>

#include "config.h"
#include "iosystem/button.h"
#include "nv.h"
#include "app-error/app_assert.h"
#include "app-error/diag.h"
#include "board.h"
#include "utils/time_count.h"

// ST HAL/CubeMX:
#include "stm32f0xx_hal.h"

// FreeRTOS:
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

// LOG:
#include "log/log.h"

#define BUTTON_COUNT              3

#define BUTTON_POLL_PERIOD_MS     20

/*
 * Below one tick pdMS_TO_TICKS() rounds to 0 and the wait stops being a wait: the task would
 * spin at its own priority and starve everything below it. Sampling faster than the tick buys
 * nothing anyway - HAL_GetTick() is the tick counter, so the timestamps the state machine
 * compares only move in whole ticks.
 */
_Static_assert(pdMS_TO_TICKS(BUTTON_POLL_PERIOD_MS) >= 1,
               "BUTTON_POLL_PERIOD_MS must be at least one RTOS tick");

/* Bytes of one button configuration, as carried by host registers 0x110/0x112/0x114. */
#define BUTTON_CFG_LEN            12

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

/* Configuration write posted by the host, applied by the task. */
typedef struct
{
	uint8_t index;
	uint8_t data[BUTTON_CFG_LEN];
} ButtonCfg_t;

static Button_T buttons[BUTTON_COUNT] = {
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


// FreeRTOS task:
static TaskHandle_t s_TaskHandle;
static StaticTask_t TaskTCB;
static StackType_t TaskStack[TASK_BTN_STACK];

// Configuration command queue, host -> task:
static QueueHandle_t s_CmdQueHandle;
static StaticQueue_t s_CmdQue;
static ButtonCfg_t s_CmdQueBuf[TASK_BTN_QUEUE_LEN];

/*============================ TASK PRIVATE ==================================*/

static ButtonFunction_T GetFuncOfButton(uint8_t b) {
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

/*
 * The press/release state machine, unchanged from the polled version - it still works on
 * HAL_GetTick() deltas, it is just no longer called from main_poll(). The only difference is
 * that a matching callback is queued for the APP task instead of being invoked right here.
 */
static void ProcessButton( uint8_t b, GPIO_PinState pinState ) {
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
	} else if ( buttons[b].tempEvent && pinState == GPIO_PIN_RESET && MS_TIME_COUNT(buttons[b].pressTimer)  > buttons[b].doublePressTime ) {
		if ( buttons[b].doublePressFunc != BUTTON_EVENT_NO_FUNC ) {
			// generate single press event only if there was no double press
			if ( buttons[b].tempEvent == BUTTON_EVENT_SINGLE_PRESS ) {
				buttons[b].event = BUTTON_EVENT_SINGLE_PRESS;
			} else if ( buttons[b].tempEvent == BUTTON_EVENT_RELEASE ) {
				buttons[b].event = BUTTON_EVENT_RELEASE;
			}
		}
		buttons[b].tempEvent = 0;
	}

	// Callback:
	if ( buttons[b].event  > oldEv )
	{
		ButtonFunction_T func = GetFuncOfButton(b);
		if ( func > BUTTON_EVENT_NO_FUNC && func < BUTTON_EVENT_FUNC_NUMBER )
		{
		  LOG_DEBUG("[BTN] Button event: btn=%u, event=%u, func=%u",
		      (unsigned)b, (unsigned)buttons[b].event, (unsigned)func);
		  button_ButtonFunc_Callback(func);
		  button_ClearEvent(b);
		}
		else if ( func >= BUTTON_EVENT_FUNC_SYS_EVENT )
		{
			/*
			 * A host function: SYS_FUNC_* encode as 0x11..0x14 and USER_FUNC* as 0x20..0x2F (see
			 * SetButtonConfiguration in pijuice.py). The firmware must NOT act on these and must
			 * NOT clear them - it publishes the event in register 0x45 and the host clears it by
			 * writing back, see CmdServerReadButtonStatus(). Clearing here destroyed the event long
			 * before the host's next poll, which is what stopped host-side power off from working.
			 */
			LOG_DEBUG("[BTN] Button event left for host: btn=%u, event=%u, func=0x%02X",
			    (unsigned)b, (unsigned)buttons[b].event, (unsigned)func);
		}
		else
		{
			/*
			 * Nothing configured for this event, so no one will ever consume it. Clearing keeps
			 * register 0x45 and the isButton bit of 0x40 from reporting it forever.
			 */
			LOG_WARNING("[BTN] Unconfigured button event: btn=%u, event=%u",
			    (unsigned)b, (unsigned)buttons[b].event);
			button_ClearEvent(b);
		}
	}

}

static void ProcessAllButtons(void) {

  const uint8_t oldDualLongPressStatus = buttons[0].staticLongPressEvent && buttons[1].staticLongPressEvent;

	ProcessButton(0, HAL_GPIO_ReadPin(BTN_SW2_PORT, BTN_SW2_PIN)); // host-facing sw1

	ProcessButton(1, HAL_GPIO_ReadPin(BTN_SW1_PORT, BTN_SW1_PIN)); // host-facing sw2

	ProcessButton(2, HAL_GPIO_ReadPin(BTN_SW3_PORT, BTN_SW3_PIN));  // host-facing sw3

	if ((buttons[0].staticLongPressEvent && buttons[1].staticLongPressEvent) > oldDualLongPressStatus)
	{
	  // Callback:
	  button_ButtonRstCfg_Callback();
	}
}

/*
 * Nothing in the state machine advances while every button is released and no double press
 * window is open: the second branch of ProcessButton() needs a pressed pin, the third needs a
 * pending tempEvent, and the first needs an edge, which arrives as an EXTI interrupt. So the
 * task can sleep indefinitely in that state instead of being polled 50 times a second.
 */
static bool AnyActive(void) {
	for (uint8_t b = 0; b < BUTTON_COUNT; b++) {
		if (buttons[b].state != GPIO_PIN_RESET || buttons[b].tempEvent != 0)
			return true;
	}
	return false;
}

/* NV offsets of the ten configuration bytes within one button's block, relative to its
 * NV_ADDR_BUTTON_PRESS_FUNC_SWx entry. The gaps are the reserved *_CONFIG slots of the press
 * and release events, which are never stored. Shared by the read and the write path below. */
static const uint8_t buttonNvOffsets[10] = { 0, 2, 4, 5, 6, 7, 8, 9, 10, 11 };

static uint8_t NvOffsetOf(uint8_t b) {
	return b * (NV_ADDR_BUTTON_PRESS_FUNC_SW2 - NV_ADDR_BUTTON_PRESS_FUNC_SW1) + NV_ADDR_BUTTON_PRESS_FUNC_SW1;
}

/*
 * Applies ten raw configuration bytes to a button. The times are stored in units of 100 ms and
 * kept in milliseconds at runtime, which is why button_GetConfig() divides them back.
 */
static void SetConfigData(uint8_t b, const uint8_t val[10]) {
	taskENTER_CRITICAL();   // button_GetConfig() reads this group from the I2C1 interrupt
	buttons[b].pressFunc = val[0];
	buttons[b].releaseFunc = val[1];
	buttons[b].singlePressFunc = val[2];
	buttons[b].singlePressTime = (uint16_t)val[3] * 100;
	buttons[b].doublePressFunc = val[4];
	buttons[b].doublePressTime = (uint16_t)val[5] * 100;
	buttons[b].longPressFunc1 = val[6];
	buttons[b].longPressTime1 = (uint16_t)val[7] * 100;
	buttons[b].longPressFunc2 = val[8];
	buttons[b].longPressTime2 = (uint16_t)val[9] * 100;
	taskEXIT_CRITICAL();
}

/* Loads one button from the NV. A block that does not read back complete is left at its
 * compiled in default. */
static void LoadConfig(uint8_t b) {
	uint8_t nvOffset = NvOffsetOf(b);
	uint8_t val[10];

	for (uint8_t i = 0; i < 10; i++) {
		if (nv_read_U8(nvOffset + buttonNvOffsets[i], &val[i]) != NV_OK)
			return;
	}

	SetConfigData(b, val);
}

/*
 * Persists a configuration and reloads it. This is the one place in the module that programs
 * the flash - previously it ran in the APP poll loop, now it runs here, so a page transfer
 * stalls button sampling rather than the whole main loop.
 */
static void ApplyConfig(const ButtonCfg_t *_pCmd) {
	uint8_t nvOffset = NvOffsetOf(_pCmd->index);
	const uint8_t *d = _pCmd->data;
	const uint8_t val[10] = { d[0], d[2], d[4], d[5], d[6], d[7], d[8], d[9], d[10], d[11] };
	for (uint8_t i = 0; i < 10; i++)
		nv_write_U8(nvOffset + buttonNvOffsets[i], val[i]);

	LoadConfig(_pCmd->index);
}

static void Task(void *parameters)
{
  (void)parameters;

  LOG_INFO("BUTTON task started");

  for (uint8_t b = 0; b < BUTTON_COUNT; b++)
    LoadConfig(b);

  TickType_t last_sample = xTaskGetTickCount();

  while (1)
  {
    if (AnyActive())
    {
      xTaskDelayUntil(&last_sample, pdMS_TO_TICKS(BUTTON_POLL_PERIOD_MS));
    }
    else
    {
      ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
      last_sample = xTaskGetTickCount();
    }

    ButtonCfg_t btn_cfg;
    bool applied = false;
    while (xQueueReceive(s_CmdQueHandle, &btn_cfg, 0) == pdTRUE)
    {
      ApplyConfig(&btn_cfg);
      applied = true;
    }

    if (applied)
      last_sample = xTaskGetTickCount();

    ProcessAllButtons();
  }
}

/*============================ PUBLIC API ====================================*/

void button_Init(void) {
	LOG_INFO("button_Init...");

	// Create the queue first. BTN sits below APP (see config.h) and button_Init() runs in the
	// APP task, so no preemption happens here - but the ordering costs nothing and survives a
	// future priority change.
	s_CmdQueHandle = xQueueCreateStatic(sizeof(s_CmdQueBuf) / sizeof(s_CmdQueBuf[0]),
	                          sizeof(s_CmdQueBuf[0]), (uint8_t*)s_CmdQueBuf, &s_CmdQue);
	ASSERT(s_CmdQueHandle != NULL);

	s_TaskHandle = xTaskCreateStatic(Task, "BTN", sizeof(TaskStack) / sizeof(StackType_t),
	                          NULL, TASK_BTN_PRIO, TaskStack, &TaskTCB);
	ASSERT(s_TaskHandle != NULL);
}

void button_NotifyFromISR(void)
{
	if (s_TaskHandle == NULL)
		return;

	BaseType_t woken = pdFALSE;
	vTaskNotifyGiveFromISR(s_TaskHandle, &woken);
	portYIELD_FROM_ISR(woken);
}

ButtonEvent_T button_GetEvent(uint8_t _b) {
	return buttons[_b].event;
}

void button_ClearEvent(uint8_t _b) {
	buttons[_b].event = 0;
}

uint8_t button_IsEvent(void) {
	return buttons[0].event || buttons[1].event || buttons[2].event;
}

void button_SetConfig(uint8_t _b, uint8_t _pData[], uint8_t _len) {
	(void)_len;

	if (_b >= BUTTON_COUNT)
		return;

	/*
	 * Only queued here: this runs in the I2C1 interrupt, and applying the configuration means
	 * ten emulated EEPROM writes that can trigger a flash page transfer.
	 */
	if (s_CmdQueHandle == NULL)
		return;   // host wrote before button_Init(); it will retry, and NV still holds the setting

	ButtonCfg_t cmd;
	cmd.index = _b;
	for (uint8_t i = 0; i < BUTTON_CFG_LEN; i++)
		cmd.data[i] = _pData[i];

	if (xPortIsInsideInterrupt() != pdFALSE) {
		BaseType_t woken = pdFALSE;
		if (xQueueSendFromISR(s_CmdQueHandle, &cmd, &woken) == pdTRUE) {
			vTaskNotifyGiveFromISR(s_TaskHandle, &woken);
		} else {
			diag_Set(DIAG_QUE_FULL_BTN);
		}
		portYIELD_FROM_ISR(woken);
	} else {
		if (xQueueSend(s_CmdQueHandle, &cmd, 0) == pdTRUE) {
			xTaskNotifyGive(s_TaskHandle);
		} else {
			LOG_ERROR("[BTN] Configuration queue full, button %u dropped", _b);
			diag_Set(DIAG_QUE_FULL_BTN);
		}
	}
}

void button_GetConfig(uint8_t _b, uint8_t _pData[], uint16_t *_pLen) {
	if (_b >= BUTTON_COUNT)
		return;

	/*
	 * Answers with the applied configuration, so a host write followed immediately by a read
	 * can still report the previous one until the task has drained the queue - the same window
	 * the polled version had, only shorter.
	 */
	_pData[0] = buttons[_b].pressFunc;
	_pData[1] = buttons[_b].pressConfig;
	_pData[2] = buttons[_b].releaseFunc;
	_pData[3] = buttons[_b].releaseConfig;
	_pData[4] = buttons[_b].singlePressFunc;
	_pData[5] = buttons[_b].singlePressTime / 100;
	_pData[6] = buttons[_b].doublePressFunc;
	_pData[7] = buttons[_b].doublePressTime / 100;
	_pData[8] = buttons[_b].longPressFunc1;
	_pData[9] = buttons[_b].longPressTime1 / 100;
	_pData[10] = buttons[_b].longPressFunc2;
	_pData[11] = buttons[_b].longPressTime2 / 100;
	*_pLen = BUTTON_CFG_LEN;
}

__attribute__((weak)) void button_ButtonFunc_Callback(ButtonFunction_T _btn_func)
{
  (void)_btn_func;
}

__attribute__((weak)) void button_ButtonRstCfg_Callback(void)
{
}
