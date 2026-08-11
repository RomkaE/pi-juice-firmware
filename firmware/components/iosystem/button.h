/*
 * button.h
 *
 *  Created on: 28.03.2017.
 *      Author: milan
 */

#ifndef BUTTON_H_
#define BUTTON_H_

#include <stdint.h>
#include <stdbool.h>

#define BUTTON_STATIC_LONG_PRESS_TIME	19600

// Mask errors:
#define BUTTON_ERR_QUEUE_FULL   (1UL << 0)  // configuration command dropped, the task did not drain

// Button event function definitions, special functions 0 - 15, user button event functions 15 - 31, other values reserved
typedef enum ButtonFunction_T {
	BUTTON_EVENT_NO_FUNC = 0,
	BUTTON_EVENT_FUNC_POWER_ON,
	BUTTON_EVENT_FUNC_POWER_OFF,
	BUTTON_EVENT_FUNC_POWER_RESET,
	BUTTON_EVENT_FUNC_NUMBER
} ButtonFunction_T;

#define BUTTON_EVENT_FUNC_USER_EVENT	0x20
#define BUTTON_EVENT_FUNC_SYS_EVENT		0x10

typedef enum ButtonEvent_T {
  BUTTON_EVENT_EMPTY = 0,
	BUTTON_EVENT_PRESS,
	BUTTON_EVENT_RELEASE,
	BUTTON_EVENT_SINGLE_PRESS,
	BUTTON_EVENT_DOUBLE_PRESS,
	BUTTON_EVENT_LONG_PRESS1,
	BUTTON_EVENT_LONG_PRESS2
} ButtonEvent_T;

void button_Init(void);

// Wakes the task from an EXTI edge. Safe from an interrupt only:
void button_NotifyFromISR(void);

ButtonEvent_T button_GetEvent(uint8_t _b);

void button_ClearEvent(uint8_t _b);

uint8_t button_IsEvent(void);

/* Host registers 0x110/0x112/0x114 - button configuration, persisted in the emulated EEPROM. */
void button_SetConfig(uint8_t _b, uint8_t _pData[], uint8_t _len);

void button_GetConfig(uint8_t _b, uint8_t _pData[], uint16_t *_pLen);

/* Reserved for the planned sleep mode - intentionally without callers, do not remove. */
int8_t button_IsActive(void);

uint32_t button_GetErrMask(bool _clear);

void button_ButtonCallback(ButtonFunction_T _btn_func);

void button_ButtonRstCfgCallback(void);

#endif /* BUTTON_H_ */
