/*
 * led.h
 *
 *  Created on: 06.12.2016.
 *      Author: milan
 */

#ifndef LED_H_
#define LED_H_

#include <stdint.h>
#include <stdbool.h>

#define LED_D1                  0

typedef enum LedFunction_T
{
  LED_NOT_USED = 0,
  LED_CHARGE_STATUS,
  LED_ON_OFF_STATUS,      // TODO - no producer yet
  LED_USER_LED,
  LED_NUMBER
} LedFunction_T;

void led_Init(void);

void led_SetRGB(uint8_t _led, uint8_t _r, uint8_t _g, uint8_t _b);

void led_SetFuncRGB(LedFunction_T _func, uint8_t _r, uint8_t _g, uint8_t _b);

void led_Stop(void);

void led_Start(void);

void led_CmdSetState(uint8_t _led, uint8_t _pData[], uint8_t _len);

void led_CmdGetState(uint8_t _led, uint8_t _pData[], uint16_t *_pLen);

void led_CmdSetBlink(uint8_t _led, uint8_t _pData[], uint8_t _len);

void led_CmdGetBlink(uint8_t _led, uint8_t _pData[], uint16_t *_pLen);

void led_CmdSetConfig(uint8_t _led, uint8_t _pData[], uint8_t _len);

void led_CmdGetConfig(uint8_t _led, uint8_t _pData[], uint16_t *_pLen);

uint8_t led_GetParamR(LedFunction_T _func);

uint8_t led_GetParamG(LedFunction_T _func);

uint8_t led_GetParamB(LedFunction_T _func);

#endif /* LED_H_ */
