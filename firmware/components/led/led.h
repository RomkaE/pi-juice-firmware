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

// Mask errors:
#define LED_ERR_QUEUE_FULL      (1UL << 0)  // command dropped, the task did not drain in time
#define LED_ERR_NOT_READY       (1UL << 1)  // command posted before led_Init() created the queue
#define LED_ERR_NV_WRITE        (1UL << 2)  // configuration did not read back valid from the NV

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

uint32_t led_GetErrMask(bool _clear);

#endif /* LED_H_ */
