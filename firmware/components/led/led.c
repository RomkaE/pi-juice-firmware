/*
 * led.c
 *
 *  Created on: 06.12.2016.
 *      Author: milan
 */

#include <stdint.h>
#include <stdbool.h>
#include "led.h"
#include "nv.h"
#include "app-error/app_error.h"
#include "app-error/app_assert.h"

// ST HAL/CubeMX:
#include "stm32f0xx_hal.h"
#include "cube-mx/tim.h"

// FreeRTOS:
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

// LOG:
#include "log/log.h"

/*
 * Host blink period unit: registers 104/105 carry the two phase durations in steps of 10 ms.
 * At configTICK_RATE_HZ 100 that is exactly one tick, so the whole blink engine works in ticks
 * and the task can sleep until the next phase boundary instead of being polled.
 */
#define LED_BLINK_UNIT_MS         10U
#define LED_BLINK_UNIT_TICKS      (pdMS_TO_TICKS(LED_BLINK_UNIT_MS))

/* Command types carried by the task queue. */
typedef enum
{
  LED_CMD_SET_RGB = 1,    // led_SetRGB()      - raw PWM override
  LED_CMD_SET_FUNC_RGB,   // led_SetFuncRGB()  - base colour of the assigned function
  LED_CMD_STATE,          // host register 102
  LED_CMD_BLINK,          // host register 104
  LED_CMD_CONFIG,         // host register 106 - the only command that touches the flash
  LED_CMD_STOP,           // led_Stop()
  LED_CMD_START           // led_Start()
} LedCmdType_T;

typedef struct
{
  uint8_t r, g, b;
} LedRgb_T;

typedef struct
{
  uint8_t func;
  uint8_t r, g, b;
} LedFuncRgb_T;

typedef struct
{
  uint8_t func;
  uint8_t paramR, paramG, paramB;
} LedCfg_T;

/* Blink request exactly as the host sends it: periods still in LED_BLINK_UNIT_MS steps. */
typedef struct
{
  uint8_t repeat;
  uint8_t r1, g1, b1, period1;
  uint8_t r2, g2, b2, period2;
} LedBlink_T;

/*
 * Queue item. `type` is kept as uint8_t rather than LedCmdType_T on purpose: an enum member
 * would force 4-byte alignment and grow the item from 11 to 16 bytes.
 * `seq` is only meaningful for the three host register commands - see the request mirror below.
 */
typedef struct
{
  uint8_t type;
  uint8_t seq;
  union
  {
    LedRgb_T     rgb;
    LedFuncRgb_T funcRgb;
    LedBlink_T   blink;
    LedCfg_T     cfg;
  } data;
} LedCmd_T;

/* Runtime state of the LED, owned by the task. */
typedef struct
{
  uint8_t r, g, b;          // base colour

  uint8_t blinkRepeat;
  uint8_t blinkR1, blinkG1, blinkB1;
  uint8_t blinkR2, blinkG2, blinkB2;
  TickType_t blinkTicks1;   // phase 1 duration, at least 1 tick
  TickType_t blinkTicks2;   // phase 2 duration, at least 1 tick
  uint16_t blinkCount;      // remaining phases, 0 - no blink running
  TickType_t blinkDeadline; // tick at which the current phase ends
} Led_T;

// Applied state, written only by the task:
static Led_T s_Led;
static LedCfg_T s_Cfg = { LED_CHARGE_STATUS, 60, 60, 100 };

/*
 * Request mirror. A host register write is applied asynchronously by the task, but the host
 * routinely reads the same register back in the next I2C transaction to verify the write, and
 * that read is served from the I2C1 interrupt. Reporting the applied state there would answer
 * with stale data - for register 106 by tens of milliseconds, because applying it can trigger
 * an emulated EEPROM page transfer (~130 variable copies plus a page erase).
 *
 * So each of the three register writes keeps its requested values plus a sequence number that
 * is bumped only after the command has been queued; the task copies the sequence number out of
 * the command it applied. A getter answers from the mirror while req != applied, and from the
 * applied state otherwise. Every one of these variables has exactly one writer, so no locking
 * is needed on either side.
 */
static LedRgb_T s_ReqState;
static LedBlink_T s_ReqBlink;
static LedCfg_T s_ReqCfg;
static volatile uint8_t s_ReqStateSeq, s_ReqBlinkSeq, s_ReqCfgSeq;
static volatile uint8_t s_AppliedStateSeq, s_AppliedBlinkSeq, s_AppliedCfgSeq;

static uint32_t s_ErrMask;  // LED_ERR_* bits, set from the task and from the I2C1 interrupt

// FreeRTOS task:
static TaskHandle_t s_TaskHandle;
static StaticTask_t TaskTCB;
static StackType_t TaskStack[512];   // TODO - remove magic number

// Command queue:
static QueueHandle_t s_QueHandle;
static StaticQueue_t s_Que;
static LedCmd_T s_QueBuf[8];         // TODO - remove magic number

// HAL instances:
extern TIM_HandleTypeDef htim3;

/* Gamma corrected duty per 8-bit intensity, stored inverted - see setRGB(). */
static const uint16_t pwm_table[] = {
    0,     27,    56,    84,    113,   141,   170,   198,
    227,   255,   284,   312,   340,   369,   397,   426,
    454,   483,   511,   540,   568,   597,   626,   657,
    688,   720,   754,   788,   824,   860,   898,   936,
    976,   1017,  1059,  1102,  1146,  1191,  1238,  1286,
    1335,  1385,  1436,  1489,  1543,  1598,  1655,  1713,
    1772,  1833,  1895,  1958,  2023,  2089,  2156,  2225,
    2296,  2368,  2441,  2516,  2592,  2670,  2750,  2831,
    2914,  2998,  3084,  3171,  3260,  3351,  3443,  3537,
    3633,  3731,  3830,  3931,  4034,  4138,  4245,  4353,
    4463,  4574,  4688,  4803,  4921,  5040,  5161,  5284,
    5409,  5536,  5665,  5796,  5929,  6064,  6201,  6340,
    6482,  6625,  6770,  6917,  7067,  7219,  7372,  7528,
    7687,  7847,  8010,  8174,  8341,  8511,  8682,  8856,
    9032,  9211,  9392,  9575,  9761,  9949,  10139, 10332,
    10527, 10725, 10925, 11127, 11332, 11540, 11750, 11963,
    12178, 12395, 12616, 12839, 13064, 13292, 13523, 13757,
    13993, 14231, 14473, 14717, 14964, 15214, 15466, 15722,
    15980, 16240, 16504, 16771, 17040, 17312, 17587, 17865,
    18146, 18430, 18717, 19006, 19299, 19595, 19894, 20195,
    20500, 20808, 21119, 21433, 21750, 22070, 22393, 22720,
    23049, 23382, 23718, 24057, 24400, 24745, 25094, 25446,
    25802, 26160, 26522, 26888, 27256, 27628, 28004, 28382,
    28765, 29150, 29539, 29932, 30328, 30727, 31130, 31536,
    31946, 32360, 32777, 33197, 33622, 34049, 34481, 34916,
    35354, 35797, 36243, 36692, 37146, 37603, 38064, 38528,
    38996, 39469, 39945, 40424, 40908, 41395, 41886, 42382,
    42881, 43383, 43890, 44401, 44916, 45434, 45957, 46484,
    47014, 47549, 48088, 48630, 49177, 49728, 50283, 50842,
    51406, 51973, 52545, 53120, 53700, 54284, 54873, 55465,
    56062, 56663, 57269, 57878, 58492, 59111, 59733, 60360,
    60992, 61627, 62268, 62912, 63561, 64215, 64873, 65535
 };

/*============================ TASK PRIVATE ==================================*/

/* TIM3 CH1/CH2/CH3 duty. Called from the task only, so the three channels never tear. */
static void setRGB(uint8_t _r, uint8_t _g, uint8_t _b)
{
  TIM3->CCR1 = pwm_table[_r];
  TIM3->CCR2 = pwm_table[_g];
  TIM3->CCR3 = pwm_table[_b];
}

/* Applies the colour that belongs to the current configuration. */
static void applyCfgColour(void)
{
  if (s_Cfg.func == LED_USER_LED)
  {
    taskENTER_CRITICAL();
    s_Led.r = s_Cfg.paramR;
    s_Led.g = s_Cfg.paramG;
    s_Led.b = s_Cfg.paramB;
    taskEXIT_CRITICAL();
    setRGB(s_Led.r, s_Led.g, s_Led.b);
  }
  else
  {
    setRGB(0, 0, 0);
  }
}

/*
 * Reads the configuration from the emulated EEPROM. A variable that is missing or fails its
 * complement check keeps the compiled in default in s_Cfg.
 */
static void loadConfig(void)
{
  LedCfg_T cfg = s_Cfg;

  (void)nv_read_U8(NV_ADDR_LED_D1_FUNC, &cfg.func);
  (void)nv_read_U8(NV_ADDR_LED_D1_PARAM_R, &cfg.paramR);
  (void)nv_read_U8(NV_ADDR_LED_D1_PARAM_G, &cfg.paramG);
  (void)nv_read_U8(NV_ADDR_LED_D1_PARAM_B, &cfg.paramB);

  taskENTER_CRITICAL();
  s_Cfg = cfg;                  // func and the three params must become visible together
  taskEXIT_CRITICAL();

  applyCfgColour();
}

/*
 * Persists a configuration and reads it back for verification. This is the one place in the
 * module that programs the flash - previously it ran inside the I2C1 interrupt, stalling the
 * bus for the whole duration of a page transfer.
 */
static void applyConfig(const LedCfg_T *_pCfg)
{
  nv_write_U8(NV_ADDR_LED_D1_FUNC, _pCfg->func);
  nv_write_U8(NV_ADDR_LED_D1_PARAM_R, _pCfg->paramR);
  nv_write_U8(NV_ADDR_LED_D1_PARAM_G, _pCfg->paramG);
  nv_write_U8(NV_ADDR_LED_D1_PARAM_B, _pCfg->paramB);

  LedCfg_T cfg = s_Cfg;         // anything that fails to read back keeps its current value
  bool valid = true;
  valid &= (nv_read_U8(NV_ADDR_LED_D1_FUNC, &cfg.func) == NV_OK);
  valid &= (nv_read_U8(NV_ADDR_LED_D1_PARAM_R, &cfg.paramR) == NV_OK);
  valid &= (nv_read_U8(NV_ADDR_LED_D1_PARAM_G, &cfg.paramG) == NV_OK);
  valid &= (nv_read_U8(NV_ADDR_LED_D1_PARAM_B, &cfg.paramB) == NV_OK);

  if (!valid)
  {
    LOG_ERROR("[LED] Configuration NV read back failed");
    taskENTER_CRITICAL();
    s_ErrMask |= LED_ERR_NV_WRITE;
    taskEXIT_CRITICAL();
  }

  taskENTER_CRITICAL();
  s_Cfg = cfg;
  taskEXIT_CRITICAL();

  applyCfgColour();
}

static void applyBlink(const LedBlink_T *_pBlink)
{
  /*
   * Clamp both phases to one tick. A zero period used to cost one main_poll() sweep per phase,
   * but an event driven task would get a zero length timeout instead and, combined with
   * repeat == 0xFF, would spin at its own priority forever and starve APP until the IWDG fires.
   */
  TickType_t ticks1 = (TickType_t)_pBlink->period1 * LED_BLINK_UNIT_TICKS;
  TickType_t ticks2 = (TickType_t)_pBlink->period2 * LED_BLINK_UNIT_TICKS;
  if (ticks1 == 0)
    ticks1 = 1;
  if (ticks2 == 0)
    ticks2 = 1;

  taskENTER_CRITICAL();
  s_Led.blinkRepeat = _pBlink->repeat;
  s_Led.blinkCount = (uint16_t)_pBlink->repeat << 1;   // two phases per blink
  s_Led.blinkR1 = _pBlink->r1;
  s_Led.blinkG1 = _pBlink->g1;
  s_Led.blinkB1 = _pBlink->b1;
  s_Led.blinkR2 = _pBlink->r2;
  s_Led.blinkG2 = _pBlink->g2;
  s_Led.blinkB2 = _pBlink->b2;
  s_Led.blinkTicks1 = ticks1;
  s_Led.blinkTicks2 = ticks2;
  taskEXIT_CRITICAL();

  if (s_Led.blinkRepeat)
  {
    setRGB(s_Led.blinkR1, s_Led.blinkG1, s_Led.blinkB1);
    s_Led.blinkDeadline = xTaskGetTickCount() + s_Led.blinkTicks1;
  }
  else
  {
    setRGB(s_Led.r, s_Led.g, s_Led.b);
  }
}

/*
 * One blink phase boundary. blinkCount counts phases, so it starts at 2 * repeat and is even
 * while colour 1 is on screen and odd while colour 2 is:
 *
 *   even count -> colour 1 is displayed, the phase lasts blinkTicks1
 *   odd  count -> colour 2 is displayed, the phase lasts blinkTicks2
 *
 * NOTE (pre-existing, preserved on purpose): with repeat == 0xFF the counter is reloaded with
 * 256 and no colour is written, so colour 2 stays on screen for one extra blinkTicks1 phase
 * every 128 blinks. Not fixed here - this change is behaviour preserving.
 */
static void processBlink(void)
{
  if (s_Led.blinkCount & 0x0001)
  {
    // Phase 2 expired.
    s_Led.blinkCount--;
    if (s_Led.blinkCount == 0)
    {
      if (s_Led.blinkRepeat == 0xFF)
      {
        s_Led.blinkCount = 256;   // infinite blink, colour intentionally left untouched
      }
      else
      {
        setRGB(s_Led.r, s_Led.g, s_Led.b);
      }
    }
    else
    {
      setRGB(s_Led.blinkR1, s_Led.blinkG1, s_Led.blinkB1);
    }
    s_Led.blinkDeadline = xTaskGetTickCount() + s_Led.blinkTicks1;
  }
  else
  {
    // Phase 1 expired.
    setRGB(s_Led.blinkR2, s_Led.blinkG2, s_Led.blinkB2);
    s_Led.blinkCount--;
    s_Led.blinkDeadline = xTaskGetTickCount() + s_Led.blinkTicks2;
  }
}

static void startPwm(void)
{
  if ((htim3.Instance->CR1 & TIM_CR1_CEN) != 0)
    return;

  if (HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1) != HAL_OK)  // LED_D1 red,   PB4
    APP_ERROR(APP_HAL_ERROR);
  if (HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2) != HAL_OK)  // LED_D1 green, PB5
    APP_ERROR(APP_HAL_ERROR);
  if (HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3) != HAL_OK)  // LED_D1 blue,  PB0
    APP_ERROR(APP_HAL_ERROR);

  loadConfig();
}

static void stopPwm(void)
{
  if ((htim3.Instance->CR1 & TIM_CR1_CEN) != TIM_CR1_CEN)
    return;

  HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
  HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_2);
  HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_3);
}

static void applyCmd(const LedCmd_T *_pCmd)
{
  switch (_pCmd->type)
  {
    case LED_CMD_SET_RGB:
      // Raw override: neither the base colour nor a running blink is touched, as before.
      setRGB(_pCmd->data.rgb.r, _pCmd->data.rgb.g, _pCmd->data.rgb.b);
      break;

    case LED_CMD_SET_FUNC_RGB:
      /*
       * Cancels a running blink. That is what makes a host blink die within 900 ms while the
       * LED is assigned to LED_CHARGE_STATUS and battery.c keeps refreshing it - preserved.
       */
      if (s_Cfg.func == _pCmd->data.funcRgb.func)
      {
        taskENTER_CRITICAL();
        s_Led.r = _pCmd->data.funcRgb.r;
        s_Led.g = _pCmd->data.funcRgb.g;
        s_Led.b = _pCmd->data.funcRgb.b;
        s_Led.blinkRepeat = 0;
        s_Led.blinkCount = 0;
        taskEXIT_CRITICAL();
        setRGB(s_Led.r, s_Led.g, s_Led.b);
      }
      break;

    case LED_CMD_STATE:
      taskENTER_CRITICAL();
      s_Led.r = _pCmd->data.rgb.r;
      s_Led.g = _pCmd->data.rgb.g;
      s_Led.b = _pCmd->data.rgb.b;
      taskEXIT_CRITICAL();
      setRGB(s_Led.r, s_Led.g, s_Led.b);
      s_AppliedStateSeq = _pCmd->seq;
      break;

    case LED_CMD_BLINK:
      applyBlink(&_pCmd->data.blink);
      s_AppliedBlinkSeq = _pCmd->seq;
      break;

    case LED_CMD_CONFIG:
      applyConfig(&_pCmd->data.cfg);
      s_AppliedCfgSeq = _pCmd->seq;
      break;

    case LED_CMD_STOP:
      stopPwm();
      break;

    case LED_CMD_START:
      startPwm();
      break;

    default:
      ASSERT(0);
      break;
  }
}

static void Task(void *parameters)
{
  (void)parameters;

  LOG_INFO("LED task started");

  // Init the PWM timer here, not in led_Init(), the same way the ANALOG task owns its ADC:
  MX_TIM3_Init();
  startPwm();

  while (1)
  {
    /*
     * Sleep until the next phase boundary, or indefinitely while the LED is static - a solid
     * LED costs zero wakeups, which is what the old 20 ms polling could not do.
     */
    TickType_t wait = portMAX_DELAY;
    if (s_Led.blinkCount != 0)
    {
      int32_t remain = (int32_t)(s_Led.blinkDeadline - xTaskGetTickCount());
      wait = (remain > 0) ? (TickType_t)remain : 0;
    }

    LedCmd_T cmd;
    if (xQueueReceive(s_QueHandle, &cmd, wait) == pdTRUE)
      applyCmd(&cmd);           // may restart, retime or cancel the blink

    /*
     * Deadline driven rather than driven by the receive result: a command arriving mid wait
     * must not shorten the current phase, and one arriving exactly at the boundary must not
     * swallow the edge.
     */
    if (s_Led.blinkCount != 0
        && (int32_t)(xTaskGetTickCount() - s_Led.blinkDeadline) >= 0)
      processBlink();
  }
}

/*============================ COMMAND POSTING ===============================*/

/*
 * The single funnel for every mutating entry point. Callable from the I2C1 interrupt, where the
 * host command handlers run, as well as from the APP task. Never blocks: an interrupt must not
 * wait, and the APP task must not be able to wait on the LED task.
 */
static bool postCmd(const LedCmd_T *_pCmd)
{
  if (s_QueHandle == NULL)
  {
    s_ErrMask |= LED_ERR_NOT_READY;   // posted before led_Init(), there is no queue yet
    return false;
  }

  BaseType_t sent;

  if (xPortIsInsideInterrupt() != pdFALSE)
  {
    BaseType_t woken = pdFALSE;
    sent = xQueueSendFromISR(s_QueHandle, _pCmd, &woken);
    portYIELD_FROM_ISR(woken);

    if (sent != pdTRUE)
    {
      /*
       * No critical section and no log from here: an interrupt cannot be preempted by the task
       * that does the read-modify-write under taskENTER_CRITICAL, and Log_Printf() would put a
       * few hundred bytes of line buffer on the stack of whatever task was interrupted.
       */
      s_ErrMask |= LED_ERR_QUEUE_FULL;
    }
  }
  else
  {
    sent = xQueueSend(s_QueHandle, _pCmd, 0);

    if (sent != pdTRUE)
    {
      LOG_ERROR("[LED] Command queue full, cmd=%u dropped", _pCmd->type);
      taskENTER_CRITICAL();
      s_ErrMask |= LED_ERR_QUEUE_FULL;
      taskEXIT_CRITICAL();
    }
  }

  return (sent == pdTRUE);
}

/*============================ PUBLIC API ====================================*/

void led_Init(void)
{
  LOG_INFO("led_Init...");

  // Create command queue. Must come first: the task outranks APP, so it preempts inside
  // xTaskCreateStatic() below and blocks on this queue before this function returns.
  s_QueHandle = xQueueCreateStatic(sizeof(s_QueBuf) / sizeof(s_QueBuf[0]),
                            sizeof(s_QueBuf[0]), (uint8_t*)s_QueBuf, &s_Que);
  ASSERT(s_QueHandle != NULL);

  // Create task:
  s_TaskHandle = xTaskCreateStatic(Task, "LED", sizeof(TaskStack) / sizeof(StackType_t),
                            NULL, 6, TaskStack, &TaskTCB);
  ASSERT(s_TaskHandle != NULL);
}

void led_SetRGB(uint8_t _led, uint8_t _r, uint8_t _g, uint8_t _b)
{
  if (_led != LED_D1)
    return;

  LedCmd_T cmd = { .type = LED_CMD_SET_RGB, .data.rgb = { _r, _g, _b } };
  postCmd(&cmd);
}

void led_SetFuncRGB(LedFunction_T _func, uint8_t _r, uint8_t _g, uint8_t _b)
{
  LedCmd_T cmd = { .type = LED_CMD_SET_FUNC_RGB,
                   .data.funcRgb = { (uint8_t)_func, _r, _g, _b } };
  postCmd(&cmd);
}

void led_Stop(void)
{
  LedCmd_T cmd = { .type = LED_CMD_STOP };
  postCmd(&cmd);
}

void led_Start(void)
{
  LedCmd_T cmd = { .type = LED_CMD_START };
  postCmd(&cmd);
}

/*
 * Now only D1 is used, so the second LED's registers (0x103, 0x105, 0x107) answer with
 * zeros of the expected length. Returning without touching *len is not an option: the
 * command server checksums whatever length the caller pre-set - 1 byte, see i2c_slave.c -
 * so the host would get a stale byte carrying a valid checksum. Same approach as
 * CmdServerReadRetiredU16() takes for the retired current registers.
 */
static void readUnused(uint8_t _pData[], uint16_t *_pLen, uint16_t _n)
{
  for (uint16_t i = 0; i < _n; i++)
    _pData[i] = 0;
  *_pLen = _n;
}

/* Configuration as it will be once everything queued has been applied. */
static uint8_t effectiveFunc(void)
{
  return (s_ReqCfgSeq != s_AppliedCfgSeq) ? s_ReqCfg.func : s_Cfg.func;
}

void led_CmdSetState(uint8_t _led, uint8_t _pData[], uint8_t _len)
{
  (void)_len;

  // Gated here rather than in the task so that a rejected write leaves the mirror untouched
  // and a read back keeps answering with the state that is really in effect.
  if (_led > LED_D1 || effectiveFunc() != LED_USER_LED)
    return;

  s_ReqState.r = _pData[0];
  s_ReqState.g = _pData[1];
  s_ReqState.b = _pData[2];

  LedCmd_T cmd = { .type = LED_CMD_STATE,
                   .seq = (uint8_t)(s_ReqStateSeq + 1),
                   .data.rgb = s_ReqState };
  if (postCmd(&cmd))
    s_ReqStateSeq = cmd.seq;    // publish the request only once it is really queued
}

void led_CmdGetState(uint8_t _led, uint8_t _pData[], uint16_t *_pLen)
{
  if (_led > LED_D1)
  {
    readUnused(_pData, _pLen, 3);
    return;
  }

  if (s_ReqStateSeq != s_AppliedStateSeq)
  {
    _pData[0] = s_ReqState.r;
    _pData[1] = s_ReqState.g;
    _pData[2] = s_ReqState.b;
  }
  else
  {
    _pData[0] = s_Led.r;
    _pData[1] = s_Led.g;
    _pData[2] = s_Led.b;
  }
  *_pLen = 3;
}

void led_CmdSetBlink(uint8_t _led, uint8_t _pData[], uint8_t _len)
{
  (void)_len;

  if (_led > LED_D1 || effectiveFunc() == LED_NOT_USED)
    return;

  s_ReqBlink.repeat = _pData[0];
  s_ReqBlink.r1 = _pData[1];
  s_ReqBlink.g1 = _pData[2];
  s_ReqBlink.b1 = _pData[3];
  s_ReqBlink.period1 = _pData[4];
  s_ReqBlink.r2 = _pData[5];
  s_ReqBlink.g2 = _pData[6];
  s_ReqBlink.b2 = _pData[7];
  s_ReqBlink.period2 = _pData[8];

  LedCmd_T cmd = { .type = LED_CMD_BLINK,
                   .seq = (uint8_t)(s_ReqBlinkSeq + 1),
                   .data.blink = s_ReqBlink };
  if (postCmd(&cmd))
    s_ReqBlinkSeq = cmd.seq;
}

void led_CmdGetBlink(uint8_t _led, uint8_t _pData[], uint16_t *_pLen)
{
  if (_led > LED_D1)
  {
    readUnused(_pData, _pLen, 9);
    return;
  }

  if (s_ReqBlinkSeq != s_AppliedBlinkSeq)
  {
    _pData[0] = s_ReqBlink.repeat;
    _pData[1] = s_ReqBlink.r1;
    _pData[2] = s_ReqBlink.g1;
    _pData[3] = s_ReqBlink.b1;
    _pData[4] = s_ReqBlink.period1;
    _pData[5] = s_ReqBlink.r2;
    _pData[6] = s_ReqBlink.g2;
    _pData[7] = s_ReqBlink.b2;
    _pData[8] = s_ReqBlink.period2;
  }
  else
  {
    // blinkCount counts phases, the host counts blinks. 255 means infinite.
    _pData[0] = s_Led.blinkRepeat < 255 ? (uint8_t)(s_Led.blinkCount >> 1) : 255;
    _pData[1] = s_Led.blinkR1;
    _pData[2] = s_Led.blinkG1;
    _pData[3] = s_Led.blinkB1;
    _pData[4] = (uint8_t)(s_Led.blinkTicks1 / LED_BLINK_UNIT_TICKS);
    _pData[5] = s_Led.blinkR2;
    _pData[6] = s_Led.blinkG2;
    _pData[7] = s_Led.blinkB2;
    _pData[8] = (uint8_t)(s_Led.blinkTicks2 / LED_BLINK_UNIT_TICKS);
  }
  *_pLen = 9;
}

void led_CmdSetConfig(uint8_t _led, uint8_t _pData[], uint8_t _len)
{
  (void)_len;

  if (_led > LED_D1)
    return;

  s_ReqCfg.func = _pData[0];
  s_ReqCfg.paramR = _pData[1];
  s_ReqCfg.paramG = _pData[2];
  s_ReqCfg.paramB = _pData[3];

  LedCmd_T cmd = { .type = LED_CMD_CONFIG,
                   .seq = (uint8_t)(s_ReqCfgSeq + 1),
                   .data.cfg = s_ReqCfg };
  if (postCmd(&cmd))
    s_ReqCfgSeq = cmd.seq;
}

void led_CmdGetConfig(uint8_t _led, uint8_t _pData[], uint16_t *_pLen)
{
  if (_led > LED_D1)
  {
    readUnused(_pData, _pLen, 4);
    return;
  }

  /*
   * While a write is in flight the requested values are reported rather than the NV verified
   * ones. If the flash write does fail, LED_ERR_NV_WRITE is raised and the answer corrects
   * itself as soon as the task has applied the command.
   */
  const LedCfg_T *p = (s_ReqCfgSeq != s_AppliedCfgSeq) ? &s_ReqCfg : &s_Cfg;
  _pData[0] = p->func;
  _pData[1] = p->paramR;
  _pData[2] = p->paramG;
  _pData[3] = p->paramB;
  *_pLen = 4;
}

uint8_t led_GetParamR(LedFunction_T _func)
{
  return (s_Cfg.func == _func) ? s_Cfg.paramR : 0;
}

uint8_t led_GetParamG(LedFunction_T _func)
{
  return (s_Cfg.func == _func) ? s_Cfg.paramG : 0;
}

uint8_t led_GetParamB(LedFunction_T _func)
{
  return (s_Cfg.func == _func) ? s_Cfg.paramB : 0;
}

uint32_t led_GetErrMask(bool _clear)
{
  taskENTER_CRITICAL();
  uint32_t mask = s_ErrMask;
  if (_clear)
    s_ErrMask &= ~mask;
  taskEXIT_CRITICAL();
  return mask;
}
