/*
 * analog.c
 *
 *  Created on: 11.12.2016.
 *      Author: milan
 */

#include <stdint.h>
#include "analog.h"
#include <to_refactor/config_switch_resistor.h>
#include <to_refactor/time_count.h>
#include "stm32f0xx_hal.h"
#include "nv.h"
#include "app-error/app_error.h"
#include "app-error/app_assert.h"

// ST HAL/CubeMX:
#include "stm32f0xx_hal.h"
#include "stm32f0xx_ll_adc.h"
#include "cube-mx/adc.h"
#include "cube-mx/tim.h"

// FreeRTOS:
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

// LOG:
#include "log/log.h"

/* mcuTemperature sensor calibration value address */
#define TEMP30_CAL_ADDR           TEMPSENSOR_CAL1_ADDR

// TODO -remove
//#define ADC_SEQUENCE_TIMEOUT_MS		5

// Frame layout = CHSELR order, ascending by channel number (SCANDIR forward):
// CH0, CH2, CH4, CH16, CH17.
#define ADC_5VPI_CHN_IDX          0
#define ADC_VBAT_CHN_IDX          1
#define ADC_POW_CHN_IDX           2
#define ADC_TEMP_INT_CHN_IDX      3
#define ADC_VREF_INT_CHN_IDX      4

#define ADC_SCAN_CHANNELS         5
#define ADC_FRAMES                64
#define ADC_BUFFER_LENGTH         (ADC_FRAMES * ADC_SCAN_CHANNELS)

/*
 * One DMA event covers half the ring. ADC_HALF_FRAMES must stay a power of two so the
 * per-channel average is a shift, not a divide - update ADC_HALF_SHIFT together with it.
 */
#define ADC_HALF_FRAMES           (ADC_FRAMES / 2)
#define ADC_HALF_SHIFT            5   // log2(ADC_HALF_FRAMES)

typedef enum
{
  EVT_CMD_BUZZER = 1,
  EVT_CMD_POWEROFF,
  EVT_CMD_DFU
} Event_t;

typedef struct
{
  uint8_t frequency;
  uint8_t mode;
  uint16_t duration;
} Buzzer_t;

typedef struct
{
  uint32_t timeout;
} PowerOff_t;

typedef struct
{
  Event_t type;
  union {
    Buzzer_t buzzer;
    PowerOff_t poweroff;
  } data;
} EventWrapper_t;

static uint16_t s_BufADC[ADC_BUFFER_LENGTH];  // raw data from ADC

static uint16_t s_AVDD;

// Measured parameters:
static uint8_t s_HwRev = HARD_REV_UNKNOWN;
static int16_t s_TempMCU = INT16_MAX;         // TODO remove magic number
static uint16_t s_RawBatt;
static uint16_t s_VBatt;      // in mV
static uint16_t s_VBattAvg;   // in mV
static uint16_t s_5VPI;       // in mV
static uint16_t s_RawPOW;

// FreeRTOS task:
static TaskHandle_t s_TaskHandle;
static StaticTask_t TaskTCB;
static StackType_t TaskStack[512];  // TODO - remove magic number

// Event queue:
static QueueHandle_t s_QueHandle;
static StaticQueue_t s_Que;
static EventWrapper_t s_QueBuf[10];   // TODO - remove magic number

// HAL instances:
extern ADC_HandleTypeDef hadc;

// Half of the ring the DMA has just finished:
static const uint16_t *volatile s_pReadyHalfBuf;

void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef *hadc_)
{
  if (hadc_->Instance != ADC1)
    return;

  s_pReadyHalfBuf = &s_BufADC[0];

  // Notify task:
  BaseType_t woken = pdFALSE;
  vTaskNotifyGiveFromISR(s_TaskHandle, &woken);
  portYIELD_FROM_ISR(woken);
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc_)
{
  if (hadc_->Instance != ADC1)
    return;

  s_pReadyHalfBuf = &s_BufADC[ADC_BUFFER_LENGTH / 2];

  // Notify task:
  BaseType_t woken = pdFALSE;
  vTaskNotifyGiveFromISR(s_TaskHandle, &woken);
  portYIELD_FROM_ISR(woken);
}

static void adc_Start(void)
{
  // Start conversion in DMA mode:
  if (HAL_ADC_Start_DMA(&hadc, (uint32_t*)s_BufADC, ADC_BUFFER_LENGTH) != HAL_OK)
    APP_ERROR(APP_HAL_ERROR);
}

static void updateAVDD(uint32_t _raw_vrefint)
{
  static uint32_t rcnt_raw_vrefint;
  if (rcnt_raw_vrefint != _raw_vrefint)
  {
    rcnt_raw_vrefint = _raw_vrefint;
    s_AVDD = ((uint32_t)VREFINT_CAL_VREF * (uint32_t)*VREFINT_CAL_ADDR) / _raw_vrefint;
    LOG_INFO("Update AVDD: vref_cal = %u, raw_vref=%u, avdd=%u", *VREFINT_CAL_ADDR, _raw_vrefint, s_AVDD);
  }
}

/*
 * One-shot board-revision probe. PA1 (ADC_CS2_CHN_IDX) carries the NCS213 current-sense amp
 * output on >=2.3 hardware (idles near 1570 counts while PA0/5V-sense sits ~3100) and a
 * plain shunt tap that tracks PA0 within a few counts on <2.3. A single comparison with a
 * wide margin (3x the largest shunt drop, 3x below the NCS idle offset) separates the two.
 * Needs the 5V rail up so both taps are energised; retries every task tick until then.
 */
static void DetectHardwareRev(void)
{
  // TODO
  /*
  if (s_HwRev != HARD_REV_UNKNOWN)
    return;
  if (!AnalogSamplesReady() || analog_Get5vPi() <= 4500)
    return;

  int32_t d = (int32_t) GetSample(ADC_CS1_CHN_IDX)
      - (int32_t) GetSample(ADC_CS2_CHN_IDX);
  s_HwRev = (d > 500) ? HARD_REV_2_3_AND_ABOVE : HARD_REV_BELOW_2_3;
  */
}

/*
 * Index of the freshest complete sample of `channel` in the DMA ring.
 *
 * The DMA counter runs down, so ADC_BUFFER_LENGTH - counter - 1 is the last cell written.
 * Truncating that to a frame boundary gives the frame in progress; if the requested channel
 * has not been converted in it yet, step back one frame.
 */

/*
 * GetSampleVoltage() lived here: a ratiometric channel-to-mV conversion that took VREFINT
 * from the same frame instead of the AnalogTask-refreshed AVDD. Its last caller was the IO1
 * analog read; removed with it. Worth restoring if a measurement ever needs to be immune to
 * AVDD moving between frames.
 */

/* Frames averaged by GetAverageBatteryVoltage(). Power of two - see the shift below. */
#define VBAT_AVERAGE_FRAMES	8
#define VBAT_AVERAGE_SHIFT	15	// 12 (ADC bits) + 3 (log2 VBAT_AVERAGE_FRAMES)

/* mV at the divider tap -> mV at the battery. */
#define VBAT_FROM_PIN_MV(v)	((uint16_t)((v) * VBAT_DIVIDER_NUM / VBAT_DIVIDER_DEN))

/*
 * Average one half of the ring (ADC_HALF_FRAMES frames) into the published values.
 * One pass, one accumulator per channel, then a shift - see ADC_HALF_SHIFT.
 *
 * VREFINT is turned into AVDD first, and every other channel is scaled against that fresh
 * AVDD, so a supply that drifts between halves does not skew the readings.
 *
 * TODO(review): channel-to-mV scaling below is carried over from the pre-DMA code
 * (Get5vIoVoltage / GetBatteryVoltage / the temperature block). Confirm against the board.
 */
static void ProcessHalf(const uint16_t *half)
{
  uint32_t acc[ADC_SCAN_CHANNELS] = {0};

  for (uint16_t f = 0; f < ADC_HALF_FRAMES; f++)
  {
    const uint16_t *frame = half + (uint32_t)f * ADC_SCAN_CHANNELS;
    for (uint8_t c = 0; c < ADC_SCAN_CHANNELS; c++)
      acc[c] += frame[c];
  }

  // AVDD:
  uint16_t rawVref = acc[ADC_VREF_INT_CHN_IDX] >> ADC_HALF_SHIFT;
  if (rawVref != 0)               // zero only before the ring has filled; the macro divides by it
    updateAVDD(rawVref);

  // Battery: tap voltage against AVDD, then back through the divider.
  s_RawBatt = acc[ADC_VBAT_CHN_IDX] >> ADC_HALF_SHIFT;
  uint32_t vbatPinMv = ((uint32_t)s_RawBatt * s_AVDD) >> 12;
  s_VBatt = VBAT_FROM_PIN_MV(vbatPinMv);
  s_VBattAvg = s_VBatt;           // the 32-frame average already is the smoothed value

  // POW_DET raw counts, consumed by the 5V-in detection in power_source.c.
  s_RawPOW = acc[ADC_POW_CHN_IDX] >> ADC_HALF_SHIFT;

  // 5V PI rail sensed through a /2 divider: >>11 == /4096 * 2.
  uint16_t raw5v = acc[ADC_5VPI_CHN_IDX] >> ADC_HALF_SHIFT;
  s_5VPI = ((uint32_t)raw5v * s_AVDD) >> 11;

  // MCU temperature from the internal sensor (avg_slope 4.3 mV/degC).
  int32_t vtemp = (((uint32_t)(acc[ADC_TEMP_INT_CHN_IDX] >> ADC_HALF_SHIFT)) * s_AVDD * 10) >> 12;
  int32_t v30 = (((uint32_t)*TEMP30_CAL_ADDR) * TEMPSENSOR_CAL_VREFANALOG) >> 12;
  s_TempMCU = (v30 - vtemp) / 43 + 30;
}

static void Task(void *parameters)
{
  (void)parameters;

  LOG_INFO("ANALOG task started");

  // TODO - read HW revision

  // Init measurement system:
  MX_ADC_Init();
  MX_TIM15_Init();

  // Check configuration:
  ASSERT(__builtin_popcount(hadc.Instance->CHSELR) == ADC_SCAN_CHANNELS);

  // ADC calibration before the first conversion:
  if (HAL_ADCEx_Calibration_Start(&hadc) != HAL_OK)
    APP_ERROR(APP_HAL_ERROR);

  // Arm the DMA (ADC now waits on the trigger), then let TIM15 TRGO pace the conversions.
  adc_Start();
  if (HAL_TIM_Base_Start(&htim15) != HAL_OK)
    APP_ERROR(APP_HAL_ERROR);

  while(1)
  {
    // Wake on a DMA half/full-transfer event; the timeout is a stall watchdog.
    if (ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(1000)) == 0)
    {
      // TODO error logs and error processing! measurement system stopped (no DMA events).
      continue;
    }

    const uint16_t *half = s_pReadyHalfBuf;
    if (half != NULL)
      ProcessHalf(half);

    // TODO - one-shot HW revision probe once the 5V rail is up.
    // DetectHardwareRev();
  }
}

void analog_Init(void)
{
  // Create task:
  s_TaskHandle = xTaskCreateStatic(Task, "ANALOG", sizeof(TaskStack)/sizeof(StackType_t),
                            NULL, 8, TaskStack, &TaskTCB);
  ASSERT(s_TaskHandle != NULL);

  // Create queue:
  s_QueHandle = xQueueCreateStatic(sizeof(s_QueBuf)/sizeof(s_QueBuf[0]),
                            sizeof(s_QueBuf[0]), (uint8_t*)s_QueBuf, &s_Que);
  ASSERT(s_QueHandle != NULL);
}

uint8_t analog_GetHwRev(void)
{
  return s_HwRev;
}

uint16_t analog_GetTempMCU(void)
{
  return s_TempMCU;
}

uint16_t analog_GetAvdd(void)
{
  return s_AVDD;
}

uint16_t analog_GetRawBatt(void)
{
  return s_RawBatt;
}

uint16_t analog_GetVBatt(void)
{
  // TODO
  /*
  uint32_t pinMv = ((uint32_t)GetSample(ADC_VBAT_CHN_IDX) * analog_GetAvdd()) >> 12;
  return VBAT_FROM_PIN_MV(pinMv);
  */
  return s_VBatt;
}

uint16_t analog_GetVBattAvg(void)
{
  // TODO
  /*
  uint32_t sum = GetSampleSum(ADC_VBAT_CHN_IDX, VBAT_AVERAGE_FRAMES);
  uint32_t pinMv = (sum * analog_GetAvdd()) >> VBAT_AVERAGE_SHIFT;
  return VBAT_FROM_PIN_MV(pinMv);
  */
  return s_VBattAvg;
}

uint16_t analog_Get5vPi()
{
  return s_5VPI;
  // TODO
//  int16_t adcAvg = GetSampleSum( , 4) >> 2;
//  return (s_AVDD > 3200 && s_AVDD < 3400) ? (adcAvg * s_AVDD) >> 11 : (adcAvg * 3300) >> 11;//adcAvg * s_AVDD / 4096 * 2;
}

uint16_t analog_GetRawPOW(void)
{
  return s_RawPOW;
}
