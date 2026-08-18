/*
 * analog.c
 *
 *  Created on: 11.12.2016.
 *      Author: milan
 */

#include <stdint.h>
#include <stdbool.h>
#include "analog.h"
#include "nv.h"
#include "app-error/app_error.h"
#include "app-error/diag.h"
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

// Frame layout = CHSELR order, ascending by channel number (SCANDIR forward):
// CH0, CH2, CH4, CH16, CH17.
#define ADC_5VPI_CHN_IDX          0
#define ADC_VBAT_CHN_IDX          1
#define ADC_PWR_CHN_IDX           2
#define ADC_TEMP_INT_CHN_IDX      3
#define ADC_VREF_INT_CHN_IDX      4

#define ADC_SCAN_CHANNELS         5
#define ADC_FRAMES                128
#define ADC_BUFFER_LENGTH         (ADC_FRAMES * ADC_SCAN_CHANNELS)

/*
 * One DMA event covers half the ring. ADC_HALF_FRAMES must stay a power of two so the
 * per-channel average is a shift, not a divide - update ADC_HALF_SHIFT together with it.
 */
#define ADC_HALF_FRAMES           (ADC_FRAMES / 2)
#define ADC_HALF_SHIFT            6   // log2(ADC_HALF_FRAMES)

/* Frames averaged by GetAverageBatteryVoltage(). Power of two - see the shift below. */
#define VBAT_AVERAGE_FRAMES 8
#define VBAT_AVERAGE_SHIFT  15  // 12 (ADC bits) + 3 (log2 VBAT_AVERAGE_FRAMES)

/* mV at the divider tap -> mV at the battery. */
#define VBAT_FROM_PIN_MV(v) ((uint16_t)((v) * VBAT_DIVIDER_NUM / VBAT_DIVIDER_DEN))

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
static int16_t s_TempMCU = INT16_MAX;         // TODO remove magic number
static uint16_t s_VBatt;      // in mV
static uint16_t s_5VPI;       // in mV
static uint16_t s_RawPWR;

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
extern TIM_HandleTypeDef htim15;

// Half of the ring the DMA has just finished. The notification count (see ulTaskNotifyTake in
// Task) is the overrun detector: >1 pending means DMA produced more halves than we consumed.
static const uint16_t *s_pReadyHalfBuf;
static bool s_fAdcOverrun;          // ADC hardware OVR: a conversion in DR was overwritten before DMA read it
static uint32_t s_LostEvents;       // running total of half-buffers dropped because the task fell behind

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

void HAL_ADC_ErrorCallback(ADC_HandleTypeDef *hadc_)
{
  if (hadc_->Instance != ADC1)
    return;

  // ADC overrun: DR was overwritten before DMA fetched it (Overrun = OVERWRITTEN keeps the stream running).
  if (hadc_->ErrorCode & HAL_ADC_ERROR_OVR)
    s_fAdcOverrun = true;
}

static void adc_Start(void)
{
  // Start conversion in DMA mode:
  if (HAL_ADC_Start_DMA(&hadc, (uint32_t*)s_BufADC, ADC_BUFFER_LENGTH) != HAL_OK)
    APP_ERROR(APP_HAL_ERROR);

  if (HAL_TIM_Base_Start(&htim15) != HAL_OK)
    APP_ERROR(APP_HAL_ERROR);
}

static void updateAVDD(uint32_t _raw_vrefint)
{
  static uint32_t rcnt_raw_vrefint;
  if (rcnt_raw_vrefint != _raw_vrefint)
  {
    rcnt_raw_vrefint = _raw_vrefint;
    s_AVDD = ((uint32_t)VREFINT_CAL_VREF * (uint32_t)*VREFINT_CAL_ADDR) / _raw_vrefint;
    LOG_VERBOSE("Update AVDD: vref_cal = %u, raw_vref=%u, avdd=%u", *VREFINT_CAL_ADDR, _raw_vrefint, s_AVDD);
    if (s_AVDD < 3250)
      LOG_WARNING("Low AVDD value: %umV", s_AVDD);
  }
}

/*
 * Average one half of the ring (ADC_HALF_FRAMES frames) into the published values.
 * One pass, one accumulator per channel, then a shift - see ADC_HALF_SHIFT.
 *
 * VREFINT is turned into AVDD first, and every other channel is scaled against that fresh
 * AVDD, so a supply that drifts between halves does not skew the readings.
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
  uint16_t raw_vbat = acc[ADC_VBAT_CHN_IDX] >> ADC_HALF_SHIFT;
  uint32_t vbatPinMv = ((uint32_t)raw_vbat * s_AVDD) >> 12;
  s_VBatt = VBAT_FROM_PIN_MV(vbatPinMv);

  // PWR_DET raw counts. Unread since the 5V-IO detection was removed, still costs a scan slot.
  s_RawPWR = acc[ADC_PWR_CHN_IDX] >> ADC_HALF_SHIFT;

  // 5V PI bus sensed through a /2 divider: >>11 == /4096 * 2.
  uint16_t raw5v = acc[ADC_5VPI_CHN_IDX] >> ADC_HALF_SHIFT;
  s_5VPI = ((uint32_t)raw5v * s_AVDD) >> 11;

  // MCU temperature from the internal sensor (avg_slope 4.3 mV/degC).
  int32_t vtemp = (((uint32_t)(acc[ADC_TEMP_INT_CHN_IDX] >> ADC_HALF_SHIFT)) * s_AVDD * 10) >> 12;
  int32_t v30 = (((uint32_t)*TEMP30_CAL_ADDR) * TEMPSENSOR_CAL_VREFANALOG) >> 12;
  s_TempMCU = (v30 - vtemp) / 43 + 30;

  analog_SamplesReady_Callback();   // last, so the hook sees a complete set
}

static void Task(void *parameters)
{
  (void)parameters;

  LOG_INFO("ANALOG task started");

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

  while(1)
  {
    // Wake on a DMA half/full-transfer event. Return value is the pending notification count:
    //   0  -> timeout, no DMA events
    //   1  -> one half ready, normal
    //  >1  -> we fell behind; only the latest half survives, (n-1) halves were lost.
    uint32_t n_events = ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(1000));
    if (n_events == 0)
    {
      LOG_ERROR("[ADC] No ADC data stream");
      diag_Set(DIAG_ADC_NO_STREAM);
      continue;
    }

    diag_Clear(DIAG_ADC_NO_STREAM);   // a notification arrived, the stream is alive

    if (--n_events)
    {
      s_LostEvents += n_events;
      LOG_ERROR("[ADC] Data processing overrun: lost events cnt=%lu", s_LostEvents);
      diag_Set(DIAG_ADC_PROC_OVERRUN);
    }

    if (s_fAdcOverrun)
    {
      LOG_ERROR("[ADC] Hardware overrun");
      s_fAdcOverrun = false;
      diag_Set(DIAG_ADC_HW_OVERRUN);
    }

    // Capture and clear the ready-half pointer atomically w.r.t. the DMA ISR so a callback
    // firing during the swap can't have its fresh pointer clobbered by the unconditional NULL.
    HAL_NVIC_DisableIRQ(DMA1_Channel1_IRQn);
    const uint16_t *half = s_pReadyHalfBuf;
    s_pReadyHalfBuf = NULL;
    HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

    if (half != NULL)
      ProcessHalf(half);
  }
}

void analog_Init(void)
{
  LOG_INFO("analog_Init...");

  // Create task:
  s_TaskHandle = xTaskCreateStatic(Task, "ANALOG", sizeof(TaskStack)/sizeof(StackType_t),
                            NULL, 8, TaskStack, &TaskTCB);
  ASSERT(s_TaskHandle != NULL);

  // Create queue:
  s_QueHandle = xQueueCreateStatic(sizeof(s_QueBuf)/sizeof(s_QueBuf[0]),
                            sizeof(s_QueBuf[0]), (uint8_t*)s_QueBuf, &s_Que);
  ASSERT(s_QueHandle != NULL);
}

uint16_t analog_GetTempMCU(void)
{
  return s_TempMCU;
}

uint16_t analog_GetAvdd(void)
{
  return s_AVDD;
}

uint16_t analog_GetVBatt(void)
{
  return s_VBatt;
}

uint16_t analog_Get5vPi()
{
  return s_5VPI;
}

uint16_t analog_GetRawPWR(void)
{
  return s_RawPWR;
}

__attribute__((weak)) void analog_SamplesReady_Callback(void)
{
}
