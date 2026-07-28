/*
 * i2c_master.c
 *
 *  Created on: Jul 28, 2026
 *      Author: Roman Egoshin
 *
 *  I2C master driver, hardwired to I2C2 (BQ2416x charger @0xD6, LC709203F fuel
 *  gauge @0x16). Interrupt-driven (no DMA - the bus is slow and traffic sparse,
 *  DMA buys nothing here). The public wrappers stay synchronous for callers:
 *  they kick off a HAL *_IT transfer and block on a completion semaphore with a
 *  timeout. The HAL master/mem callbacks (and the error callback) run in the
 *  I2C2 ISR and just give that semaphore; the wrapper then reads hi2c2.ErrorCode
 *  to tell success from a NACK/bus error and applies the retry/recovery policy.
 *
 *  One transfer is in flight at a time (the bus mutex serialises callers), so a
 *  single completion semaphore and hi2c2's own state are all the shared state
 *  the ISR path needs.
 */

#include "i2c_master.h"
#include "board.h"

// ST HAL/CubeMX:
#include "stm32f0xx_hal.h"
#include "cube-mx/i2c.h"

// FreeRTOS:
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

// LOG:
#include "log/log.h"

/* Plain re-tries before we bother touching the bus lines. */
#define I2C_MASTER_ATTEMPTS    3u

/* IT completion wait. The transfer signals the semaphore the instant it ends
 * (complete or error), so this only bounds a hung bus - generous is fine. The
 * FreeRTOS tick is 100 Hz (10 ms) here, so sub-10 ms values would round to 0;
 * keep it in tens of ms. A NACK returns via the error callback, not this. */
#define I2C_XFER_TIMEOUT_TICKS  pdMS_TO_TICKS(50)

/* Half-period of the recovery clock as a busy-loop count. At 8 MHz this is a
 * few microseconds -> ~20-30 kHz SCL. Exact value is not critical. */
#define I2C_RECOVERY_DELAY_LOOPS  40u

static uint16_t i2c_master_recoveryCount = 0;
static uint16_t i2c_master_errorCount = 0;

#define I2C_SAT(c) do { if ((c) < 0xFFFF) (c)++; } while (0)

typedef enum
{
  OP_TRANSMIT, OP_RECEIVE, OP_READMEM, OP_WRITEMEM
} MasterOp_t;

static const char *const s_opName[] = { "TX", "RX", "RDMEM", "WRMEM" };

// I2C bus mutex (serialises callers) + per-transfer completion signal:
static StaticSemaphore_t s_i2cMutexBuf;
static SemaphoreHandle_t s_i2cMutex = NULL;
static StaticSemaphore_t s_doneBuf;
static SemaphoreHandle_t s_done = NULL;

static void bus_recovery(void);

//ISR hooks (called from i2c_common dispatch, I2C2 ISR context):
void i2c_master_OnComplete(void)
{
  BaseType_t woken = pdFALSE;
  xSemaphoreGiveFromISR(s_done, &woken);
  portYIELD_FROM_ISR(woken);
}

void i2c_master_OnError(void)
{
  BaseType_t woken = pdFALSE;
  xSemaphoreGiveFromISR(s_done, &woken);
  portYIELD_FROM_ISR(woken);
}

static int hal2res(HAL_StatusTypeDef _res_hal)
{
  switch (_res_hal)
  {
    case HAL_OK:
      return I2C_OK;
    case HAL_BUSY:
      return I2C_ERR_BUSY;
    case HAL_TIMEOUT:
      return I2C_ERR_TIMEOUT;
    default:
      return I2C_ERR;
  }
}

static inline void bus_lock(void)
{
  xSemaphoreTake(s_i2cMutex, portMAX_DELAY);
}

static inline void bus_unlock(void)
{
  xSemaphoreGive(s_i2cMutex);
}

/*
 * A stuck bus benefits from recovery; a plain slave NAK does not. On a NAK
 * HAL leaves ONLY the AF flag set (device absent/busy) - re-initing for that
 * just churns. A held line shows up as our HAL_TIMEOUT, or as BERR/ARLO in the
 * error code - those we recover.
 */
static uint8_t needs_recovery(HAL_StatusTypeDef _res_hal)
{
  if (_res_hal == HAL_TIMEOUT)
  {
    return 1;
  }
  return (HAL_I2C_GetError(&hi2c2) & ~(uint32_t) HAL_I2C_ERROR_AF) != 0u;
}

static void recovery_delay(void)
{
  volatile uint32_t n = I2C_RECOVERY_DELAY_LOOPS;
  while (n--)
  {
    __NOP();
  }
}

static void bus_recovery(void)
{
  GPIO_InitTypeDef gpio = { 0 };

  I2C_SAT(i2c_master_recoveryCount);
  LOG_ERROR("[I2C2] bus stuck -> recovery/re-init (#%u)",
      (unsigned) i2c_master_recoveryCount);

  /* Drop the peripheral: it releases SCL/SDA, aborts any in-flight IT and
   * clears its error latches (BERR/ARLO/AF). MspDeInit gates the I2C2 clock
   * and disables the NVIC line, so no callback can fire past this point. */
  HAL_I2C_DeInit(&hi2c2);

  __HAL_RCC_GPIOB_CLK_ENABLE();

  /* Both lines as open-drain GPIO. In OD-output mode IDR still reflects
   * the real line level, so we can sense SDA while also driving it. */
  gpio.Mode = GPIO_MODE_OUTPUT_OD;
  gpio.Pull = GPIO_PULLUP;
  gpio.Speed = GPIO_SPEED_FREQ_LOW;
  gpio.Pin = I2C2_SCL_PIN;
  HAL_GPIO_Init(I2C2_SCL_PORT, &gpio);
  gpio.Pin = I2C2_SDA_PIN;
  HAL_GPIO_Init(I2C2_SDA_PORT, &gpio);

  /* Release both lines high (we drive '1' = high-Z on open drain). */
  HAL_GPIO_WritePin(I2C2_SCL_PORT, I2C2_SCL_PIN, GPIO_PIN_SET);
  HAL_GPIO_WritePin(I2C2_SDA_PORT, I2C2_SDA_PIN, GPIO_PIN_SET);
  recovery_delay();

  /* Up to 9 SCL pulses so a slave stuck mid-byte can finish and let SDA go. */
  for (uint8_t i = 0; i < 9; i++)
  {
    if (HAL_GPIO_ReadPin(I2C2_SDA_PORT, I2C2_SDA_PIN) == GPIO_PIN_SET)
    {
      break; /* SDA free -> bus no longer held */
    }
    HAL_GPIO_WritePin(I2C2_SCL_PORT, I2C2_SCL_PIN, GPIO_PIN_RESET);
    recovery_delay();
    HAL_GPIO_WritePin(I2C2_SCL_PORT, I2C2_SCL_PIN, GPIO_PIN_SET);
    recovery_delay();
  }

  /* STOP condition: SDA low->high while SCL is high, so any slave that was
   * still tracking the frame sees a clean bus release. */
  HAL_GPIO_WritePin(I2C2_SCL_PORT, I2C2_SCL_PIN, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(I2C2_SDA_PORT, I2C2_SDA_PIN, GPIO_PIN_RESET);
  recovery_delay();
  HAL_GPIO_WritePin(I2C2_SCL_PORT, I2C2_SCL_PIN, GPIO_PIN_SET);
  recovery_delay();
  HAL_GPIO_WritePin(I2C2_SDA_PORT, I2C2_SDA_PIN, GPIO_PIN_SET);
  recovery_delay();

  /* Back to the peripheral: MX_I2C2_Init re-runs MspInit (AF pins, NVIC)
   * and restores timing + the digital filter from cube-mx/i2c.c. */
  MX_I2C2_Init();
}

static HAL_StatusTypeDef start_xfer(MasterOp_t _op, uint8_t _dev_addr,
    uint8_t _mem_addr, uint8_t *_p_data, uint16_t _len)
{
  switch (_op)
  {
    case OP_TRANSMIT:
      return HAL_I2C_Master_Transmit_IT(&hi2c2, _dev_addr, _p_data, _len);
    case OP_RECEIVE:
      return HAL_I2C_Master_Receive_IT(&hi2c2, _dev_addr, _p_data, _len);
    case OP_READMEM:
      return HAL_I2C_Mem_Read_IT(&hi2c2, _dev_addr, _mem_addr,
          I2C_MEMADD_SIZE_8BIT, _p_data, _len);
    case OP_WRITEMEM:
      return HAL_I2C_Mem_Write_IT(&hi2c2, _dev_addr, _mem_addr,
          I2C_MEMADD_SIZE_8BIT, _p_data, _len);
    default:
      return HAL_ERROR;
  }
}

static HAL_StatusTypeDef once_xfer(MasterOp_t _op, uint8_t _dev_addr,
    uint8_t _mem_addr, uint8_t *_p_data, uint16_t _len)
{
  HAL_StatusTypeDef hal_res;
  (void) xSemaphoreTake(s_done, 0);

  hal_res = start_xfer(_op, _dev_addr, _mem_addr, _p_data, _len);
  if (hal_res != HAL_OK)
    return hal_res;

  // Wait for the bus operation to complete:
  if (xSemaphoreTake(s_done, I2C_XFER_TIMEOUT_TICKS) != pdTRUE)
    return HAL_TIMEOUT;

  // Check operation error code from HAL:
  uint32_t i2c_err = HAL_I2C_GetError(&hi2c2);
  return (i2c_err == HAL_I2C_ERROR_NONE) ? HAL_OK : HAL_ERROR;
}

static int bus_transaction(MasterOp_t _op, uint8_t _dev_addr, uint8_t _mem_addr,
    uint8_t *_p_data, uint16_t _len)
{
  HAL_StatusTypeDef hal_res = HAL_ERROR;
  uint32_t hal_err = 0;

  bus_lock();
  {
    for (int i = 0; i < I2C_MASTER_ATTEMPTS; i++)
    {
      hal_res = once_xfer(_op, _dev_addr, _mem_addr, _p_data, _len);
      if (hal_res == HAL_OK)
        break;
      if (needs_recovery(hal_res))
        bus_recovery();
    }

    if (hal_res != HAL_OK)
    {
      hal_err = HAL_I2C_GetError(&hi2c2);
      I2C_SAT(i2c_master_errorCount);
    }
  }
  bus_unlock();

  if (hal_res != HAL_OK)
  {
    LOG_WARNING("[I2C2] %s a=0x%02X r=0x%02X n=%u failed: res=%d err=0x%lX",
        s_opName[_op], _dev_addr, _mem_addr, _len, hal_res, hal_err);
  }

  return hal2res(hal_res);
}
void i2c_master_Init(void)
{
  LOG_INFO("i2c_master_Init...");
  if (s_i2cMutex == NULL)
    s_i2cMutex = xSemaphoreCreateMutexStatic(&s_i2cMutexBuf);
  if (s_done == NULL)
    s_done = xSemaphoreCreateBinaryStatic(&s_doneBuf);
  MX_I2C2_Init();
}

void i2c_master_ReInit(void)
{
  LOG_WARNING("i2c_master_ReInit...");
  bus_lock();
  bus_recovery();
  bus_unlock();
}

int i2c_master_Transmit(uint8_t _addr, uint8_t *_data, uint16_t _len)
{
  return bus_transaction(OP_TRANSMIT, _addr, 0, _data, _len);
}

int i2c_master_Receive(uint8_t _addr, uint8_t *_data, uint16_t _len)
{
  return bus_transaction(OP_RECEIVE, _addr, 0, _data, _len);
}

int i2c_master_ReadMem(uint8_t _i2c_addr, uint8_t _mem_addr, uint8_t *_data, uint16_t _len)
{
  return bus_transaction(OP_READMEM, _i2c_addr, _mem_addr, _data, _len);
}

int i2c_master_WriteMem(uint8_t _i2c_addr, uint8_t _mem_addr, uint8_t *_data, uint16_t _len)
{
  return bus_transaction(OP_WRITEMEM, _i2c_addr, _mem_addr, _data, _len);
}
