/*
 * i2c_slave.c
 *
 *  Created on: Jul 24, 2026
 *      Author: Roman Egoshin
 */

#include <stdint.h>
#include "i2c_slave.h"
#include "app-error/diag.h"
#include "board.h"         

// ST HAL/CubeMX:
#include "stm32f0xx_hal.h"
#include "cube-mx/main.h"
#include "cube-mx/i2c.h"

// FreeRTOS:
#include "FreeRTOS.h"
#include "timers.h"

#define I2C_SLAVE_BUF_SIZE      256u

static const i2c_slave_cb_t *s_cb = NULL;

static uint8_t s_OwnAddr1 = 0; // 7-bit
static uint8_t s_OwnAddr2 = 0; // 7-bit

// One in-flight transaction; all touched only from the I2C1 ISR:
static uint8_t s_RxBuf[I2C_SLAVE_BUF_SIZE];
static uint8_t s_TxBuf[I2C_SLAVE_BUF_SIZE];
static uint8_t s_RxIdx = 0;
static uint8_t s_AddrMatch = 0;
static uint8_t s_Dir = 0;
static uint8_t s_OverByte = 0;

/* --- bus-wedge watchdog ----------------------------------------------------
 * NoStretch is off, so a slave that ever stops feeding the bus holds SCL low
 * and wedges it for everyone until a power cycle. A software timer watches for
 * a transaction that never ends (real ones are well under 1 ms) or an error
 * that asked for a reset, and re-inits the peripheral with a recovery clock-out
 * - the same idea as the I2C2 master's bus_recovery(). */
#define I2C_SLAVE_XFER_MAX_MS     50u  // longer than any real transaction -> wedged
#define I2C_SLAVE_WATCH_MS        20u  // supervisor tick
#define I2C_RECOVERY_DELAY_LOOPS  40u  // ~few us half-period for the recovery clock

static volatile uint8_t  s_XferActive = 0;
static volatile uint32_t s_XferDeadline = 0;
static volatile uint8_t  s_RecoverReq = 0;

/* Bumped on every host address match, both banks. Readers compare it against their
 * own snapshot, so neither side needs a lock - see i2c_slave_GetHostActivitySeq(). */
static volatile uint8_t  s_HostActivitySeq = 0;

static StaticTimer_t s_WatchTimerBuf;
static TimerHandle_t s_WatchTimer = NULL;

// ISR hooks (called from i2c_common dispatch, I2C1 ISR context)

void i2c_slave_OnAddr(I2C_HandleTypeDef *hi2c, uint8_t _dir, uint16_t _addr)
{
  s_HostActivitySeq++;                                       // the host is alive
  s_XferActive = 1;                                          // watchdog: transaction opened
  s_XferDeadline = HAL_GetTick() + I2C_SLAVE_XFER_MAX_MS;

  s_AddrMatch = (uint8_t) _addr;
  s_Dir = _dir;

  if (_dir == I2C_DIRECTION_TRANSMIT) // master writes -> we receive
  {
    if (HAL_I2C_Slave_Seq_Receive_IT(hi2c, &s_RxBuf[s_RxIdx], 1,
        I2C_FIRST_FRAME) != HAL_OK)
    {
      diag_Set(DIAG_I2C1_SLAVE_ERR);
    }
  }
  else // master reads -> build the whole response, send it in one transfer
  {
    uint16_t len = 0;
    s_cb->on_read(s_AddrMatch, s_RxBuf[0], s_TxBuf, &len);
    if (len == 0) // nothing to say; keep the bus moving with a filler byte
    {
      s_TxBuf[0] = 0;
      len = 1;
    }
    if (HAL_I2C_Slave_Seq_Transmit_IT(hi2c, s_TxBuf, len,
        I2C_FIRST_AND_NEXT_FRAME) != HAL_OK)
    {
      diag_Set(DIAG_I2C1_SLAVE_ERR);
    }
  }
}

void i2c_slave_OnRxCplt(I2C_HandleTypeDef *hi2c)
{
  s_RxIdx++;
  if (HAL_I2C_Slave_Seq_Receive_IT(hi2c, &s_RxBuf[s_RxIdx], 1,
      I2C_NEXT_FRAME) != HAL_OK)
  {
    diag_Set(DIAG_I2C1_SLAVE_ERR);
  }
}

void i2c_slave_OnTxCplt(I2C_HandleTypeDef *hi2c)
{
  // Reached only when the master keeps clocking past the prepared block.
  s_OverByte = s_cb->on_read_over(s_AddrMatch);
  if (HAL_I2C_Slave_Seq_Transmit_IT(hi2c, &s_OverByte, 1,
      I2C_NEXT_FRAME) != HAL_OK)
  {
    diag_Set(DIAG_I2C1_SLAVE_ERR);
  }
}

void i2c_slave_OnListenCplt(I2C_HandleTypeDef *hi2c)
{
  // Only a pure write carries a payload to process; a read leaves s_dir at
  // RECEIVE and was already served through on_read()/on_read_over().
  if (s_Dir == I2C_DIRECTION_TRANSMIT)
  {
    s_cb->on_write(s_AddrMatch, s_RxBuf, s_RxIdx);
  }
  s_RxIdx = 0;
  s_XferActive = 0;                     // watchdog: transaction closed cleanly
  HAL_I2C_EnableListen_IT(hi2c);
}

/* Slave-side of the shared error path (I2C1): clear the AF flag left by a
 * master NACKing the end of a read. */
void i2c_slave_OnError(I2C_HandleTypeDef *hi2c)
{
  /* AF alone is the master's terminating NACK on a read - routine, not a fault.
   * Anything else (BERR/ARLO/OVR) is worth reporting. */
  if (hi2c->ErrorCode & ~(uint32_t)HAL_I2C_ERROR_AF)
  {
    diag_Set(DIAG_I2C1_SLAVE_ERR);
    s_RecoverReq = 1;   // BERR/ARLO/OVR can leave the bus wedged - let the watchdog re-init
  }

  __HAL_I2C_CLEAR_FLAG(hi2c, I2C_FLAG_AF);
}

void i2c_slave_SetCallbacks(const i2c_slave_cb_t *_cb)
{
  s_cb = _cb;
}

static void slave_start(void)
{
  MX_I2C1_Init(); // programs the OARs from i2c_slave_GetOwnAddress1/2()
  HAL_I2C_EnableListen_IT(&hi2c1);
}

static void recovery_delay(void)
{
  volatile uint32_t n = I2C_RECOVERY_DELAY_LOOPS;
  while (n--)
  {
    __NOP();
  }
}

/* Un-wedge and re-arm the slave. Runs in the watchdog timer task, not an ISR. */
static void slave_bus_recovery(void)
{
  GPIO_InitTypeDef gpio = { 0 };

  /* Snapshot what the bus looked like while the peripheral still owned the pins:
   * SDA low with no transaction of ours is somebody else holding the line. */
  uint8_t scl_was = (uint8_t)HAL_GPIO_ReadPin(I2C1_SCL_PORT, I2C1_SCL_PIN);
  uint8_t sda_was = (uint8_t)HAL_GPIO_ReadPin(I2C1_SDA_PORT, I2C1_SDA_PIN);
  uint8_t req_was = s_RecoverReq;
  uint8_t act_was = s_XferActive;

  diag_Set(DIAG_I2C1_SLAVE_ERR);

  /* Drop the peripheral (releases SCL/SDA, clears BERR/ARLO/AF latches) and keep
   * the ISR out while we bit-bang; MspInit re-enables the NVIC line on re-init. */
  HAL_NVIC_DisableIRQ(I2C1_IRQn);
  HAL_I2C_DeInit(&hi2c1);

  __HAL_RCC_GPIOB_CLK_ENABLE();
  gpio.Mode = GPIO_MODE_OUTPUT_OD;
  gpio.Pull = GPIO_PULLUP;
  gpio.Speed = GPIO_SPEED_FREQ_LOW;
  gpio.Pin = I2C1_SCL_PIN;
  HAL_GPIO_Init(I2C1_SCL_PORT, &gpio);
  gpio.Pin = I2C1_SDA_PIN;
  HAL_GPIO_Init(I2C1_SDA_PORT, &gpio);

  /* Release both lines (open-drain '1' = high-Z). */
  HAL_GPIO_WritePin(I2C1_SCL_PORT, I2C1_SCL_PIN, GPIO_PIN_SET);
  HAL_GPIO_WritePin(I2C1_SDA_PORT, I2C1_SDA_PIN, GPIO_PIN_SET);
  recovery_delay();

  /* Up to 9 SCL pulses so a master stuck mid-byte can finish and free SDA. */
  for (uint8_t i = 0; i < 9; i++)
  {
    if (HAL_GPIO_ReadPin(I2C1_SDA_PORT, I2C1_SDA_PIN) == GPIO_PIN_SET)
    {
      break;
    }
    HAL_GPIO_WritePin(I2C1_SCL_PORT, I2C1_SCL_PIN, GPIO_PIN_RESET);
    recovery_delay();
    HAL_GPIO_WritePin(I2C1_SCL_PORT, I2C1_SCL_PIN, GPIO_PIN_SET);
    recovery_delay();
  }

  /* STOP: SDA low->high while SCL is high, for a clean bus release. */
  HAL_GPIO_WritePin(I2C1_SCL_PORT, I2C1_SCL_PIN, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(I2C1_SDA_PORT, I2C1_SDA_PIN, GPIO_PIN_RESET);
  recovery_delay();
  HAL_GPIO_WritePin(I2C1_SCL_PORT, I2C1_SCL_PIN, GPIO_PIN_SET);
  recovery_delay();
  HAL_GPIO_WritePin(I2C1_SDA_PORT, I2C1_SDA_PIN, GPIO_PIN_SET);
  recovery_delay();

  LOG_WARNING("[I2C1] Bus recovery: was SCL=%u SDA=%u, err=%u xfer=%u, SDA now %u",
      (unsigned)scl_was, (unsigned)sda_was, (unsigned)req_was, (unsigned)act_was,
      (unsigned)HAL_GPIO_ReadPin(I2C1_SDA_PORT, I2C1_SDA_PIN));

  s_RxIdx = 0;
  s_XferActive = 0;
  s_RecoverReq = 0;

  slave_start(); // MX_I2C1_Init re-runs MspInit (AF pins, NVIC) and re-arms listen
}

static void watch_cb(TimerHandle_t _t)
{
  (void) _t;
  uint8_t wedged = s_RecoverReq;
  if (s_XferActive && (int32_t) (HAL_GetTick() - s_XferDeadline) >= 0)
  {
    wedged = 1;
  }
  if (wedged)
  {
    slave_bus_recovery();
  }
}

void i2c_slave_Init(uint8_t _addr1, uint8_t _addr2)
{
  s_OwnAddr1 = _addr1;
  s_OwnAddr2 = _addr2;
  s_RxIdx = 0;
  slave_start();

  if (s_WatchTimer == NULL)
  {
    s_WatchTimer = xTimerCreateStatic("i2cslv", pdMS_TO_TICKS(I2C_SLAVE_WATCH_MS),
        pdTRUE, NULL, watch_cb, &s_WatchTimerBuf);
  }
  if (s_WatchTimer != NULL)
  {
    xTimerStart(s_WatchTimer, 0);
  }
}

void i2c_slave_ReInit(void)
{
  HAL_I2C_DeInit(&hi2c1);
  slave_start();
}

void i2c_slave_SetOwnAddress1(uint8_t _addr1)
{
  s_OwnAddr1 = _addr1;
  i2c_slave_ReInit();
}

void i2c_slave_SetOwnAddress2(uint8_t _addr2)
{
  s_OwnAddr2 = _addr2;
  i2c_slave_ReInit();
}

uint8_t i2c_slave_GetOwnAddress1(void)
{
  return s_OwnAddr1;
}

uint8_t i2c_slave_GetOwnAddress2(void)
{
  return s_OwnAddr2;
}

uint8_t i2c_slave_GetHostActivitySeq(void)
{
  return s_HostActivitySeq;
}
