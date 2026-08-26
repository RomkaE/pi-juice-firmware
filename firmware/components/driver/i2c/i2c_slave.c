/*
 * i2c_slave.c
 *
 *  Created on: Jul 24, 2026
 *      Author: Roman Egoshin
 */

#include <stdint.h>
#include "i2c_slave.h"
#include "app-error/diag.h"

// ST HAL/CubeMX:
#include "stm32f0xx_hal.h"
#include "cube-mx/main.h"
#include "cube-mx/i2c.h"

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

// ISR hooks (called from i2c_common dispatch, I2C1 ISR context)

void i2c_slave_OnAddr(I2C_HandleTypeDef *hi2c, uint8_t _dir, uint16_t _addr)
{
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
  HAL_I2C_EnableListen_IT(hi2c);
}

/* Slave-side of the shared error path (I2C1): clear the AF flag left by a
 * master NACKing the end of a read. */
void i2c_slave_OnError(I2C_HandleTypeDef *hi2c)
{
  /* AF alone is the master's terminating NACK on a read - routine, not a fault.
   * Anything else (BERR/ARLO/OVR) is worth reporting. */
  if (hi2c->ErrorCode & ~(uint32_t)HAL_I2C_ERROR_AF)
    diag_Set(DIAG_I2C1_SLAVE_ERR);

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

void i2c_slave_Init(uint8_t _addr1, uint8_t _addr2)
{
  s_OwnAddr1 = _addr1;
  s_OwnAddr2 = _addr2;
  s_RxIdx = 0;
  slave_start();
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
