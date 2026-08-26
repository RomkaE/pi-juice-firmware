/*
 * i2c_common.c
 *
 *  Created on: Jul 28, 2026
 *      Author: Roman Egoshin
 */

#include "stm32f0xx_hal.h"

void i2c_master_OnComplete(void);
void i2c_master_OnError(void);
void i2c_slave_OnAddr(I2C_HandleTypeDef *hi2c, uint8_t TransferDirection, uint16_t AddrMatchCode);
void i2c_slave_OnTxCplt(I2C_HandleTypeDef *hi2c);
void i2c_slave_OnRxCplt(I2C_HandleTypeDef *hi2c);
void i2c_slave_OnListenCplt(I2C_HandleTypeDef *hi2c);
void i2c_slave_OnError(I2C_HandleTypeDef *hi2c);

/* -------- master (I2C2) completions ---------------------------------------*/

void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *hi2c)
{
  if (hi2c->Instance == I2C2)
  {
    i2c_master_OnComplete();
  }
}

void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
  if (hi2c->Instance == I2C2)
  {
    i2c_master_OnComplete();
  }
}

void HAL_I2C_MemTxCpltCallback(I2C_HandleTypeDef *hi2c)
{
  if (hi2c->Instance == I2C2)
  {
    i2c_master_OnComplete();
  }
}

void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
  if (hi2c->Instance == I2C2)
  {
    i2c_master_OnComplete();
  }
}

/* -------- slave (I2C1) listen-mode events ---------------------------------*/

void HAL_I2C_AddrCallback(I2C_HandleTypeDef *hi2c, uint8_t TransferDirection,
    uint16_t AddrMatchCode)
{
  if (hi2c->Instance == I2C1)
  {
    i2c_slave_OnAddr(hi2c, TransferDirection, AddrMatchCode);
  }
}

void HAL_I2C_SlaveTxCpltCallback(I2C_HandleTypeDef *hi2c)
{
  if (hi2c->Instance == I2C1)
  {
    i2c_slave_OnTxCplt(hi2c);
  }
}

void HAL_I2C_SlaveRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
  if (hi2c->Instance == I2C1)
  {
    i2c_slave_OnRxCplt(hi2c);
  }
}

void HAL_I2C_ListenCpltCallback(I2C_HandleTypeDef *hi2c)
{
  if (hi2c->Instance == I2C1)
  {
    i2c_slave_OnListenCplt(hi2c);
  }
}

/* -------- error (both units) ----------------------------------------------*/

void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c)
{
  if (hi2c->Instance == I2C2)
  {
    i2c_master_OnError();
  }
  else
  {
    i2c_slave_OnError(hi2c);
  }
}
