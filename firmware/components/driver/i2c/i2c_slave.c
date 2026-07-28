/*
 * i2c.c
 *
 *  Created on: Jul 24, 2026
 *      Author: Roman Egoshin
 */

#include <stdint.h>
#include "i2c_slave.h"
#include "to_refactor/rtc_ds1339_emu.h"
#include "to_refactor/command_server.h"

// ST HAL/CubeMX:
#include "stm32f0xx_hal.h"
#include  "cube-mx/main.h"
#include  "cube-mx/i2c.h"

static uint8_t commandReceivedFlag = 0;
static uint16_t i2cAddrMatchCode = 0;
static int16_t readCmdCode = 0;
static uint8_t aSlaveReceiveBuffer[256] = { 0 };
static uint8_t slaveTransmitBuffer[256] = { 0 };
static uint8_t ubSlaveReceiveIndex = 0;
static uint32_t uwTransferDirection = 0;
static uint8_t tstFlagi2c = 0;
static uint16_t dataLen;

void i2c_slave_OnTxCplt(I2C_HandleTypeDef *hi2c)
{
  tstFlagi2c = 9;
  dataLen = 1;
  if (i2cAddrMatchCode == hi2c->Init.OwnAddress2)
  {
    uint8_t cmd = RtcGetPointer();
    RtcDs1339ProcessRequest(I2C_DIRECTION_RECEIVE, cmd, slaveTransmitBuffer,
        &dataLen);
    RtcSetPointer(cmd + 1);
    HAL_I2C_Slave_Seq_Transmit_IT(hi2c, (uint8_t*) slaveTransmitBuffer, 1,
        I2C_NEXT_FRAME);
  }
  else
  {
    slaveTransmitBuffer[0] = 0;
    HAL_I2C_Slave_Seq_Transmit_IT(hi2c, (uint8_t*) slaveTransmitBuffer, 1,
        I2C_NEXT_FRAME);
  }
}

void i2c_slave_OnRxCplt(I2C_HandleTypeDef *hi2c1)
{
  ubSlaveReceiveIndex++;
  tstFlagi2c = 1;
  if (HAL_I2C_Slave_Seq_Receive_IT(hi2c1,
      (uint8_t*) &aSlaveReceiveBuffer[ubSlaveReceiveIndex], 1, I2C_NEXT_FRAME)
      != HAL_OK)
  {
    Error_Handler();
  }
  tstFlagi2c = 2;
}

void i2c_slave_OnAddr(I2C_HandleTypeDef *hi2c, uint8_t TransferDirection,
    uint16_t AddrMatchCode)
{
  i2cAddrMatchCode = AddrMatchCode;
  //uwTransferInitiated = 1;
  uwTransferDirection = TransferDirection;

  // First of all, check the transfer direction to call the correct Slave Interface
  if (uwTransferDirection == I2C_DIRECTION_TRANSMIT)
  {
    tstFlagi2c = 3;
    if (HAL_I2C_Slave_Seq_Receive_IT(hi2c,
        (uint8_t*) &aSlaveReceiveBuffer[ubSlaveReceiveIndex], 1,
        I2C_FIRST_FRAME) != HAL_OK)
    {
      Error_Handler();
    }
    tstFlagi2c = 4;
  }
  else
  {
    dataLen = 1;
    readCmdCode = aSlaveReceiveBuffer[0];
    slaveTransmitBuffer[0] = readCmdCode;

    if (AddrMatchCode == hi2c->Init.OwnAddress1)
    {
      if (readCmdCode >= 0x80 && readCmdCode <= 0x8F)
      {
        RtcDs1339ProcessRequest(I2C_DIRECTION_RECEIVE, readCmdCode - 0x80,
            slaveTransmitBuffer, &dataLen);
        RtcSetPointer(readCmdCode - 0x80 + dataLen);
      }
      else
      {
        CmdServerProcessRequest(MASTER_CMD_DIR_READ, slaveTransmitBuffer,
            &dataLen);
      }
      tstFlagi2c = 11;
    }
    else
    {
      if (readCmdCode <= 0x0F)
      {
        RtcDs1339ProcessRequest(I2C_DIRECTION_RECEIVE, readCmdCode,
            slaveTransmitBuffer, &dataLen);
        RtcSetPointer(readCmdCode + dataLen);
      }
      else
      {
        CmdServerProcessRequest(MASTER_CMD_DIR_READ, slaveTransmitBuffer,
            &dataLen);
      }
      tstFlagi2c = 12;
    }
    if (HAL_I2C_Slave_Seq_Transmit_IT(hi2c, (uint8_t*) slaveTransmitBuffer,
        dataLen, I2C_FIRST_AND_NEXT_FRAME) != HAL_OK)
    {
      Error_Handler();
    }
  }

  // TODO - WTF?!
//  PowerMngmtHostPollEvent();
//  MS_TIME_COUNTER_INIT(lastHostCommandTimer);
}

void i2c_slave_OnListenCplt(I2C_HandleTypeDef *hi2c)
{
  tstFlagi2c = 7;
  //uwTransferEnded = 1;
  //uwTransferDirection = I2C_GET_DIR(hi2c);
  if (uwTransferDirection == I2C_DIRECTION_TRANSMIT)
  {
    dataLen = ubSlaveReceiveIndex;
    readCmdCode = aSlaveReceiveBuffer[0];
    if (dataLen > 1)
    {
      if (i2cAddrMatchCode == (hi2c->Init.OwnAddress1 >> 1))
      {
        if (readCmdCode >= 0x80 && readCmdCode <= 0x8F)
        {
          dataLen -= 1; // first is command
          RtcDs1339ProcessRequest(I2C_DIRECTION_TRANSMIT, readCmdCode - 0x80,
              aSlaveReceiveBuffer + 1, &dataLen);
        }
        else
        {
          CmdServerProcessRequest(MASTER_CMD_DIR_WRITE, aSlaveReceiveBuffer,
              &dataLen);
          commandReceivedFlag = 1;
        }
      }
      else
      {
        if (readCmdCode <= 0x0F)
        {
          // rtc emulation range
          dataLen -= 1; // first is command
          RtcDs1339ProcessRequest(I2C_DIRECTION_TRANSMIT, readCmdCode,
              aSlaveReceiveBuffer + 1, &dataLen);
        }
        else
        {
          CmdServerProcessRequest(MASTER_CMD_DIR_WRITE, aSlaveReceiveBuffer,
              &dataLen);
          commandReceivedFlag = 1;
        }
      }
    }
  }

  ubSlaveReceiveIndex = 0;
  HAL_I2C_EnableListen_IT(hi2c);
  tstFlagi2c = 8;
}

/* Slave-side of the shared error path (dispatched by i2c_common for I2C1):
 * clear the AF flag left by a master NACKing the end of a read. */
void i2c_slave_OnError(I2C_HandleTypeDef *hi2c)
{
  __HAL_I2C_CLEAR_FLAG(hi2c, I2C_FLAG_AF);
}

void i2c_slave_Init(void)
{
  MX_I2C1_Init();
  HAL_I2C_EnableListen_IT(&hi2c1);
}

void i2c_slave_ReInit(void)
{
  HAL_I2C_DeInit(&hi2c1);
  i2c_slave_Init();
}
