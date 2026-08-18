/*
 * i2c_slave_dispatch.c
 *
 *  Created on: Jul 28, 2026
 *      Author: Roman Egoshin
 */

#include <stdint.h>
#include "i2c_slave_dispatch.h"
#include "i2c_slave.h"

#include "to_refactor/rtc_ds1339_emu.h"
#include "to_refactor/command_server.h"

// HAL for the I2C_DIRECTION_* constants the RTC/cmd-server request API expects:
#include "stm32f0xx_hal.h"

// Own-address sources:
#include "cube-mx/i2c.h" // OWN1_I2C_ADDRESS / OWN2_I2C_ADDRESS defaults
#include "nv.h"          // nv_read_U8 / NV_ADDR_OWN_ADDRESS1 / NV_ADDR_OWN_ADDRESS2

// Fill the whole read response in one shot. _buf is the transport's tx buffer
// (capacity I2C_SLAVE_BUF_SIZE); CmdServerProcessRequest reads the register from
// _buf[0], so seed it with _reg first, then let the handler overwrite with data.
static void OnRead(uint8_t _addr, uint8_t _reg, uint8_t *_buf,
    uint16_t *_len)
{
  uint16_t len = 1;
  _buf[0] = _reg;

  if (_addr == (uint8_t) (i2c_slave_GetOwnAddress1() << 1)) // primary bank
  {
    if (_reg >= 0x80 && _reg <= 0x8F)
    {
      RtcDs1339ProcessRequest(I2C_DIRECTION_RECEIVE, _reg - 0x80, _buf, &len);
      RtcSetPointer(_reg - 0x80 + len);
    }
    else
    {
      CmdServerProcessRequest(MASTER_CMD_DIR_READ, _buf, &len);
    }
  }
  else // secondary bank (DS1339 emulation)
  {
    if (_reg <= 0x0F)
    {
      RtcDs1339ProcessRequest(I2C_DIRECTION_RECEIVE, _reg, _buf, &len);
      RtcSetPointer(_reg + len);
    }
    else
    {
      CmdServerProcessRequest(MASTER_CMD_DIR_READ, _buf, &len);
    }
  }

  *_len = len;
}

// Master clocked past the prepared block. On the DS1339 bank keep streaming with
// register auto-increment; elsewhere pad with zeros.
static uint8_t ReadOver(uint8_t _addr)
{
  if (_addr == (uint8_t) (i2c_slave_GetOwnAddress2() << 1))
  {
    static uint8_t scratch[4];
    uint8_t cmd = RtcGetPointer();
    uint16_t one = 1;
    RtcDs1339ProcessRequest(I2C_DIRECTION_RECEIVE, cmd, scratch, &one);
    RtcSetPointer(cmd + 1);
    return scratch[0];
  }
  return 0;
}

static void OnWrite(uint8_t _addr, uint8_t *_buf, uint16_t _len)
{
  if (_len <= 1)
  {
    return; // just a register-pointer set, no payload
  }

  uint8_t reg = _buf[0];
  uint16_t n;

  if (_addr == i2c_slave_GetOwnAddress1()) // primary bank (7-bit match, legacy)
  {
    if (reg >= 0x80 && reg <= 0x8F)
    {
      n = _len - 1;
      RtcDs1339ProcessRequest(I2C_DIRECTION_TRANSMIT, reg - 0x80, _buf + 1, &n);
    }
    else
    {
      n = _len;
      CmdServerProcessRequest(MASTER_CMD_DIR_WRITE, _buf, &n);
    }
  }
  else // secondary bank (DS1339 emulation)
  {
    if (reg <= 0x0F)
    {
      n = _len - 1;
      RtcDs1339ProcessRequest(I2C_DIRECTION_TRANSMIT, reg, _buf + 1, &n);
    }
    else
    {
      n = _len;
      CmdServerProcessRequest(MASTER_CMD_DIR_WRITE, _buf, &n);
    }
  }
}

static const i2c_slave_cb_t s_CallBacks = {
    .on_write = OnWrite,
    .on_read = OnRead,
    .on_read_over = ReadOver,
};

/* NV stores the own address in the 8-bit (addr<<1) form. Return the 7-bit
 * address; nv_read_U8 touches the buffer only on a valid copy, so preload it
 * with the default and ignore the result. */
static uint8_t ResolveOwnAddr(uint16_t _nv_id, uint8_t _default7)
{
  uint8_t adr8 = (uint8_t) (_default7 << 1);
  (void) nv_read_U8(_nv_id, &adr8);
  return (uint8_t) (adr8 >> 1);
}

void i2c_slave_dispatch_Init(void)
{
  i2c_slave_SetCallbacks(&s_CallBacks);

  uint8_t addr1 = ResolveOwnAddr(NV_ADDR_OWN_ADDRESS1, OWN1_I2C_ADDRESS);
  uint8_t addr2 = ResolveOwnAddr(NV_ADDR_OWN_ADDRESS2, OWN2_I2C_ADDRESS);

  i2c_slave_Init(addr1, addr2);
}
