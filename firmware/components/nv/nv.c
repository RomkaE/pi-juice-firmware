/*
 * nv.c
 *
 *  Created on: 13.12.2016.
 *      Author: milan
 */
#include "nv.h"
#include "eeprom.h"

// LOG:
#include "log/log.h"

#define NV_IS_VARIABLE_VALID(var)       (((~var) & 0x00FF) == (var >> 8))

uint8_t nv_Init(void)
{
  uint8_t res = NV_ERR;
	FLASH_Unlock();
	if (EE_Init() == HAL_OK)
	  res = NV_OK;
	return res;
}

uint8_t nv_Erase(void)
{
  uint8_t res = NV_OK;
  for (uint16_t addr = NV_ADDR_START; addr < NV_ADDR_NUM; addr++)
  {
    if (EE_WriteVariable(addr, 0xFFFF) != 0)
      res = NV_ERR;
  }
  return res;
}

uint8_t nv_write_U8(uint16_t _addr, uint8_t _var)
{
  uint8_t res = NV_ERR;
  LOG_DEBUG("nv_write_U8: addr=%u, var=0x%02X", _addr, _var);
  uint16_t ee_data = (uint16_t)_var | (((uint16_t)(~_var)) << 8);
  uint16_t ee_res = EE_WriteVariable(_addr, ee_data);
  if (ee_res == 0)
    res = NV_OK;
  else
    LOG_ERROR("nv_write_U8 ERROR: addr=%u, res=0x%04X", _addr, ee_res);
  return res;
}

uint8_t nv_read_U8(uint16_t _addr, uint8_t *_p_var)
{
  uint8_t res = NV_ERR;
  uint16_t ee_data = 0;
  uint16_t ee_res = EE_ReadVariable(_addr, &ee_data);
  if (ee_res == 0)
  {
    if (NV_IS_VARIABLE_VALID(ee_data))
    {
      *_p_var = (uint8_t) (ee_data & 0x00FF);
      res = NV_OK;
    }
    else
      LOG_ERROR("nv_read_U8: invalid variable value: addr=%u, data=0x%04X", _addr, ee_data);
  }
  else
    LOG_ERROR("nv_read_U8 ERROR: addr=%u, res=0x%04X", _addr, ee_res);

  return res;
}

uint8_t nv_write_U16(uint16_t _addr, uint16_t _var)
{
  uint8_t res = NV_ERR;
  LOG_DEBUG("nv_write_U16: addr=%u, var=0x%04X", _addr, _var);
  uint16_t ee_res = EE_WriteVariable(_addr, _var);
  if (ee_res == 0)
    res = NV_OK;
  else
    LOG_ERROR("nv_write_U16 ERROR: addr=%u, res=0x%04X", _addr, ee_res);
  return res;
}

uint8_t nv_read_U16(uint16_t _addr, uint16_t *_p_var)
{
  uint8_t res = NV_ERR;
  uint16_t ee_data = 0;
  uint16_t ee_res = EE_ReadVariable(_addr, &ee_data);
  if (ee_res == 0)
  {
    *_p_var = ee_data;
    res = NV_OK;
  }
  else
    LOG_ERROR("nv_read_U16 ERROR: addr=%u, res=0x%04X", _addr, ee_res);

  return res;
}
