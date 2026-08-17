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

#define NV_IS_VARIABLE_VALID(var)       ((~(var) & 0x00FFu) == ((var) >> 8))

/* An erased cell reads back as 0xFFFF - nv_Erase() writes exactly that. */
#define NV_ERASED_DATA                  ((uint16_t)0xFFFF)

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
  LOG_DEBUG("[NV] write_U8: addr=%u, var=0x%02X", _addr, _var);
  uint16_t ee_data = (uint16_t)_var | (((uint16_t)(~_var)) << 8);
  uint16_t ee_res = EE_WriteVariable(_addr, ee_data);
  if (ee_res == 0)
    res = NV_OK;
  else
    LOG_ERROR("[NV] write_U8 failed: addr=%u, res=0x%04X", _addr, ee_res);
  return res;
}

uint8_t nv_read_U8(uint16_t _addr, uint8_t *_p_var)
{
  uint16_t ee_data = 0;
  uint16_t ee_res = EE_ReadVariable(_addr, &ee_data);

  if (ee_res != 0)
  {
    // Never written: the parameter is simply not configured.
    LOG_DEBUG("[NV] read_U8: addr=%u not stored", _addr);
    return NV_ABSENT;
  }

  if (ee_data == NV_ERASED_DATA)
  {
    LOG_DEBUG("[NV] read_U8: addr=%u erased", _addr);
    return NV_ABSENT;
  }

  if (!NV_IS_VARIABLE_VALID(ee_data))
  {
    LOG_ERROR("[NV] read_U8: addr=%u corrupted: data=0x%04X", _addr, ee_data);
    return NV_ERR;
  }

  *_p_var = (uint8_t)(ee_data & 0x00FF);
  return NV_OK;
}

uint8_t nv_write_U16(uint16_t _addr, uint16_t _var)
{
  uint8_t res = NV_ERR;
  LOG_DEBUG("[NV] write_U16: addr=%u, var=0x%04X", _addr, _var);
  uint16_t ee_res = EE_WriteVariable(_addr, _var);
  if (ee_res == 0)
    res = NV_OK;
  else
    LOG_ERROR("[NV] write_U16 failed: addr=%u, res=0x%04X", _addr, ee_res);
  return res;
}

/*
 * No complement byte here, so an erased cell cannot be told from a stored 0xFFFF - and 0xFFFF is a
 * legitimate value (battery.c encodes "capacity undefined" as exactly that). Only "no such cell"
 * reports NV_ABSENT.
 */
uint8_t nv_read_U16(uint16_t _addr, uint16_t *_p_var)
{
  uint16_t ee_data = 0;

  if (EE_ReadVariable(_addr, &ee_data) != 0)
  {
    LOG_DEBUG("[NV] read_U16: addr=%u not stored", _addr);
    return NV_ABSENT;
  }

  *_p_var = ee_data;
  return NV_OK;
}
