/*
 * i2c_master.h
 *
 *  Created on: Jul 28, 2026
 *      Author: Roman Egoshin
 */

#ifndef COMPONENTS_DRIVER_I2C_I2C_MASTER_H_
#define COMPONENTS_DRIVER_I2C_I2C_MASTER_H_

#include <stdint.h>

#define I2C_OK            (0)
#define I2C_ERR           (-1)
#define I2C_ERR_BUSY      (-2)
#define I2C_ERR_TIMEOUT   (-3)

void i2c_master_Init(void);

void i2c_master_ReInit(void);

int i2c_master_Transmit(uint8_t _addr, uint8_t *_data, uint16_t _len);

int i2c_master_Receive(uint8_t _addr, uint8_t *_data, uint16_t _len);

int i2c_master_ReadMem(uint8_t _i2c_addr, uint8_t _mem_addr, uint8_t *_data, uint16_t _len);

int i2c_master_WriteMem(uint8_t _i2c_addr, uint8_t _mem_addr, uint8_t *_data, uint16_t _len);

int i2c_master_GetI2cErrorCount(void);

#endif /* COMPONENTS_DRIVER_I2C_I2C_MASTER_H_ */
