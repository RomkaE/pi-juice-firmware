/*
 * i2c_slave.h
 *
 *  Created on: Jul 24, 2026
 *      Author: Roman Egoshin
 */

#ifndef COMPONENTS_DRIVER_I2C_I2C_SLAVE_H_
#define COMPONENTS_DRIVER_I2C_I2C_SLAVE_H_

#include <stdint.h>

typedef struct
{
  void (*on_write)(uint8_t _addr, uint8_t *_buf, uint16_t _len);
  void (*on_read)(uint8_t _addr, uint8_t _reg, uint8_t *_buf, uint16_t *_len);
  uint8_t (*on_read_over)(uint8_t _addr);
} i2c_slave_cb_t;

void i2c_slave_SetCallbacks(const i2c_slave_cb_t *_cb);

/* _addr1/_addr2 are 7-bit I2C addresses (dual-address slave). */
void i2c_slave_Init(uint8_t _addr1, uint8_t _addr2);

/* Re-apply the current configuration to the peripheral and resume listening
 * (bus recovery / after a hot address change). */
void i2c_slave_ReInit(void);

/* Change an own address on the fly (7-bit); re-applies to the peripheral and
 * resumes listen mode. */
void i2c_slave_SetOwnAddress1(uint8_t _addr1);
void i2c_slave_SetOwnAddress2(uint8_t _addr2);

uint8_t i2c_slave_GetOwnAddress1(void);
uint8_t i2c_slave_GetOwnAddress2(void);

#endif /* COMPONENTS_DRIVER_I2C_I2C_SLAVE_H_ */
