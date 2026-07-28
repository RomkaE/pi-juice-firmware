/*
 * i2c_slave.h
 *
 *  Created on: Jul 24, 2026
 *      Author: Roman Egoshin
 */

#ifndef COMPONENTS_DRIVER_I2C_I2C_SLAVE_H_
#define COMPONENTS_DRIVER_I2C_I2C_SLAVE_H_

#include "stm32f0xx_hal.h"

void i2c_slave_Init(void);

void i2c_slave_ReInit(void);

#endif /* COMPONENTS_DRIVER_I2C_I2C_SLAVE_H_ */
