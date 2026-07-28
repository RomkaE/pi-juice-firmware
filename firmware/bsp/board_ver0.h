/*
 * board_ver0.h
 *
 *  Created on:
 *      Author: Roman Egoshin
 */

#ifndef BOARD_VER0_H_
#define BOARD_VER0_H_

#include "stm32f0xx_hal.h"

// I2C1 slave:
#define I2C1_SCL_PORT   GPIOB
#define I2C1_SCL_PIN    GPIO_PIN_6
#define I2C1_SDA_PORT   GPIOB
#define I2C1_SDA_PIN    GPIO_PIN_7
#define I2C1_GPIO_AF    GPIO_AF1_I2C1

// I2C1 master:
#define I2C2_SCL_PORT   GPIOB
#define I2C2_SCL_PIN    GPIO_PIN_10
#define I2C2_SDA_PORT   GPIOB
#define I2C2_SDA_PIN    GPIO_PIN_11
#define I2C2_GPIO_AF    GPIO_AF1_I2C2

void SystemClock_Config(void);

#endif /* BOARD_VER0_H_ */
