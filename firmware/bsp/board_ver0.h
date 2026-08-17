/*
 * board_ver0.h
 *
 *  Created on:
 *      Author: Roman Egoshin
 */

#ifndef BOARD_VER0_H_
#define BOARD_VER0_H_

#include "stm32f0xx_hal.h"

/*============================ I2C ===========================================*/

// I2C1 slave (host / Raspberry Pi bus):
#define I2C1_SCL_PORT         GPIOB           // PB6
#define I2C1_SCL_PIN          GPIO_PIN_6
#define I2C1_SDA_PORT         GPIOB           // PB7
#define I2C1_SDA_PIN          GPIO_PIN_7
#define I2C1_GPIO_AF          GPIO_AF1_I2C1

// I2C2 master (on-board charger / fuel gauge / ID EEPROM):
#define I2C2_SCL_PORT         GPIOB           // PB10
#define I2C2_SCL_PIN          GPIO_PIN_10
#define I2C2_SDA_PORT         GPIOB           // PB11
#define I2C2_SDA_PIN          GPIO_PIN_11
#define I2C2_GPIO_AF          GPIO_AF1_I2C2

/*============================ CHARGER (BQ2416x) =============================*/

// Charger interrupt, EXTI on falling edge:
#define CHG_INT_PORT          GPIOF     // PF0
#define CHG_INT_PIN           GPIO_PIN_0

/*
 * TS divider control. Both pins reach the charger's TS node through 10k, with 14k from TS to
 * ground; the 1.69k that would bias TS from VDRV is marked N.P. on the schematic, so the node has
 * no source of its own and these two pins are the only thing holding it:
 *
 *   1/1 -> 3.3 * 14/19    = 2.432 V = 73.7 % -> cold
 *   1/0 -> 3.3 * 5.83/15.83 = 1.216 V = 36.8 % -> normal
 *   0/0 -> 0 V                                 -> hot
 *
 * 0/1 is the same node as 1/0, so there are three levels, not five.
 *
 * TODO(hw): keep both pins LOW. Nothing drives them, and that is deliberate - see the TS_EN
 * comment in charger_bq2416x.c. The zero is what stops the charge once the charger falls back to
 * DEFAULT mode and re-enables TS monitoring on its own. Raising them removes that protection.
 */
#define CHG_NTC_CTRL1_PORT    GPIOA     // PA6
#define CHG_NTC_CTRL1_PIN     GPIO_PIN_6
#define CHG_NTC_CTRL2_PORT    GPIOA     // PA15
#define CHG_NTC_CTRL2_PIN     GPIO_PIN_15

/*============================ POWER PATH ====================================*/

// 5V boost regulator enable, active high:
#define PWR_5V_BOOST_EN_PORT  GPIOA     // PA10
#define PWR_5V_BOOST_EN_PIN   GPIO_PIN_10

// 5V detection LDO enable, active high:
#define PWR_5V_LDO_EN_PORT    GPIOA     // PA11
#define PWR_5V_LDO_EN_PIN     GPIO_PIN_11

// VSYS output switch enable, active LOW:
#define PWR_VSYS_EN_PORT      GPIOA     // PA12
#define PWR_VSYS_EN_PIN       GPIO_PIN_12

// VSYS switch current limit select: SET = 0.5A, RESET = 2.1A:
#define PWR_VSYS_ILIM_PORT    GPIOF     // PF1
#define PWR_VSYS_ILIM_PIN     GPIO_PIN_1

// Fuel gauge interrupt. Open drain output released high, read back through IDR.
#define FG_INT_PORT           GPIOB     // PB1
#define FG_INT_PIN            GPIO_PIN_1

/*============================ HOST (Raspberry Pi) ===========================*/

// TODO - Check. The pin is not used in the circuit.
// RUN signal, open drain, active LOW pulse:
#define HOST_RUN_PORT         GPIOB     // PB13
#define HOST_RUN_PIN          GPIO_PIN_13

/*============================ BUTTONS =======================================*/

// Named after the schematic. Only SW1 is populated on the PiJuice Zero board
// TODO: button.c does NOT wire SW1 to button index 0
#define BTN_SW1_PORT          GPIOB     // PB12
#define BTN_SW1_PIN           GPIO_PIN_12
#define BTN_SW2_PORT          GPIOC     // PC13
#define BTN_SW2_PIN           GPIO_PIN_13
#define BTN_SW3_PORT          GPIOB     // PB2
#define BTN_SW3_PIN           GPIO_PIN_2

/*============================ ID EEPROM =====================================*/

// Write protect, active high:
#define EE_WP_PORT            GPIOB     // PB8
#define EE_WP_PIN             GPIO_PIN_8

// Slave address select: RESET = 0x50, SET = 0x52:
#define EE_ADDR_SEL_PORT      GPIOB     // PB3
#define EE_ADDR_SEL_PIN       GPIO_PIN_3

/*============================ ANALOG INPUTS =================================*/

// 5V bus towards the host, ADC_IN0:
#define ADC_5V_PI_PORT        GPIOA     // PA0
#define ADC_5V_PI_PIN         GPIO_PIN_0

// Battery voltage, ADC_IN2:
#define ADC_VBAT_PORT         GPIOA     // PA2
#define ADC_VBAT_PIN          GPIO_PIN_2

// 5V input presence detection, ADC_IN4:
#define ADC_PWR_DET_PORT      GPIOA     // PA4
#define ADC_PWR_DET_PIN       GPIO_PIN_4

/*============================ LEDS ==========================================*/

// LED1 (D2 on circuit and in pijuice_cli tools) - RGB on TIM15 CH1/CH2 and TIM17 CH1:
/* Unused. TIM15 is used for ADC */
/*
#define LED_D2_R_PORT         GPIOB     // PB14, TIM15_CH1
#define LED_D2_R_PIN          GPIO_PIN_14
#define LED_D2_G_PORT         GPIOB     // PB15, TIM15_CH2
#define LED_D2_G_PIN          GPIO_PIN_15
#define LED_D2_RG_GPIO_AF     GPIO_AF1_TIM15
#define LED_D2_B_PORT         GPIOB     // PB9, TIM17_CH1
#define LED_D2_B_PIN          GPIO_PIN_9
#define LED_D2_B_GPIO_AF      GPIO_AF2_TIM17
*/

// LED2 (D1 in the schematic and pijuice_cli tools)- RGB on TIM3 CH1/CH2/CH3:
#define LED_D1_R_PORT         GPIOB     // PB4, TIM3_CH1
#define LED_D1_R_PIN          GPIO_PIN_4
#define LED_D1_G_PORT         GPIOB     // PB5, TIM3_CH2
#define LED_D1_G_PIN          GPIO_PIN_5
#define LED_D1_B_PORT         GPIOB     // PB0, TIM3_CH3
#define LED_D1_B_PIN          GPIO_PIN_0
#define LED_D1_GPIO_AF        GPIO_AF1_TIM3

/*============================ USER IO =======================================*/

// IO1 - GPIO / analog / PWM on TIM14_CH1:
#define EXT_IO1_PORT          GPIOA     // PA7
#define EXT_IO1_PIN           GPIO_PIN_7
#define EXT_IO1_GPIO_AF       GPIO_AF4_TIM14

// IO2 - GPIO / PWM on TIM1_CH1, also EXTI wakeup source:
#define EXT_IO2_PORT          GPIOA     // PA8
#define EXT_IO2_PIN           GPIO_PIN_8
#define EXT_IO2_GPIO_AF       GPIO_AF2_TIM1

#endif /* BOARD_VER0_H_ */
