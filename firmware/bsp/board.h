/*
 * app.h
 *
 *  Created on: 2026
 *      Author: Roman Egoshin
 */

#ifndef BOARD_H_
#define BOARD_H_

#include <stdbool.h>
#include "stm32f0xx_hal.h"

#if !defined(BOARD_VER)
  #error "Not defined BOARD_VER"
#endif

#if BOARD_VER == 0
#include "board_ver0.h"
#elif BOARD_VER == 1
#include "board_ver1.h"
#else
  #error Wrong version board!
#endif

/*============================ PROTOTYPES ====================================*/

void bsp_Init(void);

void bsp_DeInit(void);

#endif /* BOARD_H_ */
