/*
 * board.h
 *
 *  Created on: 2026
 *      Author: Roman Egoshin
 */

#ifndef BOARD_H_
#define BOARD_H_

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

#include <stdbool.h>

void bsp_ClockConfig(void);

void bsp_Init(void);

// Starts the independent watchdog. It cannot be stopped again short of a reset.
void bsp_WdtStart(void);

void bsp_WdtRefresh(void);

void bsp_Pwr5V_SetState(bool _state);

// Last commanded state, not a pin read: PA10 is held by an RC and reads back stale.
bool bsp_Pwr5V_GetState(void);

void bsp_Pwr5V_Restore(void);

void bsp_ResetCPU(void);

void bsp_StartBootloader(void);

#endif /* BOARD_H_ */
