/*
 * app_assert.c
 *
 *  Created on:
 *      Author: Roman Egoshin
 */

/*============================ INCLUDES ======================================*/

#include "app_assert.h"

/*============================ TYPES =========================================*/


/*============================ VARIABLES =====================================*/


/*============================ PRIVATE DEFINITIONS ===========================*/


/*============================ PRIVATE PROTOTYPES ============================*/


/*============================ IMPLEMENTATION (PRIVATE FUNCTIONS) ============*/


/*============================ IMPLEMENTATION (PUBLIC FUNCTIONS) =============*/

#ifndef NDEBUG
__attribute__((weak)) void app_assert_handler()
{
  // TODO - desiable IRQ
  // TODO - reboot?
#ifdef __arm__
	// By default just break here
	asm("bkpt #0x01");
#endif
}
#endif /* NDEBUG */
