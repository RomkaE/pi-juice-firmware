/*
 * app_error.c
 *
 *  Created on:
 *      Author: Roman Egoshin
 */

/*============================ INCLUDES ======================================*/

#include "app_error.h"

/*============================ TYPES =========================================*/


/*============================ VARIABLES =====================================*/


/*============================ PRIVATE DEFINITIONS ===========================*/


/*============================ PRIVATE PROTOTYPES ============================*/


/*============================ IMPLEMENTATION (PRIVATE FUNCTIONS) ============*/


/*============================ IMPLEMENTATION (PUBLIC FUNCTIONS) =============*/

#ifdef NDEBUG

__attribute__((weak)) void app_error_handler(ErrorCode_t error_code)
{
  (void)error_code;
  // TODO
}

#else

__attribute__((weak)) void app_error_handler(ErrorCode_t error_code)
{
  (void)error_code;
  // TODO __disable_irq();
  // TODO if debug
#ifdef __arm__
    // By default just break here
    asm("bkpt #0x01");
#endif
    // TODO - reboot !?
}

#endif /* NDEBUG */
