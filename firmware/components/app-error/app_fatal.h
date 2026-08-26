/*
 * app_fatal.h
 *
 *  Created on: Aug 17, 2026
 *      Author: Roman Egoshin
 */

#ifndef APP_FATAL_H_
#define APP_FATAL_H_

/*============================ INCLUDES ======================================*/


/*============================ TYPES =========================================*/


/*============================ DEFINITIONS ===================================*/


/*============================ VARIABLES =====================================*/


/*============================ PROTOTYPES ====================================*/

/*
 * Internal to the app-error component: the tail shared by app_assert_handler() and
 * app_error_handler(). Not meant to be called from outside them.
 *
 * Returns - and only then - if the fatal path is already being walked. Everything on that path can
 * itself fail an assert (Log_Printf() does, the HAL does), and without the latch the first failure
 * would recurse until the stack ran out instead of reaching the reset.
 *
 * Otherwise never returns: LOG_FLUSH() -> bkpt when !NDEBUG -> bsp_ResetCPU().
 */
void app_fatal_Handle(void);

#endif /* APP_FATAL_H_ */
