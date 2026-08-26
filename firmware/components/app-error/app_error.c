/*
 * app_error.c
 *
 *  Created on:
 *      Author: Roman Egoshin
 */

/*============================ INCLUDES ======================================*/

#include "app_error.h"
#include "app_fatal.h"

/*============================ TYPES =========================================*/


/*============================ VARIABLES =====================================*/


/*============================ PRIVATE DEFINITIONS ===========================*/


/*============================ PRIVATE PROTOTYPES ============================*/


/*============================ IMPLEMENTATION (PRIVATE FUNCTIONS) ============*/


/*============================ IMPLEMENTATION (PUBLIC FUNCTIONS) =============*/

void app_error_handler(ErrorCode_t error_code)
{
  /*
   * Already logged by the APP_ERROR macro, together with the file and the line it fired at. Kept
   * for the debugger, same as in app_assert_handler().
   */
  (void)error_code;

  app_fatal_Handle();
}
