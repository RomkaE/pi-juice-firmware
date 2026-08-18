/*
 * app_assert.c
 *
 *  Created on:
 *      Author: Roman Egoshin
 */

/*============================ INCLUDES ======================================*/

#include "app_assert.h"
#include "app_fatal.h"

/*============================ TYPES =========================================*/


/*============================ VARIABLES =====================================*/


/*============================ PRIVATE DEFINITIONS ===========================*/


/*============================ PRIVATE PROTOTYPES ============================*/


/*============================ IMPLEMENTATION (PRIVATE FUNCTIONS) ============*/


/*============================ IMPLEMENTATION (PUBLIC FUNCTIONS) =============*/

#ifndef NDEBUG
void app_assert_handler(char *_file, int _line)
{
  /*
   * The site is already logged by the ASSERT macro that called us - only there are the file and
   * the line known. The arguments are kept so a debugger stopping on the bkpt inside
   * app_fatal_Handle() can still read them off the stack.
   */
  (void)_file;
  (void)_line;

  app_fatal_Handle();
}
#endif /* NDEBUG */
