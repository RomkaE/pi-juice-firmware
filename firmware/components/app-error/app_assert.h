/*
 * app_assert.h
 *
 *  Created on:
 *      Author: Roman Egoshin
 */

#ifndef APP_ASSERT_H_
#define APP_ASSERT_H_

/*============================ INCLUDES ======================================*/

#include "log/log.h"


/*============================ TYPES =========================================*/


/*============================ DEFINITIONS ===================================*/

#ifdef NDEBUG
  #define ASSERT(expr)  ((void)0)
#else

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Fatal: logs the site, flushes the log and resets. Never returns to the caller - except when
 * called from an interrupt, where it records the site and returns so the ISR can finish; the
 * reset then happens in thread mode. See app_error.c.
 */
void app_assert_handler(char *_file, int _line);

#ifdef __cplusplus
}
#endif

/*
 * The log line is emitted here, at the call site, so it carries the file and the line - neither of
 * which the handler could recover on its own.
 *
 * The stringified expression is deliberately not printed. configASSERT() maps to this macro, and
 * the kernel's assert expressions are fully macro-expanded before stringification - things like
 * "( pxQueue != ((void *)0) ) && !( ( pvItemToQueue == ((void *)0) ) ... )" - which came to ~30 KB
 * of .rodata across the ~260 kernel sites. File and line pin the site down well enough.
 *
 * __FILE_NAME__ rather than __FILE__ on purpose: configASSERT() maps to this macro, so the
 * FreeRTOS kernel expands it at ~260 sites and the full "../third-party/FreeRTOS-Kernel/tasks.c"
 * would be carried for every translation unit involved. The basename is what a log line needs.
 */
#define ASSERT(expr)                                                    \
  do {                                                                  \
    if ((expr) == 0)                                                    \
    {                                                                   \
      if (Log_EnterFatal())                                             \
        LOG_CRITICAL("ASSERT failed: %s:%d", __FILE_NAME__, __LINE__);  \
      app_assert_handler(__FILE_NAME__, __LINE__);                      \
    }                                                                   \
  } while (0)

#endif /* NDEBUG */

/*============================ VARIABLES =====================================*/


/*============================ PROTOTYPES ====================================*/

#endif /* APP_ASSERT_H_ */
