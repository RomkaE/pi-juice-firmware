/*
 * app_error.h
 *
 *  Created on:
 *      Author: Roman Egoshin
 */

#ifndef APP_ERROR_H_
#define APP_ERROR_H_

/*============================ INCLUDES ======================================*/

#include "log/log.h"


/*============================ TYPES =========================================*/

/* This type should be defined user */
typedef enum {
  APP_OK            = 0,
  APP_ERR           = -1,
  APP_ERR_ARG       = -2,
  APP_ERR_PARAM     = -3,
  APP_ERR_STATE     = -4,
  APP_ERR_NOMEM     = -5,
  APP_ERR_BUSY      = -6,
  APP_ERR_TIMEOUT   = -7,

  APP_ERR_RTOS_QUEUE = -8,
  APP_ERR_RTOS_TIMER = -9,
  APP_ERR_HARD_FAULT = -10,

  APP_HAL_OK        = -16,  // don't use!
  APP_HAL_ERROR     = -17,
  APP_HAL_BUSY      = -18,
  APP_HAL_TIMEOUT   = -19,

  APP_DRV_TIMEOUT   = -32,
  APP_DRV_BUS_ERR   = -33,
} ErrorCode_t;

/*============================ DEFINITIONS ===================================*/

/* Logged at the call site, same reasoning as ASSERT: only here are the file and line known. */
#define APP_ERROR(ERR_CODE) \
  do { \
    const ErrorCode_t LOCAL_ERR_CODE = (ERR_CODE);  \
    if (LOCAL_ERR_CODE != APP_OK) \
    { \
      if (Log_EnterFatal()) \
        LOG_CRITICAL("APP_ERROR: %s:%d: code=%d", __FILE_NAME__, __LINE__, (int)LOCAL_ERR_CODE); \
      app_error_handler(LOCAL_ERR_CODE);  \
    } \
  } while (0)

/*============================ VARIABLES =====================================*/


/*============================ PROTOTYPES ====================================*/

/*
 * Fatal: logs the code, flushes the log and resets. Never returns to the caller - except when
 * called from an interrupt, where it records the code and returns so the ISR can finish; the
 * reset then happens in thread mode.
 */
void app_error_handler(ErrorCode_t error_code);


#endif /* APP_ERROR_H_ */
