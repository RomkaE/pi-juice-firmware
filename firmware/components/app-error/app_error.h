/*
 * app_error.h
 *
 *  Created on:
 *      Author: Roman Egoshin
 */

#ifndef APP_ERROR_H_
#define APP_ERROR_H_

/*============================ INCLUDES ======================================*/


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

  APP_HAL_OK        = -16,  // don't use!
  APP_HAL_ERROR     = -17,
  APP_HAL_BUSY      = -18,
  APP_HAL_TIMEOUT   = -19,

  APP_DRV_TIMEOUT   = -32,
  APP_DRV_BUS_ERR   = -33,
} ErrorCode_t;

/*============================ DEFINITIONS ===================================*/

#define APP_ERROR(ERR_CODE) \
  do { \
  const ErrorCode_t LOCAL_ERR_CODE = (ERR_CODE);  \
    if (LOCAL_ERR_CODE != APP_OK) \
    { \
      app_error_handler(LOCAL_ERR_CODE);  \
    } \
  } while (0)

/*============================ VARIABLES =====================================*/


/*============================ PROTOTYPES ====================================*/

void app_error_handler(ErrorCode_t error_code);

#endif /* APP_ERROR_H_ */
