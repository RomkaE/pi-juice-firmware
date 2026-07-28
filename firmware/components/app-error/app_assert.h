/*
 * app_assert.h
 *
 *  Created on:
 *      Author: Roman Egoshin
 */

#ifndef APP_ASSERT_H_
#define APP_ASSERT_H_

/*============================ INCLUDES ======================================*/


/*============================ TYPES =========================================*/


/*============================ DEFINITIONS ===================================*/

#ifdef NDEBUG
	#define ASSERT(expr)	((void)0)
#else

#ifdef __cplusplus
extern "C" {
#endif

void app_assert_handler();

#ifdef __cplusplus
}
#endif

#define ASSERT(expr)									\
	do {												\
		if ((expr) == 0)								\
		{												\
			app_assert_handler();			            \
		}												\
	} while (0)
#endif /* NDEBUG */

/*============================ VARIABLES =====================================*/


/*============================ PROTOTYPES ====================================*/

#endif /* APP_ASSERT_H_ */
