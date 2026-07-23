
#ifndef __LOG_H
#define __LOG_H

#ifdef __cplusplus
extern "C" {
#endif

/*============================ INCLUDES ======================================*/

#include "log_config.h"

/*============================ TYPES =========================================*/


/*============================ DEFINITIONS ===================================*/

#define LOG_LEVEL_OFF			0
#define LOG_LEVEL_CRITICAL		1
#define LOG_LEVEL_ERROR			2
#define LOG_LEVEL_WARNING		3
#define LOG_LEVEL_INFO			4
#define LOG_LEVEL_DEBUG			5
#define LOG_LEVEL_VERBOSE		6
//#define LOG_LEVEL_XFER			7

#if LOG_ENABLED == 1

#define LOG_INIT(x)						log_Init(x)
#define LOG(level, format_msg, ...)		Log_Printf(level, 0, format_msg, ##__VA_ARGS__)
#define LOGN(level, format_msg, ...)	Log_Printf(level, 1, format_msg, ##__VA_ARGS__)

#define LOG_CRITICAL(format_msg, ...)	Log_Printf(LOG_LEVEL_CRITICAL, 0, format_msg, ##__VA_ARGS__)
#define LOGN_CRITICAL(format_msg, ...)	Log_Printf(LOG_LEVEL_CRITICAL, 1, format_msg, ##__VA_ARGS__)
#define LOG_ERROR(format_msg, ...)		Log_Printf(LOG_LEVEL_ERROR, 0, format_msg, ##__VA_ARGS__)
#define LOGN_ERROR(format_msg, ...)		Log_Printf(LOG_LEVEL_ERROR, 1, format_msg, ##__VA_ARGS__)
#define LOG_WARNING(format_msg, ...)	Log_Printf(LOG_LEVEL_WARNING, 0, format_msg, ##__VA_ARGS__)
#define LOGN_WARNING(format_msg, ...)	Log_Printf(LOG_LEVEL_WARNING, 1, format_msg, ##__VA_ARGS__)
#define LOG_INFO(format_msg, ...)		Log_Printf(LOG_LEVEL_INFO, 0, format_msg, ##__VA_ARGS__)
#define LOGN_INFO(format_msg, ...)		Log_Printf(LOG_LEVEL_INFO, 1, format_msg, ##__VA_ARGS__)
#define LOG_DEBUG(format_msg, ...)		Log_Printf(LOG_LEVEL_DEBUG, 0, format_msg, ##__VA_ARGS__)
#define LOGN_DEBUG(format_msg, ...)		Log_Printf(LOG_LEVEL_DEBUG, 1, format_msg, ##__VA_ARGS__)
#define LOG_VERBOSE(format_msg, ...)	Log_Printf(LOG_LEVEL_VERBOSE, 0, format_msg, ##__VA_ARGS__)
#define LOGN_VERBOSE(format_msg, ...)	Log_Printf(LOG_LEVEL_VERBOSE, 1, format_msg, ##__VA_ARGS__)
//#define LOG_XFER(format_msg, ...)		Log_Printf(LOG_LEVEL_XFER, 0, format_msg, ##__VA_ARGS__)
//#define LOGN_XFER(format_msg, ...)		Log_Printf(LOG_LEVEL_XFER, 1, format_msg, ##__VA_ARGS__)

#else

#define LOG_INIT(x)
#define LOG(level, format_msg, ...)
#define LOGN(level, format_msg, ...)
#define LOG_CRITICAL(format_msg, ...)
#define LOGN_CRITICAL(format_msg, ...)
#define LOG_ERROR(format_msg, ...)
#define LOGN_ERROR(format_msg, ...)
#define LOG_WARNING(format_msg, ...)
#define LOGN_WARNING(format_msg, ...)
#define LOG_INFO(format_msg, ...)
#define LOGN_INFO(format_msg, ...)
#define LOG_DEBUG(format_msg, ...)
#define LOGN_DEBUG(format_msg, ...)
#define LOG_VERBOSE(format_msg, ...)
#define LOGN_VERBOSE(format_msg, ...)
#define LOG_XFER(format_msg, ...)
#define LOGN_XFER(format_msg, ...)

#endif /* LOG_ENABLED */

/*============================ VARIABLES =====================================*/


/*============================ PROTOTYPES ====================================*/

void log_Init(unsigned char level);

void Log_Printf(unsigned char level, char nline, const char * format_msg, ...);

/* Only the buffered back ends need a pump task. The RTT back end writes
 * straight through from Log_Printf(). */
#if LOG_TO_CDC
int log_poll();
#endif

#ifdef __cplusplus
}
#endif

#endif /* __LOG_H */
