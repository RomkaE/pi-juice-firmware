
#ifndef RTMON_CONFIG_H_
#define RTMON_CONFIG_H_

#include "config.h"

#define RTMON_ENABLED                       ( 0 )

#define RTMON_RTT_BUFF_IDX                  ( 0 )   // RTT up-channel (shared with log)
#define RTMON_RTT_TERMINAL                  ( 1 )   // RTT virtual terminal -> separate Viewer tab

#define RTMON_TERM_PROFILE                  RTMON_TERM_SEGGER_RTT

#define RTMON_CFG_USE_STATIC_ALOCATION      ( 1 )

#define RTMON_CFG_TASK_PRIO                 TASK_RTMON_PRIO

#define RTMON_CFG_TASK_STACK_DEPTH          TASK_RTMON_STACK

#define RTMON_CFG_TASKS_MAX_COUNT           ( 16 )

#define RTMON_CFG_UPDATE_PERIOD_MS          ( 1000 )

#define RTMON_CFG_LINE_BUFF_SIZE            ( 64 )

#define RTMON_CFG_PERCENT_SCALE             ( 100 )

#define RTMON_CFG_VIEW_DEBUG_INFO           ( 0 )

#endif /* RTMON_CONFIG_H_ */
