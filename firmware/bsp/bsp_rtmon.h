
#ifndef BSP_BSP_RTMON_C_
#define BSP_BSP_RTMON_C_

#include <stdint.h>

#include "FreeRTOS.h"

void bsp_rtmon_InitRunTimer(void);

configRUN_TIME_COUNTER_TYPE bsp_rtmon_GetRunTimer(void);

#endif /* BSP_BSP_RTMON_C_ */
