
#include "bsp_rtmon.h"
//#include "cube-mx/tim.h"
// TODO

//extern TIM_HandleTypeDef htim2;

void bsp_rtmon_InitRunTimer(void)
{
//  MX_TIM2_Init();
//  HAL_TIM_Base_Start(&htim2);
}

configRUN_TIME_COUNTER_TYPE bsp_rtmon_GetRunTimer(void)
{
//  extern TIM_HandleTypeDef htim2;
//  configRUN_TIME_COUNTER_TYPE cnt = __HAL_TIM_GET_COUNTER(&htim2);
	 configRUN_TIME_COUNTER_TYPE cnt  = 0;
  return cnt;
}
