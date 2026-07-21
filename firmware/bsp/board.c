
/*============================ INCLUDES ======================================*/

#include "config.h"
#include "board.h"
#include "app-error/app_error.h"

// ST HAL/CubeMX:
#include "stm32f0xx_hal.h"
#include "cube-mx/main.h"
#include "cube-mx/i2c.h"

/*============================ TYPES =========================================*/

typedef void (*pFunction)(void);

/*============================ VARIABLES =====================================*/


/*============================ PRIVATE DEFINITIONS ===========================*/

#define SYSMEM_BOOT_BASE    0x1FFF0000UL      // System memory start address
#define DFU_MAGIC_ENTER     0xBEDA0FA5UL
#define DFU_MAGIC_EXIT      0xBC7A00A1UL

/*============================ PRIVATE PROTOTYPES ============================*/


/*============================ IMPLEMENTATION (PRIVATE FUNCTIONS) ============*/

/*============================ IMPLEMENTATION (PUBLIC FUNCTIONS) =============*/

void bsp_Init(void)
{
  HAL_Init();
  HAL_SetTickFreq(HAL_TICK_FREQ_100HZ);
  SystemClock_Config();
}

void bsp_DeInit(void)
{
}
