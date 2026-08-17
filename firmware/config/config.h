
#ifndef CONFIG_CONFIG_H_
#define CONFIG_CONFIG_H_

// MCU die temperature, currently unused:
#define ANALOG_TEMP_MCU_ENABLED         0

/*============================ TASKS =========================================*/

#define TASK_MAX_PRIORITIES         10

#define TASK_TIMER_SVC_PRIO         9
#define TASK_TIMER_SVC_STACK        384
#define TASK_TIMER_SVC_QUEUE_LEN    10

#define TASK_ANALOG_PRIO            8
#define TASK_ANALOG_STACK           512
#define TASK_ANALOG_QUEUE_LEN       10

#define TASK_APP_PRIO               7
#define TASK_APP_STACK              384
#define TASK_APP_QUEUE_LEN          16

#define TASK_BTN_PRIO               6
#define TASK_BTN_STACK              320
#define TASK_BTN_QUEUE_LEN          8

#define TASK_CHG_PRIO               5
#define TASK_CHG_STACK              384
#define TASK_CHG_QUEUE_LEN          8

#define TASK_FUEL_GAUGE_PRIO        4
#define TASK_FUEL_GAUGE_STACK       384
#define TASK_FUEL_GAUGE_QUEUE_LEN   8

// never run: led_Init() is disabled
#define TASK_LED_PRIO               3
#define TASK_LED_STACK              512
#define TASK_LED_QUEUE_LEN          8

#define TASK_RTMON_PRIO             1
#define TASK_RTMON_STACK            512

#define TASK_IDLE_STACK             192

#endif /* CONFIG_CONFIG_H_ */
