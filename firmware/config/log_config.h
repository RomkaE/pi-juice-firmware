
#ifndef CONFIG_LOG_CONFIG_H_
#define CONFIG_LOG_CONFIG_H_

#define LOG_ENABLED             1

#if LOG_ENABLED

#define LOG_TO_CDC              0
#define LOG_TO_FILE             0
#define LOG_TO_UART             0
#define LOG_TO_RTT              1

#define LOG_BUFF_LINE_SIZE      ( 64 + 32 )
//#define LOG_BUFF_SIZE           ( 256 )

#if LOG_TO_CDC
#define LOG_TUD_CDC_IF          1
#endif

#if LOG_TO_RTT
#define LOG_RTT_BUFF_IDX        0   /* RTT up-channel (physical ring buffer) */
#define LOG_RTT_TERMINAL        0   /* RTT virtual terminal shown as a Viewer tab */
#endif

#endif /* LOG_ENABLED */

#endif /* CONFIG_LOG_CONFIG_H_ */
