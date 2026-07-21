
/*============================ INCLUDES ======================================*/
#include <stdarg.h>
#include <stdio.h>
#include <stdbool.h>
#include <string.h>

#include "log.h"

#if LOG_ENABLED

#include "esc_sequence.h"
#include "board.h"
#include "app-error/app_assert.h"

#if LOG_TO_CDC || LOG_TO_FILE
#include "tusb.h"
#endif

#if LOG_TO_RTT
#include "SEGGER_RTT.h"
#endif

/*============================ TYPES =========================================*/


/*============================ VARIABLES =====================================*/

static unsigned char m_LogLevel;

#if LOG_TO_CDC
static tu_fifo_t s_ff_cdc;
static char s_ff_cdc_data[LOG_BUFF_SIZE];
static osal_semaphore_def_t sem_def;
static osal_semaphore_t sem_cdc;
#endif /* LOG_TO_CDC */

#if LOG_TO_RTT
/* Terminal switch escape: routes the following bytes to the Viewer tab
 * LOG_RTT_TERMINAL, so LOG and RTMON can share up-buffer LOG_RTT_BUFF_IDX. */
static const uint8_t s_RttTermSwitch[2] = { 0xFFu, (uint8_t)('0' + LOG_RTT_TERMINAL) };
#endif /* LOG_TO_RTT */

#if LOG_TO_FILE
static tu_fifo_t s_ff_fs;
static char s_ff_fs_data[LOG_BUFF_SIZE];
#endif /* LOG_TO_FILE */

static const char *level_strings[] = {
  "", "CRITICAL", "ERROR", "WARNING", "INFO", "DEBUG", "VERBOSE"
};

static const char *level_colors[] = {
  "", CL_LIGHTMAGENTA, CL_LIGHTRED, CL_BROWN, CL_GREEN, CL_BLUE, CL_GRAY };

//static char s_Buf[1024];

/*============================ PRIVATE DEFINITIONS ===========================*/


/*============================ PRIVATE PROTOTYPES ============================*/


/*============================ IMPLEMENTATION (PRIVATE FUNCTIONS) ============*/


/*============================ IMPLEMENTATION (PUBLIC FUNCTIONS) =============*/

#if LOG_BUFF_LINE_SIZE < 16
#error "LOG_BUFF_LINE_SIZE must be at least 16 bytes"
#endif

/* LOG_BUFF_SIZE only sizes the FIFO of the buffered back ends. */
#if LOG_TO_CDC || LOG_TO_FILE
#if LOG_BUFF_SIZE <= LOG_BUFF_LINE_SIZE
#error "LOG_BUFF_SIZE must be greater than LOG_BUFF_LINE_SIZE"
#endif
#endif

void log_Init(unsigned char level)
{
  m_LogLevel = level;

  #if LOG_TO_CDC
  {
    bool res = tu_fifo_config(&s_ff_cdc, s_ff_cdc_data, TU_ARRAY_SIZE(s_ff_cdc_data), true);
    ASSERT(res);

    sem_cdc = osal_semaphore_create(&sem_def);
  }
  #endif /* LOG_TO_CDC */
  #if LOG_TO_RTT
  {
    SEGGER_RTT_Init();
  }
  #endif /* LOG_TO_RTT */
  #if LOG_TO_FILE
  {
    bool res = tu_fifo_config(&s_ff_fs, s_ff_fs_data, TU_ARRAY_SIZE(s_ff_fs_data), 1, true);
    ASSERT(res);
  }
  #endif /* LOG_TO_FILE */
}

void Log_Printf(unsigned char level, char nline, const char* format_msg, ...)
{
  if (m_LogLevel < level)
    return;

  char line[LOG_BUFF_LINE_SIZE];
  const size_t size = LOG_BUFF_LINE_SIZE;
  unsigned int len = 0;
  int res;

  // Time stamp and log level:
  if (len < size)
  {
    extern uint32_t app_timer_get_ms(void);
    res = snprintf(&line[len], size - len, "%lu [%s%s\x1B[0m] ",
                   app_timer_get_ms(),
                   level_colors[level],
                   level_strings[level]);
    if (res > 0)
      len += res;
  }

  // Message:
  if (len < size)
  {
    va_list p_args;
    va_start(p_args, format_msg);
    res = vsnprintf(&line[len], size - len, format_msg, p_args);
    va_end(p_args);
    if (res > 0)
      len += res;
  }

  // Check and fix to max length:
  if (len > size)
    len = size;

  // Add \r\n:
  if (!nline)
  {
    if (size - len < 2)
      len = size - 2;
    line[len++] = '\r';
    line[len++] = '\n';
  }

  // Write to FIFO:
  if (len > 0)
  {
    #if LOG_TO_CDC
    {
      uint16_t wr_cnt = tu_fifo_write_n(&s_ff_cdc, line, len);
      ASSERT(wr_cnt == len);      // TODO check result. Check overflow.

      // Give semaphore:
      osal_semaphore_post(sem_cdc, false);
    }
    #endif /* LOG_TO_CDC */
    #if LOG_TO_RTT
    {
      // The terminal switch and the payload must reach the up-buffer as one
      // unit, otherwise RTMON writing to its own terminal in between would
      // make the host cross-route the streams. Check the free space up front
      // so we never emit a switch whose payload then gets dropped: if the
      // whole line does not fit, drop the line (host is not draining).
      SEGGER_RTT_LOCK();
      {
        if (SEGGER_RTT_GetAvailWriteSpace(LOG_RTT_BUFF_IDX) >= sizeof(s_RttTermSwitch) + len)
        {
          SEGGER_RTT_WriteNoLock(LOG_RTT_BUFF_IDX, s_RttTermSwitch, sizeof(s_RttTermSwitch));
          SEGGER_RTT_WriteNoLock(LOG_RTT_BUFF_IDX, line, len);
        }
      }
      SEGGER_RTT_UNLOCK();
    }
    #endif /* LOG_TO_RTT */
    #if LOG_TO_FILE
    {
      uint16_t wr_cnt = tu_fifo_write_n(&s_ff_fs, line, len);
      ASSERT(wr_cnt == len);      // TODO check result. Check overflow.
      if (wr_cnt)
        fs_write2log(&s_ff_fs);
    }
    #endif /* LOG_TO_FILE */
  }
}

#if LOG_TO_CDC
int log_poll()
{
  static bool xmit = false;

  // "State Machine":
  if (xmit || osal_semaphore_wait(sem_cdc, OSAL_TIMEOUT_WAIT_FOREVER))
      xmit = true;
  else
      return false;

  // Is FIFO empty:
  uint16_t ff_size = tu_fifo_count(&s_ff_cdc);
  xmit = ff_size != 0;
  if (!xmit)
    return false;

  // Is connected and write available:
  uint16_t cdc_size;
  if (!tud_cdc_n_connected(LOG_TUD_CDC_IF) ||
      !(cdc_size = tud_cdc_n_write_available(LOG_TUD_CDC_IF)))
  {
    osal_task_delay(100);
    // TODO - delay/wait
    return true;
  }

  uint16_t b_size = TU_MIN(cdc_size, ff_size);
  uint8_t buff[b_size];

  // Read data from FIFO:
  uint16_t ff_read_n = tu_fifo_read_n(&s_ff_cdc, buff, b_size);
  ASSERT(ff_read_n == b_size);

  // Write data to CDC:
  uint16_t ff_write_n = tud_cdc_n_write(LOG_TUD_CDC_IF, buff, ff_read_n);
  ASSERT(ff_write_n == ff_read_n);

  tud_cdc_n_write_flush(LOG_TUD_CDC_IF);

  // Input data FIFO is empty:
  if (ff_size == ff_write_n)
    xmit = false;

  return xmit;
}
#endif /* LOG_TO_CDC */


#endif /* LOG_ENABLED */
