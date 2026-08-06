/*
 * utils.c
 *
 *  Created on: Aug 6, 2026
 *      Author: Roman Egoshin
 */

#ifndef COMPONENTS_UTILS_UTILS_C_
#define COMPONENTS_UTILS_UTILS_C_

#include "utils.h"

bool utils_PostEvent(QueueHandle_t _queue, const void *_ev)
{
  ASSERT(_queue != NULL);

  BaseType_t sent;
  BaseType_t woken = pdFALSE;
  if (xPortIsInsideInterrupt() == pdTRUE)
    sent = xQueueSendFromISR(_queue, _ev, &woken);
  else
    sent = xQueueSend(_queue, _ev, 0);

  portYIELD_FROM_ISR(woken);
  return (sent == pdPASS);
}

#endif /* COMPONENTS_UTILS_UTILS_C_ */
