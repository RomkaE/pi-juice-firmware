/*
 * utils.h
 *
 *  Created on: Aug 4, 2026
 *      Author: Roman Egoshin
 */

#ifndef COMPONENTS_UTILS_UTILS_H_
#define COMPONENTS_UTILS_UTILS_H_

#include <stdbool.h>

// FreeRTOS:
#include "FreeRTOS.h"
#include "queue.h"

#define STRINGIFY_IMPL(x)   #x
#define STRINGIFY(x)        STRINGIFY_IMPL(x)

bool utils_PostEvent(QueueHandle_t _queue, const void *_ev);

#endif /* COMPONENTS_UTILS_UTILS_H_ */
