/*
 * app.h
 *
 *  Created on: Jul 20, 2026
 *      Author: Roman Egoshin
 */

#ifndef SRC_APP_H_
#define SRC_APP_H_

#include <stdint.h>
#include <stdbool.h>

/*
 * System wide event queue.
 *
 * Subsystem tasks detect, the APP task decides. A component task posts what happened and goes
 * straight back to its own job; the APP task drains the queue and runs the functional logic in
 * one place and one context. That context matters: the handlers drive the 5V boost, the RUN
 * pin and the charger, and those sequences must not interleave with each other.
 *
 * New producers add a type here and a case to the dispatcher in app.c.
 */
typedef enum
{
  APP_EVT_BUTTON = 1,           // a button event that has a configured function
  APP_EVT_BUTTON_RESET_CONFIG,  // both power buttons held down: reset the configuration
} AppEventType_t;

typedef struct
{
  uint8_t func;    // ButtonFunction_T the button is configured with
  uint8_t index;   // button index in the host facing numbering
  uint8_t event;   // ButtonEvent_T that fired
} AppEventButton_t;

typedef struct
{
  uint8_t type;    // AppEventType_T
  union
  {
    AppEventButton_t button;
  } data;
} AppEvent_t;

void app_Init(void);

/*
 * Hands an event to the APP task. Safe from an interrupt and from a task, never blocks.
 * Returns false when the queue is full, which the caller is expected to count as a fault -
 * the event is lost, not retried.
 */
bool app_PostEvent(const AppEvent_t *_pEvent);

#endif /* SRC_APP_H_ */
