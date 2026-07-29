/*
 * retained_memory.h
 *
 * Validity guard for the no_init section - the variables that are meant to survive a reset
 * (executionState, charger and IO configuration, fuel gauge state, timers, event flags).
 */

#ifndef RETAINED_MEMORY_H_
#define RETAINED_MEMORY_H_

#include <stdbool.h>

/*
 * Decides whether the current contents of the no_init section were left there by *this*
 * firmware image, and stamps the section so the next reset can tell.
 *
 * Returns 1 if the retained data may be trusted, 0 if it must be treated as uninitialised
 * and everything re-read from NV / recomputed.
 *
 * Must be called before any no_init variable is read - it is the first thing main_init()
 * does, ahead of the executionState test.
 */
bool retained_mem_Check(void);

bool retained_mem_GetStatus(void);

#endif /* RETAINED_MEMORY_H_ */
