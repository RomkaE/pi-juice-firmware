/*
 * diag.c
 *
 *  Created on: Aug 17, 2026
 *      Author: Roman Egoshin
 */

/*============================ INCLUDES ======================================*/

#include "diag.h"

// FreeRTOS:
#include "FreeRTOS.h"
#include "task.h"

/*============================ TYPES =========================================*/


/*============================ VARIABLES =====================================*/

/*
 * Retained across a reset, because the fatal path ends in one: an assert, an APP_ERROR or a
 * watchdog bite would otherwise wipe exactly the counters that explain them, and the host polling
 * 0xC3/0xC7 would see zeros with no hint that the firmware had just died and come back.
 *
 * No initialiser: the section is NOLOAD, so startup does not touch it. diag_Init() decides.
 */
static volatile uint32_t s_LiveMask __attribute__((section("no_init")));
static volatile uint8_t  s_Counters[DIAG_ID_COUNT] __attribute__((section("no_init")));

/*============================ PRIVATE DEFINITIONS ===========================*/

/*
 * The critical sections below use the FROM_ISR form deliberately, and it is not wrapped in a macro
 * on purpose: the pair has to share a local, so any wrapper would have to leak that name into the
 * caller's scope and would not be usable as a statement.
 *
 * On Cortex-M0 taskENTER/EXIT_CRITICAL_FROM_ISR() are ulSetInterruptMask()/vClearInterruptMask(),
 * a plain mrs/cpsid + msr PRIMASK pair, which is correct in task and ISR context alike - and
 * before the scheduler starts. That matters because the read path runs in the I2C1 ISR
 * (CmdServerProcessRequest is called from i2c_slave_OnAddr) while the write path is mostly task
 * context. M0 has no LDREX/STREX and no bit-banding, so there is no lock-free alternative.
 */

/*============================ PRIVATE PROTOTYPES ============================*/


/*============================ IMPLEMENTATION (PRIVATE FUNCTIONS) ============*/


/*============================ IMPLEMENTATION (PUBLIC FUNCTIONS) =============*/

void diag_Init(bool _cold_start)
{
  /*
   * Cold start, or the section moved under a new image: the contents are somebody else's.
   *
   * On a warm reset everything is kept, live bits included. They are safe to carry over because
   * every live condition is re-evaluated by its owner within a round or two of booting (the charger
   * clears its bit on entering ACTIVE, the fuel gauge on IC_INIT, the ADC as soon as the stream
   * ticks, the battery when a valid profile is set, the power protection on the first clean
   * dataset), so nothing stale can linger. DIAG_PWR_FAULT_LATCHED surviving is not a defect but
   * the point: the latch itself is what the host needs to know about.
   */
  if (_cold_start)
  {
    s_LiveMask = 0;
    for (uint8_t i = 0; i < DIAG_ID_COUNT; i++)
      s_Counters[i] = 0;
  }
}

void diag_Set(DiagId_t _id)
{
  if (_id >= DIAG_ID_COUNT)
    return;

  uint32_t lock = taskENTER_CRITICAL_FROM_ISR();

  s_LiveMask |= (1UL << _id);

  if (s_Counters[_id] < DIAG_COUNTER_MAX)
    s_Counters[_id]++;

  taskEXIT_CRITICAL_FROM_ISR(lock);
}

void diag_Clear(DiagId_t _id)
{
  if (_id >= DIAG_ID_COUNT)
    return;

  uint32_t lock = taskENTER_CRITICAL_FROM_ISR();
  s_LiveMask &= ~(1UL << _id);
  taskEXIT_CRITICAL_FROM_ISR(lock);
}

bool diag_IsSet(DiagId_t _id)
{
  if (_id >= DIAG_ID_COUNT)
    return false;

  // Single aligned word load on M0, no lock needed.
  return (s_LiveMask & (1UL << _id)) != 0;
}

uint32_t diag_GetLiveMask(void)
{
  return s_LiveMask;
}

/*
 * No lock on purpose. Byte loads and stores are single instructions on M0, so no individual
 * counter can be read torn; the only thing missing is a consistent snapshot across all 32, which
 * a rate counter does not need. Worth avoiding because this runs in the I2C1 ISR, and holding
 * PRIMASK for 32 iterations there would be a needless interrupt blackout.
 */
void diag_GetCounters(uint8_t *_p_dst)
{
  if (_p_dst == NULL)
    return;

  for (uint8_t i = 0; i < DIAG_ID_COUNT; i++)
    _p_dst[i] = s_Counters[i];
}

void diag_ClearCounters(uint32_t _mask)
{
  /* Counters first, unlocked: a byte store is atomic, and worst case a concurrent diag_Set()
   * lands right after the zeroing and the incident is counted again - which is correct, it did
   * happen. Only the 32-bit mask RMW needs the lock. */
  for (uint8_t i = 0; i < DIAG_ID_COUNT; i++)
  {
    if (_mask & (1UL << i))
      s_Counters[i] = 0;
  }

  // Acknowledging an incident also drops its live bit; a condition that still holds sets it again.
  uint32_t lock = taskENTER_CRITICAL_FROM_ISR();
  s_LiveMask &= ~_mask;
  taskEXIT_CRITICAL_FROM_ISR(lock);
}
