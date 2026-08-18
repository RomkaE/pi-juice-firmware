/*
 * app_fatal.c
 *
 *  Created on: Aug 17, 2026
 *      Author: Roman Egoshin
 */

/*============================ INCLUDES ======================================*/

#include <stdbool.h>

#include "app_fatal.h"
#include "diag.h"

/*
 * The one file of this component that reaches outside it: the reset comes from the BSP and the
 * flush from the log. app_assert.c and app_error.c stay free of both, so the failure reporting
 * itself carries no board dependency.
 */
#include "board.h"
#include "log/log.h"

/*============================ TYPES =========================================*/


/*============================ VARIABLES =====================================*/

static volatile bool s_fInFatalHandler;

/*============================ PRIVATE DEFINITIONS ===========================*/


/*============================ PRIVATE PROTOTYPES ============================*/


/*============================ IMPLEMENTATION (PRIVATE FUNCTIONS) ============*/

/*
 * Deliberately identical in task and interrupt context.
 *
 * Nothing is deferred to thread mode. The reset point is what matters, and once the reset is
 * unconditional the usual objections to working inside an ISR stop applying: blocking the ADC
 * protection loop, the charger watchdog kick or a host I2C transaction costs nothing when all three
 * are about to be reset anyway. Flushing from an ISR is therefore fine too, and worth it - it is
 * the difference between having the last lines and guessing.
 *
 * Timing: the watchdog is kicked on entry, so the flush gets a full IWDG period to itself rather
 * than whatever was left of one. Without that kick the margin was not what it looked like - the
 * fatal can land anywhere in the 250 ms refresh cycle, and if the timer task was already starved
 * the counter may be near expiry. The watchdog would then bite mid-flush: log lines lost, and the
 * reset reported as IWDG instead of the deliberate one DIAG_RESET_FATAL claims.
 */
__attribute__((noreturn)) static void Fatal(void)
{
  bsp_WdtRefresh();   // safe before bsp_WdtStart() too, see the guard in board.c

  /*
   * With no debugger attached RdOff never moves, so this always burns the full budget rather than
   * returning early. That is accepted: it happens once, immediately before a reset.
   */
  LOG_FLUSH();   // macro, so it disappears together with the log when LOG_ENABLED=0

#ifndef NDEBUG
#ifdef __arm__
  // Under a debugger stop here first. Safe to sit here: bsp_WdtStart() freezes the IWDG under the
  // same !NDEBUG condition, so the board will not reset out from under the session. Continuing
  // resumes into the reset below. Reaching this at all means a debugger is attached, so the flush
  // above has already succeeded.
  __asm volatile ("bkpt #0x01");
#endif
#endif /* NDEBUG */

  bsp_ResetCPU();
  while (1);
}

/*============================ IMPLEMENTATION (PUBLIC FUNCTIONS) =============*/

void app_fatal_Handle(void)
{
  if (s_fInFatalHandler)
    return;
  s_fInFatalHandler = true;

  /*
   * Recorded before the reset, and it survives it - this is what separates "the firmware killed
   * itself" from the config-reset button, which reaches NVIC_SystemReset() through the same
   * RCC_FLAG_SFTRST and is otherwise indistinguishable.
   */
  diag_Set(DIAG_RESET_FATAL);

  Fatal();
}
