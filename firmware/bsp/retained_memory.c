/*
 * retained_memory.c
 *
 * Why this exists.
 *
 * The linker script places the no_init section straight after .bss, with no fixed address
 * (see STM32F030CCTX_FLASH.ld). Its base is therefore the end of .bss, so *any* change in
 * the size of .bss moves every retained variable. RAM survives a reset and a reflash, so
 * after such a move the firmware reads the previous image's leftovers at the new addresses
 * and takes them for its own state.
 *
 * That is not a theoretical concern: shrinking the ADC ring buffer by 7424 bytes moved the
 * whole section down by the same amount. main_init() trusts retained data whenever
 * executionState happens to hold one of its magic values, and then ChargerInit() skips
 * reading its configuration from NV and FuelGaugeInit() skips rebuilding the SoC tables -
 * so a stale word could leave the charger disabled and the gauge reporting nonsense while
 * the purely ADC-derived readings still looked perfectly healthy.
 *
 * What is checked, and what deliberately is not.
 *
 * A CRC over the live section would not work: the retained data changes continuously
 * (timers, soc, event flags) while resets are asynchronous (watchdog, POR). Keeping a
 * reference CRC current would mean recomputing it after every write, and a reset landing
 * mid-update would still fail the comparison and throw away perfectly good state.
 *
 * What actually fails here is the *layout*, so that is what is fingerprinted: a magic word
 * plus the section base, its size, and the token's own address, cross-checked against each
 * other. The token lives inside the section, so if the section moves the token moves with
 * it, lands on foreign data and fails to match - which is exactly the detection wanted.
 *
 * This detects the corruption rather than preventing it. Prevention would mean pinning the
 * section to a fixed address in the linker script; detection is enough for correctness,
 * because an invalid verdict simply forces the cold-boot path and everything is re-read
 * from NV. The cost is losing retained state across a firmware update that changes .bss.
 */

#include <stdint.h>
#include <stdbool.h>
#include "retained_memory.h"

/* Section bounds, from the linker script. */
extern uint32_t _snoinit;
extern uint32_t _enoinit;

#define RETAINED_TOKEN_MAGIC    ((uint32_t)0x6E496552)	/* "ReIn" */
#define RETAINED_MIX            ((uint32_t)0x9E3779B9)	/* odd, so it does not lose bits */

typedef struct
{
	uint32_t magic;
	uint32_t base;
	uint32_t size;
	uint32_t check;
} RetainedToken_T;

static bool s_Status = true;

static RetainedToken_T retainedToken __attribute__((section("no_init")));

bool retained_mem_Check(void)
{
  uint32_t base = (uint32_t) &_snoinit;
  uint32_t size = (uint32_t) &_enoinit - base;
  /*
   * Ties the three fields together and to the token's own placement, so a stale token
   * that happens to carry the right magic still has to sit at the right address.
   */
  uint32_t check = RETAINED_TOKEN_MAGIC ^ (base * RETAINED_MIX)
      ^ (size * RETAINED_MIX) ^ (uint32_t) &retainedToken;

  s_Status = (retainedToken.magic == RETAINED_TOKEN_MAGIC)
      && (retainedToken.base == base) && (retainedToken.size == size)
      && (retainedToken.check == check);

  retainedToken.magic = RETAINED_TOKEN_MAGIC;
  retainedToken.base = base;
  retainedToken.size = size;
  retainedToken.check = check;

  return s_Status;
}

bool retained_mem_GetStatus(void)
{
  return s_Status;
}
