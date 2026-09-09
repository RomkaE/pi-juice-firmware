/*
 * host_wdt.c
 *
 *  Created on: Sep 9, 2026
 *      Author: Roman Egoshin
 */

/*============================ INCLUDES ======================================*/

#include <stdint.h>
#include <stdbool.h>

#include "host_wdt.h"
#include "nv.h"
#include "src/app.h"
#include "app-error/app_assert.h"
#include "app-error/app_error.h"
#include "driver/i2c/i2c_slave.h"
#include "log/log.h"

// FreeRTOS:
#include "FreeRTOS.h"
#include "timers.h"

/*============================ PRIVATE DEFINITIONS ===========================*/

/*
 * The host is expected to keep talking to us over I2C; when it goes quiet for the configured
 * period the only recovery this board has is a 5V power cycle - there is no RUN pin. What that
 * cycle looks like is the FSM's business: this module only reports the fact.
 *
 * Register 0x61 carries the period in minutes, v1.6's encoding; it is kept in whole seconds here.
 */
#define HOST_WDT_TICK_MS     1000

/* The first window after a power-up is longer: the host has to boot and start its daemon before it
 * can poll anything. The floor matters at short periods - twice a one minute period is not enough
 * for a cold boot. It drops to the configured period on the first request that arrives. */
#define HOST_WDT_GRACE_MULT     2
#define HOST_WDT_GRACE_MIN_SEC  180

/*============================ TYPES =========================================*/

/*
 * Request mirror, the idiom of CfgMirror_t in app.c in the shape register 0x61 needs: a 16 bit
 * timeout and the store bit. It earns its keep because the apply can take an EEPROM page erase,
 * long enough for the host's verifying read to overtake it - pijuice_gui.py reads the register
 * back on the very next transaction.
 */
typedef struct
{
  uint16_t requested;
  bool req_store;
  uint8_t req_seq;
  uint8_t applied_seq;
} WdtMirror_t;

/*============================ VARIABLES =====================================*/

static TimerHandle_t s_TimerHandle;
static StaticTimer_t s_Timer;

static uint16_t s_Config;        // persisted minutes, 0 when nothing is stored
static uint32_t s_PeriodSec;     // the period in effect, 0 disables
static uint32_t s_CountSec;      // seconds left
static uint8_t  s_ActSeq;        // last seen i2c_slave_GetHostActivitySeq()
static bool     s_Grace = true;  // the host has not spoken since this power-up

static WdtMirror_t s_Mirror;

/*============================ IMPLEMENTATION (PRIVATE FUNCTIONS) ============*/

/*
 * Has the host been heard since the last look? The first request also ends the boot grace: it is
 * what proves the host has finished booting.
 */
static bool SeenActivity(void)
{
  uint8_t seq = i2c_slave_GetHostActivitySeq();
  if (seq == s_ActSeq)
    return false;

  s_ActSeq = seq;
  if (s_Grace)
  {
    s_Grace = false;
    LOG_INFO("[PWR] Host WDT: first host request, boot grace over");
  }
  return true;
}

/*
 * Timer service task. It outranks the APP task, so it cannot be preempted by the arming below;
 * and the APP task only ever stores whole words here, which are atomic on M0. No lock needed.
 */
static void OnTimer(TimerHandle_t _timer)
{
  (void)_timer;

  if (SeenActivity())
  {
    s_CountSec = s_PeriodSec;   // refreshed, never with the grace multiplier
    return;
  }

  // Zero also parks the tick after an expiry, until the FSM arms the next window.
  if (s_CountSec == 0 || --s_CountSec != 0)
    return;

  LOG_ERROR("[PWR] Host WDT expired: the host has gone quiet");
  AppEvent_t evt = { .type = APP_EVT_HOST_WDT_EXPIRED };
  app_PostEvent(&evt);
}

/*============================ IMPLEMENTATION (PUBLIC FUNCTIONS) =============*/

void host_wdt_Init(void)
{
  s_TimerHandle = xTimerCreateStatic("HOST_WDT", pdMS_TO_TICKS(HOST_WDT_TICK_MS),
                       pdTRUE, NULL, OnTimer, &s_Timer);
  ASSERT(s_TimerHandle != NULL);

  // Only a complete pair counts - half a period would arm the watchdog with a bogus timeout.
  uint8_t valueL = 0, valueH = 0;
  if (nv_read_U8(NV_ADDR_HOST_WDT_CONFIGL, &valueL) != NV_OK
   || nv_read_U8(NV_ADDR_HOST_WDT_CONFIGH, &valueH) != NV_OK)
  {
    valueL = 0;   // nothing stored: watchdog disabled
    valueH = 0;
  }
  s_Config = (uint16_t)valueH << 8 | valueL;

  /* A stored config is v1.6's restore flag: it re-arms the watchdog by itself. v1.6 did that on
   * the next wake-up; here it is simply the period the FSM finds set when it enters ON. */
  s_PeriodSec = (uint32_t)s_Config * 60u;
}

void host_wdt_OnHostPowerUp(void)
{
  /* The snapshot is taken before the bus comes up, so traffic from before a cut cannot be
   * mistaken for the host having booted. */
  s_Grace = true;
  s_ActSeq = i2c_slave_GetHostActivitySeq();
}

void host_wdt_Start(void)
{
  (void)SeenActivity();   // a config write is itself a request, so it ends the grace

  if (s_PeriodSec == 0)
  {
    host_wdt_Stop();
    LOG_INFO("[PWR] Host WDT disabled");
    return;
  }

  if (s_Grace)
  {
    uint32_t grace = s_PeriodSec * HOST_WDT_GRACE_MULT;
    s_CountSec = (grace < HOST_WDT_GRACE_MIN_SEC) ? HOST_WDT_GRACE_MIN_SEC : grace;
  }
  else
  {
    s_CountSec = s_PeriodSec;
  }

  // Restarting an auto reload timer is what gives it a fresh phase.
  if (xTimerStart(s_TimerHandle, 0) != pdPASS)
    APP_ERROR(APP_ERR_RTOS_TIMER);
  else
    LOG_INFO("[PWR] Host WDT armed: %usec.", (unsigned)s_CountSec);
}

void host_wdt_Stop(void)
{
  s_CountSec = 0;
  xTimerStop(s_TimerHandle, 0);
}

bool host_wdt_HostSpoke(void)
{
  return !s_Grace;
}

void host_wdt_CmdSetConfig(uint8_t _data[], uint16_t _len)
{
  if (_len < 2)
    return;   // _len still counts the FCS byte, so this is "less than one 16 bit word"

  uint16_t cfg = ((uint16_t)_data[1] << 8) | _data[0];
  uint16_t minutes = cfg & 0x3FFF;
  if (cfg & 0x4000)
    minutes <<= 2;   // 4 minute resolution over the 16384-65536 range

  LOG_WARNING("[PWR] Rcvd CMD SetHostWDTConfig: %umin, store=%u",
      (unsigned)minutes, (unsigned)((cfg & 0x8000) != 0));

  s_Mirror.requested = minutes;
  s_Mirror.req_store = (cfg & 0x8000) != 0;
  s_Mirror.req_seq++;

  /* The NV write is left to the APP task: this runs in the I2C1 interrupt. */
  AppEvent_t evt = { .type = APP_EVT_CMD_SET_HOST_WDT_CONFIG };
  evt.hostWdt.minutes = minutes;
  evt.hostWdt.store = s_Mirror.req_store;
  evt.hostWdt.seq = s_Mirror.req_seq;
  app_PostEvent(&evt);
}

/*
 * The configured period, never the remaining time and never the doubled boot grace: 0x61 is the
 * configuration register, and a host that read back twice what it wrote would call that a bug.
 */
void host_wdt_CmdGetConfig(uint8_t _data[], uint16_t *_p_len)
{
  LOG_DEBUG("[PWR] Rcvd CMD GetHostWDTConfig");

  uint16_t d;
  bool nv;
  if (s_Mirror.req_seq != s_Mirror.applied_seq)
  {
    d = s_Mirror.requested;    // not applied yet, answer with what was asked for
    nv = s_Mirror.req_store;
  }
  else
  {
    d = s_Config ? s_Config : (uint16_t)(s_PeriodSec / 60u);
    nv = (s_Config != 0);
  }

  if (d >= 0x4000)
    d = (d >> 2) | 0x4000;

  _data[0] = d;
  _data[1] = (d >> 8) | (nv ? 0x80 : 0x00);
  *_p_len = 2;
}

/*
 * Persist the timeout when the host asked for it, and let the read-back decide: whatever ends up
 * in NV is what the watchdog runs with. APP task - the flash write must not happen in the I2C1
 * ISR that delivered the host write.
 *
 * Arming is not done here. This is the fact, the FSM owns the policy - see state_On().
 */
void host_wdt_ApplyConfig(uint16_t _minutes, bool _store, uint8_t _seq)
{
  if (_store)
  {
    if (nv_write_U8(NV_ADDR_HOST_WDT_CONFIGL, (uint8_t)_minutes) != NV_OK
     || nv_write_U8(NV_ADDR_HOST_WDT_CONFIGH, (uint8_t)(_minutes >> 8)) != NV_OK)
      LOG_ERROR("[PWR] NV write of the host watchdog config failed");

    uint8_t valueL = 0, valueH = 0;
    if (nv_read_U8(NV_ADDR_HOST_WDT_CONFIGL, &valueL) != NV_OK
     || nv_read_U8(NV_ADDR_HOST_WDT_CONFIGH, &valueH) != NV_OK)
      s_Config = 0;
    else
      s_Config = (uint16_t)valueH << 8 | valueL;

    _minutes = s_Config;
  }
  else
  {
    s_Config = 0;   // the live period is not backed by NV any more
  }

  s_PeriodSec = (uint32_t)_minutes * 60u;
  s_Mirror.applied_seq = _seq;
}
