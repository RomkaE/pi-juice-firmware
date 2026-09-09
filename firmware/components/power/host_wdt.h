/*
 * host_wdt.h
 *
 *  Created on: Sep 9, 2026
 *      Author: Roman Egoshin
 */

#ifndef COMPONENTS_POWER_HOST_WDT_H_
#define COMPONENTS_POWER_HOST_WDT_H_

/*============================ INCLUDES ======================================*/

#include <stdint.h>
#include <stdbool.h>

/*============================ PROTOTYPES ====================================*/

/* Creates the tick timer and loads the stored period. APP task, after nv_Init(). */
void host_wdt_Init(void);

/* The 5V bus is about to come up: the host has to boot before it can poll us, so the next
 * window is the doubled one. */
void host_wdt_OnHostPowerUp(void);

/* The countdown runs only while the host is powered. Both are idempotent. */
void host_wdt_Start(void);
void host_wdt_Stop(void);

/* Has the host reached us at all since the 5V bus came up? False means it never booted far
 * enough to talk, which is a different failure from a host that ran and then hung. */
bool host_wdt_HostSpoke(void);

/* Host register 0x61. Set runs in the I2C1 ISR and only posts the request; ApplyConfig does the
 * NV work in the APP task, off that ISR. */
void host_wdt_CmdSetConfig(uint8_t _data[], uint16_t _len);
void host_wdt_CmdGetConfig(uint8_t _data[], uint16_t *_p_len);
void host_wdt_ApplyConfig(uint16_t _minutes, bool _store, uint8_t _seq);

#endif /* COMPONENTS_POWER_HOST_WDT_H_ */
