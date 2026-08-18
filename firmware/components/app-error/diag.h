/*
 * diag.h
 *
 *  Created on: Aug 17, 2026
 *      Author: Roman Egoshin
 */

#ifndef DIAG_H_
#define DIAG_H_

/*============================ INCLUDES ======================================*/

#include <stdint.h>
#include <stdbool.h>

/*============================ TYPES =========================================*/

typedef enum {
  /* power path - power_manager.c protection loop, app.c FAULT state */
  DIAG_PWR_AVDD_SAG        = 0,   // AVDD below PWR_AVDD_FAULT_MV, debounced
  DIAG_PWR_5V_UNDER        = 1,   // 5V bus below PWR_5V_FAULT_MV while armed, debounced
  DIAG_PWR_VBAT_CUTOFF     = 2,   // VBAT below profile cutoff with no input source
  DIAG_PWR_FAULT_LATCHED   = 3,   // retries exhausted, staying powered off

  /* charger BQ2416x */
  DIAG_CHG_UNREACHABLE     = 4,   // CHG_ST_LOST: device does not answer
  DIAG_CHG_REG_CORRUPT     = 5,   // register read violates the invariant masks
  DIAG_CHG_WDT_KICK_FAIL   = 6,   // failed to reset the charger's own 32 s watchdog
  DIAG_CHG_DEV_FAULT       = 7,   // ChargerFaultStatus_t != NONE, code in reg 0xFA bits 1-3

  /* fuel gauge LC709203F */
  DIAG_FG_UNREACHABLE      = 8,   // FG_ST_IC_LOST
  DIAG_FG_BUS_ERR          = 9,   // transfer did not happen: NACK, timeout, BERR/ARLO
  DIAG_FG_CRC_ERR          = 10,  // transfer happened, CRC-8 mismatch
  DIAG_FG_TEMP_STALE       = 11,  // temperature unusable for FG_STALE_LIMIT rounds
  DIAG_FG_RSOC_STALE       = 12,  // charge level unusable for FG_STALE_LIMIT rounds

  /* analog / ADC */
  DIAG_ADC_NO_STREAM       = 13,  // no DMA events: the whole protection loop is dead
  DIAG_ADC_PROC_OVERRUN    = 14,  // ANALOG task fell behind, half-buffers dropped
  DIAG_ADC_HW_OVERRUN      = 15,  // ADC OVR: a conversion was overwritten in DR

  /* i2c */
  DIAG_I2C2_RECOVERY       = 16,  // on-board bus stuck, recovery/re-init ran
  DIAG_I2C2_OP_FAIL        = 17,  // an on-board transfer failed after all retries
  DIAG_I2C1_SLAVE_ERR      = 18,  // host bus error outside the master's terminating NACK
  DIAG_I2C1_FCS_BAD        = 19,  // host write dropped: checksum mismatch

  /* storage / config */
  DIAG_NV_READ_FAIL        = 20,  // emulated EEPROM read failed (NV_ABSENT is not a failure)
  DIAG_NV_WRITE_FAIL       = 21,  // emulated EEPROM write or read-back failed
  DIAG_BAT_PROFILE_INVALID = 22,  // no usable battery profile

  /* event queues - always a design defect, never expected traffic */
  DIAG_QUE_FULL_APP        = 23,
  DIAG_QUE_FULL_CHG        = 24,
  DIAG_QUE_FULL_FG         = 25,
  DIAG_QUE_FULL_BTN        = 26,
  DIAG_QUE_FULL_LED        = 27,

  DIAG_RESET_FATAL         = 28,  // an ASSERT or APP_ERROR took the board down on purpose
  DIAG_RESET_IWDG          = 29,  // the watchdog bit: something hung without reaching a handler
  DIAG_RESET_LPWR          = 30,  // illegal low-power entry; sleep modes are off, so a bug
  DIAG_RESET_HARDFAULT     = 31,  // the core took a fault; no detail, see below

  DIAG_ID_COUNT            = 32
} DiagId_t;

/*============================ DEFINITIONS ===================================*/

#define DIAG_COUNTER_MAX    ((uint8_t)0xFF)   // counters saturate, they never wrap

/*============================ VARIABLES =====================================*/


/*============================ PROTOTYPES ====================================*/

void     diag_Init(bool _cold_start);

void     diag_Set(DiagId_t _id);
void     diag_Clear(DiagId_t _id);
bool     diag_IsSet(DiagId_t _id);
uint32_t diag_GetLiveMask(void);
void     diag_GetCounters(uint8_t *_p_dst);     // writes DIAG_ID_COUNT bytes
void     diag_ClearCounters(uint32_t _mask);

#endif /* DIAG_H_ */
