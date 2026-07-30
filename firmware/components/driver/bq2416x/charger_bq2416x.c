/*
 * charger_bq2416x.c
 *
 *  Created on: 06.12.2016.
 *      Author: milan
 */

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "charger_bq2416x.h"
#include <to_refactor/fuel_gauge_lc709203f.h>
#include <to_refactor/power_source.h>
#include <to_refactor/time_count.h>
#include "nv.h"
#include "app-error/app_assert.h"
#include "src/app.h"
#include "board.h"

// ST HAL/CubeMX:
#include "stm32f0xx_hal.h"

// FreeRTOS:
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

// LOG:
#include "log/log.h"

#include "driver/i2c/i2c_master.h"

#define CHG_READ_PERIOD_MS  90  // ms
#define WD_RESET_TRSH_MS    (30000 / 3)

#define BQ2416X_OTG_LOCK_BIT  0x08

#define CHARGER_VIN_DPM_USB   6

/*
 * Below one tick pdMS_TO_TICKS() rounds to 0 and the wait stops being a wait - same reasoning
 * as button.c's BUTTON_POLL_PERIOD_MS assert.
 */
_Static_assert(pdMS_TO_TICKS(TICK_PERIOD_MS) >= 1,
               "TICK_PERIOD_MS must be at least one RTOS tick");

/* Command types carried by the task queue. */
typedef enum
{
  CHARGER_CMD_WRITE_INPUTS_CONFIG = 1,   // host register - ChargerWriteInputsConfig()
  CHARGER_CMD_WRITE_CHARGING_CONFIG,     // host register - ChargerWriteChargingConfig()
  CHARGER_CMD_SET_BAT_PROFILE            // posted by APP on APP_EVT_BATTERY_PROFILE_CHANGED
} ChargerCmdType_T;

typedef struct
{
  bool valid;
  BatteryProfile_T profile;
} ChargerCmdBatProfile_T;

typedef struct
{
  uint8_t type;
  uint8_t seq;    // meaningful for the two host config commands only
  union
  {
    uint8_t config;
    ChargerCmdBatProfile_T batProfile;
  } data;
} ChargerCmd_T;

// BQ2416x register cache. Touched only by the CHG task - no locking needed.
static uint8_t s_Regs[8] = {0x00, 0x00, 0x8C, 0x14, 0x40, 0x32, 0x00, 0x98};
static uint8_t s_Regsw[8] = {0x08, 0x08, 0x1C, 0x02, 0x00, 0x00, 0x38, 0xC0};
static uint8_t s_RegsStatusRW[8] = {0x00}; // bit0 1 - read error, bit1 1 - write error
static const uint8_t s_RegswMask[8] = {0x08, 0x0A, 0x7F, 0xFF, 0x00, 0xFF, 0x3F, 0xE9};

static uint8_t s_ChargerI2cErrorCounter = 0;
static ChargerUSBInLockoutStatus_T s_UsbInLockoutStatus = CHG_USB_IN_UNKNOWN;
static ChargerUsbInCurrentLimit_T s_ChargerUsbInCurrentLimit = CHG_IUSB_LIMIT_150MA;
static ChargerStatus_T s_ChargerStatus = CHG_NA;
static uint8_t s_ChargerNeedPoll = 0;
static bool s_PowerSourcePresent;

/*
 * Retained across a warm reset (not zeroed - see retained_memory.c for the same idea elsewhere):
 * chargerInit() only reloads these from NV when _reset (POR) is true, so a watchdog/soft reset
 * keeps whatever the host last configured instead of silently reverting to defaults.
 */
static uint8_t s_ChargerInputsConfig __attribute__((section("no_init")));
static uint8_t s_UsbInEnabled __attribute__((section("no_init")));
static uint8_t s_NoBatteryTurnOn __attribute__((section("no_init")));
static uint8_t s_ChargerInputsPrecedence __attribute__((section("no_init")));
static uint8_t s_ChargerInLimit __attribute__((section("no_init"))); // 0 - 1.5A, 1 - 2.5A
static uint8_t s_ChargerInDpm __attribute__((section("no_init")));
static uint8_t s_ChargingConfig __attribute__((section("no_init")));
static uint8_t s_ChargingEnabled __attribute__((section("no_init")));

static uint8_t s_NoBatteryOperationEnabled = 0;

// Battery profile, copied in from battery_GetProfile()/charger_CmdSetBatProfile() - never a
// live pointer into another task's memory.
static BatteryProfile_T s_BatProfile;
static bool s_BatProfileValid;

/*
 * Request mirror for the two host config registers, same idiom as led.c: a write is applied
 * asynchronously by the task, but a host read-back must not see the previous value while the
 * write is still queued.
 */
static uint8_t s_ReqInputsConfig, s_ReqChargingConfig;
static volatile uint8_t s_ReqInputsSeq, s_ReqChargingSeq;
static volatile uint8_t s_AppliedInputsSeq, s_AppliedChargingSeq;
static uint8_t s_InputsConfigReadout, s_ChargingConfigReadout; // NV-verified byte, task writes only

static uint32_t s_ReadTimeCounter;
static uint32_t s_WdTimeCounter;

static uint32_t s_ErrMask;  // CHARGER_ERR_* bits

// FreeRTOS task:
static TaskHandle_t s_TaskHandle;
static StaticTask_t s_TaskTCB;
static StackType_t s_TaskStack[512];   // TODO - remove magic number

// Command queue:
static QueueHandle_t s_QueHandle;
static StaticQueue_t s_Que;
static ChargerCmd_T s_QueBuf[4];       // TODO - remove magic number

/*============================ TASK PRIVATE ==================================*/

static HAL_StatusTypeDef regRead(uint8_t regAddress)
{
  HAL_StatusTypeDef rStatus;
  uint8_t regVal1, regVal2;
  s_RegsStatusRW[regAddress] |= 0x01;
  rStatus = (i2c_master_ReadMem(0xD6, regAddress, &regVal1, 1) == I2C_OK) ? HAL_OK : HAL_ERROR;
  if (rStatus == HAL_OK) {
    // read once more to confirm
    regVal2 = ~regVal1;
    rStatus = (i2c_master_ReadMem(0xD6, regAddress, &regVal2, 1) == I2C_OK) ? HAL_OK : HAL_ERROR;
    if (rStatus == HAL_OK) {
      if (regVal2 == regVal1) {
        s_Regs[regAddress] = regVal2;
        s_RegsStatusRW[regAddress] &= ~0x01;
        rStatus = HAL_OK;
      } else
        rStatus = HAL_ERROR;
    }
  }

  if (rStatus != HAL_OK) {
    s_ChargerI2cErrorCounter++;
    /* Escalation moved out of app main_poll: many soft failures (transport or persistent
     * readback mismatch) -> kick the bus once, then resume. */
    if (s_ChargerI2cErrorCounter > 10) {
      i2c_master_ReInit();
      s_ChargerI2cErrorCounter = 1;
    }
  } else {
    s_ChargerI2cErrorCounter = 0;
  }

  return rStatus;
}

static HAL_StatusTypeDef regWrite(uint8_t regAddress)
{
  s_RegsStatusRW[regAddress] |= 0x03;
  HAL_StatusTypeDef status = (i2c_master_WriteMem(0xD6, regAddress, &s_Regsw[regAddress], 1) == I2C_OK) ? HAL_OK : HAL_ERROR;
  if (status != HAL_OK) return status;
  // verify reading back from charger register
  status = regRead(regAddress);
  if (status == HAL_OK) {
    s_RegsStatusRW[regAddress] &= ~0x01; // read was successful
    if ((s_Regs[regAddress] & s_RegswMask[regAddress]) == (s_Regsw[regAddress] & s_RegswMask[regAddress])) {
      s_RegsStatusRW[regAddress] &= ~0x02; // write was successful
      status = HAL_OK;
    } else {
      status = HAL_ERROR;
    }
  }

  if (status != HAL_OK) {
    s_ChargerI2cErrorCounter++;
  } else {
    s_ChargerI2cErrorCounter = 0;
  }
  return status;
}

static int8_t updateRegulationVoltage(void)
{
  s_Regsw[3] &= ~0xFE;
  s_Regsw[3] |= s_ChargerInLimit << 1;
  if (s_BatProfileValid) {
    int16_t newRegVol;
    if (fuel_gauge_GetTemp() > s_BatProfile.tWarm
        && fuel_gauge_GetTempSenseConfig() != BAT_TEMP_SENSE_CONFIG_NOT_USED) {
      newRegVol = (int16_t)(s_BatProfile.regulationVoltage) - (140 / 20);
      newRegVol = newRegVol < 0 ? 0 : newRegVol;
    } else {
      newRegVol = s_BatProfile.regulationVoltage;
    }

    s_Regsw[3] |= newRegVol << 2;
  } else {
    if (!s_RegsStatusRW[0x03]) return 0;
  }

  // if there were errors in previous transfers read state from register
  if (s_RegsStatusRW[0x03]) {
    if (regRead(0x03) != HAL_OK)
      return 1;
  }

  if ((s_Regsw[3] & 0xFF) != (s_Regs[3] & 0xFF)) {
    // write regulation voltage register
    if (regWrite(0x03) != HAL_OK)
      return 2;
  }

  return 0;
}

static int8_t updateChgCurrentAndTermCurrent(void)
{
  if (s_BatProfileValid) {
    s_Regsw[5] = ((s_BatProfile.chargeCurrent > 26 ? 26 : s_BatProfile.chargeCurrent & 0x1F) << 3)
                 | (s_BatProfile.terminationCurr & 0x07);
  } else {
    s_Regsw[5] = 0;
    if (!s_RegsStatusRW[0x05]) return 0;
  }

  // if there were errors in previous transfers, or first time update, read state from register
  if (s_RegsStatusRW[0x05]) {
    if (regRead(0x05) != HAL_OK)
      return 1;
  }

  if (s_Regsw[5] != s_Regs[5]) {
    // write new value to register
    if (regWrite(0x05) != HAL_OK)
      return 2;
  }

  return 0;
}

static int8_t updateVinDPM(void)
{
  // if there were errors in previous transfers read state from register
  if (s_RegsStatusRW[0x06]) {
    if (regRead(0x06) != HAL_OK)
      return 1;
  }

  s_Regsw[6] = (uint8_t)s_ChargerInDpm | ((uint8_t)CHARGER_VIN_DPM_USB << 3);
  if (s_Regsw[6] != (s_Regs[6] & 0x3F)) {
    // write new value to register
    if (regWrite(0x06) != HAL_OK)
      return 2;
  }

  return 0;
}

static int8_t updateTempRegulationControlStatus(void)
{
  s_Regsw[7] = 0xC0; // Timer slowed by 2x when in thermal regulation, 10-9 hour fast charge, TS function disabled
  if (s_BatProfileValid) {
    if (fuel_gauge_GetTemp() < s_BatProfile.tCool
        && fuel_gauge_GetTempSenseConfig() != BAT_TEMP_SENSE_CONFIG_NOT_USED) {
      // charge current reduced to half
      s_Regsw[7] |= 0x01;
    } else {
      // normal charge current
      s_Regsw[7] &= ~0x01;
    }
  } else {
    s_Regsw[7] &= ~0x01;
    if (!s_RegsStatusRW[0x07]) return 0;
  }

  // if there were errors in previous transfers read state from register
  if (s_RegsStatusRW[0x07]) {
    if (regRead(0x07) != HAL_OK)
      return 1;
  }

  if ((s_Regsw[7] & 0xE9) != (s_Regs[7] & 0xE9)) {
    // write new value to register
    if (regWrite(0x07) != HAL_OK)
      return 2;
  }

  return 0;
}

static int8_t updateControlStatus(void)
{
  s_Regsw[2] = ((s_ChargerUsbInCurrentLimit & 0x07) << 4) | 0x0C; // usb in current limit code, Enable STAT output, Enable charge current termination
  if (s_BatProfileValid) {
    if (!s_ChargingEnabled
        || ((fuel_gauge_GetTemp() >= s_BatProfile.tHot || fuel_gauge_GetTemp() <= s_BatProfile.tCold)
            && fuel_gauge_GetTempSenseConfig() != BAT_TEMP_SENSE_CONFIG_NOT_USED)) {
      // disable charging
      s_Regsw[2] |= 0x02;
      s_Regsw[2] &= ~0x01; // clear high impedance mode
    } else {
      // enable charging
      s_Regsw[2] &= ~0x02;
      s_Regsw[2] &= ~0x01; // clear high impedance mode
      s_Regsw[2] |= 0x04;  // Enable charge current termination
    }
  } else {
    // disable charging
    s_Regsw[2] |= 0x02;
    s_Regsw[2] &= ~0x01; // clear high impedance mode
  }

  // if there were errors in previous transfers, or first time update, read state from register
  if (s_RegsStatusRW[0x02]) {
    if (regRead(0x02) != HAL_OK)
      return 1;
  }

  if ((s_Regsw[2] & 0x7F) != (s_Regs[2] & 0x7F)) {
    // write new value to register
    if (regWrite(0x02) != HAL_OK)
      return 1;
  }

  return 0;
}

static int8_t updateUSBInLockout(void)
{
  s_Regsw[1] = s_NoBatteryOperationEnabled != 0;
  if (s_UsbInEnabled && pwr5vInDetStatus == PWR_5V_IN_DETECTION_STATUS_PRESENT && (s_Regs[1] & 0x06) == 0x00) {
    s_Regsw[1] &= ~BQ2416X_OTG_LOCK_BIT;
  } else {
    s_Regsw[1] |= BQ2416X_OTG_LOCK_BIT;
  }

  s_UsbInLockoutStatus = CHG_USB_IN_UNKNOWN;

  // if there were errors in previous transfers, or first time update, read state from register
  if (s_RegsStatusRW[0x01]) {
    if (regRead(0x01) != HAL_OK)
      return 1;
  }

  if ((s_Regsw[1] & 0x09) != (s_Regs[1] & 0x09)) {
    // write new value to register
    if (regWrite(0x01) != HAL_OK)
      return 2;
  }

  s_UsbInLockoutStatus = (s_Regs[1] & BQ2416X_OTG_LOCK_BIT) ? CHG_USB_IN_LOCK : CHG_USB_IN_UNLOCK;

  return 0;
}

static void setInputsConfig(uint8_t config)
{
  s_ChargerInputsConfig = config;
  s_ChargerInputsPrecedence = config & 0x01;
  s_UsbInEnabled = (config & 0x02) == 0x02;
  s_NoBatteryTurnOn = (config & 0x04) == 0x04;
  s_ChargerInLimit = (config & 0x08) == 0x08;
  s_ChargerInDpm = (config >> 4) & 0x07;
}

/*
 * Persists a configuration and reloads a verified copy for charger_ReadInputsConfig() to serve
 * from - moved out of the I2C1 interrupt, same reasoning as led.c's applyConfig().
 */
static void applyInputsConfig(uint8_t config)
{
  setInputsConfig(config);
  if (config & 0x80)
    nv_write_U8(NV_ADDR_CHARGER_INPUTS_CONFIG, config);

  uint8_t stored = 0;
  s_InputsConfigReadout = (nv_read_U8(NV_ADDR_CHARGER_INPUTS_CONFIG, &stored) == NV_OK
                            && (s_ChargerInputsConfig & 0x7F) == (stored & 0x7F))
                          ? stored : s_ChargerInputsConfig;
}

static void applyChargingConfig(uint8_t config)
{
  s_ChargingConfig = config;
  s_ChargingEnabled = s_ChargingConfig & 0x01;
  if (config & 0x80)
    nv_write_U8(NV_ADDR_CHARGING_CONFIG, config);

  uint8_t stored = 0;
  s_ChargingConfigReadout = (nv_read_U8(NV_ADDR_CHARGING_CONFIG, &stored) == NV_OK
                             && (s_ChargingConfig & 0x7F) == (stored & 0x7F))
                            ? stored : s_ChargingConfig;
}

static void postInputPresenceEvent(bool present)
{
  AppEvent_t evt = { .type = APP_EVT_CHARGER_INPUT_PRESENCE, .data.chargerInput = { present } };
  if (app_PostEvent(&evt))
    return;

  LOG_ERROR("[CHG] APP event queue full, input presence event dropped");
  taskENTER_CRITICAL();
  s_ErrMask |= CHARGER_ERR_EVENT_LOST;
  taskEXIT_CRITICAL();
}

static void chargerInit(bool _reset)
{
  if (_reset)
  {
    if (nv_read_U8(NV_ADDR_CHARGER_INPUTS_CONFIG, &s_ChargerInputsConfig) == NV_OK) {
      setInputsConfig(s_ChargerInputsConfig);
    } else {
      // set default config
      s_ChargerInputsPrecedence = 0x01; // 5V GPIO has precedence for charging
      s_UsbInEnabled = 1; // enabled
      s_NoBatteryTurnOn = 0;
      s_ChargerInLimit = 1; // 2.5A
      s_ChargerInDpm = 0; // 4.2V
      s_ChargerInputsConfig = s_ChargerInputsPrecedence & 0x01;
      s_ChargerInputsConfig |= s_UsbInEnabled << 1;
      s_ChargerInputsConfig |= s_NoBatteryTurnOn << 2;
      s_ChargerInputsConfig |= s_ChargerInLimit << 3;
      s_ChargerInputsConfig |= s_ChargerInDpm << 4;
    }

    if (nv_read_U8(NV_ADDR_CHARGING_CONFIG, &s_ChargingConfig) == NV_OK) {
      s_ChargingEnabled = s_ChargingConfig & 0x01;
    } else {
      // set default config
      s_ChargingEnabled = 1;
      s_ChargingConfig = s_ChargingEnabled & 0x01;
    }
  }

  // Refresh the read-back cache every boot, not just on POR - otherwise a warm reset would
  // serve a stale 0 from charger_Read*Config() until the next host write.
  s_InputsConfigReadout = s_ChargerInputsConfig;
  s_ChargingConfigReadout = s_ChargingConfig;

  MS_TIME_COUNTER_INIT(s_ReadTimeCounter);
  MS_TIME_COUNTER_INIT(s_WdTimeCounter);

  s_Regsw[1] |= 0x08; // lockout usbin
  i2c_master_WriteMem(0xD6, 1, &s_Regsw[1], 1);

  // NOTE: do not place in high impedance mode, it will disable VSys mosfet, and no power to mcu
  s_Regsw[2] |= 0x02; // set control register, disable charging initially
  s_Regsw[2] |= 0x20; // Set USB limit 500mA
  i2c_master_WriteMem(0xD6, 2, &s_Regsw[2], 1);

  DelayUs(500);

  // read states
  i2c_master_ReadMem(0xD6, 0, s_Regs, 8);

  updateUSBInLockout();
  updateTempRegulationControlStatus();

  regRead(0);
  regRead(1);

  s_ChargerStatus = (s_Regs[0] >> 4) & 0x07;

  s_PowerSourcePresent = charger_IsInputPresent();
}

static void chargerTick(bool irqWake)
{
  static int8_t currRegAdr;
  s_ChargerNeedPoll = 0;

  if (updateUSBInLockout() != 0) {
    s_ChargerNeedPoll = 1;
    return;
  }

  if (irqWake) {
    // update status on interrupt
    regRead(0);
    s_ChargerNeedPoll = 1;
  }

  if (updateControlStatus() != 0) {
    s_ChargerNeedPoll = 1;
    return;
  }

  if (updateRegulationVoltage() != 0) {
    s_ChargerNeedPoll = 1;
    return;
  }

  if (updateTempRegulationControlStatus() != 0) {
    s_ChargerNeedPoll = 1;
    return;
  }

  if (updateChgCurrentAndTermCurrent() != 0) {
    s_ChargerNeedPoll = 1;
    return;
  }

  if (updateVinDPM() != 0) {
    s_ChargerNeedPoll = 1;
    return;
  }

  if (MS_TIME_COUNT(s_WdTimeCounter) > WD_RESET_TRSH_MS) {
    // reset timer
    // NOTE: reset bit must be 0 in write register image to prevent resets for other write access
    s_Regsw[0] = s_ChargerInputsPrecedence << 3;
    uint8_t resetVal = s_Regsw[0] | 0x80;
    if (i2c_master_WriteMem(0xD6, 0, &resetVal, 1) == I2C_OK) {
      MS_TIME_COUNTER_INIT(s_WdTimeCounter);
    }
  }

  // Periodically read register states from charger
  if (MS_TIME_COUNT(s_ReadTimeCounter) >= CHG_READ_PERIOD_MS) {

    if (regRead(currRegAdr) == HAL_OK) {
      currRegAdr++;
      currRegAdr &= 0x07; // 8 registers to read in circle
    }

    s_ChargerStatus = (s_Regs[0] >> 4) & 0x07;

    if (currRegAdr == 0) {
      bool present = charger_IsInputPresent();
      if (s_PowerSourcePresent != present) {
        s_PowerSourcePresent = present;
        postInputPresenceEvent(present);
      }
    }

    MS_TIME_COUNTER_INIT(s_ReadTimeCounter);
  }
}

static void applyCmd(const ChargerCmd_T *_pCmd)
{
  switch (_pCmd->type)
  {
    case CHARGER_CMD_WRITE_INPUTS_CONFIG:
      applyInputsConfig(_pCmd->data.config);
      s_AppliedInputsSeq = _pCmd->seq;
      break;

    case CHARGER_CMD_WRITE_CHARGING_CONFIG:
      applyChargingConfig(_pCmd->data.config);
      s_AppliedChargingSeq = _pCmd->seq;
      break;

    case CHARGER_CMD_SET_BAT_PROFILE:
      s_BatProfileValid = _pCmd->data.batProfile.valid;
      if (s_BatProfileValid)
        s_BatProfile = _pCmd->data.batProfile.profile;
      break;

    default:
      ASSERT(0);
      break;
  }
}

static void Task(void *parameters)
{
  bool reset = (bool)(uintptr_t)parameters;

  LOG_INFO("CHARGER task started");

  HAL_Delay(100); // after power-on, charger and fuel gauge require initialization time

  s_BatProfileValid = battery_GetProfile(&s_BatProfile);

  chargerInit(reset);

  while (1)
  {
    /*
     * Single wait, same idiom as button.c: satisfies both the ~90 ms register poll cadence
     * (via the timeout) and an immediate wake on the CHG_INT EXTI edge (via
     * charger_NotifyFromISR()). The notification count tells the two apart - see chargerTick().
     */
    uint32_t n = ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(TICK_PERIOD_MS));

    ChargerCmd_T cmd;
    while (xQueueReceive(s_QueHandle, &cmd, 0) == pdTRUE)
      applyCmd(&cmd);

    chargerTick(n > 0);
  }
}

/*============================ COMMAND POSTING ===============================*/

static bool postCmd(const ChargerCmd_T *_pCmd)
{
  if (s_QueHandle == NULL)
  {
    s_ErrMask |= CHARGER_ERR_NOT_READY;
    return false;
  }

  BaseType_t sent;

  if (xPortIsInsideInterrupt() != pdFALSE)
  {
    BaseType_t woken = pdFALSE;
    sent = xQueueSendFromISR(s_QueHandle, _pCmd, &woken);
    portYIELD_FROM_ISR(woken);

    if (sent != pdTRUE)
      s_ErrMask |= CHARGER_ERR_QUEUE_FULL;
  }
  else
  {
    sent = xQueueSend(s_QueHandle, _pCmd, 0);

    if (sent != pdTRUE)
    {
      LOG_ERROR("[CHG] Command queue full, cmd=%u dropped", _pCmd->type);
      taskENTER_CRITICAL();
      s_ErrMask |= CHARGER_ERR_QUEUE_FULL;
      taskEXIT_CRITICAL();
    }
  }

  return (sent == pdTRUE);
}

/*============================ PUBLIC API ====================================*/

void charger_Init(bool _reset)
{
  LOG_INFO("charger_Init...");

  s_QueHandle = xQueueCreateStatic(sizeof(s_QueBuf) / sizeof(s_QueBuf[0]),
                            sizeof(s_QueBuf[0]), (uint8_t*)s_QueBuf, &s_Que);
  ASSERT(s_QueHandle != NULL);

  s_TaskHandle = xTaskCreateStatic(Task, "CHG", sizeof(s_TaskStack) / sizeof(StackType_t),
                            (void*)(uintptr_t)_reset, 6, s_TaskStack, &s_TaskTCB);
  ASSERT(s_TaskHandle != NULL);
}

void charger_NotifyFromISR(void)
{
  if (s_TaskHandle == NULL)
    return;

  BaseType_t woken = pdFALSE;
  vTaskNotifyGiveFromISR(s_TaskHandle, &woken);
  portYIELD_FROM_ISR(woken);
}

ChargerStatus_T charger_GetStatus(void)
{
  return s_ChargerStatus;
}

bool charger_IsBatteryPresent(void)
{
  return !((s_Regs[1] & 0x06) == 0x04);
}

bool charger_IsInputPresent(void)
{
  return (s_Regs[0] & 0x70) && ((s_Regs[0] & 0x70) < (6 * 16));
}

uint8_t charger_GetInStat(void)
{
  return (s_Regs[1] >> 6) & 0x03;
}

uint8_t charger_GetUsbStat(void)
{
  return (s_Regs[1] >> 4) & 0x03;
}

bool charger_IsDpmModeActive(void)
{
  return (s_Regs[6] & 0x40) != 0;
}

ChargerUSBInLockoutStatus_T charger_GetUsbInLockoutStatus(void)
{
  return s_UsbInLockoutStatus;
}

uint8_t charger_GetTsFaultStatus(void)
{
  // NOTE (pre-existing, preserved on purpose): this is `s_Regs[7] & (0x06>>1)`, not
  // `(s_Regs[7]&0x06)>>1` - the original CHRGER_TS_FAULT_STATUS() macro has the same operator
  // precedence quirk. Not fixed here - this refactor is behaviour preserving.
  return s_Regs[7] & 0x06 >> 1;
}

ChargerFaultStatus_T charger_GetFaultStatus(void)
{
  return (ChargerFaultStatus_T)(s_Regs[0] & 0x07);
}

uint8_t charger_GetI2cErrorCount(void)
{
  return s_ChargerI2cErrorCounter;
}

bool charger_IsNoBatteryTurnOnEnabled(void)
{
  return s_NoBatteryTurnOn != 0;
}

void charger_TriggerNtcMonitor(NTC_MonitorTemperature_T temp)
{
  switch (temp) {
  case COLD:
    HAL_GPIO_WritePin(CHG_NTC_CTRL1_PORT, CHG_NTC_CTRL1_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(CHG_NTC_CTRL2_PORT, CHG_NTC_CTRL2_PIN, GPIO_PIN_SET);
    break;
  case COOL:
    break;
  case WARM:
    break;
  case HOT:
    HAL_GPIO_WritePin(CHG_NTC_CTRL1_PORT, CHG_NTC_CTRL1_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(CHG_NTC_CTRL2_PORT, CHG_NTC_CTRL2_PIN, GPIO_PIN_RESET);
    break;
  case NORMAL: default:
    HAL_GPIO_WritePin(CHG_NTC_CTRL1_PORT, CHG_NTC_CTRL1_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(CHG_NTC_CTRL2_PORT, CHG_NTC_CTRL2_PIN, GPIO_PIN_RESET);
    break;
  }
}

void charger_SetUsbLockout(ChargerUSBInLockoutStatus_T status)
{
  (void)status;
  s_ChargerNeedPoll |= (uint8_t)updateUSBInLockout();
}

void charger_UsbInCurrentLimitStepUp(void)
{
  if (s_ChargerUsbInCurrentLimit < CHG_IUSB_LIMIT_1500MA) s_ChargerUsbInCurrentLimit++;
}

void charger_UsbInCurrentLimitStepDown(void)
{
  if (s_ChargerUsbInCurrentLimit > CHG_IUSB_LIMIT_150MA) s_ChargerUsbInCurrentLimit--;
}

void charger_UsbInCurrentLimitSetMin(void)
{
  s_ChargerUsbInCurrentLimit = CHG_IUSB_LIMIT_150MA;
}

void charger_CmdWriteInputsConfig(uint8_t config)
{
  s_ReqInputsConfig = config;
  ChargerCmd_T cmd = { .type = CHARGER_CMD_WRITE_INPUTS_CONFIG,
                       .seq = (uint8_t)(s_ReqInputsSeq + 1),
                       .data.config = config };
  if (postCmd(&cmd))
    s_ReqInputsSeq = cmd.seq;
}

uint8_t charger_ReadInputsConfig(void)
{
  return (s_ReqInputsSeq != s_AppliedInputsSeq) ? s_ReqInputsConfig : s_InputsConfigReadout;
}

void charger_CmdWriteChargingConfig(uint8_t config)
{
  s_ReqChargingConfig = config;
  ChargerCmd_T cmd = { .type = CHARGER_CMD_WRITE_CHARGING_CONFIG,
                       .seq = (uint8_t)(s_ReqChargingSeq + 1),
                       .data.config = config };
  if (postCmd(&cmd))
    s_ReqChargingSeq = cmd.seq;
}

uint8_t charger_ReadChargingConfig(void)
{
  return (s_ReqChargingSeq != s_AppliedChargingSeq) ? s_ReqChargingConfig : s_ChargingConfigReadout;
}

void charger_CmdSetBatProfile(const BatteryProfile_T *batProfile)
{
  ChargerCmd_T cmd = { .type = CHARGER_CMD_SET_BAT_PROFILE };
  cmd.data.batProfile.valid = (batProfile != NULL);
  if (batProfile != NULL)
    cmd.data.batProfile.profile = *batProfile;
  postCmd(&cmd);
}

uint32_t charger_GetErrMask(bool _clear)
{
  taskENTER_CRITICAL();
  uint32_t mask = s_ErrMask;
  if (_clear)
    s_ErrMask &= ~mask;
  taskEXIT_CRITICAL();
  return mask;
}

__weak void charger_OnEvent_InputPresenceChanged(bool present)
{
  UNUSED(present);
}
