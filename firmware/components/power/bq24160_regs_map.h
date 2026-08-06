/*
 * bq24160_regs_map.h
 *
 *  Created on: Aug 4, 2026
 *      Author: Roman Egoshin
 */

#ifndef COMPONENTS_POWER_BQ24160_REGS_MAP_H_
#define COMPONENTS_POWER_BQ24160_REGS_MAP_H_

#include <stdint.h>

#define BQ_I2C_ADDR             0xD6
#define BQ_REG_COUNT            8

#define BQ_REG_STATUS_CONTROL   0   // charger status, watchdog reset bit
#define BQ_REG_SUPPLY_STATUS    1   // IN/USB status, OTG lockout, BATSTAT
#define BQ_REG_CONTROL          2   // USB current limit, charge enable, termination
#define BQ_REG_BAT_VOLTAGE      3   // regulation voltage, IN current limit
#define BQ_REG_VENDOR           4   // read only
#define BQ_REG_CHARGE_CURRENT   5   // fast charge and termination current
#define BQ_REG_VIN_DPM          6   // VIN-DPM thresholds, DPM status
#define BQ_REG_SAFETY_NTC       7   // safety timer, thermal regulation, TS

#define BQ_WRITABLE_MASK_INIT   { 0x08, 0x09, 0x7F, 0xFF, 0x00, 0xFF, 0x3F, 0xE9 }

// Register 0 - Status/Control
#define BQ_WD_RESET_BIT         0x80  // TMR_RST, write only
#define BQ_PRECEDENCE_BIT       0x08  // SUPPLY_SEL: 0 = IN has precedence, 1 = USB
#define BQ_STATUS_MASK          0x70  // STAT_x, read only
#define BQ_FAULT_MASK           0x07  // FAULT_x, read only

// Register 1 - Battery/Supply Status
#define BQ_INSTAT_MASK          0xC0  // read only
#define BQ_USBSTAT_MASK         0x30  // read only
#define BQ_OTG_LOCK_BIT         0x08
#define BQ_BATSTAT_MASK         0x06  // read only
#define BQ_EN_NOBATOP_BIT       0x01

// Values of the BATSTAT field, as read through .bits.batstat
#define BQ_BATSTAT_PRESENT      0     // present and normal
#define BQ_BATSTAT_OVP          1
#define BQ_BATSTAT_NOT_PRESENT  2
#define BQ_BATSTAT_NA           3

// Register 2 - Control
#define BQ_RESET_BIT            0x80  // write 1 resets every register; reads back as 1, always
#define BQ_IUSB_LIMIT_MASK      0x70  // codes match ChargerUsbInCurrentLimit_T one for one
#define BQ_IUSB_LIMIT_SHIFT     4
#define BQ_STAT_ENABLE_BIT      0x08  // EN_STAT
#define BQ_TERM_ENABLE_BIT      0x04  // TE
#define BQ_CHG_DISABLE_BIT      0x02  // CE: 1 disables charging
#define BQ_HIGH_IMPEDANCE_BIT   0x01  // HZ_MODE

// Register 3 - Control/Battery Voltage
#define BQ_VBREG_MASK           0xFC
#define BQ_VBREG_SHIFT          2
#define BQ_IINLIMIT_BIT         0x02  // IN input limit: 0 = 1.5 A, 1 = 2.5 A
#define BQ_DPDM_EN_BIT          0x01  // forces D+/D- detection, self-clearing

// Register 5 - Battery Termination/Fast Charge Current
#define BQ_ICHRG_MASK           0xF8
#define BQ_ICHRG_SHIFT          3
#define BQ_ITERM_MASK           0x07

// Register 6 - VIN-DPM Voltage/DPPM Status
#define BQ_MINSYS_STATUS_BIT    0x80  // read only
#define BQ_DPM_STATUS_BIT       0x40  // read only
#define BQ_VINDPM_USB_MASK      0x38
#define BQ_VINDPM_USB_SHIFT     3
#define BQ_VINDPM_IN_MASK       0x07

// Register 7 - Safety Timer/NTC Monitor
#define BQ_2XTMR_EN_BIT         0x80
#define BQ_TMR_MASK             0x60
#define BQ_TMR_SHIFT            5
#define BQ_TS_FAULT_SHIFT       1

// Values of the TMR field, as written through .bits.tmr
#define BQ_TMR_27MIN            0
#define BQ_TMR_6HOUR            1
#define BQ_TMR_9HOUR            2
#define BQ_TMR_OFF              3
#define BQ_TS_EN_BIT            0x08
#define BQ_TS_FAULT_MASK        0x06  // read only
#define BQ_HALF_CURRENT_BIT     0x01  // LOW_CHG


// Field ranges:
#define BQ_CHRG_VOLTAGE_MAX     47    // 4.44 V, the top of the specified range
#define BQ_CHRG_CURRENT_MAX     26    // 550 + 26 * 75 = 2500 mA, the device maximum
#define BQ_TERM_CURRENT_MAX     0x07  // three bits wide

#pragma pack(push, 1)

// Register 0:
typedef union
{
  uint8_t raw;
  struct
  {
    unsigned fault      : 3;  // 2:0  FAULT_x
    unsigned            : 1;  // 3    SUPPLY_SEL, write-only meaning
    unsigned status     : 3;  // 6:4  STAT_x
    unsigned            : 1;  // 7    TMR_RST on write, always reads 0
  } rd;
  struct
  {
    unsigned            : 3;
    unsigned supply_sel : 1;  // 3    0 - IN, 1 - USB
    unsigned            : 3;
    unsigned tmr_rst    : 1;  // 7    write 1 to kick the watchdog, always reads 0
  } wr;
} BQ24160Reg_StatusControl_t;

// Register 1:
typedef union
{
  uint8_t raw;
  struct
  {
    unsigned            : 1;
    unsigned bat_stat   : 2;  // 2:1
    unsigned            : 1;  // 3
    unsigned usb_status : 2;  // 5:4
    unsigned in_status  : 2;  // 7:6
  } rd;
  struct
  {
    unsigned en_nobatop : 1;  // 0
    unsigned            : 2;  // 2:1
    unsigned otg_lock   : 1;  // 3
    unsigned            : 2;  // 5:4
    unsigned            : 2;  // 7:6
  } rd_wr;
} BQ24160Reg_SupplyStatus_t;

// Register 2:
typedef union
{
  uint8_t raw;
  struct
  {
    unsigned hz_mode    : 1;  // 0
    unsigned ce         : 1;  // 1    1 disables charging
    unsigned te         : 1;  // 2
    unsigned en_stat    : 1;  // 3
    unsigned iusb_limit : 3;  // 6:4
    unsigned            : 1;
  } rd_wr;
  struct
  {
    unsigned            : 7;  // 6:0
    unsigned reset      : 1;  // 7    write 1 to RESET all regs, always reads 1
  } wr;
} BQ24160Reg_Control_t;

// Register 3:
typedef union
{
  uint8_t raw;
  struct
  {
    unsigned dpdm_en    : 1;  // 0    forces D+/D- detection, self-clearing
    unsigned iinlimit   : 1;  // 1    0 = 1.5 A, 1 = 2.5 A
    unsigned vbreg      : 6;  // 7:2  3.5 V offset, 20 mV per step
  } rd_wr;
} BQ24160Reg_BatVoltage_t;

/* Register 4, read only - vendor, part number, revision. No field this module acts on. */
typedef union
{
  uint8_t raw;
  struct
  {
    unsigned revision   : 3;  // 2:0
    unsigned part       : 2;  // 4:3
    unsigned vendor     : 3;  // 7:5
  } rd;
} BQ24160Reg_Vendor_t;

// Register 5:
typedef union
{
  uint8_t raw;
  struct
  {
    unsigned iterm      : 3;  // 2:0  50 mA offset, 50 mA per LSB
    unsigned ichrg      : 5;  // 7:3  550 mA offset, 75 mA per LSB
  } rd_wr;
} BQ24160Reg_ChargeCurrent_t;

// Register 6:
typedef union
{
  uint8_t raw;
  struct
  {
    unsigned            : 3;  // 2:0
    unsigned            : 3;  // 5:3
    unsigned dpm_status : 1;  // 6
    unsigned minsys     : 1;  // 7
  } rd;
  struct
  {
    unsigned vindpm_in  : 3;  // 2:0  4.20 V offset, 80 mV per LSB
    unsigned vindpm_usb : 3;  // 5:3  same scale, USB input
    unsigned            : 1;  // 6
    unsigned            : 1;  // 7
  } rd_wr;
} BQ24160Reg_VinDpm_t;

// Register 7:
typedef union
{
  uint8_t raw;
  struct
  {
    unsigned            : 1;  // 0
    unsigned ts_fault   : 2;  // 2:1
    unsigned            : 1;  // 3
    unsigned            : 1;  // 4    NA
    unsigned            : 2;  // 6:5
    unsigned            : 1;  // 7
  } rd;
  struct
  {
    unsigned low_chg    : 1;  // 0    halve the programmed charge current
    unsigned            : 2;  // 2:1
    unsigned ts_en      : 1;  // 3
    unsigned            : 1;  // 4    NA
    unsigned tmr        : 2;  // 6:5  safety timer limit
    unsigned tmr2x_en   : 1;  // 7
  } rd_wr;
} BQ24160Reg_SafetyNtc_t;

typedef union
{
  uint8_t raw[BQ_REG_COUNT];
  struct
  {
    BQ24160Reg_StatusControl_t status_control;
    BQ24160Reg_SupplyStatus_t  supply_status;
    BQ24160Reg_Control_t       control;
    BQ24160Reg_BatVoltage_t    bat_voltage;
    BQ24160Reg_Vendor_t        vendor;
    BQ24160Reg_ChargeCurrent_t charge_current;
    BQ24160Reg_VinDpm_t        vin_dpm;
    BQ24160Reg_SafetyNtc_t     safety_ntc;
  } reg;
} BQ24160Regs_t;

#pragma pack(pop)

_Static_assert(sizeof(BQ24160Reg_StatusControl_t) == 1, "A register must be one byte");
_Static_assert(sizeof(BQ24160Regs_t) == BQ_REG_COUNT, "The map must cover exactly the register file");

#endif /* COMPONENTS_POWER_BQ24160_REGS_MAP_H_ */
