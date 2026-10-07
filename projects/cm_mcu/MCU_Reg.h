#ifndef MCU_REG_H
#define MCU_REG_H

#include <stddef.h>
#include <stdint.h>

enum mcu_reg_result {
  MCU_REG_OK,
  MCU_REG_INVALID_DEVICE,
  MCU_REG_INVALID_PAGE,
  MCU_REG_INVALID_ADDRESS,
  MCU_REG_INVALID_LENGTH,
  MCU_REG_READ_ONLY,
  MCU_REG_WRITE_ONLY,
  MCU_REG_BUSY,
  MCU_REG_QUEUE_FULL,
  MCU_REG_INVALID_COMMAND, // valid address/length, unrecognized command payload
  MCU_REG_INVALID_WRITE_LENGTH, // write length != the target field's width
  MCU_REG_INVALID_VALUE,        // valid address/length, value outside the clamp
  MCU_REG_INTERNAL_ERROR,
};

enum mcu_reg_result mcu_reg_read(
    uint8_t page, uint8_t address,
    uint8_t length, uint8_t out[4]);

enum mcu_reg_result mcu_reg_write(
    uint8_t page, uint8_t address,
    const uint8_t *data, size_t length);

// Called once from InitTask.c with the raw (untruncated) value from
// ROM_SysCtlResetCauseGet(), before that value is truncated for the
// persistent error-log entry and the hardware register is cleared. Publishes
// the full 32-bit cause for page 0x00; does not change existing error-log
// behavior.
void mcu_reg_set_reset_cause(uint32_t raw_reset_cause);

// ---- Page 0x00 (System) wire layout. See MCU_REGISTER_MAP.md for the full,
// authoritative register list, including pages not implemented yet.

#define MCU_REG_PAGE_SYSTEM 0x00

#define MCU_MAP_MAJOR 1
#define MCU_MAP_MINOR 3

#define SYS_OFF_MAGIC        0x00 // 4  ASCII magic "CMCU"
#define SYS_OFF_MAP_MAJOR    0x04 // 1
#define SYS_OFF_MAP_MINOR    0x05 // 1
#define SYS_OFF_HW_REV       0x06 // 1
#define SYS_OFF_ADC_COUNT    0x07 // 1
#define SYS_OFF_CAPABILITIES 0x08 // 4
#define SYS_OFF_HEALTH       0x0c // 4
#define SYS_OFF_BOARD_ID     0x10 // 4
#define SYS_OFF_UPTIME_S     0x14 // 4
#define SYS_OFF_RESET_CAUSE  0x18 // 4
// 0x1c-0x2b (16 bytes) reserved; 0x2c-0x3f undeclared hole.
#define SYS_OFF_GIT_VERSION 0x40 // 20, NUL-padded
#define SYS_GIT_VERSION_LEN 20
// 0x54-0x77 added at map minor 1. 0x5d-0x5f is an undeclared hole.
#define SYS_OFF_FF_USER_MASK    0x54 // 4  ff_USER_mask, bit n = Firefly index n
#define SYS_OFF_FF_PRESENT_MASK 0x58 // 4  ff_PRESENT_mask
#define SYS_OFF_BUILD_TYPE      0x5c // 1  0 = release, 1 = DEBUG
#define SYS_OFF_BUILD_TIME      0x60 // 24, NUL-padded, buildTime()
#define SYS_BUILD_TIME_LEN      24
#define SYS_PAGE_USED_LEN       (SYS_OFF_BUILD_TIME + SYS_BUILD_TIME_LEN)

#define MCU_CAP_SYSTEM (1U << 0)
#define MCU_CAP_POWER  (1U << 1)
#define MCU_CAP_ALARMS (1U << 2)
#define MCU_CAP_ADC    (1U << 3)
// bit 4 reserved -- formerly ADC_TARGETS (page 0x04), dropped from the
// design entirely. Not reassigned.
#define MCU_CAP_PERSISTENT_LOG (1U << 5)
#define MCU_CAP_CONTROLS       (1U << 6)
#define MCU_CAP_CONFIG  (1U << 7) // page 0x05 readable
#define MCU_CAP_RUNTIME (1U << 8)
#define MCU_CAP_CONFIG_WRITE (1U << 9) // page 0x05 writable (Phase 2b, map minor 3)

#define MCU_HEALTH_POWER_FAULT       (1U << 0)
#define MCU_HEALTH_TEMPERATURE_ALARM (1U << 1)
#define MCU_HEALTH_VOLTAGE_ALARM     (1U << 2)
#define MCU_HEALTH_ADC_ERROR         (1U << 3)

// ---- Page 0x05 (Config) wire layout. Read-only at map minor 2, writable from 3. Five
// independent 16-bit policy values, each a naturally-aligned halfword (a single
// atomic access on Cortex-M4), so there is no generation counter.

#define MCU_REG_PAGE_CONFIG 0x05

// Temperature thresholds, int16_t degrees C. Offset = CFG_OFF_ALARM_TEMP_FF +
// 2 * (enum device): FF, DCDC, TM4C, FPGA, the same order as Tasks.h.
#define CFG_LEN_ALARM_TEMP       2
#define CFG_OFF_ALARM_TEMP_FF    0x00
#define CFG_OFF_ALARM_TEMP_DCDC  0x02
#define CFG_OFF_ALARM_TEMP_TM4C  0x04
#define CFG_OFF_ALARM_TEMP_FPGA  0x06
// Voltage-alarm threshold, uint16_t centi-percent (500 = 5 %).
#define CFG_LEN_ALARM_VOLT       2
#define CFG_OFF_ALARM_VOLT_CPCT  0x08
#define CFG_PAGE_USED_LEN        0x0a

// Write clamps (reads are unclamped: EEPROM can hold older CLI-set values).
// Raising a temperature threshold persistently reduces thermal protection;
// lowering it can force a power-down that no inhibit bit reflects. The console
// keeps its wider ranges on purpose.
#define CFG_TEMP_MIN_C         50
#define CFG_TEMP_MAX_C         100
#define CFG_VOLT_MIN_CPCT      100  // 1 %
#define CFG_VOLT_MAX_CPCT      1000 // 10 %

// ---- Page 0x06 (Runtime) wire layout. Read-only, no generation counter: every
// field is a single atomic word/byte, except the RTC, which is two words that
// a client reads time -> date -> time and retries on a second rollover.

#define MCU_REG_PAGE_RUNTIME 0x06

#define RT_OFF_HEAP_FREE               0x00 // 4  bytes
#define RT_OFF_HEAP_MIN_FREE           0x04 // 4  bytes, minimum ever free
#define RT_OFF_HEAP_TOTAL              0x08 // 4  bytes
#define RT_OFF_SYSSTACK_UNTOUCHED_WORDS 0x0c // 4  words; falling = worse
#define RT_OFF_SYSSTACK_TOTAL_WORDS    0x10 // 4  words
#define RT_OFF_ZYNQMON_TX_ENABLED      0x14 // 1  0/1
#define RT_OFF_FPGA_DONE               0x15 // 1  raw pin level, bit0 = F1, bit1 = F2
// 0x16-0x17 undeclared hole (alignment before the RTC pair)
#define RT_OFF_RTC_DATE 0x18 // 4  (year << 16) | (month << 8) | day; 0 when invalid
#define RT_OFF_RTC_TIME 0x1c // 4  (valid << 24) | (hour << 16) | (min << 8) | sec
#define RT_PAGE_USED_LEN 0x20

// ---- Page 0x03 (ADC sample) wire layout.

#define MCU_REG_PAGE_ADC 0x03

#define ADC_PAGE_VALUES_OFF 0x00
#define ADC_PAGE_VALUES_LEN (ADC_CHANNEL_COUNT * 2) // 42

// ---- Page 0x02 (Alarm) wire layout. No generation counter: each field is
// either a single atomic word/byte write from one task, or a group of
// bytes always written together within one function call by one task -- see
// MCU_FIRMWARE_IMPLEMENTATION_PLAN.md for the full reasoning.

#define MCU_REG_PAGE_ALARM 0x02

#define ALM_OFF_TEMP_STATE 0x00 // 1  temperature-alarm task FSM state (0-4)
#define ALM_OFF_VOLT_STATE 0x01 // 1  voltage-alarm task FSM state (0-4)
#define ALM_OFF_STATUS_T   0x04 // 4  status_T bitmap
#define ALM_OFF_WARN_LATCH 0x08 // 4  warnLatch bitmap
#define ALM_OFF_VOLT_GEN   0x0c // 1  currentVoltStatus[GEN]
#define ALM_OFF_VOLT_FPGA1 0x0d // 1  currentVoltStatus[FPGA1]
#define ALM_OFF_VOLT_FPGA2 0x0e // 1  currentVoltStatus[FPGA2]
#define ALM_PAGE_USED_LEN  0x0f

// ---- Page 0x01 (Power) wire layout. Keeps a generation counter, unlike
// pages 0x02/0x03 -- see MCU_FIRMWARE_IMPLEMENTATION_PLAN.md for why these
// fields are causally correlated and need one.

#define MCU_REG_PAGE_POWER 0x01

#define PWR_OFF_GENERATION    0x00 // 4
#define PWR_OFF_STATE         0x04 // 1  power_system_state (0-10)
#define PWR_OFF_FLAGS         0x05 // 1  see PWR_FLAG_* below
#define PWR_OFF_LIVE_MASK     0x08 // 4  supply_bitset, freshest reading
#define PWR_OFF_EXPECTED_MASK 0x0c // 4  supply_ok_mask: full/final target,
                                   //    not a per-sequencing-level submask
#define PWR_OFF_IGNORE_MASK   0x10 // 4
#define PWR_OFF_FAILED_MASK   0x14 // 4  latched
#define PWR_OFF_SUPPLY_COUNT  0x18 // 1  N_PS_OKS (12 on REV2/REV3)
#define PWR_OFF_SUPPLY_STATES 0x1c // 12, one byte each, enum ps_state
#define PWR_PAGE_USED_LEN     (PWR_OFF_SUPPLY_STATES + 12)

#define PWR_FLAG_BLADE_POWER_EN       (1U << 0)
#define PWR_FLAG_CLI_INHIBIT          (1U << 1)
#define PWR_FLAG_PROGCOM_INHIBIT      (1U << 2) // ASSERT/RELEASE_PROGCOM_POWER_INHIBIT, page 0x7f
#define PWR_FLAG_FAULT_LATCH          (1U << 3)
#define PWR_FLAG_ALARM_SHUTDOWN_LATCH (1U << 4)
#define PWR_FLAG_F1_ENABLE            (1U << 5)
#define PWR_FLAG_F2_ENABLE            (1U << 6)

// ---- Page 0x7f (Control) wire layout. Write-only: one command byte at
// offset 0x00. See MCU_REGISTER_MAP.md for the safety-review context.

#define MCU_REG_PAGE_CONTROL 0x7f
#define CTRL_OFF_COMMAND     0x00 // 1

#define CTRL_CMD_ASSERT_PROGCOM_POWER_INHIBIT  1
#define CTRL_CMD_RELEASE_PROGCOM_POWER_INHIBIT 2
#define CTRL_CMD_CLEAR_POWER_FAULT             3
#define CTRL_CMD_CLEAR_ALARM_LATCHES           4
// Sticky: nothing re-enables ZynqMon transmit except the CLI ("zmon enable")
// or a reboot.
#define CTRL_CMD_ZYNQMON_ENABLE_TRANSMIT  5
#define CTRL_CMD_ZYNQMON_DISABLE_TRANSMIT 6

#endif // MCU_REG_H
