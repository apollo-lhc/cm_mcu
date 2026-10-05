#include "MCU_Reg.h"

#include <assert.h>
#include <stdio.h>
#include <string.h>

#include "FreeRTOS.h"
#include "task.h"

#include <time.h>

#include "AlarmUtilities.h"
#include "FireflyUtils.h"
#include "Tasks.h"
#include "ZynqMonTask.h"
#include "common/pinsel.h"
#include "common/power_ctl.h"
#include "driverlib/hibernate.h"
#include "driverlib/rom.h"
#include "inc/hw_hibernate.h"
#include "inc/hw_types.h"

// Page 0x01's per-supply state array assumes REV2/REV3's supply count; this
// module isn't built for REV1 (no ProgCom UART support there).
static_assert(N_PS_OKS == 12, "MC page 0x01 wire width assumes N_PS_OKS == 12");

// Cached full-width reset cause, set once from InitTask.c before the
// hardware register is cleared. See mcu_reg_set_reset_cause().
static uint32_t cached_reset_cause = 0U;

void mcu_reg_set_reset_cause(uint32_t raw_reset_cause)
{
  cached_reset_cause = raw_reset_cause;
}

static void put_u16_le(uint8_t *dst, uint16_t value)
{
  dst[0] = (uint8_t)(value >> 0);
  dst[1] = (uint8_t)(value >> 8);
}

static void put_u32_le(uint8_t *dst, uint32_t value)
{
  dst[0] = (uint8_t)(value >> 0);
  dst[1] = (uint8_t)(value >> 8);
  dst[2] = (uint8_t)(value >> 16);
  dst[3] = (uint8_t)(value >> 24);
}

// Field spans for page 0x00 (System), used to validate that a requested
// [address, address+length) run lies within a single declared field and
// does not cross into a hole or a neighboring field. Keep in sync with the
// offsets in MCU_Reg.h and with MCU_REGISTER_MAP.md.
struct field_span {
  uint8_t offset;
  uint8_t size;
};

static const struct field_span SYSTEM_FIELDS[] = {
    {SYS_OFF_MAGIC, 4},
    {SYS_OFF_MAP_MAJOR, 1},
    {SYS_OFF_MAP_MINOR, 1},
    {SYS_OFF_HW_REV, 1},
    {SYS_OFF_ADC_COUNT, 1},
    {SYS_OFF_CAPABILITIES, 4},
    {SYS_OFF_HEALTH, 4},
    {SYS_OFF_BOARD_ID, 4},
    {SYS_OFF_UPTIME_S, 4},
    {SYS_OFF_RESET_CAUSE, 4},
    {SYS_OFF_GIT_VERSION, SYS_GIT_VERSION_LEN},
    {SYS_OFF_FF_USER_MASK, 4},
    {SYS_OFF_FF_PRESENT_MASK, 4},
    {SYS_OFF_BUILD_TYPE, 1},
    {SYS_OFF_BUILD_TIME, SYS_BUILD_TIME_LEN},
};

static_assert(sizeof(__TIME__ ", " __DATE__) <= SYS_BUILD_TIME_LEN,
              "build-time string does not fit SYS_OFF_BUILD_TIME");

static enum mcu_reg_result validate_span(const struct field_span *fields, size_t nfields,
                                         uint8_t address, uint8_t length)
{
  for (size_t i = 0; i < nfields; ++i) {
    uint8_t start = fields[i].offset;
    uint16_t end = (uint16_t)start + fields[i].size; // exclusive
    if (address >= start && address < end) {
      if ((uint16_t)address + length > end)
        return MCU_REG_INVALID_LENGTH;
      return MCU_REG_OK;
    }
  }
  return MCU_REG_INVALID_ADDRESS;
}

static void build_system_page(uint8_t buf[SYS_PAGE_USED_LEN])
{
  memset(buf, 0, SYS_PAGE_USED_LEN);

  buf[SYS_OFF_MAGIC + 0] = 'C';
  buf[SYS_OFF_MAGIC + 1] = 'M';
  buf[SYS_OFF_MAGIC + 2] = 'C';
  buf[SYS_OFF_MAGIC + 3] = 'U';

  buf[SYS_OFF_MAP_MAJOR] = MCU_MAP_MAJOR;
  buf[SYS_OFF_MAP_MINOR] = MCU_MAP_MINOR;

  uint32_t rev, id;
  get_board_info(&rev, &id);
  buf[SYS_OFF_HW_REV] = (uint8_t)rev;
  buf[SYS_OFF_ADC_COUNT] = ADC_CHANNEL_COUNT;

  // Only SYSTEM, POWER, ALARMS, ADC, CONTROLS, CONFIG (read-only), and RUNTIME
  // are implemented so far; the rest stay clear until their page is actually
  // implemented, per the register-map design rule.
  put_u32_le(buf + SYS_OFF_CAPABILITIES, MCU_CAP_SYSTEM | MCU_CAP_POWER | MCU_CAP_ALARMS |
                                             MCU_CAP_ADC | MCU_CAP_CONTROLS | MCU_CAP_CONFIG |
                                             MCU_CAP_RUNTIME);

  uint32_t health = 0U;
  if (getPowerControlState() == POWER_FAILURE)
    health |= MCU_HEALTH_POWER_FAULT;
  if (getTempAlarmStatus() != 0U)
    health |= MCU_HEALTH_TEMPERATURE_ALARM;
  if (getVoltAlarmStatus() != 0U)
    health |= MCU_HEALTH_VOLTAGE_ALARM;
  // MCU_HEALTH_ADC_ERROR: no source published yet; left clear rather than
  // fabricated.
  put_u32_le(buf + SYS_OFF_HEALTH, health);

  put_u32_le(buf + SYS_OFF_BOARD_ID, id);
  put_u32_le(buf + SYS_OFF_UPTIME_S, (uint32_t)pdTICKS_TO_S(xTaskGetTickCount()));
  put_u32_le(buf + SYS_OFF_RESET_CAUSE, cached_reset_cause);

  // GIT_DESCRIBE is `git describe --dirty --always --tags` only (Makefile),
  // not the full gitVersion()/FIRMWARE_VERSION string. snprintf truncates
  // and NUL-terminates; the memset above already zero-padded the rest.
  snprintf((char *)(buf + SYS_OFF_GIT_VERSION), SYS_GIT_VERSION_LEN, "%s", GIT_DESCRIBE);

  put_u32_le(buf + SYS_OFF_FF_USER_MASK, ff_USER_mask);
  put_u32_le(buf + SYS_OFF_FF_PRESENT_MASK, ff_PRESENT_mask);
#ifdef DEBUG
  buf[SYS_OFF_BUILD_TYPE] = 1U;
#else
  buf[SYS_OFF_BUILD_TYPE] = 0U;
#endif
  snprintf((char *)(buf + SYS_OFF_BUILD_TIME), SYS_BUILD_TIME_LEN, "%s", buildTime());
}

static enum mcu_reg_result mcu_local_read0(uint8_t address, uint8_t length, uint8_t out[4])
{
  enum mcu_reg_result r = validate_span(SYSTEM_FIELDS,
                                        sizeof(SYSTEM_FIELDS) / sizeof(SYSTEM_FIELDS[0]),
                                        address, length);
  if (r != MCU_REG_OK)
    return r;

  uint8_t buf[SYS_PAGE_USED_LEN];
  build_system_page(buf);
  memcpy(out, buf + address, length);
  return MCU_REG_OK;
}

// Page 0x03 (ADC sample): one field spanning the whole 42-byte channel
// array. Channels are independent and uncorrelated, and each 2-byte read is
// a single atomic access, so (unlike page 0x00) there is no per-channel
// boundary to enforce beyond the field's own span -- see
// MCU_FIRMWARE_IMPLEMENTATION_PLAN.md's "Coherent publication" section.
static const struct field_span ADC_FIELDS[] = {
    {ADC_PAGE_VALUES_OFF, ADC_PAGE_VALUES_LEN},
};

static void build_adc_page(uint8_t buf[ADC_PAGE_VALUES_LEN])
{
  for (int i = 0; i < ADC_CHANNEL_COUNT; ++i) {
    __fp16 v = (__fp16)getADCvalue(i);
    memcpy(buf + i * 2, &v, sizeof v);
  }
}

static enum mcu_reg_result mcu_local_read3(uint8_t address, uint8_t length, uint8_t out[4])
{
  enum mcu_reg_result r = validate_span(ADC_FIELDS,
                                        sizeof(ADC_FIELDS) / sizeof(ADC_FIELDS[0]),
                                        address, length);
  if (r != MCU_REG_OK)
    return r;

  uint8_t buf[ADC_PAGE_VALUES_LEN];
  build_adc_page(buf);
  memcpy(out, buf + address, length);
  return MCU_REG_OK;
}

// Field spans for page 0x05 (Config). The widths are #defines in MCU_Reg.h so
// the client contract test can read them without parsing this initializer.
static const struct field_span CONFIG_FIELDS[] = {
    {CFG_OFF_ALARM_TEMP_FF, CFG_LEN_ALARM_TEMP},
    {CFG_OFF_ALARM_TEMP_DCDC, CFG_LEN_ALARM_TEMP},
    {CFG_OFF_ALARM_TEMP_TM4C, CFG_LEN_ALARM_TEMP},
    {CFG_OFF_ALARM_TEMP_FPGA, CFG_LEN_ALARM_TEMP},
    {CFG_OFF_ALARM_VOLT_CPCT, CFG_LEN_ALARM_VOLT},
};

// The four temperature offsets are derived from enum device, so its order must
// stay FF, DCDC, TM4C, FPGA (the same order the CLI and the EEPROM table use).
static_assert(FF == 0 && DCDC == 1 && TM4C == 2 && FPGA == 3,
              "page 0x05 temperature offsets assume enum device order FF, DCDC, TM4C, FPGA");
static_assert(CFG_OFF_ALARM_TEMP_DCDC == CFG_OFF_ALARM_TEMP_FF + CFG_LEN_ALARM_TEMP &&
                  CFG_OFF_ALARM_TEMP_TM4C == CFG_OFF_ALARM_TEMP_DCDC + CFG_LEN_ALARM_TEMP &&
                  CFG_OFF_ALARM_TEMP_FPGA == CFG_OFF_ALARM_TEMP_TM4C + CFG_LEN_ALARM_TEMP,
              "page 0x05 temperature fields must be contiguous");

static void build_config_page(uint8_t buf[CFG_PAGE_USED_LEN])
{
  memset(buf, 0, CFG_PAGE_USED_LEN);
  // int16_t -> uint16_t is a bit-pattern conversion, so negative thresholds
  // (older CLI-set EEPROM content) read back in two's complement.
  for (int dev = FF; dev <= FPGA; ++dev)
    put_u16_le(buf + CFG_OFF_ALARM_TEMP_FF + CFG_LEN_ALARM_TEMP * dev,
               (uint16_t)getAlarmTemperature((enum device)dev));

  // Clamp before the cast: converting an out-of-range float to an integer is
  // undefined, and nothing structurally keeps alarmVolt inside the 0.01-0.50
  // band the CLI enforces. `!(v >= 0)` also catches NaN. Rounded by hand to
  // avoid pulling in libm (lrintf is not otherwise used in this firmware).
  float v = getAlarmVoltageThres();
  if (!(v >= 0.f))
    v = 0.f;
  else if (v > 6.5535f)
    v = 6.5535f;
  put_u16_le(buf + CFG_OFF_ALARM_VOLT_CPCT, (uint16_t)(v * 10000.f + 0.5f));
}

static enum mcu_reg_result mcu_local_read5(uint8_t address, uint8_t length, uint8_t out[4])
{
  enum mcu_reg_result r = validate_span(CONFIG_FIELDS,
                                        sizeof(CONFIG_FIELDS) / sizeof(CONFIG_FIELDS[0]),
                                        address, length);
  if (r != MCU_REG_OK)
    return r;

  uint8_t buf[CFG_PAGE_USED_LEN];
  build_config_page(buf);
  memcpy(out, buf + address, length);
  return MCU_REG_OK;
}

// Field spans for page 0x06 (Runtime). 0x16-0x17 is an undeclared hole.
static const struct field_span RUNTIME_FIELDS[] = {
    {RT_OFF_HEAP_FREE, 4},
    {RT_OFF_HEAP_MIN_FREE, 4},
    {RT_OFF_HEAP_TOTAL, 4},
    {RT_OFF_SYSSTACK_UNTOUCHED_WORDS, 4},
    {RT_OFF_SYSSTACK_TOTAL_WORDS, 4},
    {RT_OFF_ZYNQMON_TX_ENABLED, 1},
    {RT_OFF_FPGA_DONE, 1},
    {RT_OFF_RTC_DATE, 4},
    {RT_OFF_RTC_TIME, 4},
};

static void build_runtime_page(uint8_t buf[RT_PAGE_USED_LEN])
{
  memset(buf, 0, RT_PAGE_USED_LEN);
  put_u32_le(buf + RT_OFF_HEAP_FREE, (uint32_t)xPortGetFreeHeapSize());
  put_u32_le(buf + RT_OFF_HEAP_MIN_FREE, (uint32_t)xPortGetMinimumEverFreeHeapSize());
  put_u32_le(buf + RT_OFF_HEAP_TOTAL, (uint32_t)configTOTAL_HEAP_SIZE);
  put_u32_le(buf + RT_OFF_SYSSTACK_UNTOUCHED_WORDS, (uint32_t)SystemStackWaterHighWaterMark());
  put_u32_le(buf + RT_OFF_SYSSTACK_TOTAL_WORDS, (uint32_t)SYSTEM_STACK_SIZE);
  buf[RT_OFF_ZYNQMON_TX_ENABLED] = getZynqMonTransmitEnabled() ? 1U : 0U;
  // Raw pin levels: no bus, no semaphore, no write path.
  buf[RT_OFF_FPGA_DONE] = (uint8_t)((read_gpio_pin(_F1_FPGA_DONE) ? 1U : 0U) |
                                    (read_gpio_pin(_F2_FPGA_DONE) ? 2U : 0U));

  // Validity is the hardware's own latch (as the `rtc` CLI command uses), not
  // log.c's "tm_year < 120" plausibility test; the two can disagree. When
  // invalid, both words stay zero so valid == 0.
  uint32_t cal1 = HWREG(HIB_CAL1);
  if (cal1 & HIB_CAL1_VALID) {
    struct tm now;
    ROM_HibernateCalendarGet(&now);
    put_u32_le(buf + RT_OFF_RTC_DATE, ((uint32_t)(now.tm_year + 1900) << 16) |
                                          ((uint32_t)(now.tm_mon + 1) << 8) |
                                          (uint32_t)now.tm_mday);
    put_u32_le(buf + RT_OFF_RTC_TIME, (1U << 24) | ((uint32_t)now.tm_hour << 16) |
                                          ((uint32_t)now.tm_min << 8) | (uint32_t)now.tm_sec);
  }
}

static enum mcu_reg_result mcu_local_read6(uint8_t address, uint8_t length, uint8_t out[4])
{
  enum mcu_reg_result r = validate_span(RUNTIME_FIELDS,
                                        sizeof(RUNTIME_FIELDS) / sizeof(RUNTIME_FIELDS[0]),
                                        address, length);
  if (r != MCU_REG_OK)
    return r;

  uint8_t buf[RT_PAGE_USED_LEN];
  build_runtime_page(buf);
  memcpy(out, buf + address, length);
  return MCU_REG_OK;
}

// Field spans for page 0x02 (Alarm). Unlike page 0x03, fields are not
// uniform width and are not contiguous, so a full span table is needed --
// same shape as page 0x00's SYSTEM_FIELDS.
static const struct field_span ALARM_FIELDS[] = {
    {ALM_OFF_TEMP_STATE, 1},
    {ALM_OFF_VOLT_STATE, 1},
    {ALM_OFF_STATUS_T, 4},
    {ALM_OFF_WARN_LATCH, 4},
    {ALM_OFF_VOLT_GEN, 1},
    {ALM_OFF_VOLT_FPGA1, 1},
    {ALM_OFF_VOLT_FPGA2, 1},
};

static void build_alarm_page(uint8_t buf[ALM_PAGE_USED_LEN])
{
  memset(buf, 0, ALM_PAGE_USED_LEN);

  buf[ALM_OFF_TEMP_STATE] = (uint8_t)getTempAlarmTaskState();
  buf[ALM_OFF_VOLT_STATE] = (uint8_t)getVoltAlarmTaskState();
  put_u32_le(buf + ALM_OFF_STATUS_T, getTempAlarmStatus());
  put_u32_le(buf + ALM_OFF_WARN_LATCH, getWarnLatch());
  buf[ALM_OFF_VOLT_GEN] = getVoltStatusGroup(GEN);
  buf[ALM_OFF_VOLT_FPGA1] = getVoltStatusGroup(FPGA1);
  buf[ALM_OFF_VOLT_FPGA2] = getVoltStatusGroup(FPGA2);
}

static enum mcu_reg_result mcu_local_read2(uint8_t address, uint8_t length, uint8_t out[4])
{
  enum mcu_reg_result r = validate_span(ALARM_FIELDS,
                                        sizeof(ALARM_FIELDS) / sizeof(ALARM_FIELDS[0]),
                                        address, length);
  if (r != MCU_REG_OK)
    return r;

  uint8_t buf[ALM_PAGE_USED_LEN];
  build_alarm_page(buf);
  memcpy(out, buf + address, length);
  return MCU_REG_OK;
}

// Field spans for page 0x01 (Power).
static const struct field_span POWER_FIELDS[] = {
    {PWR_OFF_GENERATION, 4},
    {PWR_OFF_STATE, 1},
    {PWR_OFF_FLAGS, 1},
    {PWR_OFF_LIVE_MASK, 4},
    {PWR_OFF_EXPECTED_MASK, 4},
    {PWR_OFF_IGNORE_MASK, 4},
    {PWR_OFF_FAILED_MASK, 4},
    {PWR_OFF_SUPPLY_COUNT, 1},
    {PWR_OFF_SUPPLY_STATES, 12},
};

static void build_power_page(uint8_t buf[PWR_PAGE_USED_LEN])
{
  memset(buf, 0, PWR_PAGE_USED_LEN);
  const struct power_snapshot_t *snap = getPowerSnapshot();

  put_u32_le(buf + PWR_OFF_GENERATION, snap->generation);
  buf[PWR_OFF_STATE] = snap->fsm_state;

  uint8_t flags = 0;
  if (snap->blade_power_en)
    flags |= PWR_FLAG_BLADE_POWER_EN;
  if (snap->cli_inhibit)
    flags |= PWR_FLAG_CLI_INHIBIT;
  if (snap->progcom_inhibit)
    flags |= PWR_FLAG_PROGCOM_INHIBIT;
  if (snap->fault_latch)
    flags |= PWR_FLAG_FAULT_LATCH;
  if (snap->alarm_shutdown_latch)
    flags |= PWR_FLAG_ALARM_SHUTDOWN_LATCH;
  if (snap->f1_enable)
    flags |= PWR_FLAG_F1_ENABLE;
  if (snap->f2_enable)
    flags |= PWR_FLAG_F2_ENABLE;
  buf[PWR_OFF_FLAGS] = flags;

  put_u32_le(buf + PWR_OFF_LIVE_MASK, snap->live_mask);
  put_u32_le(buf + PWR_OFF_EXPECTED_MASK, snap->expected_mask);
  put_u32_le(buf + PWR_OFF_IGNORE_MASK, snap->software_ignore_mask);
  put_u32_le(buf + PWR_OFF_FAILED_MASK, snap->failed_mask);
  buf[PWR_OFF_SUPPLY_COUNT] = N_PS_OKS;
  for (int i = 0; i < N_PS_OKS; ++i)
    buf[PWR_OFF_SUPPLY_STATES + i] = snap->supply_states[i];
}

static enum mcu_reg_result mcu_local_read1(uint8_t address, uint8_t length, uint8_t out[4])
{
  enum mcu_reg_result r = validate_span(POWER_FIELDS,
                                        sizeof(POWER_FIELDS) / sizeof(POWER_FIELDS[0]),
                                        address, length);
  if (r != MCU_REG_OK)
    return r;

  uint8_t buf[PWR_PAGE_USED_LEN];
  build_power_page(buf);
  memcpy(out, buf + address, length);
  return MCU_REG_OK;
}

// Page 0x7f (Control): write-only, one command byte. Commands 3/4 reuse
// existing internal messages already reachable via the unauthenticated CLI
// (power_ctl clearfail / alarm_ctl clear); commands 1/2 are the only new
// input to PowerSupplyTask's FSM added for this page (progcom_inhibit_request
// in PowerSupplyTask.c). All sends are non-blocking: a full queue is real
// backpressure and should surface to the remote client immediately.
static enum mcu_reg_result mcu_local_write7f(uint8_t address, const uint8_t *data, size_t length)
{
  if (address != CTRL_OFF_COMMAND)
    return MCU_REG_INVALID_ADDRESS;
  if (length != 1)
    return MCU_REG_INVALID_LENGTH;

  if (data[0] == CTRL_CMD_CLEAR_ALARM_LATCHES) {
    // matches alarm_ctl clear exactly: same two independent sends, same
    // accepted partial-failure risk if one queue is full and the other isn't
    uint32_t msg = ALM_CLEAR_ALL;
    BaseType_t ok1 = xQueueSendToBack(tempAlarmTask.xAlmQueue, &msg, 0);
    BaseType_t ok2 = xQueueSendToBack(voltAlarmTask.xAlmQueue, &msg, 0);
    return (ok1 == pdPASS && ok2 == pdPASS) ? MCU_REG_OK : MCU_REG_QUEUE_FULL;
  }

  if (data[0] == CTRL_CMD_ZYNQMON_ENABLE_TRANSMIT ||
      data[0] == CTRL_CMD_ZYNQMON_DISABLE_TRANSMIT) {
    // ZynqMonTask has its own queue, so these cannot join the xPwrQueue switch
    // below. The CLI sends the same messages with a 10 ms timeout; ProgCom
    // uses 0 ticks per this page's non-blocking rule.
    uint32_t zmsg = (data[0] == CTRL_CMD_ZYNQMON_ENABLE_TRANSMIT) ? ZYNQMON_ENABLE_TRANSMIT
                                                                   : ZYNQMON_DISABLE_TRANSMIT;
    return (xQueueSendToBack(xZynqMonQueue, &zmsg, 0) == pdPASS) ? MCU_REG_OK : MCU_REG_QUEUE_FULL;
  }

  uint32_t msg;
  switch (data[0]) {
    case CTRL_CMD_ASSERT_PROGCOM_POWER_INHIBIT:
      msg = PS_PROGCOM_OFF;
      break;
    case CTRL_CMD_RELEASE_PROGCOM_POWER_INHIBIT:
      msg = PS_PROGCOM_ON;
      break;
    case CTRL_CMD_CLEAR_POWER_FAULT:
      msg = PS_ANYFAIL_ALARM_CLEAR; // matches power_ctl clearfail exactly
      break;
    default:
      return MCU_REG_INVALID_COMMAND;
  }
  return (xQueueSendToBack(xPwrQueue, &msg, 0) == pdPASS) ? MCU_REG_OK : MCU_REG_QUEUE_FULL;
}

enum mcu_reg_result
mcu_reg_read(uint8_t page, uint8_t address, uint8_t length, uint8_t out[4])
{
  if (length < 1 || length > 4)
    return MCU_REG_INVALID_LENGTH;

  switch (page) {
    case MCU_REG_PAGE_SYSTEM:
      return mcu_local_read0(address, length, out);
    case MCU_REG_PAGE_POWER:
      return mcu_local_read1(address, length, out);
    case MCU_REG_PAGE_ALARM:
      return mcu_local_read2(address, length, out);
    case MCU_REG_PAGE_ADC:
      return mcu_local_read3(address, length, out);
    case MCU_REG_PAGE_CONFIG:
      return mcu_local_read5(address, length, out);
    case MCU_REG_PAGE_RUNTIME:
      return mcu_local_read6(address, length, out);
    case MCU_REG_PAGE_CONTROL:
      return MCU_REG_WRITE_ONLY;
    default:
      return MCU_REG_INVALID_PAGE;
  }
}

enum mcu_reg_result
mcu_reg_write(uint8_t page, uint8_t address, const uint8_t *data, size_t length)
{
  switch (page) {
    case MCU_REG_PAGE_SYSTEM:
    case MCU_REG_PAGE_POWER:
    case MCU_REG_PAGE_ALARM:
    case MCU_REG_PAGE_ADC:
    case MCU_REG_PAGE_CONFIG:
    case MCU_REG_PAGE_RUNTIME:
      return MCU_REG_READ_ONLY;
    case MCU_REG_PAGE_CONTROL:
      return mcu_local_write7f(address, data, length);
    default:
      return MCU_REG_INVALID_PAGE;
  }
}
