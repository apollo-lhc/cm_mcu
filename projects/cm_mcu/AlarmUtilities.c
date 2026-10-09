#include "AlarmUtilities.h"
#include "Tasks.h"
#include "MonitorTask.h"
#include "FireflyUtils.h"
#include "MCU_Reg.h"

#include "common/log.h"
#include "common/pinsel.h"
#include "common/utils.h"
#include <assert.h>
#include <math.h>
#include <stdbool.h>

///////////////////////////////////////////////////////////
//
// Temperature Alarms
//
///////////////////////////////////////////////////////////

extern struct MonitorTaskArgs_t fpga_args;

#define ALM_OVERTEMP_THRESHOLD 5
// if the temperature is above the threshold by OVERTEMP_THRESHOLD
// a shutdown message is sent

// Hysteresis deadband (degrees C) for clearing a warn condition. A device's
// warn bit is set as soon as its temperature rises above the threshold, but is
// only cleared once the temperature drops at least this far below it. This
// prevents the alarm from flapping when a sensor dithers right at the limit
// (e.g. Firefly integer temps reading 55<->56 against a 55 C threshold).
#define ALM_TEMP_HYSTERESIS 3

// current value of the thresholds in integer degrees C (initialized to compile-time defaults;
// overwritten at startup by loadAlarmTemperaturesFromEEPROM() if EEPROM has valid data)
static int16_t alarmTemp[4] = {INITIAL_ALARM_TEMP_FF, INITIAL_ALARM_TEMP_DCDC,
                               INITIAL_ALARM_TEMP_TM4C, INITIAL_ALARM_TEMP_FPGA};

// EEPROM addresses for each device's alarm temperature, indexed by enum device
static const uint32_t alarmTempAddr[4] = {
    ADDR_TEMP_FF,   // FF   = 0
    ADDR_TEMP_DCDC, // DCDC = 1
    ADDR_TEMP_TM4C, // TM4C = 2
    ADDR_TEMP_FPGA, // FPGA = 3
};
// current value of temperatures
static float currentTemp[4] = {0.f, 0.f, 0.f, 0.f};

// Status flags of the temperature alarm task
static uint32_t status_T = 0x0;

// Latched per-device warn state for the hysteresis deadband. Uses the same
// ALM_STAT_*_OVERTEMP bit positions as status_T.
static uint32_t warnLatch = 0x0;

// Clear the warning latch.
void clearWarnLatch(void)
{
  warnLatch = 0x0;
}

// Evaluate one device's over-temperature condition with hysteresis.
// excess = (measured temp - threshold) in degrees C. Updates status_T and the
// warnLatch for `bit` and returns this device's contribution to retval:
//   0 = normal, 1 = warn, 2 = error (excess beyond ALM_OVERTEMP_THRESHOLD).
// The warn state latches: set above the threshold, held inside the deadband,
// cleared only once excess falls below -ALM_TEMP_HYSTERESIS.
static int evalTempHyst(float excess, uint32_t bit)
{
  bool warn;
  if (excess > 0.f)
    warn = true; // above threshold: set
  else if (excess < -(float)ALM_TEMP_HYSTERESIS)
    warn = false; // clearly below: clear
  else
    warn = (warnLatch & bit) != 0; // in the deadband: hold previous state

  if (!warn) {
    warnLatch &= ~bit;
    return 0;
  }
  warnLatch |= bit;
  status_T |= bit;
  return (excess > ALM_OVERTEMP_THRESHOLD) ? 2 : 1;
}

// read-only, so no need to use queue
uint32_t getTempAlarmStatus(void)
{
  return status_T;
}

uint32_t getWarnLatch(void)
{
  return warnLatch;
}

// Published by the TALM task's own GenericAlarmTask() instance; see
// struct GenericAlarmParams_t's published_state field.
static enum alarm_task_state tempAlarmState = ALM_INIT;

enum alarm_task_state getTempAlarmTaskState(void)
{
  return tempAlarmState;
}

int16_t getAlarmTemperature(enum device theDevice)
{
  return alarmTemp[theDevice];
}

void setAlarmTemperature(enum device theDevice, int16_t temperature)
{
  alarmTemp[theDevice] = temperature;
  // Zero-extend int16_t to uint32_t for EEPROM storage.
  // 0xFFFFFFFF is reserved as the uninitialized-EEPROM sentinel and cannot
  // be produced by zero-extension (upper 16 bits are always 0).
  // The gatekeeper programs the word only if it differs from what is stored,
  // so repeated sets do not wear the EEPROM.
  write_eeprom_if_diff((uint32_t)(uint16_t)temperature, alarmTempAddr[theDevice]);
}

// Queue the EEPROM write first and change the live threshold only if it was
// accepted, so a "queue full" reply to the client means nothing changed.
bool setAlarmTemperatureTry(enum device theDevice, int16_t temperature)
{
  if (!write_eeprom_if_diff_try((uint32_t)(uint16_t)temperature, alarmTempAddr[theDevice]))
    return false;
  alarmTemp[theDevice] = temperature;
  return true;
}

// Load alarm temperature thresholds from EEPROM into the alarmTemp[] array.
// If a slot reads 0xFFFFFFFF (uninitialized EEPROM), the compile-time default
// already present in alarmTemp[] is kept unchanged.
// Must be called after the EEPROM gatekeeper task and its queues are running.
void loadAlarmTemperaturesFromEEPROM(void)
{
  for (int i = 0; i < 4; ++i) {
    uint32_t raw = read_eeprom_single(alarmTempAddr[i]);
    if (raw == 0xFFFFFFFFU) {
      continue; // uninitialized — keep compile-time default
    }
    alarmTemp[i] = (int16_t)(uint16_t)raw;
  }
}

// check the current temperature status.
// returns +1 for warning, +2 or higher for error
int TempStatus(void)
{
  int retval = 0;
  status_T = 0x0U;

  // microcontroller
  currentTemp[TM4C] = getADCvalue(ADC_INFO_TEMP_ENTRY);
  retval += evalTempHyst(currentTemp[TM4C] - getAlarmTemperature(TM4C), ALM_STAT_TM4C_OVERTEMP);

  // DCDC. The first command is READ_TEMPERATURE_1.
  // I am assuming it stays that way!!!!!!!!
  currentTemp[DCDC] = -99.0f;
  // one bit per LGA80D device/page: set while its temperature is out of range,
  // so the warning is logged once per transition rather than every cycle
  static uint32_t dcdcInvalidMask = 0;
  configASSERT(dcdc_args.n_devices * dcdc_args.n_pages <= 32);
  for (int ps = 0; ps < dcdc_args.n_devices; ++ps) {
    for (int page = 0; page < dcdc_args.n_pages; ++page) {
      size_t index =
          ps * (dcdc_args.n_commands * dcdc_args.n_pages) + page * dcdc_args.n_commands + 0;
      float thistemp = dcdc_args.pm_values[index];
      if (thistemp <= -999.f)
        continue; // sentinel
      uint32_t bit = 1UL << (ps * dcdc_args.n_pages + page);
      if (!(thistemp >= LGA80D_TEMP_MIN_C && thistemp <= LGA80D_TEMP_MAX_C)) {
        if (!(dcdcInvalidMask & bit)) {
          const char *sign;
          int tens, fractions;
          float_to_ints(thistemp, &sign, &tens, &fractions);
          log_warn(LOG_ALM, "LGA80D %s page %d: invalid temp %s%d.%02d C; ignoring\r\n",
                   dcdc_args.devices[ps].name, page, sign, tens, fractions);
          dcdcInvalidMask |= bit;
        }
        continue;
      }
      if (dcdcInvalidMask & bit) {
        log_info(LOG_ALM, "LGA80D %s page %d: temp valid again\r\n", dcdc_args.devices[ps].name,
                 page);
        dcdcInvalidMask &= ~bit;
      }
      if (thistemp > currentTemp[DCDC])
        currentTemp[DCDC] = thistemp;
    }
  }
  retval += evalTempHyst(currentTemp[DCDC] - getAlarmTemperature(DCDC), ALM_STAT_DCDC_OVERTEMP);
#ifdef REV1
  // tests below here require the power to be on
  if (getPowerControlState() != POWER_ON) {
    // FPGA and Firefly are not evaluated when power is off; clear their latched
    // warn state so they start fresh when power returns.
    warnLatch &= ~(ALM_STAT_FPGA_OVERTEMP | ALM_STAT_FIREFLY_OVERTEMP);
    return retval;
  }
#endif // REV1

  // FPGA
  // loop over the two FPGAs and take the max temp.
  // we loop over all entries in the pm_values for fpga_args.
  // Currently there are only FPGA temperatures here. If that
  // changes this will be wrong. Can check the name of the
  // register in that case....
  currentTemp[FPGA] = -99.0f;
  for (int i = 0; i < fpga_args.n_values; ++i) {
    float thistemp = fpga_args.pm_values[i];
    if (thistemp > currentTemp[FPGA]) {
      currentTemp[FPGA] = thistemp;
    }
  }
#if defined REV2 || defined(REV3)
  // now check the FPGA diode temperatures as measured by the ADC on the MCU
  // these are always valid (MCU ADC readout). We didn't add this til Rev2.
  currentTemp[FPGA] = MAX(currentTemp[FPGA], getADCvalue(ADC_INFO_F1_TEMP_ENTRY));
  currentTemp[FPGA] = MAX(currentTemp[FPGA], getADCvalue(ADC_INFO_F2_TEMP_ENTRY));
#endif // REV2 or 3
  retval += evalTempHyst(currentTemp[FPGA] - getAlarmTemperature(FPGA), ALM_STAT_FPGA_OVERTEMP);
#if defined REV2 || defined(REV3)
  // tests below here require the power to be on
  if (getPowerControlState() != POWER_ON) {
    // Firefly is not evaluated when power is off; clear its latched warn state.
    warnLatch &= ~ALM_STAT_FIREFLY_OVERTEMP;
    return retval;
  }
#endif // REV2 or 3
  // Fireflies. These are reported as ints but we are asked
  // to report a float.
  // if stale we ignore
  if (isFFStale()) {
    // not evaluated this cycle; clear latched warn state.
    warnLatch &= ~ALM_STAT_FIREFLY_OVERTEMP;
    return retval;
  }
  int16_t imax_ff_temp = -99;
  for (size_t i = 0; i < NFIREFLIES; ++i) {
    int16_t v = getFFtemp(i);
    if (v == FF_TEMP_INVALID) {
      log_debug(LOG_ALM, "Firefly %zu: current temp is invalid (raw 0x%04x)\r\n", i,
                getFFtempRaw(i));
      continue;
    }
    if (v > imax_ff_temp)
      imax_ff_temp = v;
  }
  // keep a copy of current temp for external display, but do
  // calculations in int.
  currentTemp[FF] = (float)imax_ff_temp;
  retval += evalTempHyst((float)(imax_ff_temp - getAlarmTemperature(FF)), ALM_STAT_FIREFLY_OVERTEMP);
  return retval;
}

// this function logs the temperature error. It doesn't know the context of the error,
// it just logs the current temperature values.
void TempErrorLog(void)
{
  log_warn(LOG_ALM, "Temperature high: status: 0x%04x MCU: %d F: %d FF:%d PS:%d\r\n",
           status_T, (int)currentTemp[TM4C], (int)currentTemp[FPGA],
           (int)currentTemp[FF], (int)currentTemp[DCDC]);
  errbuffer_temp_high((uint8_t)currentTemp[TM4C], (uint8_t)currentTemp[FPGA],
                      (uint8_t)currentTemp[FF], (uint8_t)currentTemp[DCDC]);
}

// the EEPROM entry is EBUF_TEMP_NORMAL either way
void TempClearErrorLog(bool fault_latched)
{
  if (fault_latched)
    log_info(LOG_ALM, "Temperature fault no longer detected; power-off latched until alarms cleared\r\n");
  else
    log_info(LOG_ALM, "Temperature error cleared\r\n");
  errbuffer_put(EBUF_TEMP_NORMAL, 0);
}

struct GenericAlarmParams_t tempAlarmTask = {
    .checkStatus = &TempStatus,
    .errorlog_registererror = &TempErrorLog,
    .errorlog_clearerror = &TempClearErrorLog,
    .clearHysteresis = &clearWarnLatch,
    .stack_size = 4096,
    .led_warn_msg = &LED_STATUS_WARN,
    .led_alarm_msg = &LED_STATUS_ALARM,
    .led_normal_msg = &LED_STATUS_NORMAL,
    .published_state = &tempAlarmState,
};

///////////////////////////////////////////////////////////
//
// Voltage Alarms
//
///////////////////////////////////////////////////////////

// current value of the threshold, in centi-percent (500 = +/-5.00 %). This is
// also the EEPROM and ProgCom unit; VoltStatus() is the only place it is
// converted to a fraction.
#define INITIAL_ALARM_VOLT_CPCT 500 // +/-5% from the ADC thresholds
static uint16_t alarmVoltCpct = INITIAL_ALARM_VOLT_CPCT;

uint16_t getAlarmVoltageThresCpct(void)
{
  return alarmVoltCpct;
}

// Zero-extended to 32 bits in EEPROM, so 0xFFFFFFFF stays the uninitialized
// sentinel.
void setAlarmVoltageThresCpct(uint16_t cpct)
{
  alarmVoltCpct = cpct;
  write_eeprom_if_diff(cpct, ADDR_ALARM_VOLT);
}

bool setAlarmVoltageThresCpctTry(uint16_t cpct)
{
  if (!write_eeprom_if_diff_try(cpct, ADDR_ALARM_VOLT))
    return false;
  alarmVoltCpct = cpct;
  return true;
}

// Load the voltage alarm threshold from EEPROM into alarmVoltCpct.
// An uninitialized or out-of-range word (valid range is ProgCom's
// CFG_VOLT_MIN_CPCT-CFG_VOLT_MAX_CPCT, which the CLI shares) keeps the
// compile-time default.
// Must be called after the EEPROM gatekeeper task and its queues are running.
void loadAlarmVoltageFromEEPROM(void)
{
  uint32_t raw = read_eeprom_single(ADDR_ALARM_VOLT);
  if (raw < CFG_VOLT_MIN_CPCT || raw > CFG_VOLT_MAX_CPCT) {
    return; // uninitialized (0xFFFFFFFF) or corrupt: keep the default
  }
  alarmVoltCpct = (uint16_t)raw;
}

// current status of voltages
static uint8_t currentVoltStatus[3] = {0U, 0U, 0U};

// Status flags of the voltage alarm task
static uint32_t status_V = 0x0;
// worst offender of the last VoltStatus() call: signed deviation from target in
// percent (negative = low), plus the channel and its measured/target values
static float excess_volt = 0.0f;
static int excess_volt_which_ch = 0;
static float excess_volt_now = 0.0f;
static float excess_volt_target = 0.0f;
// copy of the above taken when an alarm is registered. Unlike the live values it
// survives VoltStatus() resetting them (e.g. power off or POWER_FAILURE), so the
// failing rail is still visible after a fault. Cleared on ALM_CLEAR_ALL.
static bool latch_volt_valid = false;
static float latch_volt_pct = 0.0f;
static int latch_volt_ch = 0;
static float latch_volt_now = 0.0f;
static float latch_volt_target = 0.0f;

void clearVoltAlarmLatch(void)
{
  latch_volt_valid = false;
}

bool getVoltAlarmLatch(int *ch, float *now, float *target, float *pct)
{
  // prevent tearing with a critical section. 
  taskENTER_CRITICAL();

  if (!latch_volt_valid) {
    taskEXIT_CRITICAL();
    return false;
  }
  *ch = latch_volt_ch;
  *now = latch_volt_now;
  *target = latch_volt_target;
  *pct = latch_volt_pct;

  taskEXIT_CRITICAL();
  return true;
}
// read-only, so no need to use queue
uint32_t getVoltAlarmStatus(void)
{
  return status_V;
}

uint8_t getVoltStatusGroup(enum powdevice which)
{
  return currentVoltStatus[which];
}

// Published by the VALM task's own GenericAlarmTask() instance; see
// struct GenericAlarmParams_t's published_state field.
static enum alarm_task_state voltAlarmState = ALM_INIT;

enum alarm_task_state getVoltAlarmTaskState(void)
{
  return voltAlarmState;
}
// check the current voltage status.
// returns 1 for one warning rail, 2 for two warning rails or one severe rail
// these flags represent positions into thestruct ADC_Info_t ADCs[] array in
// the ADCMonitorTask.c
#ifdef REV1
#define VALM_BASE_MASK    0x00025U // management powers, e.g. 12V and M3V3
#define VALM_GEN_MASK     0x0001AU // common powers
#define VALM_F1_MASK      0xCCC80U // F1-specific
#define VALM_F2_MASK      0x33340U // F2-specific
#define VALM_ALL_MASK     (VALM_BASE_MASK | VALM_GEN_MASK | VALM_F1_MASK | VALM_F2_MASK)
#define VALM_HIGHEST_V_CH 19 // highest channel that contains a voltage, 0 based counting
#elif defined(REV2) || defined(REV3)
#define VALM_BASE_MASK    0x003U  // management powers, e.g. 12V and M3V3
#define VALM_GEN_MASK     0x001CU // common powers
#define VALM_F1_MASK      0x01E0U // F1-specific
#define VALM_F2_MASK      0x1E00U // F2-specific
#define VALM_ALL_MASK     (VALM_BASE_MASK | VALM_GEN_MASK | VALM_F1_MASK | VALM_F2_MASK)
#define VALM_HIGHEST_V_CH 12 // highest channel that contains a voltage, 0 based counting
#endif                       // REV2 or 3
int VoltStatus(void)
{

  // compile-time sanity check on the flags being unique.
  // I need the +1 in the 1<xx since the highest channel is 0-based counting.
  static_assert((VALM_BASE_MASK ^ VALM_GEN_MASK ^ VALM_F1_MASK ^ VALM_F2_MASK) == ((1 << (VALM_HIGHEST_V_CH + 1)) - 1),
                "VALM masks not unique");

  bool f1_enable = isFPGAF1_PRESENT();
  bool f2_enable = isFPGAF2_PRESENT();

  int warn_rails = 0;
  bool severe_rail = false;
  status_V = 0x0U;

  // change what we do, if power is on or not.
  enum power_system_state currPsState = getPowerControlState();

  if (!((currPsState == POWER_ON) || (currPsState == POWER_OFF))) { // in flux. Skip.
    return 0;
  }

  // set up mask for which channels to worry about
  uint32_t ch_mask = VALM_BASE_MASK; // always true
  if (currPsState == POWER_ON) {
    ch_mask |= VALM_GEN_MASK; // common power
    if (f1_enable) {
      ch_mask |= VALM_F1_MASK;
    }
    if (f2_enable) {
      ch_mask |= VALM_F2_MASK;
    }
  }
  // Loop over ADC values.
  const float threshold = (float)getAlarmVoltageThresCpct() / 10000.f; // fraction
  const float fault_threshold = 2.f * threshold;
  uint32_t ch_alm_mask = 0x0U;
  excess_volt = 0.0f; // reset, so a cleared alarm doesn't report stale data
  excess_volt_which_ch = 0;
  excess_volt_now = 0.0f;
  excess_volt_target = 0.0f;
  // VALM_HIGHEST_V_CH is 0-based, so the highest channel must be included
  for (int i = 0; i <= VALM_HIGHEST_V_CH; ++i) {
    // check if the current channel contains a voltage measurement we care about
    if (!(ch_mask & (0x1U << i))) {
      continue; // if not, continue to then ext loop iteration
    }
    float target_value = getADCtargetValue(i);
    float now_value = getADCvalue(i);
    float excess = (now_value - target_value) / target_value;
    float aexcess = ABS(excess);

    if (aexcess > threshold) {
      ++warn_rails;
      if (aexcess > fault_threshold) {
        severe_rail = true;
      }
      ch_alm_mask |= (0x1U << i);               // mark bit for failing supply
      if (aexcess * 100.f > ABS(excess_volt)) { // keep the worst offender, in percent
        excess_volt = excess * 100.f;
        excess_volt_which_ch = i;
        excess_volt_now = now_value;
        excess_volt_target = target_value;
      }
      const char *sign;
      int tens, frac;
      float_to_ints(excess * 100, &sign, &tens, &frac);
      log_debug(LOG_ALM, "VoltAlm: %s: %s%02d.%02d %% off target\r\n", getADCname(i), sign, tens,
                frac);
    }
  }
  // record which rails failed, per group, for the EEPROM error buffer. The shift
  // makes the bits group-relative; the ctz of each mask is a compile-time constant.
  //
  // This packing is only lossless on REV2/3, where each group mask is contiguous
  // and at most 5 bits wide once shifted down. On REV1 the F1/F2 masks are
  // interleaved rather than contiguous, so shifting by ctz leaves a 13-bit value
  // (0xCCC80 >> 7 == 0x1999) and (0x33340 >> 6 == 0xCCD), and the uint8_t cast
  // silently drops the high bits: the F1/F2 entries in the error log are partial.
  // status_V below is unaffected -- it tests the masks directly -- so the alarm
  // itself is still correct on REV1; only the logged detail is lossy. REV1 is
  // deprecated, so this is recorded rather than fixed. A fix would need a
  // bit-gather, not a shift.
#define VALM_GRP(msk) (uint8_t)((ch_alm_mask & (msk)) >> __builtin_ctz(msk))
  currentVoltStatus[GEN] = VALM_GRP(VALM_BASE_MASK | VALM_GEN_MASK);
  currentVoltStatus[FPGA1] = VALM_GRP(VALM_F1_MASK);
  currentVoltStatus[FPGA2] = VALM_GRP(VALM_F2_MASK);
#undef VALM_GRP
  status_V = 0x0U;
  if (ch_alm_mask & (VALM_BASE_MASK | VALM_GEN_MASK)) {
    status_V |= ALM_STAT_GEN_OVERVOLT;
  }
  if (ch_alm_mask & VALM_F1_MASK) {
    status_V |= ALM_STAT_FPGA1_OVERVOLT;
  }
  if (ch_alm_mask & VALM_F2_MASK) {
    status_V |= ALM_STAT_FPGA2_OVERVOLT;
  }

  // Severity counts individual rails; groups above are diagnostics only.
  return (severe_rail || warn_rails > 1) ? 2 : warn_rails;
}

void VoltErrorLog(void)
{
  // called on NORMAL->WARN and again on WARN->FAULT, so after a fault this holds
  // the reading that tripped it
  latch_volt_pct = excess_volt;
  latch_volt_ch = excess_volt_which_ch;
  latch_volt_now = excess_volt_now;
  latch_volt_target = excess_volt_target;
  latch_volt_valid = true;
  if (ABS(excess_volt) > 2.0f) {
    const char *pct_sign, *now_sign, *tgt_sign;
    int pct_tens, pct_frac, now_tens, now_frac, tgt_tens, tgt_frac;
    float_to_ints(excess_volt, &pct_sign, &pct_tens, &pct_frac);
    float_to_ints(excess_volt_now, &now_sign, &now_tens, &now_frac);
    float_to_ints(excess_volt_target, &tgt_sign, &tgt_tens, &tgt_frac);
    log_warn(LOG_ALM,
             "Voltage %s: status: 0x%04x %s (ADC ch %02d) %s%d.%02d V, target %s%d.%02d V, %s%02d.%02d %% off\r\n",
             (excess_volt < 0.f) ? "low" : "high", status_V, getADCname(excess_volt_which_ch),
             excess_volt_which_ch, now_sign, now_tens, now_frac, tgt_sign, tgt_tens, tgt_frac,
             (*pct_sign != '\0') ? pct_sign : "+", pct_tens, pct_frac);
  }
  // add voltage status as a data field in eeprom rather than its value
  errbuffer_volt_high((uint8_t)currentVoltStatus[GEN], (uint8_t)currentVoltStatus[FPGA1],
                      (uint8_t)currentVoltStatus[FPGA2]);
}

// the EEPROM entry is EBUF_VOLT_NORMAL either way. "no longer detected" rather
// than "normal": with power off the faulting rail is no longer measured at all.
void VoltClearErrorLog(bool fault_latched)
{
  if (fault_latched)
    log_info(LOG_ALM, "Voltage fault no longer detected; power-off latched until alarms cleared\r\n");
  else
    log_info(LOG_ALM, "Voltage normal\r\n");
  errbuffer_put(EBUF_VOLT_NORMAL, 0);
}

struct GenericAlarmParams_t voltAlarmTask = {
    .checkStatus = &VoltStatus,
    .errorlog_registererror = &VoltErrorLog,
    .errorlog_clearerror = &VoltClearErrorLog,
    .clearHysteresis = &clearVoltAlarmLatch,
    .stack_size = 4096,
    .published_state = &voltAlarmState,
};

///////////////////////////////////////////////////////////
//
// Current Alarms
//
///////////////////////////////////////////////////////////
