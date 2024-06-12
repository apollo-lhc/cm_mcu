#include "AlarmUtilities.h"
#include "Tasks.h"
#include "MonitorTask.h"
#include "FireflyUtils.h"

#include "common/log.h"
#include "common/pinsel.h"
#include <assert.h>
#include <math.h>

///////////////////////////////////////////////////////////
//
// Temperature Alarms
//
///////////////////////////////////////////////////////////

extern struct MonitorTaskArgs_t fpga_args;

#define INITIAL_ALARM_TEMP_FF   45.0f // in Celsius duh
#define INITIAL_ALARM_TEMP_DCDC 70.0f
#define INITIAL_ALARM_TEMP_TM4C 70.0f
#define INITIAL_ALARM_TEMP_FPGA 70.0f
#define ALM_OVERTEMP_THRESHOLD  5.0f
// if the temperature is above the threshold by OVERTEMP_THRESHOLD
// a shutdown message is sent

// current value of the thresholds
static float alarmTemp[4] = {INITIAL_ALARM_TEMP_FF, INITIAL_ALARM_TEMP_DCDC,
                             INITIAL_ALARM_TEMP_TM4C, INITIAL_ALARM_TEMP_FPGA};
// current value of temperatures
static float currentTemp[4] = {0.f, 0.f, 0.f, 0.f};

// Status flags of the temperature alarm task
static uint32_t status_T = 0x0;

// read-only, so no need to use queue
uint32_t getTempAlarmStatus(void)
{
  return status_T;
}

float getAlarmTemperature(enum device theDevice)
{
  return alarmTemp[theDevice];
}
void setAlarmTemperature(enum device theDevice, float temperature)
{
  alarmTemp[theDevice] = temperature;
}

// check the current temperature status.
// returns +1 for warning, +2 or higher for error
int TempStatus(void)
{
  int retval = 0;
  status_T = 0x0U;

  // microcontroller
  currentTemp[TM4C] = getADCvalue(ADC_INFO_TEMP_ENTRY);
  float excess_temp = currentTemp[TM4C] - getAlarmTemperature(TM4C);
  if (excess_temp > 0.f) { // over temperature
    status_T |= ALM_STAT_TM4C_OVERTEMP;
    retval++;
    if (excess_temp > ALM_OVERTEMP_THRESHOLD)
      ++retval;
  }

  // FPGA
  if (fpga_args.n_devices == 2) {
    currentTemp[FPGA] = MAX(fpga_args.pm_values[0], fpga_args.pm_values[1]);
  }
  else {
    currentTemp[FPGA] = fpga_args.pm_values[0];
  }
  excess_temp = currentTemp[FPGA] - getAlarmTemperature(FPGA);
  if (excess_temp > 0.f) {
    status_T |= ALM_STAT_FPGA_OVERTEMP;
    retval++;
    if (excess_temp > ALM_OVERTEMP_THRESHOLD)
      ++retval;
  }

  // DCDC. The first command is READ_TEMPERATURE_1.
  // I am assuming it stays that way!!!!!!!!
  currentTemp[DCDC] = -99.0f;
  for (int ps = 0; ps < dcdc_args.n_devices; ++ps) {
    for (int page = 0; page < dcdc_args.n_pages; ++page) {
      size_t index =
          ps * (dcdc_args.n_commands * dcdc_args.n_pages) + page * dcdc_args.n_commands + 0;
      float thistemp = dcdc_args.pm_values[index];
      if (thistemp > currentTemp[DCDC])
        currentTemp[DCDC] = thistemp;
    }
  }
  excess_temp = currentTemp[DCDC] - getAlarmTemperature(DCDC);
  if (excess_temp > 0.f) {
    status_T |= ALM_STAT_DCDC_OVERTEMP;
    retval++;
    if (excess_temp > ALM_OVERTEMP_THRESHOLD)
      ++retval;
  }

  // Fireflies. These are reported as ints but we are asked
  // to report a float.
  BaseType_t imax_ff_temp = -99;
  for (size_t i = 0; i < NFIREFLIES; ++i) {
    int8_t v = getFFtemp(i);
    if (v > imax_ff_temp)
      imax_ff_temp = v;
  }
  currentTemp[FF] = (float)imax_ff_temp;
  excess_temp = currentTemp[FF] - getAlarmTemperature(FF);
  if (excess_temp > 0.f) {
    status_T |= ALM_STAT_FIREFLY_OVERTEMP;
    retval++;
    if (excess_temp > ALM_OVERTEMP_THRESHOLD)
      ++retval;
  }
  return retval;
}

void TempErrorLog(void)
{
  log_warn(LOG_ALM, "Temperature high: status: 0x%04x MCU: %d F: %d FF:%d PS:%d\r\n",
           status_T, (int)currentTemp[TM4C], (int)currentTemp[FPGA],
           (int)currentTemp[FF], (int)currentTemp[DCDC]);
  errbuffer_temp_high((uint8_t)currentTemp[TM4C], (uint8_t)currentTemp[FPGA],
                      (uint8_t)currentTemp[FF], (uint8_t)currentTemp[DCDC]);
}

void TempClearErrorLog(void)
{
  log_info(LOG_ALM, "Temperature normal\r\n");
  errbuffer_put(EBUF_TEMP_NORMAL, 0);
}

struct GenericAlarmParams_t tempAlarmTask = {
    .checkStatus = &TempStatus,
    .errorlog_registererror = &TempErrorLog,
    .errorlog_clearerror = &TempClearErrorLog,
    .stack_size = 4096,
};

///////////////////////////////////////////////////////////
//
// Voltage Alarms
//
///////////////////////////////////////////////////////////

// current value of the thresholds
#define INITIAL_ALARM_VOLT_PERCENT 0.05f // +/-5% from the ADC thresholds
static float alarmVolt = INITIAL_ALARM_VOLT_PERCENT;

float getAlarmVoltageThres(void)
{
  return alarmVolt;
}
void setAlarmVoltageThres(float voltthres)
{
  alarmVolt = voltthres;
}

// current status of voltages
static uint8_t currentVoltStatus[3] = {0U, 0U, 0U};

// Status flags of the voltage alarm task
static uint32_t status_V = 0x0;
// fractional value of a high voltage than an expected ADC value
static float excess_volt = 0.0f;
static int excess_volt_which_ch = 0;
// read-only, so no need to use queue
uint32_t getVoltAlarmStatus(void)
{
  return status_V;
}
// check the current voltage status.
// returns +1 for warning, +2 or higher for error
// these flags represent positions into thestruct ADC_Info_t ADCs[] array in
// the ADCMonitorTask.c
#ifdef REV1
#define VALM_BASE_MASK    0x00025U // management powers, e.g. 12V and M3V3
#define VALM_GEN_MASK     0x0001AU // common powers
#define VALM_F1_MASK      0xCCC80U // F1-specific
#define VALM_F2_MASK      0x33340U // F2-specific
#define VALM_ALL_MASK     (VALM_BASE_MASK | VALM_GEN_MASK | VALM_F1_MASK | VALM_F2_MASK)
#define VALM_HIGHEST_V_CH 19 // highest channel that contains a voltage, 0 based counting
#elif REV2
#define VALM_BASE_MASK    0x003U  // management powers, e.g. 12V and M3V3
#define VALM_GEN_MASK     0x001CU // common powers
#define VALM_F1_MASK      0x01E0U // F1-specific
#define VALM_F2_MASK      0x1E00U // F2-specific
#define VALM_ALL_MASK     (VALM_BASE_MASK | VALM_GEN_MASK | VALM_F1_MASK | VALM_F2_MASK)
#define VALM_HIGHEST_V_CH 12 // highest channel that contains a voltage, 0 based counting
#endif                       // REV2
int VoltStatus(void)
{

  // compile-time sanity check on the flags being unique.
  // I need the +1 in the 1<xx since the highest channel is 0-based counting.
  static_assert((VALM_BASE_MASK ^ VALM_GEN_MASK ^ VALM_F1_MASK ^ VALM_F2_MASK) == ((1 << (VALM_HIGHEST_V_CH + 1)) - 1), "VALM masks not unique");

  bool f1_enable = isFPGAF1_PRESENT();
  bool f2_enable = isFPGAF2_PRESENT();

  int retval = 0;
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
  const float threshold = getAlarmVoltageThres();
  uint32_t ch_alm_mask = 0x0U;
  for (int i = 0; i < VALM_HIGHEST_V_CH; ++i) {
    // check if the current channel contains a voltage measurement we care about
    if (!(ch_mask & (0x1U << i))) {
      continue; // if not, continue to then ext loop iteration
    }
    float target_value = getADCtargetValue(i);
    float now_value = getADCvalue(i);
    float excess = (now_value - target_value) / target_value;

    if (ABS(excess) > threshold) {
      ch_alm_mask |= (0x1U << i); // mark bit for failing supply
      int tens, frac;
      float_to_ints(excess * 100, &tens, &frac);
      log_debug(LOG_ALM, "VoltAlm: %s: %02d.%02d %% off target\r\n", getADCname(i), tens, frac);
    }
  }
  status_V = 0x0U;
  if (ch_alm_mask & (VALM_BASE_MASK | VALM_GEN_MASK)) {
    status_V |= ALM_STAT_GEN_OVERVOLT;
    ++retval;
  }
  if (ch_alm_mask & VALM_F1_MASK) {
    status_V |= ALM_STAT_FPGA1_OVERVOLT;
    ++retval;
  }
  if (ch_alm_mask & VALM_F2_MASK) {
    status_V |= ALM_STAT_FPGA2_OVERVOLT;
    ++retval;
  }

  return retval;
}

void VoltErrorLog(void)
{
  int tens, frac;
  float_to_ints(excess_volt, &tens, &frac);
  if (excess_volt > 2.0f) {
    log_warn(LOG_ALM, "Voltage high: status: 0x%04x at ADC ch %02d now +%02d.%02d %% off\r\n",
             status_V, excess_volt_which_ch, tens, frac);
  }
  errbuffer_volt_high((uint8_t)currentVoltStatus[GEN], (uint8_t)currentVoltStatus[FPGA1], (uint8_t)currentVoltStatus[FPGA2]); // add voltage status as a data field in eeprom rather than its value
}

void VoltClearErrorLog(void)
{
  log_info(LOG_ALM, "Voltage normal\r\n");
  errbuffer_put(EBUF_VOLT_NORMAL, 0);
}

struct GenericAlarmParams_t voltAlarmTask = {
    .checkStatus = &VoltStatus,
    .errorlog_registererror = &VoltErrorLog,
    .errorlog_clearerror = &VoltClearErrorLog,
    .stack_size = 4096,
};

///////////////////////////////////////////////////////////
//
// Current Alarms
//
///////////////////////////////////////////////////////////
