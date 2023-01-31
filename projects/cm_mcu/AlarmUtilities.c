#include "AlarmUtilities.h"
#include "Tasks.h"
#include "MonitorTask.h"

#include "common/log.h"

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
uint32_t getTempAlarmStatus()
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
int TempStatus()
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

void TempErrorLog()
{
  log_warn(LOG_ALM, "Temperature high: status: 0x%04x MCU: %d F: %d FF:%d PS:%d\r\n",
           status_T, (int)currentTemp[TM4C], (int)currentTemp[FPGA],
           (int)currentTemp[FF], (int)currentTemp[DCDC]);
  errbuffer_temp_high((uint8_t)currentTemp[TM4C], (uint8_t)currentTemp[FPGA],
                      (uint8_t)currentTemp[FF], (uint8_t)currentTemp[DCDC]);
}

void TempClearErrorLog()
{
  log_info(LOG_ALM, "Temperature normal\r\n");
  errbuffer_put(EBUF_TEMP_NORMAL, 0);
}

static QueueHandle_t const xTempAlarmQueue = 0;

struct GenericAlarmParams_t tempAlarmTask = {
    .xAlmQueue = xTempAlarmQueue,
    .checkStatus = &TempStatus,
    .errorlog_registererror = &TempErrorLog,
    .errorlog_clearerror = &TempClearErrorLog,
    .stack_size = 176,
};

///////////////////////////////////////////////////////////
//
// Voltage Alarms
//
///////////////////////////////////////////////////////////

// current status of voltages
static uint8_t currentVoltStatus[4] = {0.0, 0.0, 0.0, 0.0}; // FIXME : bitmasks for voltage statuses

// Status flags of the voltage alarm task
static uint32_t status_V = 0x0;
// read-only, so no need to use queue
uint32_t getVoltAlarmStatus()
{
  return status_V;
}
// check the current voltage status.
// returns +1 for warning, +2 or higher for error
int VoltStatus()
{
#ifndef REV2                      // REV1
  uint8_t TM4C_VOLTAGE_MASK = 63; // 0b111111 by default
#else                             // REV2
  uint8_t TM4C_VOLTAGE_MASK = 31; // 0b11111 by default
#endif                            // REV 2
  int retval = 0;
  status_V = 0x0U;
  uint8_t tm4c_bitmask = 0;
  uint8_t fpga_bitmask = 0;
  // bool is_excess_volt = 0;
  bool is_alarm_volt = 0;
  int debug_ch = -1; // FIX ME

  // microcontroller
  if (getPowerControlState() != POWER_ON) {
#ifndef REV2               // REV1
    TM4C_VOLTAGE_MASK = 5; // 0b000101 only allows other powers off except M3V3 and 12V
#else                      // REV2
    TM4C_VOLTAGE_MASK = 3;        // 0b00011 only allows other powers off except M3V3 and 12V
#endif                     // REV 2
  }
  for (int ch = ADC_INFO_TM4C_VCC_INIT_CH; ch < ADC_INFO_TM4C_VCC_FIN_CH; ++ch) {

    float threshold = getADCtargetValue(ch) * 0.10f;
    float now_value = getADCvalue(ch);
    float excess = now_value - getADCtargetValue(ch);
    if (excess > threshold) {                                      // if this ADC voltage is greater than a target value by 10%
      tm4c_bitmask += (1 << (ch - ADC_INFO_TM4C_VCC_INIT_CH + 1)); // first to last bit corresponds to status of low to high ADC voltage channel of tm4c
      is_alarm_volt = 1;
      debug_ch = ch;
    }
  }

  if (is_alarm_volt) { // over voltage among one of tm4c power supplies by +10% of its threshold
    log_debug(LOG_ALM, "alarm volt at ADC ch : %d\r\n", debug_ch);
    status_V |= ALM_STAT_TM4C_OVERVOLT;
    retval++;
  }

  currentVoltStatus[TM4C] = tm4c_bitmask & TM4C_VOLTAGE_MASK; // applies a mask with power-off exceptions

  is_alarm_volt = 0;
  for (int ch = ADC_INFO_FPGA_VCC_INIT_CH; ch < ADC_INFO_FPGA_VCC_FIN_CH; ++ch) {

    float threshold = getADCtargetValue(ch) * 0.10f;
    float now_value = getADCvalue(ch);
    float excess = now_value - getADCtargetValue(ch);
    if (excess > threshold) {                                      // if this ADC voltage is greater than a target value by 10%
      fpga_bitmask += (1 << (ch - ADC_INFO_FPGA_VCC_INIT_CH + 1)); // first to last bit corresponds to status of low to high ADC voltage channel of tm4c
      is_alarm_volt = 1;
      debug_ch = ch;
    }
  }

  if (is_alarm_volt) { // over voltage among one of tm4c power supplies by +10% of its threshold
    log_debug(LOG_ALM, "alarm volt at ADC ch : %d\r\n", debug_ch);
    status_V |= ALM_STAT_FPGA_OVERVOLT;
    retval++;
  }

  currentVoltStatus[FPGA] = fpga_bitmask;

  return retval;
}

void VoltErrorLog()
{
  log_warn(LOG_ALM, "Voltage high: status: 0x%04x MCU: %d FPGAs:%d\r\n",
           status_V, (int)currentVoltStatus[TM4C], (int)currentVoltStatus[FPGA]);
  errbuffer_volt_high((uint8_t)currentVoltStatus[TM4C], (uint8_t)currentVoltStatus[FPGA]); // add voltage status as a data field in eeprom rather than its value
}

void VoltClearErrorLog()
{
  log_info(LOG_ALM, "Voltage normal\r\n");
  errbuffer_put(EBUF_VOLT_NORMAL, 0);
}

static QueueHandle_t const xVoltAlarmQueue = 0;

struct GenericAlarmParams_t voltAlarmTask = {
    .xAlmQueue = xVoltAlarmQueue,
    .checkStatus = &VoltStatus,
    .errorlog_registererror = &VoltErrorLog,
    .errorlog_clearerror = &VoltClearErrorLog,
    .stack_size = 35,
};

///////////////////////////////////////////////////////////
//
// Current Alarms
//
///////////////////////////////////////////////////////////
