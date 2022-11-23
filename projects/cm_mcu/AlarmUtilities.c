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
};

///////////////////////////////////////////////////////////
//
// Voltage Alarms
//
///////////////////////////////////////////////////////////

#define INITIAL_ALARM_VOLT_DCDC 70.0f // FIXME : copy from temp
#define INITIAL_ALARM_VOLT_TM4C 70.0f // FIXME : copy from temp
#define ALM_OVERVOLT_THRESHOLD  5.0f  // FIXME : copy from temp
// current value of the thresholds
static float alarmVolt[2] = {INITIAL_ALARM_VOLT_DCDC,
                             INITIAL_ALARM_VOLT_TM4C};
// current value of voltages
static float currentVolt[2] = {0.f, 0.f}; // FIXME : copy from temp

// Status flags of the voltage alarm task
static uint32_t status_V = 0x0;

// read-only, so no need to use queue
uint32_t getVoltAlarmStatus()
{
  return status_V;
}

float getAlarmVoltage(enum device theDevice)
{
  return alarmVolt[theDevice];
}
void setAlarmVoltage(enum device theDevice, float voltage)
{
  alarmVolt[theDevice] = voltage;
}

// check the current voltage status.
// returns +1 for warning, +2 or higher for error
int VoltStatus()
{
  int retval = 0;
  status_V = 0x0U;

  // microcontroller
  currentVolt[TM4C] = getADCvalue(ADC_INFO_VCC_INIT_CH); // start with the adc value from the first voltage channel
  for (int ch = ADC_INFO_VCC_INIT_CH + 1; ch < ADC_INFO_VCC_FIN_CH + 1; ++ch) {
    if (getADCvalue(ch) > currentVolt[TM4C])
      currentVolt[TM4C] = getADCvalue(ch); // find the maximum among voltage channels
  }

  float excess_volt = currentVolt[TM4C] - getAlarmVoltage(TM4C);
  if (excess_volt > 0.f) { // over temperature
    status_V |= ALM_STAT_TM4C_OVERVOLT;
    retval++;
    if (excess_volt > ALM_OVERVOLT_THRESHOLD)
      ++retval;
  }

  // DCDC. The first command is READ_VOUT.
  // I am assuming it stays that way!!!!!!!!
  currentVolt[DCDC] = -99.0f; // FIXME : copy from temp
  for (int ps = 0; ps < dcdc_args.n_devices; ++ps) {
    for (int page = 0; page < dcdc_args.n_pages; ++page) {
      size_t index =
          ps * (dcdc_args.n_commands * dcdc_args.n_pages) + page * dcdc_args.n_commands + 3;
      float thisvolt = dcdc_args.pm_values[index];
      if (thisvolt > currentVolt[DCDC])
        currentVolt[DCDC] = thisvolt;
    }
  }
  excess_volt = currentVolt[DCDC] - getAlarmVoltage(DCDC);
  if (excess_volt > 0.f) {
    status_V |= ALM_STAT_DCDC_OVERVOLT;
    retval++;
    if (excess_volt > ALM_OVERVOLT_THRESHOLD)
      ++retval;
  }

  return retval;
}

void VoltErrorLog()
{
  log_warn(LOG_ALM, "Voltage high: status: 0x%04x MCU: %d PS:%d\r\n",
           status_V, (int)currentVolt[TM4C], (int)currentVolt[DCDC]);
  // errbuffer_volt_high((uint8_t)currentVolt[TM4C],(uint8_t)currentVolt[DCDC]); //FIXME : necessary?
}

void VoltClearErrorLog()
{
  log_info(LOG_ALM, "Voltage normal\r\n");
  // errbuffer_put(EBUF_TEMP_NORMAL, 0); //FIXME : Is this line necessary? and what is EBUF_*_NORMAL?
}

static QueueHandle_t const xVoltAlarmQueue = 0;

struct GenericAlarmParams_t voltAlarmTask = {
    .xAlmQueue = xVoltAlarmQueue,
    .checkStatus = &VoltStatus,
    .errorlog_registererror = &VoltErrorLog,
    .errorlog_clearerror = &VoltClearErrorLog,
};

///////////////////////////////////////////////////////////
//
// Current Alarms
//
///////////////////////////////////////////////////////////
