#include "AlarmUtilities.h"
#include "Tasks.h"
#include "MonitorTask.h"

#include "common/log.h"

#define LOG_FACILITY LOG_ALM

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
uint32_t getAlarmStatus()
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
// static float tm4c_temp, max_fpga, max_dcdc_temp;
// static int8_t imax_ff_temp;

int TempStatus()
{
  int retval = 0;
  status_T = 0x0U;

  // microcontroller
  currentTemp[TM4C] = getADCvalue(ADC_INFO_TEMP_ENTRY);
  if (currentTemp[TM4C] > getAlarmTemperature(TM4C)) {
    status_T |= ALM_STAT_TM4C_OVERTEMP;
    retval++;
  }

  // FPGA
  if (fpga_args.n_devices == 2) {
    currentTemp[FPGA] = MAX(fpga_args.pm_values[0], fpga_args.pm_values[1]);
  }
  else {
    currentTemp[FPGA] = fpga_args.pm_values[0];
  }
  if (currentTemp[FPGA] > getAlarmTemperature(FPGA)) {
    status_T |= ALM_STAT_FPGA_OVERTEMP;
    retval++;
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
  if (currentTemp[DCDC] > getAlarmTemperature(DCDC)) {
    status_T |= ALM_STAT_DCDC_OVERTEMP;
    retval++;
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
  if (currentTemp[FF] > getAlarmTemperature(FF)) {
    status_T |= ALM_STAT_FIREFLY_OVERTEMP;
    retval++;
  }
  return retval;
}

void TempErrorLog()
{
  log_warn("Temperature high: status: 0x%04x MCU: %d F: %d FF:%d PS:%d\r\n",
      status_T, (int)currentTemp[TM4C], (int)currentTemp[FPGA],
      (int)currentTemp[FF], (int)currentTemp[DCDC]);
  errbuffer_temp_high((uint8_t)currentTemp[TM4C], (uint8_t)currentTemp[FPGA],
                      (uint8_t)currentTemp[FF], (uint8_t)currentTemp[DCDC]);
}

void TempClearErrorLog()
{
  log_info("Temperature normal\r\n");
  errbuffer_put(EBUF_TEMP_NORMAL, 0);
}

static QueueHandle_t const xTempAlarmQueue = 0;

struct GenericAlarmParams_t tempAlarmTask = {
    .xAlmQueue = xTempAlarmQueue,
    .checkStatus = &TempStatus,
    .errorlog_registererror = &TempErrorLog,
    .errorlog_clearerror = &TempClearErrorLog,
};
