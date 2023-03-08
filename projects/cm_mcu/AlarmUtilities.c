#include "AlarmUtilities.h"
#include "Tasks.h"
#include "MonitorTask.h"

#include "common/log.h"
#include "common/pinsel.h"

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
int VoltStatus(void)
{
  bool f1_enable = isFPGAF1_PRESENT();
  bool f2_enable = isFPGAF2_PRESENT();

#ifndef REV2                       // REV1
  uint8_t GEN_VOLTAGE_MASK = 0x3f; // 0b111111 by default
#else                              // REV2
  uint8_t GEN_VOLTAGE_MASK = 0x1f; // 0b11111 by default
#endif                             // REV 2

  int retval = 0;
  status_V = 0x0U;
  uint8_t gen_bitmask = 0;
  uint8_t fpga1_bitmask = 0;
  uint8_t fpga2_bitmask = 0;
  uint8_t is_alarm_volt = 0;

  // microcontroller and general power

  if (getPowerControlState() != POWER_ON) {
#ifndef REV2                // REV1
    GEN_VOLTAGE_MASK = 0x5; // 0b000101 only allows other powers off except M3V3 and 12V
#else                       // REV2
    GEN_VOLTAGE_MASK = 0x3;        // 0b00011 only allows other powers off except M3V3 and 12V
#endif                      // REV 2
  }

#ifdef REV2
  while (getADCvalue(ADC_INFO_GEN_VCC_4V0_CH) < 0.01f) { // somehow the initial value of VCC 4V0 is 0 after reboot and can screw up the logic below
    vTaskDelay(pdMS_TO_TICKS(10));                       // delay 10 ms
  }
#endif

  for (int ch = ADC_INFO_GEN_VCC_INIT_CH; ch < ADC_INFO_GEN_VCC_FIN_CH + 1; ++ch) {

    float threshold = getAlarmVoltageThres();
    float now_value = getADCvalue(ch);
    float excess = (now_value - getADCtargetValue(ch)) / getADCtargetValue(ch);
    int tens, frac;
    float_to_ints(excess*100, &tens, &frac);
    if (excess > 0.0f) {
      is_alarm_volt = 1;
      excess_volt = excess * 100;
      excess_volt_which_ch = ch;
    }

    if ((excess > threshold && excess > 0.0f) || (excess * -1.0f > threshold && excess < 0.0f)) { // if this ADC voltage is greater/lower than a target value by getAlarmVoltageThres()*100%
      gen_bitmask += (1 << (ch - ADC_INFO_GEN_VCC_INIT_CH));                                      // first to last bit corresponds to status of low to high ADC voltage channel of mcu/general
      is_alarm_volt = 2;
      log_debug(LOG_ALM, "alarm volt at ADC ch : %02d now %02d.%02d %% off target\r\n", ch, tens, frac); // over voltage among one of fpga power supplies by +/- getAlarmVoltageThres()*100%% of its threshold
    }
  }

  if (is_alarm_volt > 0) {
    retval++;
    if (is_alarm_volt == 2) {
      status_V |= ALM_STAT_GEN_OVERVOLT;
      ++retval;
    }
  }

  currentVoltStatus[GEN] = gen_bitmask & GEN_VOLTAGE_MASK; // applies a mask with power-off exceptions

  if (retval > 0)
    return retval;
  else
    retval = 0;

  is_alarm_volt = 0;
  status_V = 0x0U;

  int n_fpga_half_ch = (ADC_INFO_FPGA_VCC_FIN_CH - ADC_INFO_FPGA_VCC_INIT_CH + 1) / 2;
  int ADC_INFO_FPGA2_VCC_INIT_CH = n_fpga_half_ch + ADC_INFO_FPGA_VCC_INIT_CH;
  for (int ch = ADC_INFO_FPGA_VCC_INIT_CH; ch < ADC_INFO_FPGA_VCC_FIN_CH + 1; ++ch) {
    if ((!f1_enable) || (!f2_enable && ch > (ADC_INFO_FPGA2_VCC_INIT_CH - 1))) // check if fpga1/2 is on the board. currently fpga1 takes the first half of adc outputs in this indexing
      break;
    float threshold = getAlarmVoltageThres();
    float now_value = getADCvalue(ch);
    float excess = (now_value - getADCtargetValue(ch)) / getADCtargetValue(ch);
    int tens, frac;
    float_to_ints(excess*100, &tens, &frac);
    if (excess > 0.0f) {
      is_alarm_volt = 1;
      excess_volt = excess * 100;
      excess_volt_which_ch = ch;
    }
    if (ch > (ADC_INFO_FPGA2_VCC_INIT_CH - 1)) {
      if ((excess > threshold && excess > 0.0f) || (excess * -1.0f > threshold && excess < 0.0f)) { // if this ADC voltage is greater/lower than a target value by getAlarmVoltageThres()*100%
        fpga2_bitmask += (1 << (ch - ADC_INFO_FPGA2_VCC_INIT_CH));                                  // first to last bit corresponds to status of low to high ADC voltage channel of fpga2
        is_alarm_volt = 2;
        log_debug(LOG_ALM, "alarm volt at ADC ch : %02d now +/- %02d.%02d %% off target\r\n", ch, tens, frac); // over voltage among one of fpga power supplies by +/-getAlarmVoltageThres()*100% of its threshold
      }
    }
    else {
      if ((excess > threshold && excess > 0.0f) || (excess * -1.0f > threshold && excess < 0.0f)) { // if this ADC voltage is greater/lower than a target value by getAlarmVoltageThres()*100%
        fpga1_bitmask += (1 << (ch - ADC_INFO_FPGA_VCC_INIT_CH));                                   // first to last bit corresponds to status of low to high ADC voltage channel of fpga2
        is_alarm_volt = 2;
        log_debug(LOG_ALM, "alarm volt at ADC ch : %02d now +/- %02d.%02d %% off target\r\n", ch, tens, frac); // over voltage among one of fpga power supplies by +/-getAlarmVoltageThres()*100% of its threshold
      }
    }
  }

  if (is_alarm_volt > 0) {
    retval++;
    if (is_alarm_volt == 2) {
      status_V |= ALM_STAT_FPGA_OVERVOLT;
      ++retval;
    }
  }

  currentVoltStatus[FPGA1] = fpga1_bitmask;
  currentVoltStatus[FPGA2] = fpga2_bitmask;

  return retval;
}

void VoltErrorLog(void)
{
  int tens, frac;
  float_to_ints(excess_volt, &tens, &frac);
  log_warn(LOG_ALM, "Voltage high: status: 0x%04x at ADC ch %02d now +%02d.%02d %% off\r\n",
           status_V, excess_volt_which_ch, tens, frac);
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
