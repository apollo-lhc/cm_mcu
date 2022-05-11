/*
 * SensorControl.c
 *
 *  Created on: Jan 14, 2021
 *      Author: fatimayousuf
 */


#include "SensorControl.h"
#include "common/smbus_helper.h"

// this command takes no arguments since there is only one command
// right now.
BaseType_t sensor_summary(int argc, char **argv, char* m)
{
  int copied = 0;
  // collect all sensor information
  // highest temperature for each
  // Firefly
  // FPGA
  // DCDC
  // TM4C
  float tm4c_temp = getADCvalue(ADC_INFO_TEMP_ENTRY);
  int tens, frac;
  float_to_ints(tm4c_temp, &tens, &frac);
  copied += snprintf(m + copied, SCRATCH_SIZE - copied, "MCU %02d.%02d\r\n", tens, frac);
  // Fireflies. These are reported as ints but we are asked
  // to report a float.
  int8_t imax_temp = -99.0;
  for (int i = 0; i < NFIREFLIES; ++i) {
    int8_t v = getFFtemp(i);
    if (v > imax_temp)
      imax_temp = v;
  }
  copied += snprintf(m + copied, SCRATCH_SIZE - copied, "FIREFLY %02d.0\r\n", imax_temp);
  // FPGAs.
  float max_fpga;
  if (fpga_args.n_devices == 2)
    max_fpga = MAX(fpga_args.pm_values[0], fpga_args.pm_values[1]);
  else
    max_fpga = fpga_args.pm_values[0];
  float_to_ints(max_fpga, &tens, &frac);
  copied += snprintf(m + copied, SCRATCH_SIZE - copied, "FPGA %02d.%02d\r\n", tens, frac);

  // DCDC. The first command is READ_TEMPERATURE_1.
  // I am assuming it stays that way!!!!!!!!
  float max_temp = -99.0;
  for (int ps = 0; ps < dcdc_args.n_devices; ++ps) {
    for (int page = 0; page < dcdc_args.n_pages; ++page) {
      float thistemp = dcdc_args.pm_values[ps * (dcdc_args.n_commands * dcdc_args.n_pages) +
                                           page * dcdc_args.n_commands + 0];
      if (thistemp > max_temp)
        max_temp = thistemp;
    }
  }
  float_to_ints(max_temp, &tens, &frac);
  copied += snprintf(m + copied, SCRATCH_SIZE - copied, "REG %02d.%02d\r\n", tens, frac);

  return pdFALSE;
}

// dump monitor information
BaseType_t psmon_ctl(int argc, char **argv, char* m)
{
  int s = SCRATCH_SIZE;
  BaseType_t i1 = strtol(argv[1], NULL, 10);

  if (i1 < 0 || i1 >= dcdc_args.n_commands) {
    snprintf(m, s, "%s: Invalid argument, must be between 0 and %d\r\n", argv[0],
             dcdc_args.n_commands - 1);
    return pdFALSE;
  }
  // update times, in seconds
  TickType_t now = pdTICKS_TO_MS(xTaskGetTickCount()) / 1000;
  TickType_t last = pdTICKS_TO_MS(dcdc_args.updateTick) / 1000;
  int copied = 0;
  if (checkStale(last, now)) {
    int mins = (now - last) / 60;
    copied += snprintf(m + copied, SCRATCH_SIZE - copied,
                       "%s: stale data, last update %d minutes ago\r\n", argv[0], mins);
  }
  copied += snprintf(m + copied, SCRATCH_SIZE - copied, "%s (0x%02x)\r\n",
      dcdc_args.commands[i1].name, dcdc_args.commands[i1].command);
  for (int ps = 0; ps < dcdc_args.n_devices; ++ps) {
    copied +=
        snprintf(m + copied, SCRATCH_SIZE - copied, "SUPPLY %s\r\n", dcdc_args.devices[ps].name);
    for (int page = 0; page < dcdc_args.n_pages; ++page) {
      float val = dcdc_args.pm_values[ps * (dcdc_args.n_commands * dcdc_args.n_pages) +
                                      page * dcdc_args.n_commands + i1];
      int tens, frac;
      float_to_ints(val, &tens, &frac);
      copied += snprintf(m + copied, SCRATCH_SIZE - copied, "VALUE %02d.%02d\t", tens, frac);
    }
    copied += snprintf(m + copied, SCRATCH_SIZE - copied, "\r\n");
  }

  return pdFALSE;
}

// send power control commands
extern struct gpio_pin_t oks[];
BaseType_t power_ctl(int argc, char **argv, char* m)
{
  int s = SCRATCH_SIZE;

  uint32_t message;
  if (strncmp(argv[1], "on", 2) == 0) {
    message = PS_ON; // turn on power supply
  }
  else if (strncmp(argv[1], "off", 3) == 0) {
    message = PS_OFF; // turn off power supply
  }
  else if (strncmp(argv[1], "clearfail", 9) == 0) {
    message = PS_ANYFAIL_ALARM_CLEAR;
  }
  else if (strncmp(argv[1], "status", 5) == 0) { // report status to UART
    int copied = 0;
    bool f1_enable = (isFPGAF1_PRESENT());
    bool f2_enable = (isFPGAF2_PRESENT());
    static int i = 0;
    if (i == 0) {
      copied += snprintf(m + copied, SCRATCH_SIZE - copied,
                         "%s:\r\nF1_ENABLE:\t%d\r\n"
                         "F2_ENABLE:\t%d\r\n",
                         argv[0], f1_enable, f2_enable);
      copied += snprintf(m + copied, SCRATCH_SIZE - copied, "State machine state: %s\r\n",
                         getPowerControlStateName(getPowerControlState()));
    }
    for (; i < N_PS_OKS; ++i) {
      enum ps_state j = getPSStatus(i);
      char *c;
      switch (j) {
        case PWR_UNKNOWN:
          c = "PWR_UNKNOWN";
          break;
        case PWR_ON:
          c = "PWR_ON";
          break;
        case PWR_OFF:
          c = "PWR_OFF";
          break;
        case PWR_DISABLED:
          c = "PWR_DISABLED";
          break;
        case PWR_FAILED:
          c = "PWR_FAILED";
          break;
        default:
          c = "UNKNOWN";
          break;
      }

      copied +=
          snprintf(m + copied, SCRATCH_SIZE - copied, "%16s: %s\r\n", oks[i].name, c);
      if ((SCRATCH_SIZE - copied) < 20 && (i < N_PS_OKS)) {
        ++i;
        return pdTRUE;
      }
    }
    i = 0;
    return pdFALSE;
  }
  else {
    snprintf(m, s, "power_ctl: invalid argument %s received\r\n", argv[1]);
    return pdFALSE;
  }
  // Send a message to the power supply task, if needed
  xQueueSendToBack(xPwrQueue, &message, pdMS_TO_TICKS(10));
  m[0] = '\0'; // no output from this command

  return pdFALSE;
}

// takes 1-2 arguments
BaseType_t alarm_ctl(int argc, char **argv, char* m)
{
  int s = SCRATCH_SIZE;
  if (argc < 2) {
    snprintf(m, s, "%s: need one or more arguments\r\n", argv[0]);
    return pdFALSE;
  }

  uint32_t message;
  if (strncmp(argv[1], "clear", 4) == 0) {
    message = ALM_CLEAR_ALL; // clear all alarms
    xQueueSendToBack(tempAlarmTask.xAlmQueue, &message, pdMS_TO_TICKS(10));
    m[0] = '\0'; // no output from this command

    return pdFALSE;
  }
  else if (strncmp(argv[1], "status", 5) == 0) { // report status to UART
    int copied = 0;
    copied += snprintf(m + copied, SCRATCH_SIZE - copied, "%s: ALARM status\r\n", argv[0]);
    uint32_t stat = getAlarmStatus();
    copied += snprintf(m + copied, SCRATCH_SIZE - copied, "Raw: 0x%08lx\r\n", stat);

    float ff_val = getAlarmTemperature(FF);
    int tens, frac;
    float_to_ints(ff_val, &tens, &frac);
    copied +=
        snprintf(m + copied, SCRATCH_SIZE - copied, "TEMP FFLY: %s \t Threshold: %02d.%02d\r\n",
                 (stat & ALM_STAT_FIREFLY_OVERTEMP) ? "ALARM" : "GOOD", tens, frac);

    float fpga_val = getAlarmTemperature(FPGA);
    float_to_ints(fpga_val, &tens, &frac);
    copied +=
        snprintf(m + copied, SCRATCH_SIZE - copied, "TEMP FPGA: %s \t Threshold: %02d.%02d\r\n",
                 (stat & ALM_STAT_FPGA_OVERTEMP) ? "ALARM" : "GOOD", tens, frac);

    float dcdc_val = getAlarmTemperature(DCDC);
    float_to_ints(dcdc_val, &tens, &frac);
    copied +=
        snprintf(m + copied, SCRATCH_SIZE - copied, "TEMP DCDC: %s \t Threshold: %02d.%02d\r\n",
                 (stat & ALM_STAT_DCDC_OVERTEMP) ? "ALARM" : "GOOD", tens, frac);

    float tm4c_val = getAlarmTemperature(TM4C);
    float_to_ints(tm4c_val, &tens, &frac);
    copied +=
        snprintf(m + copied, SCRATCH_SIZE - copied, "TEMP TM4C: %s \t Threshold: %02d.%02d\r\n",
                 (stat & ALM_STAT_TM4C_OVERTEMP) ? "ALARM" : "GOOD", tens, frac);
    configASSERT(copied < SCRATCH_SIZE);

    return pdFALSE;
  }
  else if (strcmp(argv[1], "settemp") == 0) {
    if (argc != 4) {
      snprintf(m, s, "Invalid command\r\n");
      return pdFALSE;
    }
    float newtemp = (float)strtol(argv[3], NULL, 10);
    char *device = argv[2];
    if (!strcmp(device, "ff")) {
      setAlarmTemperature(FF, newtemp);
      snprintf(m, s, "%s: set Firefly alarm temperature to %s\r\n", argv[0], argv[3]);
      return pdFALSE;
    }
    if (!strcmp(device, "fpga")) {
      setAlarmTemperature(FPGA, newtemp);
      snprintf(m, s, "%s: set FPGA alarm temperature to %s\r\n", argv[0], argv[3]);
      return pdFALSE;
    }
    if (!strcmp(device, "dcdc")) {
      setAlarmTemperature(DCDC, newtemp);
      snprintf(m, s, "%s: set DCDC alarm temperature to %s\r\n", argv[0], argv[3]);
      return pdFALSE;
    }
    if (!strcmp(device, "tm4c")) {
      setAlarmTemperature(TM4C, newtemp);
      snprintf(m, s, "%s: set TM4C alarm temperature to %s\r\n", argv[0], argv[3]);
      return pdFALSE;
    }
    else {
      snprintf(m, s, "%s is not a valid device.\r\n", argv[2]);
      return pdFALSE;
    }
  }
  else {
    snprintf(m, s, "%s: invalid argument %s received\r\n", argv[0], argv[1]);
    return pdFALSE;
  }
  return pdFALSE;
}

// send LED commands
BaseType_t led_ctl(int argc, char **argv, char* m)
{

  BaseType_t i1 = strtol(argv[1], NULL, 10);

  BaseType_t ones = i1%10;
  BaseType_t tens = i1/10; // integer truncation

  uint32_t message = HUH; // default: message not understood
  if ( ones < 5 && tens>0 && tens<4) {
    message = i1;
  }
  // Send a message to the LED task
  xQueueSendToBack(xLedQueue, &message, pdMS_TO_TICKS(10));
  m[0] = '\0'; // no output from this command

  return pdFALSE;
}

// this command takes no arguments
BaseType_t adc_ctl(int argc, char **argv, char* m)
{
  int copied = 0;

  static int whichadc = 0;
  if (whichadc == 0) {
    copied += snprintf(m + copied, SCRATCH_SIZE - copied, "ADC outputs\r\n");
  }
  for (; whichadc < 21; ++whichadc) {
    float val = getADCvalue(whichadc);
    int tens, frac;
    float_to_ints(val, &tens, &frac);
    copied += snprintf(m + copied, SCRATCH_SIZE - copied, "%14s: %02d.%02d\r\n",
                       getADCname(whichadc), tens, frac);
    if ((SCRATCH_SIZE - copied) < 20 && (whichadc < 20)) {
      ++whichadc;
      return pdTRUE;
    }
  }
  whichadc = 0;
  return pdFALSE;
}

// this command takes up to two arguments
BaseType_t ff_ctl(int argc, char **argv, char* m)
{
  // argument handling
  int copied = 0;
  static int whichff = 0;

  if (whichff == 0) {
    // check for stale data
    TickType_t now = pdTICKS_TO_MS(xTaskGetTickCount()) / 1000;
    TickType_t last = pdTICKS_TO_MS(getFFupdateTick()) / 1000;
    if (checkStale(last, now)) {
      int mins = (now - last) / 60;
      copied += snprintf(m + copied, SCRATCH_SIZE - copied,
                         "%s: stale data, last update %d minutes ago\r\n", argv[0], mins);
    }
  }
  // parse command based on how many arguments it has
  if (argc == 1) { // default command: temps
    if (whichff == 0) {
      copied += snprintf(m + copied, SCRATCH_SIZE - copied, "FF temperatures\r\n");
    }
    for (; whichff < NFIREFLIES; ++whichff) {
      int8_t val = getFFtemp(whichff);
      const char *name = getFFname(whichff);
      if (isEnabledFF(whichff)) // val > 0 )
        copied += snprintf(m + copied, SCRATCH_SIZE - copied, "%17s: %2d", name, val);
      else // dummy value
        copied += snprintf(m + copied, SCRATCH_SIZE - copied, "%17s: %2s", name, "--");
      bool isTx = (strstr(name, "Tx") != NULL);
      if (isTx)
        copied += snprintf(m + copied, SCRATCH_SIZE - copied, "\t");
      else
        copied += snprintf(m + copied, SCRATCH_SIZE - copied, "\r\n");
      if ((SCRATCH_SIZE - copied) < 20) {
        ++whichff;
        return pdTRUE;
      }
    }
    if (whichff % 2 == 1) {
      m[copied++] = '\r';
      m[copied++] = '\n';
      m[copied] = '\0';
    }
    whichff = 0;
  } // argc == 1
  else if ( argc == 2 ) {
    uint8_t code;
    if (strncmp(argv[1], "suspend", 4) == 0) {
     code = FFLY_SUSPEND;
    }
    else if (strncmp(argv[1], "resume", 4) == 0) {
      code = FFLY_RESUME;
    }
    else {
      snprintf(m+copied, SCRATCH_SIZE-copied, "%s: %s not understood", argv[0], argv[1]);
      return pdFALSE;
    }
    uint32_t message = (code & FF_MESSAGE_CODE_MASK) << FF_MESSAGE_CODE_OFFSET;
    xQueueSendToBack(xFFlyQueueIn, &message, pdMS_TO_TICKS(10));
    snprintf(m + copied, SCRATCH_SIZE - copied, "%s: command %s  sent.\r\n", argv[0], argv[1]);

  }
  else {
    int whichFF = 0;
    // handle the channel number first
    if (strncmp(argv[argc - 1], "all", 3) == 0) {
      whichFF = NFIREFLIES;
    }
    else { // commands with arguments. The last argument is always which FF module.
      whichFF = strtol(argv[argc - 1], NULL, 10);
      if (whichFF >= NFIREFLIES || (whichFF == 0 && strncmp(argv[argc - 1], "0", 1) != 0)) {
        snprintf(m + copied, SCRATCH_SIZE - copied, "%s: choose ff number less than %d\r\n",
                 argv[0], NFIREFLIES);
        return pdFALSE;
      }
    }
    // now process various commands.
    if (argc == 4) { // command + three arguments
      bool receiveAnswer = false;
      uint8_t code;
      uint32_t data = (whichFF & FF_MESSAGE_CODE_REG_FF_MASK) << FF_MESSAGE_CODE_REG_FF_OFFSET;
      if (strncmp(argv[1], "cdr", 3) == 0) {
        code = FFLY_DISABLE_CDR; // default: disable
        if (strncmp(argv[2], "on", 2) == 0) {
          code = FFLY_ENABLE_CDR;
        }
      }
      else if (strncmp(argv[1], "xmit", 4) == 0) {
        code = FFLY_DISABLE_TRANSMITTER;
        if (strncmp(argv[2], "on", 2) == 0) {
          code = FFLY_ENABLE_TRANSMITTER;
        }
      }
      else if (strncmp(argv[1], "rcvr", 4) == 0) {
        code = FFLY_DISABLE;
        if (strncmp(argv[2], "on", 2) == 0) {
          code = FFLY_ENABLE;
        }
      }
      // Add here
      else if (strncmp(argv[1], "regr", 4) == 0) {
        code = FFLY_READ_REGISTER;
        receiveAnswer = true;
        if (whichFF == NFIREFLIES) {
          copied += snprintf(m + copied, SCRATCH_SIZE - copied, "%s: cannot read all registers\r\n",
                             argv[0]);
          return pdFALSE;
        }
        // register number
        uint8_t regnum = strtol(argv[2], NULL, 16);
        copied +=
            snprintf(m + copied, SCRATCH_SIZE - copied, "%s: reading FF %s, register 0x%x\r\n",
                     argv[0], getFFname(whichFF), regnum);
        // pack channel and register into the data
        data |= ((regnum & FF_MESSAGE_CODE_REG_REG_MASK) << FF_MESSAGE_CODE_REG_REG_OFFSET);
      }
      else {
        snprintf(m + copied, SCRATCH_SIZE - copied, "%s: command %s not recognized\r\n", argv[0],
                 argv[1]);
        return pdFALSE;
      }
      uint32_t message =
          ((code & FF_MESSAGE_CODE_MASK) << FF_MESSAGE_CODE_OFFSET) | (data & FF_MESSAGE_DATA_MASK);
      xQueueSendToBack(xFFlyQueueIn, &message, pdMS_TO_TICKS(10));
      snprintf(m + copied, SCRATCH_SIZE - copied, "%s: command %s %s sent.\r\n", argv[0], argv[1],
               argv[2]);
      if (receiveAnswer) {
        BaseType_t f = xQueueReceive(xFFlyQueueOut, &message, pdMS_TO_TICKS(5000));
        if (f == pdTRUE) {
          uint8_t retcode = (message>>24) & 0xFFU;
          uint8_t value = message & 0xFFU;
          copied += snprintf(m + copied, SCRATCH_SIZE - copied,
                   "%s: Command returned 0x%x (ret %d - \"%s\").\r\n", argv[0], value,
                   retcode, SMBUS_get_error(retcode));
          UBaseType_t n = uxQueueMessagesWaiting(xFFlyQueueOut);
          if ( n > 0 ) {
            copied += snprintf(m+copied, SCRATCH_SIZE-copied,
                "%s: still have %lu messages in the queue\r\n", argv[0], n);
          }
        }
        else
          snprintf(m + copied, SCRATCH_SIZE - copied, "%s: Command failed (queue).\r\n", argv[0]);
      }
    }                     // argc == 4
    else if (argc == 5) { // command + five arguments
      // register write. model:
      // ff regw reg# val (0-23|all)
      // register read/write commands
      if (strncmp(argv[1], "regw", 4) == 0) {
        uint8_t code = FFLY_WRITE_REGISTER;
        // the two additional arguments
        // register number
        // value to be written
        uint8_t regnum = strtol(argv[2], NULL, 16);
        uint16_t value = strtol(argv[3], NULL, 16);
        uint8_t channel = whichFF;
        if (channel == NFIREFLIES) {
          channel = 0; // silently fall back to first channel
        }
        // pack channel, register and value into the data
        uint32_t data =
            ((regnum & FF_MESSAGE_CODE_REG_REG_MASK) << FF_MESSAGE_CODE_REG_REG_OFFSET) |
            ((value & FF_MESSAGE_CODE_REG_DAT_MASK) << FF_MESSAGE_CODE_REG_DAT_OFFSET) |
            ((channel & FF_MESSAGE_CODE_REG_FF_MASK) << FF_MESSAGE_CODE_REG_FF_OFFSET);
        uint32_t message = ((code & FF_MESSAGE_CODE_MASK) << FF_MESSAGE_CODE_OFFSET) |
                           ((data & FF_MESSAGE_DATA_MASK) << FF_MESSAGE_DATA_OFFSET);
        xQueueSendToBack(xFFlyQueueIn, &message, pdMS_TO_TICKS(10));
        snprintf(m + copied, SCRATCH_SIZE - copied,
                 "%s: write val 0x%x to register 0x%x, FF %d.\r\n", argv[0], value, regnum,
                 channel);
        whichFF = 0;
      } // end regw
      else if (strncmp(argv[1], "test", 4) == 0) { // test code
        uint8_t code = FFLY_TEST_READ;
        if (whichFF == NFIREFLIES) {
          copied += snprintf(m + copied, SCRATCH_SIZE - copied,
                             "%s: cannot test read all registers\r\n", argv[0]);
          return pdFALSE;
        }
        uint8_t regnum = strtol(argv[2], NULL, 16);   // which register
        uint8_t charsize = strtol(argv[3], NULL, 10); // how big
        uint32_t message =
            ((code & FF_MESSAGE_CODE_MASK) << FF_MESSAGE_CODE_OFFSET) |
            (((regnum & FF_MESSAGE_CODE_TEST_REG_MASK) << FF_MESSAGE_CODE_TEST_REG_OFFSET) |
             ((charsize & FF_MESSAGE_CODE_TEST_SIZE_MASK) << FF_MESSAGE_CODE_TEST_SIZE_OFFSET) |
             ((whichFF & FF_MESSAGE_CODE_TEST_FF_MASK) << FF_MESSAGE_CODE_TEST_FF_OFFSET));
        xQueueSendToBack(xFFlyQueueIn, &message, pdMS_TO_TICKS(10));
        snprintf(m + copied, SCRATCH_SIZE - copied, "%s: command %s sent.\r\n", argv[0], argv[1]);
        whichFF = 0;
      }
      else {
        snprintf(m + copied, SCRATCH_SIZE - copied, "%s: command %s not understood\r\n", argv[0],
                 argv[1]);
        return pdFALSE;
      }
    } // argc == 5
    else {
      snprintf(m + copied, SCRATCH_SIZE - copied, "%s: command %s not understood\r\n", argv[0],
               argv[1]);
      return pdFALSE;
    }
  }
  return pdFALSE;
}

BaseType_t ff_status(int argc, char **argv, char* m)
{
  int copied = 0;

  static int whichff = 0;
  if (whichff == 0) {
    copied += snprintf(m + copied, SCRATCH_SIZE - copied, "FIREFLY STATUS:\r\n");
  }
  for (; whichff < NFIREFLIES; ++whichff) {

    const char *name = getFFname(whichff);
    if (!isEnabledFF(whichff)){
      copied+=snprintf(m + copied, SCRATCH_SIZE - copied, "%17s   --", name);
    }
    else {
      uint8_t status = getFFstatus(whichff);
      copied += snprintf(m + copied, SCRATCH_SIZE - copied, "%17s 0x%02x ", name, status);
    }

    bool isTx = (strstr(name, "Tx") != NULL);
    if (isTx)
      copied += snprintf(m + copied, SCRATCH_SIZE - copied, "\t");
    else
      copied += snprintf(m + copied, SCRATCH_SIZE - copied, "\r\n");

    if ((SCRATCH_SIZE - copied) < 20 && (whichff < 25)) {
      ++whichff;
      return pdTRUE;
    }
  }
  whichff = 0;
  return pdFALSE;
}

BaseType_t ff_los_alarm(int argc, char **argv, char* m) {
  int copied = 0;

  static int whichff = 0;
  if (whichff == 0) {
    copied += snprintf(m + copied, SCRATCH_SIZE - copied, "FIREFLY LOS ALARM:\r\n");
  }
  for (; whichff < NFIREFLIES; ++whichff) {
    const char *name = getFFname(whichff);
    copied += snprintf(m + copied, SCRATCH_SIZE - copied, "%s ", name);
    if (!isEnabledFF(whichff)){
      copied+=snprintf(m + copied, SCRATCH_SIZE - copied, "------------");
    }
    else{
      for (size_t i = 0; i<8; i++) {
        int alarm = getFFlos(whichff, i)? 1:0;
        copied+=snprintf(m + copied, SCRATCH_SIZE - copied, "%d", alarm);
      }
      if (strstr(name, "XCVR") == NULL) {
        for (size_t i = 8; i<12; i++) {
          int alarm = getFFlos(whichff, i)? 1:0;
          copied+=snprintf(m + copied, SCRATCH_SIZE - copied, "%d", alarm);
        }
      }
      else{
        copied+=snprintf(m + copied, SCRATCH_SIZE - copied, "    ");
      }

    }

    bool isTx = (strstr(name, "Tx") != NULL);
    if (isTx)
      copied += snprintf(m + copied, SCRATCH_SIZE - copied, "\t");
    else
      copied += snprintf(m + copied, SCRATCH_SIZE - copied, "\r\n");

    if ((SCRATCH_SIZE - copied) < 20 && (whichff < 25)) {
      ++whichff;
      return pdTRUE;
    }
  }
  whichff = 0;
  return pdFALSE;
}

BaseType_t ff_cdr_lol_alarm(int argc, char **argv, char* m) {
  int copied = 0;

  static int whichff = 0;
  if (whichff == 0) {
    copied += snprintf(m + copied, SCRATCH_SIZE - copied, "FIREFLY CDR LOL ALARM:\r\n");
  }
  for (; whichff < NFIREFLIES; ++whichff) {
    const char *name = getFFname(whichff);
    copied += snprintf(m + copied, SCRATCH_SIZE - copied, "%s ", name);
    if (!isEnabledFF(whichff)){
      copied+=snprintf(m + copied, SCRATCH_SIZE - copied, "------------");
    }
    else{
      for (size_t i = 0; i<8; i++) {
        int alarm = getFFlol(whichff, i)? 1:0;
        copied+=snprintf(m + copied, SCRATCH_SIZE - copied, "%d", alarm);
      }
      if (strstr(name, "XCVR") == NULL) {
        for (size_t i = 8; i<12; i++) {
          int alarm = getFFlol(whichff, i)? 1:0;
          copied+=snprintf(m + copied, SCRATCH_SIZE - copied, "%d", alarm);
        }
      }
      else{
        copied+=snprintf(m + copied, SCRATCH_SIZE - copied, "    ");
      }
    }

    bool isTx = (strstr(name, "Tx") != NULL);
    if (isTx)
      copied += snprintf(m + copied, SCRATCH_SIZE - copied, "\t");
    else
      copied += snprintf(m + copied, SCRATCH_SIZE - copied, "\r\n");

    if ((SCRATCH_SIZE - copied) < 20 && (whichff < 25)) {
      ++whichff;
      return pdTRUE;
    }
  }
  whichff = 0;
  return pdFALSE;
}

BaseType_t fpga_ctl(int argc, char **argv, char* m)
{
  if (argc == 2) {
    if (strncmp(argv[1], "done", 4) == 0) { // print out value of done pins
      int f1_done_ = read_gpio_pin(_F1_FPGA_DONE);
      int f2_done_ = read_gpio_pin(_F2_FPGA_DONE);
      snprintf(m, SCRATCH_SIZE, "F1_DONE* = %d\r\nF2_DONE* = %d\r\n", f1_done_, f2_done_);
      return pdFALSE;
    }
    else {
      snprintf(m, SCRATCH_SIZE, "%s: invalid command %s\r\n", argv[0], argv[1]);
      return pdFALSE;
    }
  }
  else if (argc != 1) {
    // error, invalid
    snprintf(m, SCRATCH_SIZE, "%s: invalid argument count %d\r\n", argv[0], argc);
    return pdFALSE;
  }
  else {
    int copied = 0;
    static int whichfpga = 0;
    int howmany = fpga_args.n_devices * fpga_args.n_pages;
    if (whichfpga == 0) {
      TickType_t now = pdTICKS_TO_MS(xTaskGetTickCount()) / 1000;
      TickType_t last = pdTICKS_TO_MS(getFFupdateTick()) / 1000;
      if (checkStale(last, now)) {
        int mins = (now - last) / 60;
        copied += snprintf(m + copied, SCRATCH_SIZE - copied,
                           "%s: stale data, last update %d minutes ago (%ld, %ld)\r\n", argv[0], mins,
                           now, last);
      }

      copied += snprintf(m + copied, SCRATCH_SIZE - copied, "FPGA monitors\r\n");
      copied += snprintf(m + copied, SCRATCH_SIZE - copied, "%s\r\n", fpga_args.commands[0].name);
    }

    for (; whichfpga < howmany; ++whichfpga) {
      float val = fpga_args.pm_values[whichfpga];
      int tens, frac;
      float_to_ints(val, &tens, &frac);

      copied += snprintf(m + copied, SCRATCH_SIZE - copied, "%5s: %02d.%02d",
                         fpga_args.devices[whichfpga].name, tens, frac);
      if (whichfpga % 2 == 1)
        copied += snprintf(m + copied, SCRATCH_SIZE - copied, "\r\n");
      else
        copied += snprintf(m + copied, SCRATCH_SIZE - copied, "\t");
      if ((SCRATCH_SIZE - copied) < 20) {
        ++whichfpga;
        return pdTRUE;
      }
    }
    if (whichfpga % 2 == 1) {
      m[copied++] = '\r';
      m[copied++] = '\n';
      m[copied] = '\0';
    }
    whichfpga = 0;
    return pdFALSE;
  }
}

// This command takes 1 argument, either k or v
BaseType_t fpga_reset(int argc, char **argv, char* m)
{
  int copied = 0;
  const TickType_t delay = 1 / portTICK_PERIOD_MS; // 1 ms delay

  if (strcmp(argv[1], "f2") == 0) {
    write_gpio_pin(F2_FPGA_PROGRAM, 0x1);
    vTaskDelay(delay);
    write_gpio_pin(F2_FPGA_PROGRAM, 0x0);
    copied += snprintf(m + copied, SCRATCH_SIZE - copied, "VU7P has been reset\r\n");
  }
  if (strcmp(argv[1], "f1") == 0) {
    write_gpio_pin(F1_FPGA_PROGRAM, 0x1);
    vTaskDelay(delay);
    write_gpio_pin(F1_FPGA_PROGRAM, 0x0);
    copied += snprintf(m + copied, SCRATCH_SIZE - copied, "KU15P has been reset\r\n");
  }
  return pdFALSE;
}

extern struct MonitorTaskArgs_t dcdc_args;
extern struct dev_i2c_addr_t pm_addrs_dcdc[];
extern struct pm_command_t extra_cmds[]; // LocalTasks.c

// Read out registers from LGA80D
BaseType_t psmon_reg(int argc, char **argv, char *m)
{
  int copied = 0;
  int page = strtol(argv[1], NULL, 10); // which supply within the LGA08D
  int which = page / 10;
  page = page % 10;
  if (page < 0 || page > 1) {
    copied += snprintf(m + copied, SCRATCH_SIZE - copied, "%s: page %d must be between 0-1\r\n",
                       argv[0], page);
    return pdFALSE;
  }
  if (which < 0 || which > (NSUPPLIES_PS-1)) {
    copied += snprintf(m + copied, SCRATCH_SIZE - copied, "%s: device %d must be between 0-%d\r\n",
                       argv[0], which, (NSUPPLIES_PS-1));
    return pdFALSE;
  }
  UBaseType_t regAddress  = strtoul(argv[2], NULL, 16);
  copied += snprintf(m + copied, SCRATCH_SIZE - copied, "%s: page %d of device %s, reg 0x%02lx\r\n", argv[0],
                     page, pm_addrs_dcdc[which].name, regAddress);



  // acquire the semaphore
  while (xSemaphoreTake(dcdc_args.xSem, (TickType_t)10) == pdFALSE)
    ;
  uint8_t ui8page = page;
  // page register
  int r = apollo_pmbus_rw(&g_sMaster1, &eStatus1, false, &pm_addrs_dcdc[which], &extra_cmds[0], &ui8page);
  if (r) {
    Print("error in psmon_reg (page)\r\n");
  }
  // read register, 2 bytes
  uint8_t thevalue[2] = {0,0};
  struct pm_command_t thecmd = {regAddress, 2, "dummy", "", PM_STATUS};
  r = apollo_pmbus_rw(&g_sMaster1, &eStatus1, true, &pm_addrs_dcdc[which], &thecmd, thevalue);
  if (r) {
    Print("error in psmon_reg (regr)\r\n");
  }
  uint16_t vv = (thevalue[0] | (thevalue[1]<<8));
  copied += snprintf(m + copied, SCRATCH_SIZE - copied, "%s: read value 0x%04x\r\n",
      argv[0], vv);

  // release the semaphore
  xSemaphoreGive(dcdc_args.xSem);
  return pdFALSE;

}

