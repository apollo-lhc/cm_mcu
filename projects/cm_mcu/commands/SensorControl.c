/*
 * SensorControl.c
 *
 *  Created on: Jan 14, 2021
 *      Author: fatimayousuf
 */


#include <SensorControl.h>

// this command takes no arguments since there is only one command
// right now.
static BaseType_t sensor_summary(int argc, char **argv, char m)
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

// send power control commands
extern struct gpio_pin_t oks[];
static BaseType_t power_ctl(int argc, char **argv,  char m)
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
    bool ku_enable = (read_gpio_pin(TM4C_DIP_SW_1) == 1);
    bool vu_enable = (read_gpio_pin(TM4C_DIP_SW_2) == 1);
    static int i = 0;
    if (i == 0) {
      copied += snprintf(m + copied, SCRATCH_SIZE - copied,
                         "%s:\r\nVU_ENABLE:\t%d\r\n"
                         "KU_ENABLE:\t%d\r\n",
                         argv[0], vu_enable, ku_enable);
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
          snprintf(m + copied, SCRATCH_SIZE - copied, "%15s: %s\r\n", pin_names[oks[i].name], c);
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
static BaseType_t alarm_ctl(int argc, char **argv, char m)
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
    int32_t stat = getAlarmStatus();
    copied += snprintf(m + copied, SCRATCH_SIZE - copied, "Raw: 0x%08x\r\n", stat);

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
static BaseType_t led_ctl(int argc, char **argv, char m)
{

  BaseType_t i1 = strtol(argv[1], NULL, 10);
  configASSERT(i1 != 99);

  uint32_t message = HUH; // default: message not understood
  if (i1 == 0) {          // ToDo: make messages less clunky. break out color.
    message = RED_LED_OFF;
  }
  else if (i1 == 1) {
    message = RED_LED_ON;
  }
  else if (i1 == 2) {
    message = RED_LED_TOGGLE;
  }
  else if (i1 == 3) {
    message = RED_LED_TOGGLE3;
  }
  else if (i1 == 4) {
    message = RED_LED_TOGGLE4;
  }
  // Send a message to the LED task
  xQueueSendToBack(xLedQueue, &message, pdMS_TO_TICKS(10));
  m[0] = '\0'; // no output from this command

  return pdFALSE;
}

// this command takes no arguments
static BaseType_t adc_ctl(int argc, char **argv, char m)
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
static BaseType_t ff_ctl(int argc, char **argv, char m)
{
  // argument handling
  int copied = 0;
  static int whichff = 0;

  if (whichff == 0) {
    // check for stale data
    TickType_t now = pdTICKS_TO_MS(xTaskGetTickCount()) / 1000;
    TickType_t last = pdTICKS_TO_MS(getFFupdateTick()) / 1000;
    if ((now - last) > 60) {
      int mins = (now - last) / 60;
      copied += snprintf(m + copied, SCRATCH_SIZE - copied,
                         "%s: stale data, last update %d minutes ago\r\n", argv[0], mins);
    }
  }
  // parse command based on how many arguments it has
  if (argc == 1) { // default command: temps
    uint32_t ff_config = read_eeprom_single(EEPROM_ID_FF_ADDR);
    if (whichff == 0) {
      copied += snprintf(m + copied, SCRATCH_SIZE - copied, "FF temperatures\r\n");
    }
    for (; whichff < NFIREFLIES; ++whichff) {
      int8_t val = getFFtemp(whichff);
      const char *name = getFFname(whichff);
      if ((1 << whichff) & ff_config) // val > 0 )
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
      ;
      ;
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
        BaseType_t f = xQueueReceive(xFFlyQueueOut, &message, pdMS_TO_TICKS(500));
        if (f == pdTRUE)
          snprintf(m + copied, SCRATCH_SIZE - copied, "%s: Command returned 0x%x.\r\n", argv[0],
                   message & 0xFFU);
        else
          snprintf(m + copied, SCRATCH_SIZE - copied, "%s: Command failed.\r\n", argv[0]);
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

      } // end regw
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

static BaseType_t ff_status(int argc, char **argv, char m)
{
  int copied = 0;

  static int whichff = 0;
  if (whichff == 0) {
    copied += snprintf(m + copied, SCRATCH_SIZE - copied, "FIREFLY STATUS:\r\n");
  }
  for (; whichff < 25; ++whichff) {
    int8_t status = getFFstatus(whichff);
    const char *name = getFFname(whichff);
    copied += snprintf(m + copied, SCRATCH_SIZE - copied, "%s %02d", name, status);

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

static BaseType_t fpga_ctl(int argc, char **argv, char m)
{
  if (argc == 2) {
    if (strncmp(argv[1], "done", 4) == 0) { // print out value of done pins
      int ku_done_ = read_gpio_pin(_K_FPGA_DONE);
      int vu_done_ = read_gpio_pin(_V_FPGA_DONE);
      snprintf(m, SCRATCH_SIZE, "KU_DONE* = %d\r\nVU_DONE* = %d\r\n", ku_done_, vu_done_);
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
      if ((now - last) > 60) {
        int mins = (now - last) / 60;
        copied += snprintf(m + copied, SCRATCH_SIZE - copied,
                           "%s: stale data, last update %d minutes ago\r\n", argv[0], mins);
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

static BaseType_t fpga_reset(int argc, char **argv, char m)
{
  int copied = 0;
  const TickType_t delay = 1 / portTICK_PERIOD_MS; // 1 ms delay

  if (strcmp(argv[1], "v") == 0) {
    write_gpio_pin(V_FPGA_PROGRAM, 0x1);
    vTaskDelay(delay);
    write_gpio_pin(V_FPGA_PROGRAM, 0x0);
    copied += snprintf(m + copied, SCRATCH_SIZE - copied, "VU7P has been reset\r\n");
  }
  if (strcmp(argv[1], "k") == 0) {
    write_gpio_pin(K_FPGA_PROGRAM, 0x1);
    vTaskDelay(delay);
    write_gpio_pin(K_FPGA_PROGRAM, 0x0);
    copied += snprintf(m + copied, SCRATCH_SIZE - copied, "KU15P has been reset\r\n");
  }
  return pdFALSE;
}
