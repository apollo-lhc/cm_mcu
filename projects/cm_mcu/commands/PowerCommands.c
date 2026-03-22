/*
 * PowerCommands.c
 *
 * Power supply CLI command handlers, extracted from SensorControl.c
 * and CommandLineTask.c.
 */

#include <string.h>

#include "commands/PowerCommands.h"
#include "commands/parameters.h"
#include "common/utils.h"
#include "common/smbus_units.h"
#include "common/smbus_helper.h"
#include "MonUtils.h"
#include "Semaphore.h"
#include "Tasks.h"
#include "projdefs.h"

// dump monitor information
BaseType_t psmon_ctl(int argc, char **argv, char *m)
{
  BaseType_t i1 = strtol(argv[1], NULL, 10);

  if (i1 < 0 || i1 >= dcdc_args.n_commands) {
    snprintf(m, SCRATCH_SIZE, "%s: Invalid argument, must be between 0 and %d\r\n", argv[0],
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

// power control state names
static const char *const power_control_state_names[] = {
#define X(name) #name,
    X_MACRO_PS_STATES
#undef X
};

BaseType_t power_ctl(int argc, char **argv, char *m)
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
      copied += snprintf(m + copied, SCRATCH_SIZE - copied, "External Alarm: %d\r\n",
                         getPowerControlExternalAlarmState());
    }
    for (; i < N_PS_OKS; ++i) {
      enum ps_state j = getPSStatus(i);
      const char *c = power_control_state_names[j];

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
  else if (strncmp(argv[1], "igmask", 6) == 0) { // report ignore mask to UART
    int copied = 0;
    static int i = 0;
    uint16_t ignore_mask = getPowerControlIgnoreMask();
    for (; i < N_PS_OKS; ++i) {
      BaseType_t ignored = (ignore_mask & (0x1U << i)) != 0;
      copied += snprintf(m + copied, SCRATCH_SIZE - copied, "%-16s: %ld\r\n", oks[i].name, ignored);
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

extern struct MonitorTaskArgs_t dcdc_args;
extern struct dev_i2c_addr_t pm_addrs_dcdc[N_PM_ADDRS_DCDC];
extern struct pm_command_t extra_cmds[N_EXTRA_CMDS]; // LocalTasks.c

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
  if (which < 0 || which > (NSUPPLIES_PS - 1)) {
    copied += snprintf(m + copied, SCRATCH_SIZE - copied, "%s: device %d must be between 0-%d\r\n",
                       argv[0], which, (NSUPPLIES_PS - 1));
    return pdFALSE;
  }
  UBaseType_t regAddress = strtoul(argv[2], NULL, 16);
  copied += snprintf(m + copied, SCRATCH_SIZE - copied, "%s: page %d of device %s, reg 0x%02lx\r\n", argv[0],
                     page, pm_addrs_dcdc[which].name, regAddress);

  // acquire the semaphore
  if (acquireI2CSemaphore(dcdc_args.xSem) == pdFAIL) {
    snprintf(m + copied, SCRATCH_SIZE - copied, "%s: could not get semaphore in time\r\n", argv[0]);
    return pdFALSE;
  }
  uint8_t ui8page = page;
  // page register
  int r = apollo_pmbus_rw(&g_sMaster1, &eStatus1, false, &pm_addrs_dcdc[which], &extra_cmds[0], &ui8page);
  if (r) {
    Print("error in psmon_reg (page)\r\n");
  }
  // read register, 2 bytes
  uint8_t thevalue[2] = {0, 0};
  struct pm_command_t thecmd = {regAddress, 2, "dummy", "", PM_STATUS};
  r = apollo_pmbus_rw(&g_sMaster1, &eStatus1, true, &pm_addrs_dcdc[which], &thecmd, thevalue);
  if (r) {
    Print("error in psmon_reg (regr)\r\n");
  }
  uint16_t vv = (thevalue[0] | (thevalue[1] << 8));
  copied += snprintf(m + copied, SCRATCH_SIZE - copied, "%s: read value 0x%04x\r\n",
                     argv[0], vv);

  // release the semaphore
  if (xSemaphoreGetMutexHolder(dcdc_args.xSem) == xTaskGetCurrentTaskHandle()) {
    xSemaphoreGive(dcdc_args.xSem);
  }
  return pdFALSE;
}

typedef struct __attribute__((packed)) {
  linear11_val_t v_in;
  uint16_t v_out;
  linear11_val_t i_out;
  linear11_val_t i_out_max;
  linear11_val_t duty_cycle;
  linear11_val_t temperature;
  linear11_val_t unused1;
  linear11_val_t freq;
  uint8_t v_out_status;
  uint8_t i_out_status;
  uint8_t input_status;
  uint8_t temperature_status;
  uint8_t cml_status;
  uint8_t mfr_status;
  uint8_t flash_status;
  uint8_t unused[9];
} snapshot_t;

BaseType_t sn_all(int argc, char **argv, char *m)
{
  int which = 0;
  int page_d = 0;
  for (; which < N_PM_ADDRS_DCDC; ++which) {
    for (; page_d < 2; ++page_d) { // for reading two pages per device
      bool reset = false;
      reset = true;
      uint8_t sn[32];
      snapdump(&pm_addrs_dcdc[which], page_d, sn, reset);
    }
  }
  return pdFALSE;
}

BaseType_t snapshot(int argc, char **argv, char *m)
{
  _Static_assert(sizeof(snapshot_t) == 32, "sizeof snapshot_t");
  int copied = 0;
  int page = strtol(argv[1], NULL, 10); // which LGA08D
  int which = page / 10;
  page = page % 10;
  if (page < 0 || page > 1) {
    snprintf(m + copied, SCRATCH_SIZE - copied, "%s: page %d must be between 0-1\r\n",
             argv[0], page + 1);
    return pdFALSE;
  }
  if (which < 0 || which > (NSUPPLIES_PS - 1)) {
    snprintf(m + copied, SCRATCH_SIZE - copied, "%s: device %d must be between 0-%d\r\n",
             argv[0], which, (NSUPPLIES_PS - 1));
    return pdFALSE;
  }
  copied += snprintf(m + copied, SCRATCH_SIZE - copied, "%s: page %d of device %s\r\n", argv[0],
                     page, pm_addrs_dcdc[which].name);

  bool reset = false;
  int ireset = strtol(argv[2], NULL, 10);
  if (ireset == 1)
    reset = true;

  uint8_t sn[32];
  snapdump(&pm_addrs_dcdc[which], page, sn, reset);
  snapshot_t *p0 = (snapshot_t *)&sn[0];
  int tens, fraction;
  float_to_ints(linear11_to_float(p0->v_in), &tens, &fraction);
  copied += snprintf(m + copied, SCRATCH_SIZE - copied, "VIN  = %d.%02d\r\n", tens, fraction);
  float_to_ints(linear16u_to_float(p0->v_out), &tens, &fraction);
  copied += snprintf(m + copied, SCRATCH_SIZE - copied, "VOUT = %d.%02d\r\n", tens, fraction);
  float_to_ints(linear11_to_float(p0->i_out), &tens, &fraction);
  copied += snprintf(m + copied, SCRATCH_SIZE - copied, "IOUT = %d.%02d\r\n", tens, fraction);
  float_to_ints(linear11_to_float(p0->i_out_max), &tens, &fraction);
  copied += snprintf(m + copied, SCRATCH_SIZE - copied, "IOUT MAX = %d.%02d\r\n", tens, fraction);
  float_to_ints(linear11_to_float(p0->duty_cycle), &tens, &fraction);
  copied += snprintf(m + copied, SCRATCH_SIZE - copied, "duty cycle = %d.%02d\r\n", tens, fraction);
  float_to_ints(linear11_to_float(p0->temperature), &tens, &fraction);
  copied += snprintf(m + copied, SCRATCH_SIZE - copied, "TEMP = %d.%02d\r\n", tens, fraction);
  float_to_ints(linear11_to_float(p0->freq), &tens, &fraction);
  copied +=
      snprintf(m + copied, SCRATCH_SIZE - copied, "switching freq = %d.%02d\r\n", tens, fraction);
  copied +=
      snprintf(m + copied, SCRATCH_SIZE - copied, "VOUT  STATUS: 0x%02x\r\n", p0->v_out_status);
  copied +=
      snprintf(m + copied, SCRATCH_SIZE - copied, "IOUT  STATUS: 0x%02x\r\n", p0->i_out_status);
  copied +=
      snprintf(m + copied, SCRATCH_SIZE - copied, "INPUT STATUS: 0x%02x\r\n", p0->input_status);
  copied += snprintf(m + copied, SCRATCH_SIZE - copied, "TEMP  STATUS: 0x%02x\r\n",
                     p0->temperature_status);
  copied += snprintf(m + copied, SCRATCH_SIZE - copied, "CML   STATUS: 0x%02x\r\n", p0->cml_status);
  copied += snprintf(m + copied, SCRATCH_SIZE - copied, "MFR   STATUS: 0x%02x\r\n", p0->mfr_status);
  copied +=
      snprintf(m + copied, SCRATCH_SIZE - copied, "flash STATUS: 0x%02x\r\n", p0->flash_status);

  return pdFALSE;
}
