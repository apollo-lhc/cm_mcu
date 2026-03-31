/*
 * CommandLineTask.c
 *
 *  Created on: Apr 7, 2019
 *      Author: wittich, rzou
 */

// Include commands
#include <strings.h>
#include "commands/AlarmCommands.h"
#include "commands/BoardCommands.h"
#include "commands/BufferCommands.h"
#include "commands/ClockCommands.h"
#include "commands/EEPROMCommands.h"
#include "commands/FireflyCommands.h"
#include "commands/FPGACommands.h"
#include "commands/I2CCommands.h"
#include "commands/PowerCommands.h"
#include "commands/SensorControl.h"
#include "commands/SoftwareCommands.h"
#include "commands/ZynqCommands.h"
#include "Semaphore.h"

static char m[SCRATCH_SIZE];

static BaseType_t help_command_fcn(int argc, char **, char *m);

////////////////////////////////////////////////////////////////////////

static const char *const pcWelcomeMessage =
    "CLI based on microrl.\r\nType \"help\" to view a list of registered commands.\r\n";

struct command_t {
  const char *commandstr;
  BaseType_t (*interpreter)(int argc, char **, char *m);
  const char *helpstr;
  const int num_args;
};

#define NUM_COMMANDS (sizeof(commands) / sizeof(commands[0]))
static struct command_t commands[] = {
    {"adc", adc_ctl, "Display ADC measurements\r\n", 0},
    {"alm", alarm_ctl,
     "args: clear|status|settemp|resettemp|setvoltthres|#\r\n"
     "  status  -- show alarms/thresh\r\n"
     "  settemp [ff|fpga|dcdc|tm4c] T -- set temp thresh T C (EEPROM)\r\n"
     "  resettemp [ff|fpga|dcdc|tm4c|all] -- reset temp thresh default\r\n"
     "  setvoltthres PCT -- set volt alarm +/-PCT%%\r\n"
     "  clear -- clear alarm state\r\n",
     -1},
    {"bootloader", bl_ctl, "Call bootloader\r\n", 0},
#if defined(REV2) || defined(REV3)
    {"clearclk", clearclk_ctl,
     "Reset clk sticky bits\r\n", 0},
    {"clk_freq_fpga", clk_freq_fpga_cmd, "read out special FPGA i2c regs\r\n", 2}, // need to call with which fpga you are checking
    {"clkmon", clkmon_ctl, "CLK chips' status, id:0-4\r\n", 1},
    {"clkname", clk_prog_name, "CLK chip program, id:0-4\r\n", 1},
    {"clkprog", init_load_clock_ctl, "args: <id> <reset>\r\nLoad clock chip program, id:0-4\r\n", 2},
    {"clkreset", clk_reset, "Reset clock chip, id:0-4\r\n", 1},
#endif // REV2
    {"eeprom_info", eeprom_info, "Prints information about the EEPROM\r\n", 0},
    {"eeprom_read", eeprom_read,
     "args: <address>\r\nReads 4 bytes from EEPROM. Address should be a multiple of 4\r\n",
     1},
    {"eeprom_write", eeprom_write,
     "args: <address> <data>\r\nWrites <data> to <address> in EEPROM. <address> should be "
     "a multiple of 4\r\n",
     2},
    {"errorlog_entry", errbuff_in,
     "args: <data>\r\nManual entry of 2-byte code into the eeprom error logger\r\n", 1},
    {"errorlog", errbuff_out,
     "args: <n>\r\nPrints last n entries in the eeprom error logger\r\n", 1},
    {"errorlog_info", errbuff_info,
     "Prints information about the eeprom error logger\r\n", 0},
    {"errorlog_reset", errbuff_reset,
     "Resets the eeprom error logger\r\n", 0},
    {"first_mcu", set_mcu_id, "args: <board #> <revision #>\r\n Detect first-time setup of MCU and prompt loading internal EEPROM configuration\r\n", 4},
#if defined(REV2) || defined(REV3)
    {"fpga_flash", fpga_flash, "args # (1|2):  Program FPGA 1(2) via a programmed flash\r\n",
     1},
#endif // REV2
    {"fpga_reset", fpga_reset, "Reset F1 (f1) or F2 (f2) FPGA\r\n", 1},
    {"ff", ff_ctl,
     "args: (xmit|cdr on/off (0-23|all)) | regw reg# val (0-23|all) | regr reg# (0-23)\r\n"
     " Firefly controlling and monitoring commands\r\n",
     -1},
#if defined(REV2) || defined(REV3)
    {
        "ff_reset",
        ff_reset,
        "Reset FF, args: 1 or 2 for F1 or F2\r\n",
        1,
    },
#endif // REV2
    {
        "ff_status",
        ff_status,
        "Show FF status\r\n",
        0,
    },
    {
        "ff_los",
        ff_los_alarm,
        "Show FF loss of signal alarms\r\n",
        0,
    },
    {
        "ff_cdr_lol",
        ff_cdr_lol_alarm,
        "Show FF CDR loss of lock alarms\r\n",
        0,
    },
#if defined(REV2) || defined(REV3)
    {
        "ff_cdr_ena",
        ff_cdr_enable_status,
        "Show FF CDR enable status\r\n",
        0,
    },
    {
        "ff_ch_dis",
        ff_ch_disable_status,
        "Show FF ch disable status\r\n",
        0,
    },
    {
        "ff_mux_reset",
        ff_mux_reset,
        "reset ff muxes, 1 or 2 for F1 or F2\r\n",
        1,
    },
    {
        "ff_dump_names",
        ff_dump_names,
        "dump name registers\r\n",
        0,
    },
    {
        "ff_optpow",
        ff_optpow,
        "Show avg FF optical power\r\n",
        0,
    },
    {
        "ff_optpow_dev",
        ff_optpow_dev,
        "Show dev FF optical power\r\n",
        1,
    },
    {
        "ff_volts",
        ff_v3v3,
        "Show FF 3v3 mon\r\n",
        0,
    },
#endif // REV2
    {
        "ff_temp",
        ff_temp,
        "Show FF temperatures\r\n",
        0,
    },
    {"fpga", fpga_ctl, "Show state of FPGAs\r\n",
     -1},
    {
        "gpio",
        gpio_ctl,
        "Get or set GPIO pin\r\n",
        -1,
    },
    {"help", help_command_fcn, "This help command\r\n", -1},
    {"id", board_id_info, "Print board ID info\r\n", 0},
    {"i2cr", i2c_ctl_r,
     "args: <dev> <address> <number of bytes>\r\nRead I2C controller. Addr in hex\r\n", 3},
    {"i2crr", i2c_ctl_reg_r,
     "i2crr <dev> <address> <n reg addr bytes> <reg addr> <n data bytes> \r\n Read I2C controller. Addr in hex\r\n", 5},
    {"i2cw", i2c_ctl_w, "i2cw <dev> <address> <number of bytes> <value>\r\n Write I2C controller\r\n",
     4},
    {"i2cwr", i2c_ctl_reg_w,
     "args: <dev> <address> <number of reg bytes> <reg> <number of bytes>\r\nWrite I2C controller\r\n", 6},
    {
        "i2c_scan",
        i2c_scan,
        "Scan current I2C bus\r\n",
        1,
    },
#if defined(REV2) || defined(REV3)
    {
        "jtag_sm",
        jtag_sm_ctl,
        "(on|off) set the JTAG from SM or not\r\n",
        -1,
    },
    {
        "loadclock",
        init_load_clock_ctl,
        "args: 0-4\r\n",
        1,
    },
#endif // REV2
    {
        "log",
        log_ctl,
        "args: (<fac> FTL|ERR|WRN|INF|DBG|TRC)|dump|status|quiet\r\nConfigure log\r\n",
        -1,
    },
    {"led", led_ctl, "Manipulate red LED\r\n", 1},
    {"mem", mem_ctl, "Size of heap\r\n", 0},
    {
        "pwr",
        power_ctl,
        "args: (on|off|status|clearfail)\r\nTurn on or off all power, get status or clear "
        "failures\r\n",
        1,
    },
    {"psmon", psmon_ctl, "Show state of power supplies\r\n", 1},
    {"psreg", psmon_reg, "<which> <reg>. which: LGA80D (10*dev+page), reg: reg address in hex\r\n", 2},
    {"restart_mcu", restart_mcu, "Restart the microcontroller\r\n", 0},
    {"semaphore", sem_ctl, "args: (none)|<i2cdev 1-6> <take|release>\r\nTake or release a semaphore\r\n", -1},
    {"snapshot", snapshot,
     "args:# (0|1)\r\nDump snapshot register. #: which LGA80D (10*dev+page). 0|1 reset bool\r\n",
     2},
    {"sn_all", sn_all, "reset all LGA80D snapshots\r\n", 0},
    {
        "set_id",
        set_board_id,
        "args: <passwd> <addr> <data>\r\nAllows the user to set the board id "
        "information\r\n",
        3,
    },
    {
        "set_id_password",
        set_board_id_password,
        "One-time use: sets password for ID block\r\n",
        0,
    },
    {
        "stack_usage",
        stack_ctl,
        "Print out system stack high water mark\r\n",
        0,
    },
    {
        "taskinfo",
        taskInfo,
        "Info about FreeRTOS tasks\r\n",
        0,
    },
    {"taskstats",
     TaskStatsCommand,
     "Show state of each FreeRTOS task\r\n", 0},
#if defined(REV2) || defined(REV3)
    {
        "time",
        time_ctl,
        "(set HH:MM:SS MM/DD/YYYY|<none)\r\nRTC set and display\r\n",
        -1,
    },
#endif // REV2
    {"uptime", uptime, "Uptime in minutes\r\n", 0},
    {"version", ver_ctl, "MCU firmware version\r\n", 0},
#if defined(REV2) || defined(REV3)
    {"v38", v38_ctl, "Control 3V8 supply. Args: on|off 1|2\r\n", 2},
#endif // REV2
#if 0
    {"watchdog", watchdog_ctl, "Display status of the watchdog task\r\n", 0},
#endif
    {
        "zmon",
        zmon_ctl,
#ifdef ZYNQMON_TEST_MODE
        "args:(on|off|status|debug1|debug2|debugraw|normal|sendone|(settest <sensor> <val>))\r\n"
#else
        "args:(on|off)\r\n"
#endif // ZYNQMON_TEST_MODE
        " Control ZynqMon task\r\n",
        -1,
    },

};

#ifdef REV1
static void U4Print(const char *str)
{
  UARTPrint(UART4_BASE, str);
}
static void U1Print(const char *str)
{
  UARTPrint(UART1_BASE, str);
}
#elif defined(REV2) || defined(REV3) // REV 2 or 3
static void U0Print(const char *str)
{
  UARTPrint(UART0_BASE, str);
}
#endif

struct microrl_user_data_t {
  uint32_t uart_base;
};

static BaseType_t help_command_fcn(int argc, char **argv, char *m)
{
  int copied = 0;
  if (argc == 1) {
    static int i = 0;
    for (; i < NUM_COMMANDS; ++i) {
      int left = SCRATCH_SIZE - copied;
      // need room for command string, help string, newlines, etc, and trailing \0
      unsigned int len = strlen(commands[i].helpstr) + strlen(commands[i].commandstr) + 7;
      if (left < len) {
        return pdTRUE;
      }
      copied += snprintf(m + copied, SCRATCH_SIZE - copied, "%s:\r\n %s",
                         commands[i].commandstr, commands[i].helpstr);
    }
    i = 0;
    return pdFALSE;
  }
  else { // help on a specific command.
    // help for any command that matches the entered command
    static int j = 0;
    for (; j < NUM_COMMANDS; ++j) {
      if (strncmp(commands[j].commandstr, argv[1], strlen(argv[1])) == 0) {
        int left = SCRATCH_SIZE - copied;
        // need room for command string, help string, newlines, etc, and trailing \0
        unsigned int len = strlen(commands[j].helpstr) + strlen(commands[j].commandstr) + 7;
        if (left < len) {
          return pdTRUE;
        }
        copied += snprintf(m + copied, SCRATCH_SIZE - copied, "%s:\r\n %s",
                           commands[j].commandstr, commands[j].helpstr);
      }
    }
    j = 0;
    if (copied == 0) {
      snprintf(m + copied, SCRATCH_SIZE - copied,
               "%s: No command starting with %s found\r\n", argv[0], argv[1]);
    }
    return pdFALSE;
  }
}

static int execute(void *p, int argc, char **argv)
{
  struct microrl_user_data_t *userdata = p;
  uint32_t base = userdata->uart_base;

  UARTPrint(base, "\r\n"); // the microrl does not terminate the active command

  // find the command in the list
  // argc here includes the actual command itself, so the
  // number of supplied arguments is argc-1
  for (int i = 0; i < NUM_COMMANDS; ++i) {
    if (strncmp(commands[i].commandstr, argv[0], 256) == 0) {
      if ((argc == commands[i].num_args + 1) || commands[i].num_args < 0) {
        int retval = commands[i].interpreter(argc, argv, m);
        if (m[0] != '\0')
          UARTPrint(base, m);
        while (retval == pdTRUE) {
          retval = commands[i].interpreter(argc, argv, m);
          if (m[0] != '\0')
            UARTPrint(base, m);
        }
        m[0] = '\0';
        return 0;
      }
      else {
        snprintf(m, SCRATCH_SIZE,
                 "Wrong number of arguments for command %s: %d expected, got %d\r\n", argv[0],
                 commands[i].num_args, argc - 1);
        UARTPrint(base, m);
        return 0;
      }
    }
  }
  UARTPrint(base, "Command unknown: ");
  UARTPrint(base, argv[0]);
  UARTPrint(base, "\r\n");

  return 0;
}

// The actual task
void vCommandLineTask(void *pvParameters)
{
  uint8_t cRxedChar;

  configASSERT(pvParameters != 0);

  CommandLineTaskArgs_t *args = pvParameters;
  StreamBufferHandle_t uartStreamBuffer = args->UartStreamBuffer;
  uint32_t uart_base = args->uart_base;

  UARTPrint(uart_base, pcWelcomeMessage);
  struct microrl_user_data_t rl_userdata = {
      .uart_base = uart_base,
  };

#ifdef REV1
  void (*printer)(const char *) = U4Print;
#elif defined(REV2) || defined(REV3) // Rev 2 or 3
  void (*printer)(const char *) = U0Print;
#endif

  struct microrl_config rl_config = {
      .print = printer, // default to front panel
      // set callback for execute
      .execute = execute,
      .prompt_str = "% ",
      .prompt_length = 2,
      .userdata = &rl_userdata,
  };
#ifdef REV1
  // this is a hack
  if (uart_base == UART1_BASE) {
    rl_config.print = U1Print; // switch to Zynq
  }
#endif // REV1
  microrl_t rl;
  microrl_init(&rl, &rl_config);
  microrl_set_execute_callback(&rl, execute);
  microrl_insert_char(&rl, ' '); // this seems to be necessary?

  for (;;) {
    /* This implementation reads a single character at a time.  Wait in the
       Blocked state until a character is received. */
    xStreamBufferReceive(uartStreamBuffer, &cRxedChar, 1, portMAX_DELAY);
    microrl_insert_char(&rl, cRxedChar);

    // monitor stack usage for this task
    CHECK_TASK_STACK_USAGE(args->stack_size);
  }
}
