// FreeRTOS task for programmatic interface to the CM.
// The Interface is a UART and the commands have a simple structure
/*
The remote host initiates all transactions by transmitting a command as a string containing a series of fields separated by spaces:

- command: "r" = read, "w" = write
- device type: "DC" = DC-DC converter, "FF" = Firefly, "CL" = clock devices, "MC" = MCU,
  "FP" = FPGA generic I2C register (device number 0 = F1, 1 = F2; page must be 0)
- device number: hex number representing the device number (one byte)
- page: hex number which is the page address (one byte)
- address: hex number that is the register address within the selected page (one byte)
- for read commands: optional number of bytes to read (1-4, default 1).
- for write commands: one or several data bytes to write.
- line terminator: "\n" symbol (code 0x0A). A preceding "\r" is tolerated and ignored.

Reads return one to four bytes. Omitting the read length preserves the legacy one-byte behavior.

If the command was a read command, the MCU responds with the data read out from the device
by transmitting a string in the following format:

- start marker: "d" + space symbol
- one or several hex numbers that are the data bytes read from the device, separated by spaces
- line terminator: "\n" symbol (code 0x0A)

If the command was a write command, the MCU responds with the following:
- confirmation token "c"
- line terminator: "\n" symbol (code 0x0A)

If MCU encountered an error, it responds with the following:

- error marker: "e" + space symbol
- a string with error description. Example: "invalid address"
- line terminator: "\n" symbol (code 0x0A)
*/

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include <string.h>

#include "FreeRTOSConfig.h" // IWYU pragma: keep
#include "ProgComTask.h"

#if defined(REV2) || defined(REV3)

#include "InterruptHandlers.h"
#include "Tasks.h"
#include "FireflyUtils.h"
#include "FPGAUtils.h"
#include "I2CCommunication.h"
#include "Semaphore.h"
#include "MonitorTask.h"
#include "MonitorTaskI2C.h"
#include "clocksynth.h"
#include "common/LocalUart.h"
#include "common/smbus_helper.h"

// size of the buffer holding an incoming command line
#define CMD_SZ 256
// size of the buffer holding a response. Longest is an error string.
#define PROGCOM_RESP_SZ 64
// maximum number of data bytes accepted on a single write command. The
// underlying I2C helpers cap out at MAX_BYTES (4) per transaction.
#define PROGCOM_MAX_DATA 4

extern const struct dev_i2c_addr_t pm_addrs_dcdc[N_PM_ADDRS_DCDC]; // LocalTasks.c
extern const struct pm_command_t extra_cmds[N_EXTRA_CMDS];         // LocalTasks.c

enum progcom_op_t {
  PROGCOM_OP_READ,
  PROGCOM_OP_WRITE,
};

enum progcom_dev_t {
  PROGCOM_DEV_DCDC, // "DC"
  PROGCOM_DEV_FF,   // "FF"
  PROGCOM_DEV_CLK,  // "CL"
  PROGCOM_DEV_MCU,  // "MC"
  PROGCOM_DEV_FPGA, // "FP"
};

// a parsed command line
struct progcom_cmd_t {
  enum progcom_op_t op;
  enum progcom_dev_t dev;
  uint8_t dev_num;                // device number within the device type
  uint8_t page;                   // page register value
  uint8_t address;                // register address within the page
  uint8_t read_len;               // bytes requested by a read (1-4)
  uint8_t data[PROGCOM_MAX_DATA]; // write payload; empty for reads
  size_t ndata;                   // number of valid bytes in data[]
};

static uint8_t hex_digit(char c)
{
  if (c >= '0' && c <= '9')
    return c - '0';
  if (c >= 'A' && c <= 'F')
    return c - 'A' + 10;
  if (c >= 'a' && c <= 'f')
    return c - 'a' + 10;
  return 0; // or handle invalid input appropriately
}

// hex_digit() cannot report failure (0 is a valid result), so callers must
// validate the character first.
static inline bool is_hex_digit(char c)
{
  return (c >= '0' && c <= '9') || (c >= 'A' && c <= 'F') || (c >= 'a' && c <= 'f');
}

// ---------------------------------------------------------------------------
// Command parser
//
// Grammar (fields separated by one or more spaces, line terminator already
// stripped by the caller):
//   r <DC|FF|CL|MC|FP> <devnum> <page> <addr> [<length>]
//   w <DC|FF|CL|MC|FP> <devnum> <page> <addr> <data> ...
// All numeric fields are one or two hex digits, i.e. a single byte.
// ---------------------------------------------------------------------------

// end of the command: NUL, or either half of the line terminator
static inline bool progcom_at_end(const char *p)
{
  return *p == '\0' || *p == '\r' || *p == '\n';
}

static inline const char *progcom_skip_spaces(const char *p)
{
  while (*p == ' ' || *p == '\t')
    ++p;
  return p;
}

// Parse a one- or two-digit hex byte. The token must be delimited by a space
// or the end of the command; anything else is a syntax error.
// Returns a pointer just past the token, or NULL on error.
static inline const char *progcom_parse_byte(const char *p, uint8_t *out)
{
  if (!is_hex_digit(*p))
    return NULL;
  uint32_t v = hex_digit(*p++);
  if (is_hex_digit(*p)) {
    v = (v << 4) | hex_digit(*p++);
  }
  if (*p != ' ' && *p != '\t' && !progcom_at_end(p))
    return NULL; // trailing junk, e.g. "1G" or "123"
  *out = (uint8_t)v;
  return p;
}

// Parse a full command line into cmd. Returns NULL on success, or a short
// human-readable description of the failure suitable for the "e " response.
// The input is not modified; the caller keeps ownership of it.
static const char *progcom_parse(const char *line, struct progcom_cmd_t *cmd)
{
  if (line == NULL || cmd == NULL)
    return "internal error";

  memset(cmd, 0, sizeof(*cmd));
  cmd->read_len = 1;
  const char *p = progcom_skip_spaces(line);

  // command: r or w
  switch (*p) {
    case 'r':
    case 'R':
      cmd->op = PROGCOM_OP_READ;
      break;
    case 'w':
    case 'W':
      cmd->op = PROGCOM_OP_WRITE;
      break;
    default:
      return "invalid command";
  }
  ++p;
  if (*p != ' ' && *p != '\t')
    return "invalid command";
  p = progcom_skip_spaces(p);

  // device type: two letters
  if (strncmp(p, "DC", 2) == 0)
    cmd->dev = PROGCOM_DEV_DCDC;
  else if (strncmp(p, "FF", 2) == 0)
    cmd->dev = PROGCOM_DEV_FF;
  else if (strncmp(p, "CL", 2) == 0)
    cmd->dev = PROGCOM_DEV_CLK;
  else if (strncmp(p, "MC", 2) == 0)
    cmd->dev = PROGCOM_DEV_MCU;
  else if (strncmp(p, "FP", 2) == 0)
    cmd->dev = PROGCOM_DEV_FPGA;
  else
    return "invalid device type";
  p += 2;
  if (*p != ' ' && *p != '\t')
    return "invalid device type";
  p = progcom_skip_spaces(p);

  // device number, page, register address
  p = progcom_parse_byte(p, &cmd->dev_num);
  if (p == NULL)
    return "invalid device number";
  p = progcom_skip_spaces(p);

  p = progcom_parse_byte(p, &cmd->page);
  if (p == NULL)
    return "invalid page";
  p = progcom_skip_spaces(p);

  p = progcom_parse_byte(p, &cmd->address);
  if (p == NULL)
    return "invalid address";
  p = progcom_skip_spaces(p);

  if (cmd->op == PROGCOM_OP_READ) {
    if (!progcom_at_end(p)) {
      p = progcom_parse_byte(p, &cmd->read_len);
      if (p == NULL)
        return "invalid read length";
      p = progcom_skip_spaces(p);
      if (!progcom_at_end(p))
        return "read takes one length byte";
    }
    if (cmd->read_len < 1 || cmd->read_len > PROGCOM_MAX_DATA)
      return "read length must be 1-4";
  }
  else {
    // remaining tokens are write data bytes
    while (!progcom_at_end(p)) {
      if (cmd->ndata >= PROGCOM_MAX_DATA)
        return "too many data bytes";
      p = progcom_parse_byte(p, &cmd->data[cmd->ndata]);
      if (p == NULL)
        return "invalid data byte";
      ++cmd->ndata;
      p = progcom_skip_spaces(p);
    }
  }

  if (cmd->op == PROGCOM_OP_WRITE && cmd->ndata == 0)
    return "write with no data";

  return NULL; // success
}

// ---------------------------------------------------------------------------
// Device access. Each helper returns NULL on success or an error string.
// On a successful read cmd->read_len bytes are stored in out[].
// ---------------------------------------------------------------------------

// Buffer for error messages that embed the SMBus error string. A single static
// buffer is safe here: only ProgComTask calls into these helpers.
static char progcom_errbuf[56];

static const char *progcom_i2c_error(const char *what, int r)
{
  snprintf(progcom_errbuf, sizeof(progcom_errbuf), "%s: %s", what, SMBUS_get_error((tSMBusStatus)r));
  return progcom_errbuf;
}

// Fireflies: read_arbitrary_ff_register()/write_arbitrary_ff_register() already
// handle bus selection, the mux, the page select byte and the semaphore.
static const char *progcom_access_ff(const struct progcom_cmd_t *cmd, uint8_t *out)
{
  if (cmd->dev_num >= NFIREFLIES)
    return "invalid Firefly device number";
  if (!isEnabledFF(cmd->dev_num))
    return "Firefly not enabled";

  // FF register numbers are packed as page<<8 | address. The page is only
  // written to the page select byte for addresses above FF_PAGE_SELECT_BYTE;
  // see ff_select_page_if_needed() in commands/FireflyCommands.c.
  uint16_t packed_reg = ((uint16_t)cmd->page << 8) | cmd->address;

  int r;
  if (cmd->op == PROGCOM_OP_READ) {
    r = (int16_t)read_arbitrary_ff_register(packed_reg, cmd->dev_num, out, cmd->read_len);
  }
  else {
    if (cmd->ndata != 1)
      return "Firefly write takes one data byte";
    r = write_arbitrary_ff_register(packed_reg, cmd->data[0], cmd->dev_num);
  }

  if (r == 0)
    return NULL;
  if (r == SEM_ACCESS_ERROR)
    return "could not get I2C semaphore";
  if (r < 0)
    return "Firefly access error";
  return progcom_i2c_error(cmd->op == PROGCOM_OP_READ ? "read failed" : "write failed", r);
}

// LGA80D DC-DC converters, via PMBus. apollo_pmbus_rw() selects the mux itself,
// so we only need the semaphore and the PAGE command.
static const char *progcom_access_dcdc(const struct progcom_cmd_t *cmd, uint8_t *out)
{
  if (cmd->dev_num >= NSUPPLIES_PS)
    return "invalid DCDC device number";
  if (cmd->page >= NPAGES_PS)
    return "invalid DCDC page";

  if (acquireI2CSemaphore(i2c1_sem) == pdFAIL)
    return "could not get I2C semaphore";

  const char *err = NULL;
  uint8_t page = cmd->page;
  // extra_cmds[0] is the PMBus PAGE command (register 0x0, one byte)
  tSMBusStatus r = apollo_pmbus_rw(&g_sMaster1, &eStatus1, false, &pm_addrs_dcdc[cmd->dev_num],
                                   &extra_cmds[0], &page);
  if (r != SMBUS_OK) {
    err = progcom_i2c_error("page select failed", r);
  }
  else if (cmd->op == PROGCOM_OP_READ) {
    struct pm_command_t thecmd = {cmd->address, cmd->read_len, "progcom", "", PM_STATUS};
    r = apollo_pmbus_rw(&g_sMaster1, &eStatus1, true, &pm_addrs_dcdc[cmd->dev_num], &thecmd, out);
    if (r != SMBUS_OK)
      err = progcom_i2c_error("read failed", r);
  }
  else {
    struct pm_command_t thecmd = {cmd->address, cmd->ndata, "progcom", "", PM_STATUS};
    uint8_t data[PROGCOM_MAX_DATA];
    memcpy(data, cmd->data, cmd->ndata); // apollo_pmbus_rw() wants a writable buffer
    r = apollo_pmbus_rw(&g_sMaster1, &eStatus1, false, &pm_addrs_dcdc[cmd->dev_num], &thecmd, data);
    if (r != SMBUS_OK)
      err = progcom_i2c_error("write failed", r);
  }

  if (xSemaphoreGetMutexHolder(i2c1_sem) == xTaskGetCurrentTaskHandle())
    xSemaphoreGive(i2c1_sem);
  return err;
}

// Clock synthesizers. Same shape as clear_clk_stickybits()/getClockProgram()
// in clocksynth.c: mux select, page select, transaction, mux clear.
static const char *progcom_access_clk(const struct progcom_cmd_t *cmd, uint8_t *out)
{
  if (cmd->dev_num >= NDEVICES_CLK)
    return "invalid clock device number";

  const uint8_t mux_addr = clk_moni2c_addrs[cmd->dev_num].mux_addr;
  const uint8_t mux_bit = clk_moni2c_addrs[cmd->dev_num].mux_bit;
  const uint8_t dev_addr = clk_moni2c_addrs[cmd->dev_num].dev_addr;

  if (acquireI2CSemaphore(i2c2_sem) == pdFAIL)
    return "could not get I2C semaphore";

  const char *err = NULL;
  int r = apollo_i2c_ctl_w(CLOCK_I2C_DEV, mux_addr, 1, 0x1U << mux_bit);
  if (r != SMBUS_OK) {
    err = progcom_i2c_error("mux select failed", r);
  }
  else {
    r = apollo_i2c_ctl_reg_w(CLOCK_I2C_DEV, dev_addr, 1, CLOCK_CHANGEPAGE_REG_ADDR, 1, cmd->page);
    if (r != SMBUS_OK) {
      err = progcom_i2c_error("page select failed", r);
    }
    else if (cmd->op == PROGCOM_OP_READ) {
      uint32_t packed_data = 0;
      r = apollo_i2c_ctl_reg_r(CLOCK_I2C_DEV, dev_addr, 1, cmd->address,
                               cmd->read_len, &packed_data);
      if (r != SMBUS_OK)
        err = progcom_i2c_error("read failed", r);
      else {
        for (size_t i = 0; i < cmd->read_len; ++i)
          out[i] = (uint8_t)((packed_data >> (i * 8)) & 0xFFU);
      }
    }
    else {
      // apollo_i2c_ctl_reg_w() sends the packed data least significant byte first
      uint32_t packed_data = 0;
      for (size_t i = 0; i < cmd->ndata; ++i)
        packed_data |= ((uint32_t)cmd->data[i]) << (i * 8);
      r = apollo_i2c_ctl_reg_w(CLOCK_I2C_DEV, dev_addr, 1, cmd->address, cmd->ndata, packed_data);
      if (r != SMBUS_OK)
        err = progcom_i2c_error("write failed", r);
    }
    apollo_i2c_ctl_w(CLOCK_I2C_DEV, mux_addr, 1, 0x0U); // clear the mux
  }

  if (xSemaphoreGetMutexHolder(i2c2_sem) == xTaskGetCurrentTaskHandle())
    xSemaphoreGive(i2c2_sem);
  return err;
}

// FPGA generic I2C diagnostic-register block. fpga_i2c_reg_r/w() (FPGAUtils.c)
// handle the mux select/clear; we only own the semaphore. Page is required to
// be zero: a future wider-address capability belongs in an explicit protocol
// extension, not a silent reinterpretation of this field.
static const char *progcom_access_fpga(const struct progcom_cmd_t *cmd, uint8_t *out)
{
  if (cmd->dev_num > 1)
    return "invalid FPGA device number";
  if (cmd->page != 0)
    return "invalid FPGA page";

  if (acquireI2CSemaphore(i2c5_sem) == pdFAIL)
    return "could not get I2C semaphore";

  const char *err = NULL;
  int r;
  if (cmd->op == PROGCOM_OP_READ) {
    uint32_t packed_data = 0;
    r = fpga_i2c_reg_r(cmd->dev_num, cmd->address, cmd->read_len, &packed_data);
    if (r != 0)
      err = progcom_i2c_error("read failed", r);
    else {
      for (size_t i = 0; i < cmd->read_len; ++i)
        out[i] = (uint8_t)((packed_data >> (i * 8)) & 0xFFU);
    }
  }
  else {
    uint32_t packed_data = 0;
    for (size_t i = 0; i < cmd->ndata; ++i)
      packed_data |= ((uint32_t)cmd->data[i]) << (i * 8);
    r = fpga_i2c_reg_w(cmd->dev_num, cmd->address, cmd->ndata, packed_data);
    if (r != 0)
      err = progcom_i2c_error("write failed", r);
  }

  if (xSemaphoreGetMutexHolder(i2c5_sem) == xTaskGetCurrentTaskHandle())
    xSemaphoreGive(i2c5_sem);
  return err;
}

// Parse and execute one command line, then send the response out the UART.
static void progcom_handle_line(uint32_t uart_base, const char *line)
{
  struct progcom_cmd_t cmd;
  uint8_t value[PROGCOM_MAX_DATA] = {0};

  const char *err = progcom_parse(line, &cmd);
  if (err == NULL) {
    switch (cmd.dev) {
      case PROGCOM_DEV_DCDC:
        err = progcom_access_dcdc(&cmd, value);
        break;
      case PROGCOM_DEV_FF:
        err = progcom_access_ff(&cmd, value);
        break;
      case PROGCOM_DEV_CLK:
        err = progcom_access_clk(&cmd, value);
        break;
      case PROGCOM_DEV_MCU:
        err = "MCU device not implemented";
        break;
      case PROGCOM_DEV_FPGA:
        err = progcom_access_fpga(&cmd, value);
        break;
      default:
        err = "unknown device type";
        break;
    }
  }

  char response[PROGCOM_RESP_SZ];
  if (err != NULL)
    snprintf(response, sizeof(response), "e %s\n", err);
  else if (cmd.op == PROGCOM_OP_READ) {
    size_t used = (size_t)snprintf(response, sizeof(response), "d");
    for (size_t i = 0; i < cmd.read_len; ++i)
      used += (size_t)snprintf(response + used, sizeof(response) - used, " %02X", value[i]);
    snprintf(response + used, sizeof(response) - used, "\n");
  }
  else
    snprintf(response, sizeof(response), "c\n");

  UARTPrint(uart_base, response);
}

// Accumulate one character. On the line terminator the command is executed and
// the response sent. An over-long line is discarded up to the next terminator.
static void add_progcom_char(uint32_t uart_base, uint8_t c)
{
  static char cmd_buffer[CMD_SZ];
  static size_t cmd_index = 0;
  static bool too_long = false;

  if (c == '\n') {
    if (too_long) {
      UARTPrint(uart_base, "e command too long\n");
    }
    else if (cmd_index > 0) {
      if (cmd_buffer[cmd_index - 1] == '\r')
        --cmd_index; // tolerate \r\n line endings
      cmd_buffer[cmd_index] = '\0';
      if (cmd_index > 0)
        progcom_handle_line(uart_base, cmd_buffer);
    }
    cmd_index = 0;
    too_long = false;
    return;
  }

  // incoming char, not command terminator
  if (cmd_index < sizeof(cmd_buffer) - 1)
    cmd_buffer[cmd_index++] = (char)c;
  else
    too_long = true; // discard the rest of the line
}

// FreeRTOS Task itself
void ProgComTask(void *pvParameters)
{
  // cast the parameters to the correct type
  ProgComTaskArgs_t *args = (ProgComTaskArgs_t *)pvParameters;

  uint8_t cRxedChar;
  for (;;) {
    /* This implementation reads a single character at a time.  Wait in the
    Blocked state until a character is received. */
    xStreamBufferReceive(args->UartStreamBuffer, &cRxedChar, 1,
                         portMAX_DELAY);
    add_progcom_char(args->uart_base, cRxedChar);
    // monitor stack usage for this task
    CHECK_TASK_STACK_USAGE(args->stack_size);
  }
}

#else // REV1 -- there is no programmatic UART on Rev 1 hardware

typedef int progcom_not_supported_on_rev1_t;

#endif // REV2 || REV3
