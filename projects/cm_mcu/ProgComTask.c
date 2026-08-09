// FreeRTOS task for programmatic interface to the CM.
// The Interface is a UART and the commands have a simple structure
/*
The remote host initiates all transactions by transmitting a command as a string containing a series of fields separated by spaces:

- command: "r" = read, "w" = write
- device type: "DC" = DC-DC converter, "FF" = Firefly, "CL" = clock devices
- device number: hex number representing the device number (one byte)
- page: hex number which is the page address (one byte)
- address: hex number that is the register address within the selected page (one byte)
- only for write commands: data to write: one or several hex numbers that are the data bytes to be written.
- line terminator: "\n" symbol (code 0x0A)

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
- optionally, but strongly suggested: a string with error description. 
  Example: "invalid address"
- line terminator: "\n" symbol (code 0x0A)
*/

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include <string.h>

#include "FreeRTOSConfig.h" // IWYU pragma: keep
#include "InterruptHandlers.h"
#include "Tasks.h"
#include "FireflyUtils.h"
#include "I2CCommunication.h"
#include "Semaphore.h"
#include "MonitorTask.h"
#include "MonitorTaskI2C.h"

struct ProgComTaskArgs_t {
  StreamBufferHandle_t UartStreamBuffer;
  uint32_t uart_base;
  size_t stack_size;
};

#define CMD_SZ 256
#define POWER_I2C_BASE              1
#define POWER_PAGE_COMMAND         0x0
#define CLOCK_I2C_BASE              2
#define CLOCK_PAGE_COMMAND         0x0FF // dummy
#define FF_F1_I2C_BASE             3
#define FF_F2_I2C_BASE             4
#define FF_PAGE_COMMAND             0x7F // 127

extern const struct dev_i2c_addr_t pm_addrs_dcdc[N_PM_ADDRS_DCDC];

// maximum number of data bytes accepted on a single write command
#define PROGCOM_MAX_DATA 16

enum progcom_op_t {
  PROGCOM_OP_READ,
  PROGCOM_OP_WRITE,
};

enum progcom_dev_t {
  PROGCOM_DEV_DCDC, // "DC"
  PROGCOM_DEV_FF,   // "FF"
  PROGCOM_DEV_CLK,  // "CL"
  PROGCOM_DEV_MCU,  // "MC"
};

// a parsed command line
struct progcom_cmd_t {
  enum progcom_op_t op;
  enum progcom_dev_t dev;
  uint8_t dev_num;                  // device number within the device type
  uint8_t page;                     // page register value
  uint8_t address;                  // register address within the page
  uint8_t data[PROGCOM_MAX_DATA];   // write payload; empty for reads
  size_t ndata;                     // number of valid bytes in data[]
};

static uint8_t hex_digit(char c)
{
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    return 0;  // or handle invalid input appropriately
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
// Grammar (fields separated by one or more spaces, line terminator optional
// since the caller may have stripped it):
//   <r|w> <DC|FF|CL|MC> <devnum> <page> <addr> [<data> ...]
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
const char *progcom_parse(const char *line, struct progcom_cmd_t *cmd)
{
  if (line == NULL || cmd == NULL)
    return "internal error";

  memset(cmd, 0, sizeof(*cmd));
  const char *p = progcom_skip_spaces(line);

  // command: r or w
  switch (*p) {
    case 'r':
      cmd->op = PROGCOM_OP_READ;
      break;
    case 'w':
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

  // remaining tokens are data bytes (write only)
  while (!progcom_at_end(p)) {
    if (cmd->ndata >= PROGCOM_MAX_DATA)
      return "too many data bytes";
    p = progcom_parse_byte(p, &cmd->data[cmd->ndata]);
    if (p == NULL)
      return "invalid data byte";
    ++cmd->ndata;
    p = progcom_skip_spaces(p);
  }

  if (cmd->op == PROGCOM_OP_WRITE && cmd->ndata == 0)
    return "write with no data";
  if (cmd->op == PROGCOM_OP_READ && cmd->ndata != 0)
    return "read takes no data bytes";

  return NULL; // success
}

// Execute a parsed read command. Returns NULL on success, or an error string.
// On success, writes up to PROGCOM_MAX_DATA bytes to out_data and sets *nout.
static const char *progcom_execute_read(const struct progcom_cmd_t *cmd, uint8_t *out_data, size_t *nout)
{
  uint8_t i2c_dev = 0;
  const struct dev_i2c_addr_t *addr = NULL;
  uint8_t dev_addr = 0;
  uint8_t mux_addr = 0;
  uint8_t mux_bit = 0;
  *nout = 0;

  // Select I2C bus and device based on device type
  switch (cmd->dev) {
    case PROGCOM_DEV_DCDC:
      i2c_dev = POWER_I2C_BASE;
      if (cmd->dev_num >= N_PM_ADDRS_DCDC)
        return "invalid DCDC device number";
      addr = &pm_addrs_dcdc[cmd->dev_num];
      dev_addr = addr->dev_addr;
      mux_addr = addr->mux_addr;
      mux_bit = addr->mux_bit;
      break;
    case PROGCOM_DEV_FF:
      if (cmd->dev_num < NFIREFLIES_F1) {
        i2c_dev = FF_F1_I2C_BASE;
      } else if (cmd->dev_num < NFIREFLIES_F1 + NFIREFLIES_F2) {
        i2c_dev = FF_F2_I2C_BASE;
      } else {
        return "invalid Firefly device number";
      }
      // Firefly uses dev_moni2c_addr_t, not dev_i2c_addr_t
      // For now, use a simplified approach - direct I2C read
      // TODO: integrate with FireflyUtils for proper register access
      dev_addr = 0x50; // Example default address
      mux_addr = 0x70; // Example mux address
      mux_bit = cmd->dev_num & 0x7;
      break;
    case PROGCOM_DEV_CLK:
      i2c_dev = CLOCK_I2C_BASE;
      if (cmd->dev_num >= NDEVICES_CLK)
        return "invalid clock device number";
      // clk_moni2c_addrs is dev_moni2c_addr_t, but we only need dev_addr field
      // which is at the same offset as in dev_i2c_addr_t
      dev_addr = clk_moni2c_addrs[cmd->dev_num].dev_addr;
      mux_addr = clk_moni2c_addrs[cmd->dev_num].mux_addr;
      mux_bit = clk_moni2c_addrs[cmd->dev_num].mux_bit;
      break;
    case PROGCOM_DEV_MCU:
      return "MCU read not implemented";
    default:
      return "unknown device type";
  }

  // Acquire I2C semaphore
  SemaphoreHandle_t sem = getSemaphore(i2c_dev);
  if (sem == NULL || acquireI2CSemaphore(sem) == pdFAIL) {
    return "failed to acquire I2C semaphore";
  }

  tSMBusStatus r;
  // Step 1: Set mux (if applicable)
  if (mux_addr != 0) {
    r = apollo_i2c_ctl_w(i2c_dev, mux_addr, 1, 0x1U << mux_bit);
    if (r != SMBUS_OK) {
      xSemaphoreGive(sem);
      return "mux select failed";
    }
  }

  // Step 2: Set page register
  r = apollo_i2c_ctl_reg_w(i2c_dev, dev_addr, 1, cmd->page, 1, cmd->page);
  if (r != SMBUS_OK) {
    xSemaphoreGive(sem);
    return "page select failed";
  }

  // Step 3: Read data from register address
  uint8_t data[PROGCOM_MAX_DATA];
  r = apollo_i2c_ctl_reg_r(i2c_dev, dev_addr, 1, cmd->address, 1, (uint32_t *)data);
  xSemaphoreGive(sem);

  if (r != SMBUS_OK) {
    return "read failed";
  }

  out_data[0] = data[0];
  *nout = 1;
  return NULL; // success
}

// Execute a parsed write command. Returns NULL on success, or an error string.
static const char *progcom_execute_write(const struct progcom_cmd_t *cmd)
{
  uint8_t i2c_dev = 0;
  const struct dev_i2c_addr_t *addr = NULL;
  uint8_t dev_addr = 0;
  uint8_t mux_addr = 0;
  uint8_t mux_bit = 0;

  if (cmd->ndata == 0 || cmd->ndata > PROGCOM_MAX_DATA)
    return "invalid data length";

  // Select I2C bus and device based on device type
  switch (cmd->dev) {
    case PROGCOM_DEV_DCDC:
      i2c_dev = POWER_I2C_BASE;
      if (cmd->dev_num >= N_PM_ADDRS_DCDC)
        return "invalid DCDC device number";
      addr = &pm_addrs_dcdc[cmd->dev_num];
      dev_addr = addr->dev_addr;
      mux_addr = addr->mux_addr;
      mux_bit = addr->mux_bit;
      break;
    case PROGCOM_DEV_FF:
      if (cmd->dev_num < NFIREFLIES_F1) {
        i2c_dev = FF_F1_I2C_BASE;
      } else if (cmd->dev_num < NFIREFLIES_F1 + NFIREFLIES_F2) {
        i2c_dev = FF_F2_I2C_BASE;
      } else {
        return "invalid Firefly device number";
      }
      // Firefly uses dev_moni2c_addr_t, not dev_i2c_addr_t
      // For now, use a simplified approach - direct I2C write
      // TODO: integrate with FireflyUtils for proper register access
      dev_addr = 0x50; // Example default
      mux_addr = 0x70;
      mux_bit = cmd->dev_num & 0x7;
      break;
    case PROGCOM_DEV_CLK:
      i2c_dev = CLOCK_I2C_BASE;
      if (cmd->dev_num >= NDEVICES_CLK)
        return "invalid clock device number";
      // clk_moni2c_addrs is dev_moni2c_addr_t, extract fields individually
      dev_addr = clk_moni2c_addrs[cmd->dev_num].dev_addr;
      mux_addr = clk_moni2c_addrs[cmd->dev_num].mux_addr;
      mux_bit = clk_moni2c_addrs[cmd->dev_num].mux_bit;
      break;
    case PROGCOM_DEV_MCU:
      return "MCU write not implemented";
    default:
      return "unknown device type";
  }

  // Acquire I2C semaphore
  SemaphoreHandle_t sem = getSemaphore(i2c_dev);
  if (sem == NULL || acquireI2CSemaphore(sem) == pdFAIL) {
    return "failed to acquire I2C semaphore";
  }

  tSMBusStatus r;
  // Step 1: Set mux (if applicable)
  if (mux_addr != 0) {
    r = apollo_i2c_ctl_w(i2c_dev, mux_addr, 1, 0x1U << mux_bit);
    if (r != SMBUS_OK) {
      xSemaphoreGive(sem);
      return "mux select failed";
    }
  }

  // Step 2: Set page register
  r = apollo_i2c_ctl_reg_w(i2c_dev, dev_addr, 1, cmd->page, 1, cmd->page);
  if (r != SMBUS_OK) {
    xSemaphoreGive(sem);
    return "page select failed";
  }

  // Step 3: Write data to register address
  uint32_t packed_data = 0;
  for (size_t i = 0; i < cmd->ndata && i < 4; ++i) {
    packed_data |= ((uint32_t)cmd->data[i]) << (i * 8);
  }
  r = apollo_i2c_ctl_reg_w(i2c_dev, dev_addr, 1, cmd->address, cmd->ndata, packed_data);
  xSemaphoreGive(sem);

  if (r != SMBUS_OK) {
    return "write failed";
  }

  return NULL; // success
}

void add_progcom_char(uint8_t c)
{
  // handle received character and process command when \r\n is received
  static char cmd_buffer[CMD_SZ];
  static size_t cmd_index = 0;

  // check for carriage return and line feed, signaling end of a command
  if (c == '\n' && cmd_index > 0 && cmd_buffer[cmd_index - 1] == '\r') {
    if (cmd_index > 0) {
      cmd_buffer[cmd_index] = '\0'; // null-terminate the command string

      // Parse the command using the structured parser
      struct progcom_cmd_t cmd;
      const char *parse_err = progcom_parse(cmd_buffer, &cmd);
      if (parse_err != NULL) {
        // Send error response
        char response[CMD_SZ];
        snprintf(response, sizeof(response), "e %s\r\n", parse_err);
        xStreamBufferSend(xUART7StreamBuffer, response, strlen(response), portMAX_DELAY);
        cmd_index = 0;
        return;
      }

      // Execute the command
      char response[CMD_SZ];
      response[0] = '\0';

      if (cmd.op == PROGCOM_OP_READ) {
        uint8_t data[PROGCOM_MAX_DATA];
        size_t nout = 0;
        const char *exec_err = progcom_execute_read(&cmd, data, &nout);
        if (exec_err != NULL) {
          snprintf(response, sizeof(response), "e %s\r\n", exec_err);
        } else {
          // Format response: "d <hex_bytes>\r\n"
          size_t pos = 0;
          response[pos++] = 'd';
          response[pos++] = ' ';
          for (size_t i = 0; i < nout; ++i) {
            if (i > 0) response[pos++] = ' ';
            pos += snprintf(response + pos, sizeof(response) - pos, "%02X", data[i]);
          }
          response[pos++] = '\r';
          response[pos++] = '\n';
          response[pos] = '\0';
        }
      } else if (cmd.op == PROGCOM_OP_WRITE) {
        const char *exec_err = progcom_execute_write(&cmd);
        if (exec_err != NULL) {
          snprintf(response, sizeof(response), "e %s\r\n", exec_err);
        } else {
          snprintf(response, sizeof(response), "c\r\n");
        }
      }

      // Send response back to the UART stream buffer
      if (response[0] != '\0') {
        xStreamBufferSend(xUART7StreamBuffer, response, strlen(response), portMAX_DELAY);
      }
    }
    cmd_index = 0; // reset index for next command
  } else {
    // incoming char, not command terminator
    if (cmd_index < sizeof(cmd_buffer) - 1) {
      cmd_buffer[cmd_index++] = c;
    } else {
      // buffer overflow, reset and ignore
      cmd_index = 0;
    }
  }
}
// FreeRTOS Task itself
void ProgComTask(void *pvParameters)
{
  // cast the parameters to the correct type
  struct ProgComTaskArgs_t *args = (struct ProgComTaskArgs_t *)pvParameters;
  StreamBufferHandle_t uartStreamBuffer = args->UartStreamBuffer;
  uint32_t uart_base = args->uart_base;
  (void)uart_base; // avoid unused variable warning

  uint8_t cRxedChar;
  for (;;) {
    /* This implementation reads a single character at a time.  Wait in the
    Blocked state until a character is received. */
    xStreamBufferReceive(uartStreamBuffer, &cRxedChar, 1,
                         portMAX_DELAY);
    add_progcom_char(cRxedChar);
    // monitor stack usage for this task
    CHECK_TASK_STACK_USAGE(args->stack_size);
  }
}