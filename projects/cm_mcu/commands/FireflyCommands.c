/*
 * FireflyCommands.c
 *
 * Firefly CLI command handlers, extracted from SensorControl.c.
 */

#include <stdlib.h>
#include <strings.h>
#include <string.h>
#include <assert.h>

#include "commands/FireflyCommands.h"
#include "commands/parameters.h"
#include "FireflyUtils.h"
#include "I2CCommunication.h"
#include "MonI2C_addresses.h"
#include "MonUtils.h"
#include "common/log.h"
#include "common/utils.h"
#include "common/smbus_helper.h"
#include "Semaphore.h"
#include "Tasks.h"
#include "portmacro.h"
#include "projdefs.h"

// A NOTE ABOUT THE FIREFLY REGISTER ADDRESSES
// If you look at the memory maps in the data sheets for the firefly devices, you see that
// the listed addresses for the internal registers are grouped into pages.
// The lowest 7 bits of address are always the same, regardless of the value stored in the
// page select register.
// the page select register is always accessible at byte 0x7f (127).
// Bytes above 127 (128 and following) vary in their meaning depending on the value set in the
// page register. that is to say, byte 128, e.g., is not uniquely defined.
// not all page register values are valid.
// we therefore encode the page number in the register address as follows:
// lowest 8 bits: register address, 0-255.
// if the register address is > 127, the next 8 bits are the page number.
// next 8 bits: page register number.
// This is used in both the read and write functions.
struct ff_i2c_access_t {
  int ff;
  int i2c_device;
  SemaphoreHandle_t sem;
};

static int ff_find_by_name(const char *name)
{
  for (int ff = 0; ff < NFIREFLIES; ++ff) {
    if (strncmp(ff_moni2c_addrs[ff].name, name, 10) == 0)
      return ff;
  }
  return -1;
}

static int ff_i2c_device_for_index(int ff)
{
  return ff < NFIREFLIES_F1 ? I2C_DEVICE_F1 : I2C_DEVICE_F2;
}

static SemaphoreHandle_t ff_i2c_sem_for_device(int i2c_device)
{
  return i2c_device == I2C_DEVICE_F2 ? i2c3_sem : i2c4_sem;
}

static int ff_select_mux(int ff, int i2c_device)
{
  uint8_t muxmask = 0x1U << ff_moni2c_addrs[ff].mux_bit;
  int res = apollo_i2c_ctl_w(i2c_device, ff_moni2c_addrs[ff].mux_addr, 1, muxmask);
  if (res != 0) {
    log_warn(LOG_SERVICE, "%s: Mux writing error %d (%s) (ff=%s) ...\r\n", __func__, res,
             SMBUS_get_error(res), ff_moni2c_addrs[ff].name);
  }
  return res;
}

static int ff_clear_mux(int ff, int i2c_device)
{
  int res = apollo_i2c_ctl_w(i2c_device, ff_moni2c_addrs[ff].mux_addr, 1, 0x0U);
  if (res != 0) {
    log_warn(LOG_SERVICE, "%s: Mux clear error %d (%s) (ff=%s) ...\r\n", __func__, res,
             SMBUS_get_error(res), ff_moni2c_addrs[ff].name);
  }
  return res;
}

static int ff_select_page_if_needed(int ff, int i2c_device, uint16_t *reg)
{
  int res = 0;
  if ((*reg & 0xFFU) > FF_PAGE_SELECT_BYTE) {
    uint8_t page = (*reg >> 8) & 0xFFU;
    res = apollo_i2c_ctl_reg_w(i2c_device, ff_moni2c_addrs[ff].dev_addr, 1,
                               FF_PAGE_SELECT_BYTE, 1, page);
    if (res) {
      log_warn(LOG_SERVICE, "%s: FF page write error %d (%s) (ff=%s) ...\r\n", __func__, res,
               SMBUS_get_error(res), ff_moni2c_addrs[ff].name);
    }
  }
  *reg &= 0x00FFU; // select out the register number, bottom 8 bits
  return res;
}

static int ff_begin_i2c_access(const char *name, int i2c_device, struct ff_i2c_access_t *access)
{
  int ff = ff_find_by_name(name);
  if (ff < 0) {
    return -2; // no match found
  }

  access->ff = ff;
  access->i2c_device = i2c_device;
  access->sem = ff_i2c_sem_for_device(i2c_device);

  if (acquireI2CSemaphore(access->sem) == pdFAIL) {
    log_warn(LOG_SERVICE, "could not get semaphore in time\r\n");
    return SEM_ACCESS_ERROR;
  }

  return ff_select_mux(ff, i2c_device);
}

static int ff_end_i2c_access(struct ff_i2c_access_t *access, int res)
{
  if (!res) {
    res = ff_clear_mux(access->ff, access->i2c_device);
  }

  if (xSemaphoreGetMutexHolder(access->sem) == xTaskGetCurrentTaskHandle())
    xSemaphoreGive(access->sem);
  return res;
}

int read_ff_register(const char *name, uint16_t packed_reg_addr, uint8_t *value, size_t size, int i2c_device)
{
  memset(value, 0, size);
  struct ff_i2c_access_t access;
  int res = ff_begin_i2c_access(name, i2c_device, &access);
  if (res == -2)
    return res;

  if (!res) {
    // Read from register.  if the register number is > FF_PAGE_SELECT_BYTE (0x7FU), we
    // must first write the page number to page select byte.
    res = ff_select_page_if_needed(access.ff, i2c_device, &packed_reg_addr);
    uint32_t uidata;
    res += apollo_i2c_ctl_reg_r(i2c_device, ff_moni2c_addrs[access.ff].dev_addr, 1,
                                packed_reg_addr, size, &uidata);
    for (int i = 0; i < size; ++i) {
      value[i] = (uint8_t)((uidata >> (i * 8)) & 0xFFU);
    }
    if (res != 0) {
      log_warn(LOG_SERVICE, "%s: FF Regread error %d (%s) (ff=%s) ...\r\n", __func__, res,
               SMBUS_get_error(res), ff_moni2c_addrs[access.ff].name);
    }
  }

  return ff_end_i2c_access(&access, res);
}

// see comments above read_ff_register
static int write_ff_register(const char *name, uint16_t reg, uint16_t value, int size, int i2c_device)
{
  configASSERT(size <= 2);
  struct ff_i2c_access_t access;
  int res = ff_begin_i2c_access(name, i2c_device, &access);
  if (res == -2)
    return res;

  // write to register. First word is reg address, then the data.
  // increment size to account for the register address
  if (!res) {
    // If the register number is > FF_PAGE_SELECT_BYTE (0x7FU), we
    // must first write the page number to page select byte.
    res = ff_select_page_if_needed(access.ff, i2c_device, &reg);
    res += apollo_i2c_ctl_reg_w(i2c_device, ff_moni2c_addrs[access.ff].dev_addr, 1, reg, size, (uint32_t)value);
    if (res != 0) {
      log_warn(LOG_SERVICE, "%s: FF writing error %d (%s) (ff=%s) ...\r\n", __func__, res,
               SMBUS_get_error(res), ff_moni2c_addrs[access.ff].name);
    }
  }

  return ff_end_i2c_access(&access, res);
}

typedef int (*ff_apply_fn)(int ff, int i2c_dev, void *ctx);

static bool ff_name_has(int ff, const char *needle)
{
  return strstr(ff_moni2c_addrs[ff].name, needle) != NULL;
}

static bool ff_is_xcvr(int ff)
{
  return ff_name_has(ff, "XCVR");
}

static bool ff_is_tx(int ff)
{
  return ff_name_has(ff, "Tx");
}

static bool ff_is_rx(int ff)
{
  return ff_name_has(ff, "Rx");
}

static int ff_for_each_selected(int num_ff, ff_apply_fn apply, void *ctx)
{
  int ret = 0;
  int i = num_ff;
  int imax = num_ff + 1;

  if (num_ff == NFIREFLIES) {
    i = 0;
    imax = NFIREFLIES;
  }

  for (; i < imax; ++i) {
    if (!isEnabledFF(i)) // skip the FF if it's not enabled via the FF config
      continue;
    ret += apply(i, ff_i2c_device_for_index(i), ctx);
  }

  return ret;
}

static int ff_apply_transmit_disable(int ff, int i2c_dev, void *ctx)
{
  uint16_t value = *(uint16_t *)ctx;

  if (ff_is_xcvr(ff)) {
    // only 4 LSB matter, so mask out others.
    return write_ff_register(ff_moni2c_addrs[ff].name, ECU0_25G_XVCR_TX_DISABLE_REG, value & 0xFU, 1, i2c_dev);
  }
  if (ff_is_tx(ff)) { // same for all 12 channel parts
    return write_ff_register(ff_moni2c_addrs[ff].name, ECU0_14G_TX_DISABLE_REG, value, 2, i2c_dev);
  }
  return 0;
}

static int disable_transmit(bool disable, int num_ff)
{
  uint16_t value = disable ? 0xffffU : 0x0U; // see data sheet re bit arrangement; do not use 0xfffU
  return ff_for_each_selected(num_ff, ff_apply_transmit_disable, &value);
}

static int ff_apply_receiver_disable(int ff, int i2c_dev, void *ctx)
{
  uint16_t value = *(uint16_t *)ctx;

  if (ff_is_xcvr(ff)) {
    return write_ff_register(ff_moni2c_addrs[ff].name, ECU0_25G_XVCR_RX_DISABLE_REG, value & 0x000fU, 1, i2c_dev);
  }
  if (ff_is_rx(ff)) { // Same for CERNB vs 25G
    return write_ff_register(ff_moni2c_addrs[ff].name, ECU0_14G_RX_DISABLE_REG, value, 2, i2c_dev);
  }
  return 0;
}

static int disable_receivers(bool disable, int num_ff)
{
  uint16_t value = disable ? 0xfffU : 0x0U;
  return ff_for_each_selected(num_ff, ff_apply_receiver_disable, &value);
}

static int ff_apply_cdr(int ff, int i2c_dev, void *ctx)
{
  uint8_t value = *(uint8_t *)ctx;

  if (ff_is_xcvr(ff)) {
    return write_ff_register(ff_moni2c_addrs[ff].name, ECU0_25G_XVCR_CDR_REG, value, 1, i2c_dev);
  }

  uint16_t value16 = value == 0 ? 0U : 0xffffU; // 12 channel parts use one bit per channel
  return write_ff_register(ff_moni2c_addrs[ff].name, ECU0_25G_TXRX_CDR_REG, value16, 2, i2c_dev);
}

static int set_xcvr_cdr(uint8_t value, int num_ff)
{
  return ff_for_each_selected(num_ff, ff_apply_cdr, &value);
}

static int write_arbitrary_ff_register(uint16_t regnumber, uint8_t value, int num_ff)
{
  if (num_ff >= NFIREFLIES) {
    return -1;
  }

  if (!isEnabledFF(num_ff)) { // skip the FF if it's not enabled via the FF config
    log_warn(LOG_SERVICE, "Skip writing to disabled FF %d\r\n", num_ff);
  }

  int ret = 0;
  int ret1 = write_ff_register(ff_moni2c_addrs[num_ff].name, regnumber, value, 1,
                               ff_i2c_device_for_index(num_ff));
  if (ret1) {
    log_warn(LOG_SERVICE, "%s: error %s\r\n", __func__, SMBUS_get_error(ret1));
    ret += ret1;
  }
  return ret;
}

// read a SINGLE firefly register, size bytes (up to 4 bytes)
uint16_t read_arbitrary_ff_register(uint16_t regnumber, int num_ff, uint8_t *value, uint8_t size)
{
  if (num_ff >= NFIREFLIES) {
    return -1;
  }
  int ret = read_ff_register(ff_moni2c_addrs[num_ff].name, regnumber, value, size,
                             ff_i2c_device_for_index(num_ff));
  return ret;
}

#if defined(REV2) || defined(REV3)
// reset firefly devices. The resets are ganged together,
// so you can only reset all of them at once, for those
// attached to F1 or F2
// I/O expanders are different in Rev2 and Rev3 for optics, see schematic pages 4.05 and 4.06
#define FF_RESET_MUX_ADDR     0x71
#define FF_RESET_MUX_BIT_MASK (0x1 << 6)
#define FF_RESET_IOEXP_ADDR   0x21
#if defined(REV2)
#define FF_RESET_IOEXP_REG_ADDR 0x3        // output port 1
#define FF_RESET_IOEXP_REG_BIT  (0x1 << 0) // bit P10, i.e., bit 0 of output port 1
#elif defined(REV3)
#define FF_RESET_IOEXP_REG_ADDR 0x2        // output port 0
#define FF_RESET_IOEXP_REG_BIT  (0x1 << 7) // bit P07, i.e., bit 7 of output port 0
#endif                                     // REV
BaseType_t ff_reset(int argc, char **argv, char *m)
{
  int copied = 0;
  BaseType_t which_fpga = strtol(argv[1], NULL, 10);
  if (which_fpga != 1 && which_fpga != 2) {
    copied += snprintf(m + copied, SCRATCH_SIZE - copied, "%s: arg must 1 or 2\r\n", argv[0]);
    return pdFALSE;
  }
  // grab semaphore
  // grab the semaphore to ensure unique access to I2C controller
  // otherwise, block its operations indefinitely until it's available
  SemaphoreHandle_t s = i2c4_sem;
  uint8_t i2c_dev = 4;
  if (which_fpga == 2) {
    s = i2c3_sem;
    i2c_dev = 3;
  }
  if (acquireI2CSemaphore(s) == pdFAIL) {
    log_warn(LOG_SERVICE, "could not get semaphore in time\r\n");
    return pdFALSE;
  }
  // select the appropriate output for the mux
  apollo_i2c_ctl_w(i2c_dev, FF_RESET_MUX_ADDR, 1, FF_RESET_MUX_BIT_MASK);
  // read/modify/write the reset register
  uint32_t reset_reg;
  int ret = apollo_i2c_ctl_reg_r(i2c_dev, FF_RESET_IOEXP_ADDR, 1, FF_RESET_IOEXP_REG_ADDR, 1, &reset_reg);
  // set reset bit 0 which is active low
  reset_reg &= ~FF_RESET_IOEXP_REG_BIT;
  ret += apollo_i2c_ctl_reg_w(i2c_dev, FF_RESET_IOEXP_ADDR, 1, FF_RESET_IOEXP_REG_ADDR, 1, reset_reg);
  // wait a tick
  vTaskDelay(pdMS_TO_TICKS(1));
  // clear the active low reset bit
  reset_reg |= FF_RESET_IOEXP_REG_BIT;
  ret += apollo_i2c_ctl_reg_w(i2c_dev, FF_RESET_IOEXP_ADDR, 1, FF_RESET_IOEXP_REG_ADDR, 1, reset_reg);
  // release the semaphore
  if (xSemaphoreGetMutexHolder(s) == xTaskGetCurrentTaskHandle())
    xSemaphoreGive(s);
  if (ret) {
    copied += snprintf(m + copied, SCRATCH_SIZE - copied, "%s: error %d\r\n", argv[0], ret);
  }
  else {
    copied += snprintf(m + copied, SCRATCH_SIZE - copied, "%s: reset complete F%ld\r\n", argv[0], which_fpga);
  }
  return pdFALSE;
}
// reset the muxes for the firefly devices
BaseType_t ff_mux_reset(int argc, char **argv, char *m)
{
  int copied = 0;
  BaseType_t which_fpga = strtol(argv[1], NULL, 10);
  if (which_fpga != 1 && which_fpga != 2) {
    copied += snprintf(m + copied, SCRATCH_SIZE - copied, "%s: arg must 1 or 2\r\n", argv[0]);
    return pdFALSE;
  }
  // grab semaphore
  // grab the semaphore to ensure unique access to I2C controller
  // otherwise, block its operations indefinitely until it's available
  SemaphoreHandle_t s = i2c4_sem;
  if (which_fpga == 2) {
    s = i2c3_sem;
  }
  if (acquireI2CSemaphore(s) == pdFAIL) {
    log_warn(LOG_SERVICE, "could not get semaphore in time\r\n");
    return pdFALSE;
  }
  // select the appropriate output for the mux
  int pin = _F1_OPTICS_I2C_RESET;
  if (which_fpga == 2)
    pin = _F2_OPTICS_I2C_RESET;
  // toggle the pin
  write_gpio_pin(pin, 0); // active low signal
  vTaskDelay(pdMS_TO_TICKS(1));
  write_gpio_pin(pin, 1); // active low signal

  if (xSemaphoreGetMutexHolder(s) == xTaskGetCurrentTaskHandle())
    xSemaphoreGive(s);
  copied += snprintf(m + copied, SCRATCH_SIZE - copied, "%s: mux reset complete F%ld\r\n", argv[0], which_fpga);
  return pdFALSE;
}
#endif // REV2

BaseType_t ff_status(int argc, char **argv, char *m)
{
  // argument handling
  int copied = 0;

  static int whichff = 0;

  if (whichff == 0) {
    // check for stale data
    TickType_t now = pdTICKS_TO_S(xTaskGetTickCount());

    if (isFFStale()) {
      TickType_t last = pdTICKS_TO_S(getFFupdateTick(isFFStale()));
      int mins = (now - last) / 60;
      copied += snprintf(m + copied, SCRATCH_SIZE - copied,
                         "%s: stale data, last update %d minutes ago\r\n", argv[0], mins);
    }
    copied += snprintf(m + copied, SCRATCH_SIZE - copied, "FF STATUS:\r\n");
  }

#if defined(REV2) || defined(REV3)
  int nTx = -1; // order of Tx ch
  // print out the "present" bits on first pass
  if (whichff == 0) {
    copied += snprintf(m + copied, SCRATCH_SIZE - copied, "PRESENT:\r\n");
    char *ff_bitmask_names[4] = {"1_12", "1_4 ", "2_12", "2_4 "};
    for (int i = 0; i < 4; ++i) {
      copied += snprintf(m + copied, SCRATCH_SIZE - copied, "F%s: 0x%02lx\r\n", ff_bitmask_names[i],
                         getFFpresentbit(i));
    }
  }
#endif // REV2
  for (; whichff < NFIREFLIES; ++whichff) {
    if (isEnabledFF(whichff)) {
      uint8_t val = get_FF_STATUS_REG_data(whichff);
      copied += snprintf(m + copied, SCRATCH_SIZE - copied, "%17s: 0x%02x", ff_moni2c_addrs[whichff].name, val);
    }
    else { // empty value
      copied += snprintf(m + copied, SCRATCH_SIZE - copied, "%17s: %4s", ff_moni2c_addrs[whichff].name, "--");
    }
    bool isTx = (strstr(ff_moni2c_addrs[whichff].name, "Tx") != NULL);
    if (isTx) {
#ifdef REV1
      copied += snprintf(m + copied, SCRATCH_SIZE - copied, "\t");
#elif defined(REV2) || defined(REV3)
      nTx += 1;
      uint8_t ff_4v0_sel = 1 << (nTx % (NFIREFLIES_IT_F1 / 2));
      if (nTx < (NFIREFLIES_IT_F1 / 2))
        ff_4v0_sel &= f1_ff12xmit_4v0_sel;
      else
        ff_4v0_sel &= f2_ff12xmit_4v0_sel;
      copied += snprintf(m + copied, SCRATCH_SIZE - copied, " 3v8?(%x) \t", ff_4v0_sel >> (nTx % (NFIREFLIES_IT_F1 / 2)));
#endif // REV2
    }
    else {
      copied += snprintf(m + copied, SCRATCH_SIZE - copied, "\r\n");
    }
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

  return pdFALSE;
}

// ff_table_row_fn: signature for a single-row formatter used by ff_table_print.
//
// Each row function is called once per firefly device per CLI invocation. It is
// responsible for appending one entry (name + value) to the output buffer `m`
// starting at offset `copied`, and for appending the row terminator via
// ff_table_append_row_end (which emits "\t" for Tx devices so that the Tx and Rx
// columns appear side-by-side, and "\r\n" for everything else).
//
// Return value: the new value of `copied` after appending this row.
typedef int (*ff_table_row_fn)(char *m, int copied, int whichff);

// clang-format off
//
// MACRO: FF_TABLE_CMD
//
// Expands to a complete BaseType_t CLI command handler that pages through all
// firefly devices using ff_table_print.
//
// The CLI framework calls a command handler repeatedly while it returns pdTRUE,
// passing the same output buffer `m` each time. ff_table_print uses a cursor
// (`whichff`) to remember which device to resume from, returning pdTRUE when
// the buffer is nearly full and pdFALSE when all devices have been printed.
//
// `whichff` is declared `static` so it persists across repeated calls for the
// same multi-page output. Because each macro expansion produces a distinct
// function body, each expanded function gets its own independent static variable
// — there is no sharing between, e.g., ff_los_alarm and ff_temp.
//
// Parameters:
//   fn_name  - name of the generated function (e.g. ff_los_alarm)
//   row_fn   - ff_table_row_fn to call for each device
//   title    - header line printed before the first device entry
//   min_rem  - minimum bytes that must remain in the buffer before we stop and
//              return pdTRUE (leave room for the next row before it overflows)
//   stale    - if true, prepend a staleness warning when the monitoring task
//              has not updated the cached register values recently
#define FF_TABLE_CMD(fn_name, row_fn, title, min_rem, stale)                    \
  BaseType_t fn_name(int argc, char **argv, char *m)                            \
  {                                                                               \
    (void)argc;                                                                   \
    static int whichff = 0;                                                       \
    return ff_table_print(m, argv[0], title, &whichff, min_rem, stale, row_fn); \
  }

// MACRO: FF_U16_HEX_ROW_FN
//
// Expands to a static row function that reads one uint16_t monitoring register
// and prints it as "   <device name>: 0xXXXX" followed by the appropriate
// row terminator (tab for Tx devices, CRLF for others).
//
// WHY A MACRO INSTEAD OF A FUNCTION POINTER?
// The per-revision monitoring data accessors (e.g. get_FF_LOS_ALARM_data) are
// defined in MonI2C_addresses.h as preprocessor macros, not as real functions.
// get_FF_LOS_ALARM_data(which) expands to a ternary that selects the F1 or F2
// accessor depending on the device index. A macro has no address, so it cannot
// be passed as a `uint16_t (*getter)(int)` function pointer.
//
// A macro cannot be used as a function pointer (it has no address), so the
// earlier approach of passing a `uint16_t (*getter)(int)` to a shared helper
// would have required a real wrapper function for each accessor. This macro
// avoids that by inlining the accessor expression directly into the row body.
//
// NAMING COLLISION WARNING:
// The macro parameter must NOT be named `name`. The struct field accessed as
// `ff_moni2c_addrs[whichff].name` contains the literal token `name`; if the
// macro parameter were also called `name`, the preprocessor would substitute it
// everywhere, turning `.name` into the expanded parameter value. The parameter
// is therefore called `fn_name` to avoid this silent corruption.
//
// Parameters:
//   fn_name     - name of the generated static function
//   getter_expr - expression that evaluates to a uint16_t; must use the local
//                 variable `whichff` (the function parameter) as the device index
#define FF_U16_HEX_ROW_FN(fn_name, getter_expr)                                \
  static int fn_name(char *m, int copied, int whichff)                          \
  {                                                                               \
    uint16_t val = (getter_expr);                                                \
    copied += snprintf(m + copied, SCRATCH_SIZE - copied, "%17s: 0x%04x",       \
                       ff_moni2c_addrs[whichff].name, val);                      \
    return ff_table_append_row_end(m, copied, whichff);                          \
  }
// clang-format on

// ff_table_abort_current: out-of-band signal from a row function to ff_table_print.
//
// Normally a row function returns normally and ff_table_print continues to the
// next device. When a row function encounters a fatal per-device error (e.g. an
// I2C read failure) it sets this flag to true before returning; ff_table_print
// then resets the cursor and returns pdFALSE immediately, abandoning the rest
// of the table rather than printing garbage for remaining devices.
static bool ff_table_abort_current;

// Firefly devices are laid out in the address table as interleaved Tx/Rx pairs.
// To display them in two columns (Tx on the left, Rx on the right), Tx rows
// are terminated with a tab instead of CRLF; the following Rx row then lands
// on the same terminal line.
static bool ff_table_is_tx(int whichff)
{
  return strstr(ff_moni2c_addrs[whichff].name, "Tx") != NULL;
}

static int ff_table_append_row_end(char *m, int copied, int whichff)
{
  if (ff_table_is_tx(whichff))
    return copied + snprintf(m + copied, SCRATCH_SIZE - copied, "\t");
  return copied + snprintf(m + copied, SCRATCH_SIZE - copied, "\r\n");
}

// Prepends a human-readable staleness warning to the output if the FireflyTask
// monitoring loop has not updated the cached register values within the expected
// window. isFFStale() returns the index of the stale monitoring group (non-zero
// = stale), and getFFupdateTick() returns the tick count of its last update.
static int ff_table_append_stale_warning(char *m, int copied, const char *name)
{
  if (isFFStale()) {
    TickType_t now = pdTICKS_TO_S(xTaskGetTickCount());
    TickType_t last = pdTICKS_TO_S(getFFupdateTick(isFFStale()));
    int mins = (now - last) / 60;
    copied += snprintf(m + copied, SCRATCH_SIZE - copied,
                       "%s: stale data, last update %d minutes ago\r\n", name, mins);
  }
  return copied;
}

// ff_table_print: generic paged table printer for firefly monitoring data.
//
// The CLI output buffer `m` is only SCRATCH_SIZE (1024) bytes. With up to
// NFIREFLIES devices per table, a single call cannot always fit the entire
// output. ff_table_print therefore acts as a coroutine: it fills `m` up to
// `min_remaining` bytes from the end, saves its position in `*cursor`, and
// returns pdTRUE to signal the CLI framework that more output is pending.
// The framework calls the command handler again; the handler passes the same
// `cursor` pointer and ff_table_print resumes from where it left off.
// When all devices have been printed, `*cursor` is reset to 0 and pdFALSE
// is returned so the framework knows the command is done.
//
// The two-column layout (Tx tab-separated from Rx) means that if we stop mid-
// line (i.e. cursor is odd after the loop), we emit a trailing CRLF to avoid
// leaving the terminal in a broken state.
//
// Parameters:
//   m                    - output buffer (SCRATCH_SIZE bytes)
//   stale_name           - command name used in the staleness warning prefix
//   title                - header line emitted once at the start (*cursor == 0)
//   cursor               - persistent position across calls; caller owns storage
//   min_remaining        - stop adding rows when fewer than this many bytes remain
//   include_stale_warning- whether to call ff_table_append_stale_warning on first call
//   row_fn               - called once per device to format one table row
static BaseType_t ff_table_print(char *m, const char *stale_name, const char *title,
                                 int *cursor, int min_remaining, bool include_stale_warning,
                                 ff_table_row_fn row_fn)
{
  int copied = 0;
  ff_table_abort_current = false;

  if (*cursor == 0) {
    if (include_stale_warning)
      copied = ff_table_append_stale_warning(m, copied, stale_name);
    copied += snprintf(m + copied, SCRATCH_SIZE - copied, "%s\r\n", title);
  }

  for (; *cursor < NFIREFLIES; ++(*cursor)) {
    copied = row_fn(m, copied, *cursor);
    if (ff_table_abort_current) {
      ff_table_abort_current = false;
      *cursor = 0;
      return pdFALSE;
    }
    if ((SCRATCH_SIZE - copied) < min_remaining) {
      ++(*cursor);
      return pdTRUE;
    }
  }

  if (*cursor % 2 == 1) {
    m[copied++] = '\r';
    m[copied++] = '\n';
    m[copied] = '\0';
  }
  *cursor = 0;
  return pdFALSE;
}

// The following blocks generate the simple alarm/status display commands.
//
// Each get_FF_*_data accessor is a macro defined in MonI2C_addresses.h that
// dispatches between the F1 and F2 firefly banks based on the device index.
// For example, get_FF_LOS_ALARM_data(which) expands to:
//   (which < NFIREFLIES_F1) ? get_FF_F1_LOS_ALARM_data(which)
//                           : get_FF_F2_LOS_ALARM_data(which - NFIREFLIES_F1)
//
// The F1 and F2 functions read from separate in-memory monitoring buffers that
// are populated by the FireflyTask monitoring loop (MonitorTaskI2C.c). All
// values are therefore cached; these commands never perform I2C transactions.
//
// Row functions (FF_U16_HEX_ROW_FN) — one per register of interest.
// REV1 hardware exposes a smaller register set than REV2/REV3; the four
// alarm registers below (TX fault, RX power, temperature alarm, VCC alarm) are
// only present in REV2 and REV3 and are guarded accordingly.
// clang-format off
FF_U16_HEX_ROW_FN(ff_los_alarm_row,  get_FF_LOS_ALARM_data(whichff))  // Loss-of-signal per channel
FF_U16_HEX_ROW_FN(ff_cdr_enable_row, get_FF_CDR_ENABLE_data(whichff)) // Clock/data recovery enable bits
#if defined(REV2) || defined(REV3)
FF_U16_HEX_ROW_FN(ff_tx_fault_alarm_row,    get_FF_TX_FAULT_ALARM_data(whichff))    // Transmitter fault flags
FF_U16_HEX_ROW_FN(ff_rx_power_alarm_row,    get_FF_RX_POWER_ALARM_data(whichff))    // Received optical power alarm
FF_U16_HEX_ROW_FN(ff_temperature_alarm_row, get_FF_TEMPERATURE_ALARM_data(whichff)) // On-device temperature alarm flag
FF_U16_HEX_ROW_FN(ff_vcc_alarm_row,         get_FF_VCC3V3_ALARM_data(whichff))      // 3.3 V supply voltage alarm flag
#endif // REV2 || REV3

// Command functions (FF_TABLE_CMD) — one per CLI command, each paired with its row function above:
FF_TABLE_CMD(ff_los_alarm, ff_los_alarm_row, "FIREFLY LOS ALARM:", 20, true)
#if defined(REV2) || defined(REV3)
FF_TABLE_CMD(ff_tx_fault_alarm,    ff_tx_fault_alarm_row,    "FIREFLY TX FAULT ALARM:",    20, true)
FF_TABLE_CMD(ff_rx_power_alarm,    ff_rx_power_alarm_row,    "FIREFLY RX FAULT ALARM:",    20, true)
FF_TABLE_CMD(ff_temperature_alarm, ff_temperature_alarm_row, "FIREFLY TEMPERATURE ALARM:", 20, true)
FF_TABLE_CMD(ff_vcc_alarm,         ff_vcc_alarm_row,         "FIREFLY VCC ALARM:",         20, true)
#endif // REV2 || REV3
// clang-format on

static int ff_ch_disable_row(char *m, int copied, int whichff)
{
  copied += snprintf(m + copied, SCRATCH_SIZE - copied, "%17s: ",
                     ff_moni2c_addrs[whichff].name);
  if (isEnabledFF(whichff)) {
    uint16_t val = get_FF_CHANNEL_DISABLE_data(whichff);
    copied += snprintf(m + copied, SCRATCH_SIZE - copied, "0x%04x", val);
  }
  else {
    copied += snprintf(m + copied, SCRATCH_SIZE - copied, "  --  ");
  }
  return ff_table_append_row_end(m, copied, whichff);
}

// ff_ch_disable_row has custom logic (prints "--" for disabled devices) so it is
// written out explicitly above; only the boilerplate command wrapper is generated here.
FF_TABLE_CMD(ff_ch_disable_status, ff_ch_disable_row, "FIREFLY CHANNEL DISABLE:", 20, true)

static int ff_cdr_lol_alarm_row(char *m, int copied, int whichff)
{
  copied += snprintf(m + copied, SCRATCH_SIZE - copied, "%17s: ",
                     ff_moni2c_addrs[whichff].name);
  if (isEnabledFF(whichff) && (FireflyType(whichff) == DEVICE_25G12 || FireflyType(whichff) == DEVICE_25G4)) {
    uint16_t val = get_FF_CDR_LOL_ALARM_data(whichff);
    copied += snprintf(m + copied, SCRATCH_SIZE - copied, "0x%04x", val);
  }
  else {
    copied += snprintf(m + copied, SCRATCH_SIZE - copied, "  --  ");
  }
  return ff_table_append_row_end(m, copied, whichff);
}

// ff_cdr_lol_alarm_row and ff_power_alarm_row also have custom logic (they skip
// non-25G devices) so their row functions are written explicitly; command wrappers only:
FF_TABLE_CMD(ff_cdr_lol_alarm, ff_cdr_lol_alarm_row, "FIREFLY CDR LOL ALARM:", 30, true)
FF_TABLE_CMD(ff_cdr_enable_status, ff_cdr_enable_row, "FF CDR Enable:", 20, true)

static int ff_power_alarm_row(char *m, int copied, int whichff)
{
  copied += snprintf(m + copied, SCRATCH_SIZE - copied, "%17s: ", ff_moni2c_addrs[whichff].name);
  if (isEnabledFF(whichff) && (FireflyType(whichff) == DEVICE_25G12 || FireflyType(whichff) == DEVICE_25G4)) {
    uint16_t val0 = get_FF_POWER_ALARM_0_data(whichff);
    uint16_t val1 = get_FF_POWER_ALARM_1_data(whichff);
    copied += snprintf(m + copied, SCRATCH_SIZE - copied, "0x%04x 0x%04x", val0, val1);
  }
  else {
    copied += snprintf(m + copied, SCRATCH_SIZE - copied, "  --        --");
  }
  return ff_table_append_row_end(m, copied, whichff);
}

FF_TABLE_CMD(ff_power_alarm_status, ff_power_alarm_row, "FIREFLY POWER ALARM:", 30, true)

static int ff_temp_row(char *m, int copied, int whichff)
{
  if (isEnabledFF(whichff)) {
    uint8_t val = get_FF_TEMPERATURE_data(whichff);
    copied += snprintf(m + copied, SCRATCH_SIZE - copied, "%17s: %2d", ff_moni2c_addrs[whichff].name, val);
  }
  else {
    copied += snprintf(m + copied, SCRATCH_SIZE - copied, "%17s: %2s", ff_moni2c_addrs[whichff].name, "--");
  }
  return ff_table_append_row_end(m, copied, whichff);
}

// ff_temp_row and ff_v3v3_row have custom formatting (decimal degrees / voltage
// rather than hex) so they are written out explicitly; command wrappers only:
FF_TABLE_CMD(ff_temp, ff_temp_row, "FF Temperature:", 20, true)

// loop over all channels on all devices and show optical power
BaseType_t ff_optpow(int argc, char **argv, char *m)
{
  // takes no arguments
  static int i = 0;
  int copied = 0;
  if (i == 0) {
    copied += snprintf(m, SCRATCH_SIZE, "FF average Optical Power (uW)\r\n");
  }
  for (; i < NFIREFLIES; ++i) {
    bool isTx = (strstr(ff_moni2c_addrs[i].name, "Tx") != NULL);
    if (isEnabledFF(i) && !isTx) {
      float val = getFFavgoptpow(i);
      int tens, frac;
      float_to_ints(val, &tens, &frac);
      copied += snprintf(m + copied, SCRATCH_SIZE - copied, "%17s: % 5d.%02d",
                         ff_moni2c_addrs[i].name, tens, frac);
    }
    else {
      copied += snprintf(m + copied, SCRATCH_SIZE - copied, "%17s:     ---",
                         ff_moni2c_addrs[i].name);
    }
    if (isTx) {
      copied += snprintf(m + copied, SCRATCH_SIZE - copied, "\t\t");
    }
    else {
      copied += snprintf(m + copied, SCRATCH_SIZE - copied, "\r\n");
    }
    if ((SCRATCH_SIZE - copied) < 40) {
      ++i;
      return pdTRUE;
    }
  } // loop over NFIREFLIES
  i = 0;
  return pdFALSE;
}

// show optical power for all channels in a single device
BaseType_t ff_optpow_dev(int argc, char **argv, char *m)
{
  // takes one argument
  BaseType_t whichFF = strtol(argv[1], NULL, 10);
  int copied = 0;
  if (whichFF >= NFIREFLIES) {
    copied += snprintf(m, SCRATCH_SIZE, "%s: choose ff number less than %d\r\n",
                       argv[0], NFIREFLIES);
    return pdFALSE;
  }
  copied += snprintf(m, SCRATCH_SIZE, "FF %s Optical Power (uW)\r\n", ff_moni2c_addrs[whichFF].name);
  int nchannels = FireflyType(whichFF) == DEVICE_25G4 ? 4 : 12;
  for (int i = 0; i < nchannels; ++i) {
    float val = getFFoptpow(whichFF, i);
    int tens, frac;
    float_to_ints(val, &tens, &frac);
    copied += snprintf(m + copied, SCRATCH_SIZE - copied, "Ch %02d: % 5d.%02d\r\n", i, tens, frac);
  }
  return pdFALSE;
}

// this command takes up to two arguments. control firefly devices
BaseType_t ff_ctl(int argc, char **argv, char *m)
{
  // argument handling
  int copied = 0;

  if (argc == 2) {
    snprintf(m + copied, SCRATCH_SIZE - copied, "%s: %s not understood", argv[0], argv[1]);
    return pdFALSE;
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
      int channel = whichFF;
      if (channel > NFIREFLIES)
        channel = NFIREFLIES;
      if (strncmp(argv[1], "cdr", 3) == 0) {
        uint8_t val = 0x00U; // default to off
        if (strncmp(argv[2], "on", 2) == 0) {
          val = 0xFF;
        }
        set_xcvr_cdr(val, channel);
        return pdFALSE;
      }
      else if (strncmp(argv[1], "xmit", 4) == 0) {
        bool disable = true;
        if (strncmp(argv[2], "on", 2) == 0) {
          disable = false;
        }
        disable_transmit(disable, channel);
        return pdFALSE;
      }
      else if (strncmp(argv[1], "rcvr", 4) == 0) {
        bool disable = true;
        if (strncmp(argv[2], "on", 2) == 0) {
          disable = false;
        }
        disable_receivers(disable, channel);
        return pdFALSE;
      }
      // Add here
      else if (strncmp(argv[1], "regr", 4) == 0) {
        if (whichFF == NFIREFLIES) {
          copied += snprintf(m + copied, SCRATCH_SIZE - copied, "%s: cannot read all registers\r\n",
                             argv[0]);
          return pdFALSE;
        }
        // register number
        uint16_t regnum = strtol(argv[2], NULL, 16);
        copied +=
            snprintf(m + copied, SCRATCH_SIZE - copied, "%s: reading FF %s, register 0x%x\r\n",
                     argv[0], ff_moni2c_addrs[whichFF].name, regnum);
        uint8_t value;
        int ret = read_arbitrary_ff_register(regnum, channel, &value, 1);
        if (ret != 0) {
          copied += snprintf(m + copied, SCRATCH_SIZE - copied, "read_ff_reg failed with %d\r\n", ret);
          if (ret == SEM_ACCESS_ERROR) {
            snprintf(m + copied, SCRATCH_SIZE - copied, "please release semaphore \r\n");
          }
          return pdFALSE;
        }
        copied += snprintf(m + copied, SCRATCH_SIZE - copied,
                           "%s: Command returned 0x%x (ret %d - \"%s\").\r\n", argv[0], value,
                           ret, SMBUS_get_error(ret));
      }
      else {
        snprintf(m + copied, SCRATCH_SIZE - copied, "%s: command %s not recognized\r\n", argv[0],
                 argv[1]);
        return pdFALSE;
      }

    }                     // argc == 4
    else if (argc == 5) { // command + five arguments
      // register write. model:
      // ff regw reg# val (0-23|all)
      // register read/write commands
      if (strncmp(argv[1], "regw", 4) == 0) {
        // the two additional arguments
        // register number
        // value to be written
        uint16_t regnum = strtol(argv[2], NULL, 16);
        uint16_t value = strtol(argv[3], NULL, 16);
        uint8_t channel = whichFF;
        if (channel == NFIREFLIES) {
          channel = 0; // silently fall back to first channel
        }
        int ret = write_arbitrary_ff_register(regnum, value, channel);
        if (ret != 0) {
          copied += snprintf(m + copied, SCRATCH_SIZE - copied, "write_ff_reg failed with %d\r\n", ret);
          if (ret == SEM_ACCESS_ERROR) {
            snprintf(m + copied, SCRATCH_SIZE - copied, "please release semaphore \r\n");
          }
          return pdFALSE;
        }
        snprintf(m + copied, SCRATCH_SIZE - copied,
                 "%s: write val 0x%x to register 0x%x, FF %d.\r\n", argv[0], value, regnum,
                 channel);
        whichFF = 0;
        return pdFALSE;
      }
      // end regw
#ifdef FF_TEST_DEBUG
      else if (strncmp(argv[1], "test", 4) == 0) { // test code
        if (whichFF == NFIREFLIES) {
          copied += snprintf(m + copied, SCRATCH_SIZE - copied,
                             "%s: cannot test read all registers\r\n", argv[0]);
          return pdFALSE;
        }
        uint8_t regnum = strtol(argv[2], NULL, 16);   // which register
        uint8_t charsize = strtol(argv[3], NULL, 10); // how big
#define CHARLENGTH 64
        if (charsize > CHARLENGTH) {
          charsize = CHARLENGTH;
        }
        if (whichFF > NFIREFLIES) {
          whichFF = 0;
        }
        char tmp[CHARLENGTH];
        snprintf(tmp, CHARLENGTH, "FF %s (%d)\r\n", ff_moni2c_addrs[whichFF].name, whichFF);
        Print(tmp);
        snprintf(tmp, CHARLENGTH, "Register %d (size %d)\r\n", regnum, charsize);
        Print(tmp);
        uint8_t regdata[CHARLENGTH];
        memset(regdata, 'x', CHARLENGTH);
        regdata[charsize - 1] = '\0';
        int i2c_dev;
        if (whichFF < NFIREFLIES_F1) {
          i2c_dev = I2C_DEVICE_F1;
        }
        else {
          i2c_dev = I2C_DEVICE_F2;
        }
        int ret = read_ff_register(ff_moni2c_addrs[whichFF].name, regnum, &regdata[0], charsize, i2c_dev);
        if (ret != 0) {
          snprintf(tmp, CHARLENGTH, "read_ff_reg failed with %d\r\n", ret);
          Print(tmp);
          return pdFALSE;
        }
        regdata[CHARLENGTH - 1] = '\0'; // Sanity check
        Print((char *)regdata);
        Print("\r\n");
        whichFF = 0;
        return pdFALSE;
      }
      else {
        snprintf(m + copied, SCRATCH_SIZE - copied, "%s: command %s not understood\r\n", argv[0],
                 argv[1]);
        return pdFALSE;
      }
#endif // FF_TEST_DEBUG
    }  // argc == 5
    else {
      snprintf(m + copied, SCRATCH_SIZE - copied, "%s: command %s not understood\r\n", argv[0],
               argv[1]);
      return pdFALSE;
    }
  }
  return pdFALSE;
}

// firefly 3.3V monitor dumper
static int ff_v3v3_row(char *m, int copied, int whichff)
{
  copied += snprintf(m + copied, SCRATCH_SIZE - copied, "%17s: ", ff_moni2c_addrs[whichff].name);
  if (isEnabledFF(whichff)) {
    float val = (float)__builtin_bswap16(get_FF_VCC3V3_data(whichff)) * 100e-6f; // LSB is 100uV
    int tens, frac;
    float_to_ints(val, &tens, &frac);
    copied += snprintf(m + copied, SCRATCH_SIZE - copied, "% 2d.%02d", tens, frac);
  }
  else {
    copied += snprintf(m + copied, SCRATCH_SIZE - copied, " --- ");
  }
  return ff_table_append_row_end(m, copied, whichff);
}

FF_TABLE_CMD(ff_v3v3, ff_v3v3_row, "FF 3V3 Mon:", 50, true)

static int ff_dump_names_row(char *m, int copied, int whichff)
{
  copied += snprintf(m + copied, SCRATCH_SIZE - copied, "%17s: ", ff_moni2c_addrs[whichff].name);
  if (isEnabledFF(whichff)) { // process if enabled
    char name[FF_VENDOR_COUNT_FF12 + 1];
    memset(name, '\0', FF_VENDOR_COUNT_FF12 + 1);
    int type = FireflyType(whichff);
    int startReg = FF_VENDOR_START_BIT_FFDAQ;
    if (type == DEVICE_CERNB || type == DEVICE_25G12) {
      startReg = FF_VENDOR_START_BIT_FF12;
    }
    int ret = 0;
    for (unsigned char c = 0; c < FF_VENDOR_COUNT_FF12 / 4; ++c) { // read name 4 chars at a time
      uint8_t v[4];
      ret += read_arbitrary_ff_register(startReg + 4 * c, whichff, v, 4);
      name[4 * c] = v[0];
      name[4 * c + 1] = v[1];
      name[4 * c + 2] = v[2];
      name[4 * c + 3] = v[3];
    }
    if (ret != 0) {
      ff_table_abort_current = true;
      return copied + snprintf(m + copied, SCRATCH_SIZE - copied, "read failed\r\n");
    }
    copied += snprintf(m + copied, SCRATCH_SIZE - copied, "%s", name);
  }
  else {
    copied += snprintf(m + copied, SCRATCH_SIZE - copied, "--------------");
  }
  return ff_table_append_row_end(m, copied, whichff);
}

// this command takes no arguments
BaseType_t ff_dump_names(int argc, char **argv, char *m)
{
  (void)argc;
  // ensure at compile-time that the vendor count is the same
  static_assert(FF_VENDOR_COUNT_FF12 == FF_VENDOR_COUNT_FFDAQ, "Vendor count mismatch");
  static int i = 0;
  char title[64];
  snprintf(title, sizeof(title), "%s: ID registers", argv[0]);
  return ff_table_print(m, argv[0], title, &i, 45, false, ff_dump_names_row);
}

// read the Firefly firmware register at address 111-113 on page 0. This
// register exists on the CERN-B and 12x25G parts, but does not exist
// on the 4x25G part.
#define FF_FW_REG_ADDR 111
#define FF_FW_REG_SIZE 3
static int ff_fw_reg_row(char *m, int copied, int whichff)
{
  copied += snprintf(m + copied, SCRATCH_SIZE - copied, "%17s: ", ff_moni2c_addrs[whichff].name);
  if (isEnabledFF(whichff) && FireflyType(whichff) != DEVICE_25G4) { // only read if enabled and if not 4x25G
    uint8_t fw_reg[FF_FW_REG_SIZE];
    int ret = read_arbitrary_ff_register(FF_FW_REG_ADDR, whichff, fw_reg, FF_FW_REG_SIZE);
    if (ret != 0) {
      ff_table_abort_current = true;
      return copied + snprintf(m + copied, SCRATCH_SIZE - copied, "read failed\r\n");
    }
    copied += snprintf(m + copied, SCRATCH_SIZE - copied, "0x%02x%02x%02x", fw_reg[0], fw_reg[1], fw_reg[2]);
  }
  else {
    copied += snprintf(m + copied, SCRATCH_SIZE - copied, "   ---   ");
  }
  return ff_table_append_row_end(m, copied, whichff);
}

BaseType_t ff_fw_reg(int argc, char **argv, char *m)
{
  (void)argc;
  static int whichff = 0;
  char title[64];
  snprintf(title, sizeof(title), "%s: Firmware registers", argv[0]);
  return ff_table_print(m, argv[0], title, &whichff, 30, false, ff_fw_reg_row);
}
