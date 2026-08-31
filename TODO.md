# TODO: Large-Scale Improvements for `projects/cm_mcu`

## High Priority

### 1. Thread Safety: Global Mutable State Without Synchronization

The most serious systemic issue. Multiple shared variables are written by one task and read by others with no locking:

- `currentTemp[]`, `status_T` in `AlarmUtilities.c` — written by alarm callbacks, read by status checks
- `currentState`, `external_alarm` in `PowerSupplyTask.c` — read by alarm, CLI, and Zynq tasks
- `ff_PRESENT_mask`, `ff_USER_mask` in `FireflyUtils.c` — written in InitTask, read in alarm/monitor tasks

**Action:** Introduce mutexes (or use `taskENTER_CRITICAL`/`taskEXIT_CRITICAL` for short reads) to protect shared state. Consolidate related globals into structs with an associated mutex, e.g. a `struct alarm_state` protected by a single mutex.

### 2. Pervasive Copy-Paste Code

Several subsystems have nearly identical blocks repeated many times:

| Location | Duplication | Scope |
|---|---|---|
| `ADCMonitorTask.c` | 4 identical ADC trigger/wait/read blocks | ~70 lines each |
| `FireflyUtils.c:getFFoptpow()` | 12 identical `case` blocks in a switch | ~50 lines |
| `ZynqMonTask.c:zm_set_firefly_info()` | 6 nearly identical getter+stale-check blocks | ~70 lines |
| `InterruptHandlers.c` | 3 identical UART ISRs (UART0, UART1, UART4) | ~20 lines each |

**Action:** Refactor each into a data-driven loop or a parameterized helper function. For example, `getFFoptpow()` becomes a simple loop over channel index; the ADC blocks become a loop over a config struct array.

### 3. I2C Semaphore Polling Loop

`acquireI2CSemaphoreTime()` in `Semaphore.c` calls `xSemaphoreTake()` in a loop up to 500 times, sleeping between attempts. FreeRTOS semaphores already block efficiently.

```c
// Current: polls 500 times
while (xSemaphoreTake(s, tickWaits) == pdFALSE) {
    if (++tries > MAX_TRIES) { ... }
}
```

**Action:** Replace with a single `xSemaphoreTake(s, totalTimeout)`.

### 4. Hardware Revision Abstraction

`#ifdef REV1` / `#ifdef REV2` / `#ifdef REV3` conditionals are scattered through many files, especially `readFFpresent()` (~170 lines of revision-specific bit-shifting), `PowerSupplyTask` (different power levels), and `ADCMonitorTask` (different channel counts).

**Action:** Create a board abstraction layer — a `board_config.h` per revision that defines tables/constants (I2C address maps, ADC channel lists, power supply masks, Firefly bit positions). Task code reads from these tables instead of branching on `#ifdef`. This makes adding a REV4 much easier and makes all revision paths testable.

### 5. Monitor Task Data Array Indexing — No Bounds Checking

Both `MonitorTask.c` and `MonitorTaskI2C.c` compute array indices with 3-level nesting:

```c
int index = ps * (n_commands * n_pages) + page * n_commands + c;
```

No bounds validation. A configuration change could cause a silent buffer overflow.

**Action:** Add a bounds-check macro or function wrapping the index calculation. Consider a struct-based accessor instead of raw flat-array indexing.

## Medium Priority

### 7. CLI Pagination State via `static` Variables

Commands that produce multi-line output use `static int i = 0` for pagination state. In REV1 there are two CLI task instances sharing these statics — a race condition.

**Action:** Move pagination state into a per-instance context struct passed to command handlers, or into the `CommandLineTask` instance data.

### 8. ADC Task: Hard Assert on Timeout

`ADCMonitorTask.c` calls `configASSERT(ulNotificationValue == 1)` after waiting for ADC conversion. If any conversion times out (even transiently), the MCU resets with no diagnostics.

**Action:** Replace the assert with error logging and graceful degradation (mark affected channels as stale, continue monitoring the rest).

### 9. Watchdog Task is Disabled

`WatchdogTask.c` has the actual hardware watchdog feed commented out (`//hardware_watchdog_feed()`). The task runs, tracks check-ins, but never actually resets the hardware watchdog.

**Action:** Either enable it properly (with appropriate task registration across all tasks) or remove the dead code. A disabled watchdog in production firmware is a reliability gap.

### 10. `readFFpresent()` — Monolithic Bit Manipulation

This ~170-line function contains hardcoded bit shifts (`<< 23`, `<< 21`, `<< 17`, etc.) with no named constants, deeply nested across three revision branches.

**Action:** Define the bit positions in a per-revision table (array of structs with `{register, shift, mask}`) and iterate over it. This turns 170 lines of fragile bit manipulation into a ~20-line loop.

### 11. Power Supply Task Acknowledged Hacks

`PowerSupplyTask.c` contains several self-documented hacks and FIXMEs:

- Lines 126-132: Force both FPGAs enabled if neither detected (`// HACK`)
- Line 213: `// HACK THIS NEEDS TO BE FIXED TODO FIXME`
- Lines 335, 408: additional FIXMEs about REV2 transitions

**Action:** Audit each hack, determine if the underlying issue has been resolved by hardware changes, and either formalize the workaround with proper documentation or remove it.

### 12. EEPROM Error Handling

The EEPROMTask gatekeeper ignores return codes from ROM EEPROM functions. A failed write is silently lost. The `unlock -> write -> lock` sequence for Block 1 is not atomic — another task could interleave.

**Action:** Check return codes and report errors (via error buffer or return queue). Consider making the unlock/write/lock sequence a single compound message type to guarantee atomicity.

## Lower Priority

### 13. Misplaced GPIO Logic in LED Task

`LedTask.c` handles `PS_BAD`/`PS_GOOD` messages by writing `BLADE_POWER_OK` GPIO directly. This belongs in `PowerSupplyTask`.

**Action:** Move GPIO writes to PowerSupplyTask; LED task should only control LEDs.

### 14. ZynqMonTask `zm_set_psmon()` Bug

Line 580 uses loop variable `l` instead of data index `ll` in the NaN-check condition.

**Action:** Fix `data[l].data.f` to `data[ll].data.f`.

### 15. I2CSlaveTask Computes Hottest Temperature on Every Read

`getSlaveData()` loops over all Firefly or DCDC devices on every I2C register read from the external master. No caching.

**Action:** Cache hottest-temperature values, updated periodically by the monitor tasks.

### 16. Magic Number Cleanup

Retry counts, delays, and channel indices are scattered throughout the codebase as bare literals. Examples:

- `MAX_TRIES = 500` (Semaphore.c) vs `I2C_MAX_TRIES = 25` (I2CCommunication.c)
- `pdMS_TO_TICKS(250)` polling interval (MonitorTask.c)
- Hardcoded `5` suppliers in loop (SensorControl.c)

**Action:** Extract all magic numbers into named `#define` constants in appropriate headers.

### 17. SMBus ISR Silently Overwrites Real NACK Status with `SMBUS_OK`

`SMBusMasterIntHandlerCore()` (`InterruptHandlers.c:198`) does `*status = SMBusMasterIntProcess(master);`
unconditionally, on every interrupt for a transaction. For a device that address-NACKs (e.g. an
unprogrammed FPGA SLR sysmon address), the *first* interrupt correctly returns `SMBUS_ADDR_ACK_ERROR`
(`common/smbus.c:2406`), but the transfer-in-progress flag is still set at that point (a STOP is issued
instead, `smbus.c:2387-2389`), so `SMBusMasterIntHandlerCore` does not yet notify the waiting task. The
STOP's completion fires a *second* interrupt, which lands in the `SMBUS_STATE_IDLE` case
(`smbus.c:2433-2451`), clears the flag, and falls through to the function's default `return(SMBUS_OK)`
(`smbus.c:3249`). That second write to `*status` clobbers the real error, and — because it coincides with
the flag finally clearing — it's the value the waiting task actually observes via `i2c_wait_for_transfer`.

Effect: `apollo_i2c_ctl_reg_w`/`_r` (`I2CCommunication.c`) report `SMBUS_OK` for transactions that
genuinely NACK'd. Every `r != SMBUS_OK` check downstream (e.g. the stale-data invalidation in
`MonitorTask.c`) is checking a status that was already corrupted before it got there, so those checks
never fire. Concretely: `fpga_args.pm_values[]` (`FPGACommands.c`'s `fpga` command, and the max-temp scan
in `AlarmUtilities.c`) never gets invalidated when an FPGA SLR goes from programmed back to absent after
a reset/reprogram — the last successfully-read temperature just sits there looking live.

This is a shared ISR used by all six I2C buses, not FPGA-specific — already flagged as open item #3 in
[`i2c_lockup_notes.md`](i2c_lockup_notes.md) ("NACK error recovery overwrites error status with
SMBUS_OK"), which notes it "masks real errors" but was not part of the fixes landed in PR #284.

**Action:** Latch the genuine error status (`SMBUS_ADDR_ACK_ERROR`/`SMBUS_DATA_ACK_ERROR`) the first time
it's seen and don't let the subsequent STOP-completion interrupt's default `SMBUS_OK` overwrite it —
e.g. only assign `*status` in `SMBusMasterIntHandlerCore` when the transfer is actually complete, or have
`SMBusMasterIntProcess` remember a latched error across calls for the same transaction. Needs care: this
is core interrupt-handler code touching every I2C bus in the system.
