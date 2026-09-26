# TODO: Large-Scale Improvements for `projects/cm_mcu`

**Last verified against the tree:** 2026-09-26, branch `fix/ff-register-sizes` @ `9c62f0b`.
Line numbers in the open and retracted items were re-checked against the source at that commit.
Items under **Resolved** quote the code as it stood *before* the fix, so their line numbers are
historical and deliberately left alone.

**Not on this branch:** item 17, and the `UART7IntHandler` half of item 2, describe the ProgCom
work, which lives on `feature/sm_uart` — `ProgComTask.c` does not exist here. Their
`ProgComTask.c` / `InterruptHandlers.c` line numbers stay anchored to `feature/sm_uart` @ `6ee7f58`
and are marked inline.

Item numbers are kept stable across revisions of this file (so #6 is absent, and resolved or
retracted items move to the bottom rather than being deleted — #1 was demoted to Lower Priority and
mostly retracted on 2026-08-12).

## High Priority

### 2. Pervasive Copy-Paste Code — OPEN (partly reduced)

| Location | Duplication | Scope |
|---|---|---|
| `ADCMonitorTask.c:235,253,269,285` | 4 identical ADC trigger/wait/read blocks | ~16 lines each |
| `FireflyUtils.c:385` `getFFoptpow()` | 12 identical `case` blocks in a switch | ~40 lines |
| `ZynqMonTask.c:381` `zm_set_firefly_info()` | 6 nearly identical getter + stale-check blocks | ~70 lines |
| `InterruptHandlers.c:51,94,135` | **3** near-identical UART ISRs (UART0, UART1, UART4) | ~40 lines each |

Three near-identical UART ISRs on this branch; `feature/sm_uart` adds a fourth (`UART7IntHandler`
for ProgCom), so the duplication grows rather than shrinks when that work lands. That makes the
ISRs the best candidate of the four rows.

**Action:** Refactor each into a data-driven loop or a parameterized helper. `getFFoptpow()` becomes
a loop over channel index; the ADC blocks become a loop over a config struct array; the UART ISRs
become one body parameterized by `{base, queue, notify-target}`.

### 3. I2C Semaphore Polling Loop — OPEN

`acquireI2CSemaphoreTime()` (`Semaphore.c:63-78`) calls `xSemaphoreTake()` in a loop up to
`MAX_TRIES` (500, `Semaphore.c:61`) times. FreeRTOS semaphores already block efficiently, and since
the loop passes the *same* `tickWaits` each iteration the effective timeout is `500 × tickWaits` —
almost certainly not what any caller intends.

**Action:** Replace with a single `xSemaphoreTake(s, totalTimeout)`.

### 5. Monitor Task Data Array Indexing — No Bounds Checking — OPEN (one of two fixed)

`MonitorTask.c:134` still computes a flat index with 3-level nesting and no bounds validation:

```c
int index = ps * (args->n_commands * args->n_pages) + page * args->n_commands + c;
```

`MonitorTaskI2C.c` has since gained a `devtype` bound check (`:145-148`) but still carries a
`FIXME: this is backwards` on the `31 - __builtin_clz(devtype_mask)` selection.

**Action:** Add a bounds-check macro or accessor wrapping the index calculation in `MonitorTask.c`.
Separately, resolve the `devtype_mask` FIXME — a wrong device type silently reads the wrong command
table.

## Medium Priority

### 7. CLI Pagination State via `static` Variables — OPEN, but closed for free by REV1 retirement

Nine handlers keep pagination state in function-local `static`s: `BufferCommands.c:40`,
`ClockCommands.c:93`, `FireflyCommands.c:437,580,845,1129,1161`, `PowerCommands.c:84,111`. Related:
`CommandLineTask.c:24` shares one `static char m[SCRATCH_SIZE]` scratch buffer —
same root cause, worse consequence.

**The race exists only on REV1.** The second CLI task is created inside `#ifdef REV1`:

```c
xTaskCreate(vCommandLineTask, "CLIZY", 512, &cli_uart, tskIDLE_PRIORITY + 3, NULL);
#ifdef REV1
xTaskCreate(vCommandLineTask, "CLIFP", 384, &cli_uart4, tskIDLE_PRIORITY + 3, NULL);
#endif // REV1
```
(`cm_mcu.c:283-286`; the `cli_uart4` args are likewise REV1-only, `:253-259`.)

REV2/REV3 run a single CLI task, so the `static`s have exactly one user and there is no race today
on the shipping revisions. **Retiring REV1 closes this item outright** — no
refactor needed.

**Action:** Do nothing now. If REV1 retirement is abandoned, revisit — and then fix the shared
scratch buffer first, since a torn CLI response is worse than confused pagination.

### 8. ADC Task: Hard Assert on Timeout — OPEN

`ADCMonitorTask.c` calls `configASSERT(0)` on notification timeout at `:246`, `:262`, `:279`,
`:296`, and `configASSERT(got == …)` on short reads at `:242`, `:258`, `:275`, `:292`. Any transient
timeout resets the MCU with no diagnostics.

**Action:** Replace the timeout asserts with error logging and graceful degradation (mark affected
channels stale, keep monitoring the rest). The length asserts are arguably fine to keep under DEBUG.

### 9. Watchdog Subsystem is Entirely Dead — OPEN (needs a decision, not a fix)

Not merely "the feed is commented out" — the subsystem is dead at six independent levels, and no
watchdog protection exists today at all:

| Layer | Location | State |
|---|---|---|
| Task creation | `cm_mcu.c:302` | commented out — `WatchdogTask` never runs |
| Hardware feed | `WatchdogTask.c:40` | commented out |
| Hardware peripheral init | *nowhere in the tree* | `WDOG0` is never clocked, enabled, or `WatchdogResetEnable`d; the only `WATCHDOG0_BASE` reference is inside the dead feed |
| Task registration | `MonitorTaskI2C.c:38` | only `kWatchdogTaskID_MonitorI2CTask` registers; `FireFly`, `XiMon`, `PSMon` IDs are declared (`Tasks.h:310-313`) but unused |
| Task feeding | `MonitorTaskI2C.c:226` | the sole `task_watchdog_feed_task()` call site is commented out |
| CLI status | `SoftwareCommands.c:175-181` | `watchdog_ctl` is inside `#if 0` |

**The naive re-enable is guaranteed to brick the board into a reset loop.** Rows 4 and 5 are
inconsistent by construction: `s_registered_tasks` gets bit 1 set, `s_fed_tasks` never does, so

```c
if ((s_fed_tasks & s_registered_tasks) == s_registered_tasks)  // (0 & 2) == 2 → always false
```

always takes the else branch. Uncommenting only `cm_mcu.c:302` + `WatchdogTask.c:40` (plus adding
the missing peripheral init) yields a reset every watchdog period, unconditionally.

Two further hazards for any revival:

- `MonitorTaskI2C` unregisters/re-registers around power-down (`:101`, `:110`) — the correct
  pattern. No other task does. Registering `FireFly`/`XiMon`/`PSMon` without equivalent power-state
  handling would stall the watchdog whenever the board is powered down.
- `task_watchdog_{register,unregister,feed}_task()` use `taskDISABLE_INTERRUPTS()` /
  `taskENABLE_INTERRUPTS()`, which unconditionally re-enable rather than save/restore. Safe in the
  current task-context-only call sites; unsafe the moment a feed is added to an ISR or nested in a
  critical section.

**This needs a decision, not an incremental fix.** Three defensible options:

1. **Delete it.** Remove `WatchdogTask.c`, the four `kWatchdogTaskID_*` enums, the `task_watchdog_*`
   prototypes, the `MonitorTaskI2C` register/unregister calls, and the `#if 0` CLI command. Honest
   about the status quo and stops the file from implying a working safety net exists. Costs nothing
   operationally, since nothing works today.
2. **Coarse watchdog only.** Enable `WDOG0` and feed it from a single low-priority task or the idle
   hook, with no per-task bookkeeping. Catches total scheduler death and priority-inversion
   starvation; does *not* catch one hung task. Near-zero misfire risk, small and reviewable.
3. **Full per-task watchdog.** The original intent. Requires a feed call in every registered task's
   main loop *and* power-state-aware register/unregister for each, or it misfires.

**Option 3 was evaluated and rejected (wittich).** The coordination cost of getting per-task
register/unregister correct across all power states was judged not worth the benefit. The
half-finished state in the tree is the residue of that decision, not an unfinished task — the
subsystem stalled precisely at `MonitorTaskI2C`, the one task where the power-state handling was
worked out. Do not reopen option 3 without new motivation; the reason it looks abandoned is that it
was, deliberately.

Remaining choice is (1) delete or (2) coarse-only. Recommendation: (1), unless coarse scheduler-death
protection is independently wanted — (2) adds a live reset path that has to be right, to catch
failure modes that are visible through other channels on this board.

### 10. `readFFpresentSignals()` — Monolithic Bit Manipulation — OPEN (renamed)

The function was split in `310def6`: `readFFpresent()` (`FireflyUtils.c:281`) is now a 4-line
wrapper, but the body moved to `readFFpresentSignals()` (`:96`), still ~185 lines of hardcoded bit
shifts with no named constants, across three revision branches.

**Action:** Define the bit positions in a per-revision table (array of `{register, shift, mask}`)
and iterate. Turns ~185 lines of fragile bit manipulation into a ~20-line loop. Overlaps with #4.

### 11. Power Supply Task Acknowledged Hacks — OPEN (one FIXME has since gone)

`PowerSupplyTask.c`:

- `:126-132` — force both FPGAs enabled if neither detected (`// HACK` … `// end HACK`)
- `:190` — `// TODO: what about > 1 message` on the `xQueueReceive` in the main loop
- `:213` — `bool ignorefail = false; // HACK THIS NEEDS TO BE FIXED TODO FIXME`
- `:335` — `case POWER_L3ON: { // FIXME allow this transition to fail on Rev2`

The second REV2-transition FIXME previously noted around line 408 is no longer present.

**Action:** Audit each, determine whether hardware changes have resolved the underlying issue, and
either formalize the workaround with documentation or remove it.

### 12. EEPROM Error Handling — OPEN

`EEPROMTask.c` ignores the return codes of `MAP_EEPROMProgram()` (`:28`) and `MAP_EEPROMRead()`
(`:35`, `:44`). A failed write is silently lost. The Block 1 `unlock → write → lock` sequence
(`:79-87`) is not atomic — another task's message could interleave between the unlock and the lock.

**Action:** Check return codes and report errors (error buffer or return queue). Make the
unlock/write/lock sequence a single compound message type to guarantee atomicity.

## Lower Priority

### 1. Firefly Enable-Mask Pair Read Non-Atomically — OPEN (demoted from High; scope corrected)

*This item previously claimed a broad "global mutable state without synchronization" problem across
`currentTemp[]`, `status_T`, `currentState` and the FF masks, and called it the most serious
systemic issue. That was wrong and has been retracted — see "Retracted" below. Only the following
survives, and it is minor.*

`setFFmask()` (`FireflyUtils.c:41-54`) writes two globals in sequence:

```c
ff_USER_mask = current;
ff_PRESENT_mask = data;
```

`isEnabledFF()` (`:287-295`) reads both in one expression:

```c
if (!((1 << ff) & ff_PRESENT_mask) || !((1 << ff) & ff_USER_mask))
```

This is the one place in the listed set with a genuine **two-variable invariant** read without
atomicity, so a reader can observe the new USER mask against the old PRESENT mask. It is a real
cross-task race: `readFFpresent()` → `setFFmask()` is called at **runtime** from
`PowerSupplyTask.c:409` during a power transition (not only at init, as this item used to claim),
while readers span the CLI (`FireflyCommands.c`, 10 sites), the monitor tasks
(`MonUtils.c:199,214`, as `presentCallback`) and, on `feature/sm_uart`, `ProgComTask.c:245`.

**Consequence is negligible.** For at most one cycle a Firefly is misclassified: wrongly enabled →
an I2C read to an absent device → NACK, already handled by the monitor error path; wrongly disabled
→ one skipped sample. Nothing latches, nothing is corrupted.

**Action (optional):** Pack the two masks into a single `uint32_t` pair updated with one store, or
wrap `setFFmask()`/`isEnabledFF()` in a brief `taskENTER_CRITICAL`. Low value — fix only if touching
this code anyway.

**Separate, non-concurrency nit:** `currentState` (`PowerSupplyTask.c:37`) is a non-`static` global
despite `getPowerControlState()` existing. Make it `static` for encapsulation. This is a style fix,
not a race fix.

### 15. I2CSlaveTask Computes Hottest Temperature on Every Read — OPEN

`getSlaveData()` (`I2CSlaveTask.c:48`) loops over all Fireflies at register `0x16` (`:82-101`) and
over all DCDC devices × pages at `0x18` (`:100-119`) on every I2C register read from the external
master. No caching.

**Action:** Cache the hottest-temperature values, updated periodically by the monitor tasks. Note
`AlarmUtilities.c` already computes exactly these maxima into `currentTemp[FF]` and
`currentTemp[DCDC]` every 50 ms — reuse them rather than adding a third copy (coordinate with #1,
which needs to make those reads safe anyway).

### 16. Magic Number Cleanup — OPEN (one reference corrected)

- `MAX_TRIES = 500` (`Semaphore.c:61`) vs `I2C_MAX_TRIES = 25` (`I2CCommunication.c`) — see #3
- `pdMS_TO_TICKS(250)` polling interval (`MonitorTask.c`)
- Hardcoded 5 suppliers: `ZynqMonTask.c:562`, `for (int j = 0; j < 5; ++j) // FIXME hardcoded value`
  (this was previously mis-attributed to `SensorControl.c`)
- `cm_mcu.c:10` — `// TODO: break this out into separate files. Too much clutter here.`

**Action:** Extract into named `#define` constants in the appropriate headers.

### 17. ProgCom I2C Stall Can Starve the UART7 RX Path — OPEN (partial mitigation landed)

*Line numbers in this item are against `feature/sm_uart` @ `6ee7f58`; none of this code is on the
current branch.*

`progcom_access_clk()` (`ProgComTask.c:403`) and `progcom_access_fpga()` (`:452`) each hold a bus
mutex (`i2c2_sem`, `i2c5_sem`) across 3-4 chained I2C transactions (mux select, page select, data,
mux clear). Any transaction hitting an unresponsive/NAK'ing device blocks the full
`I2C_TIMEOUT_MS` (250 ms, `I2CCommunication.c:74,120`) via `ulTaskNotifyTake()` — worst case
~750-1000 ms per ProgCom command, all inside `ProgComTask`, the same task that drains
`xUART7StreamBuffer`.

While stalled, `UART7IntHandler` keeps pushing incoming bytes into the 128-byte buffer with nobody
to service it. `9bbc2aa` ("overflow in uart7") landed detection for the resulting overflow: the ISR
sets `progcom_rx_overflow` (`InterruptHandlers.c:50,120,127`) when `xStreamBufferSendFromISR()`
can't take every byte, and `add_progcom_char()` (`ProgComTask.c:543`) discards through the next line
and replies `"e uart overflow, resync\n"` instead of silently splicing two commands together into a
different, valid-looking one. **That closes the corruption failure mode, not the stall.** Two things
are still true:

- The command in flight during the stall is simply lost — the Zynq side must notice the error
  string and retry; nothing here retries automatically.
- Any other task sharing the same I2C bus (e.g. `MonitorTaskI2C` polling clock-synth or FPGA status)
  blocks on the same mutex for the same window, so one unresponsive ProgCom target can also stall
  unrelated monitoring on that bus.

**Action:** Fully closing this means moving the I2C work off `ProgComTask` onto a separate
queue-driven worker so the task servicing UART7 never blocks on I2C, or bounding the exposure with a
protocol-level ack/retry. Both are larger, cross-cutting changes — treat as a separate follow-up from
the overflow-detection mitigation already landed.

## Retracted

### 4. Hardware Revision Abstraction Layer — DROPPED (superseded by the REV1 retirement plan)

Dropped 2026-08-12. Plan of record is to **retire REV1**; a new abstraction layer is not the right
response to what remains. Measured burden at the time of the decision:

| | REV1 refs | REV2 refs | REV3 refs |
|---|---|---|---|
| `projects/cm_mcu` + `common` | 114 | 166 | 121 |

The three examples this item used to cite do not support it:

- **`PowerSupplyTask.c`** — exactly *one* `#ifdef REV1` (`:359`). The "different power levels per
  revision" claim was overstated.
- **`ADCMonitorTask.c`** — 4 revision sites, but two (`:119`, `:121`) are a REV2-vs-REV3 split.
  Retiring REV1 removes 2 of 4 and leaves the divergence that actually matters.
- **`readFFpresentSignals()`** (`FireflyUtils.c:96`) — a genuine three-way split (`:129`, `:141`,
  `:178`, `:189`, `:230`), reducing to two-way. Real, but this is item #10's problem, not an
  argument for a new layer.

The actual REV1 hotspots were never named by this item: `LocalTasks.c` (20 refs) and `cm_mcu.c` (14).

**The decisive argument is not "REV1 leaves, so it shrinks."** ~30 REV2-vs-REV3 branch sites survive
REV1 retirement, concentrated in `LocalTasks.c`, `FireflyUtils.c`, `MonUtils.c` and the generated
address tables. But that residue is almost entirely **data** — I2C addresses, device counts, clock
chip identities, Firefly populations — and the repo already has *three* mechanisms for exactly that:

- `common/gpio_pins_rev*.def` + `common/pinout_rev*.c` + `TM4C129x_rev*.pinmux`
- YAML-generated `MonI2C_addresses.c` (from `MON_I2C*.yml`)
- YAML-generated `ZynqMon_addresses.c/.h` (from `PL_MEM*.yml`)

A `board_config.h` layer would be a fourth mechanism redundant with these. Note also that there is
no `gpio_pins_rev3.def` / `pinout_rev3.c` — REV3 already shares REV2's, which is evidence the
existing mechanism handles revision sharing fine.

**If revision divergence becomes painful again, extend the existing per-revision `.def` / pinout /
YAML tables. Do not introduce a new abstraction layer.**

**Bonus, tracked in item 7:** the second CLI task instance is REV1-only (`cm_mcu.c:283-286`), so
REV1 retirement also closes item 7's race for free.

### 1a. "Unsynchronized `currentTemp[]` / `status_T` / `currentState`" — NOT A DEFECT

Retracted 2026-08-12 after checking the actual access patterns. Recorded here so it does not get
re-filed from a fresh read of the declarations.

- **Aligned 32-bit access is single-copy atomic on ARMv7-M.** `float`, `uint32_t` and `enum` reads
  never tear. The only possible hazard in this whole class is a mixed-vintage *snapshot* across
  several variables, never a garbage value.
- **`currentTemp[4]` has no cross-task reader whatsoever.** It is `static` (`AlarmUtilities.c:51`)
  with no accessor. All references are inside `TempStatus()` (writes) and `TempErrorLog()` (reads);
  those, plus `TempClearErrorLog()`, are reachable only as function pointers in the `tempAlarmTask`
  struct (`:254-257`), bound to the single `"TALM"` task (`cm_mcu.c:300`). Single task throughout.
- Even hypothetically, a mixed snapshot would be harmless: each sensor is compared against its own
  independent threshold, so there is no cross-element invariant to violate. Getting TM4C from sweep
  *n* and DCDC from sweep *n-1* changes no decision.
- **`status_T` and `warnLatch`** are likewise single-task read-modify-write (only the temp alarm
  task); the volt alarm task uses separate state (`currentVoltStatus[]`, `AlarmUtilities.c:284`).
  The sole cross-task read is `getTempAlarmStatus()` from `AlarmCommands.c:44` (CLI) — an atomic
  `uint32_t` load feeding a diagnostic printout. A mutex could not make that value fresher than the
  50 ms alarm period already does.
- **`currentState`** has ~18 readers across many tasks, but every consumer does
  `if (getPowerControlState() != POWER_ON)`. That is an inherent TOCTOU: the state may change the
  instant after the check whether or not a lock is held. Locking cannot fix it; re-checking each
  cycle (which the monitor tasks already do) is the correct and existing mitigation.

**Lesson for future entries in this file:** "no lock is held" is not a finding. A concurrency item
needs a concrete failure mode — a torn value, a lost update, or a violated multi-variable invariant
— demonstrated against real call sites. Absent that, do not file it.

## Resolved

### 13. Misplaced GPIO Logic in LED Task — RESOLVED

`LedTask.c` no longer references `BLADE_POWER_OK` or handles `PS_BAD`/`PS_GOOD`. The GPIO writes now
live in `common/power_ctl.c` (`:173`, `:188`, `:205`, `:242`), driven from the power path. The LED
task only drives LEDs.

### 14. ZynqMonTask `zm_set_psmon()` Bug

Fixed by PR #286

Line 580 uses loop variable `l` instead of data index `ll` in the NaN-check condition.

**Action:** Fix `data[l].data.f` to `data[ll].data.f`.

### 21. SMBus ISR Silently Overwrites Real NACK Status with `SMBUS_OK`

(Fixed by PR #293)

`SMBusMasterIntHandlerCore()` (`InterruptHandlers.c:195`) does `*status = SMBusMasterIntProcess(master);`
unconditionally, on every interrupt for a transaction. For a device that address-NACKs (e.g. an
unprogrammed FPGA SLR sysmon address), the *first* interrupt correctly returns `SMBUS_ADDR_ACK_ERROR`
(`common/smbus.c:2406`), but the transfer-in-progress flag is still set at that point (a STOP is issued
instead, `smbus.c:2387-2389`), so `SMBusMasterIntHandlerCore` does not yet notify the waiting task. The
STOP's completion fires a *second* interrupt, which lands in the `SMBUS_STATE_IDLE` case
(`smbus.c:2433-2451`), clears the flag, and falls through to the function's default `return(SMBUS_OK)`
(`smbus.c:3250`). That second write to `*status` clobbers the real error, and — because it coincides with
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

### 20. Duplicate `RX_POWER_ALARM` / `POWER_ALARM_0` Entries

`MON_I2C_rev{2,3}.yml` defined `RX_POWER_ALARM` and `POWER_ALARM_0` as byte-for-byte identical entries
(same address, size, mask, devicetypes) — `POWER_ALARM_0` was added later as part of a rename/rework
and the old entry was never removed. The duplication was not harmless: these are latched clear-on-read
registers, and `MonitorTaskI2C` walks the command table in order within one poll pass, so the earlier
`RX_POWER_ALARM` read cleared the bytes and the later `POWER_ALARM_0` read stored 0. `val0` in
`ff_power_alarm_status` therefore read `0x0000` on every 25G4 and 25G12 part, always.

Resolved by deleting `RX_POWER_ALARM` (kept `POWER_ALARM_0`/`POWER_ALARM_1`, which together cover the
full 3-byte 25G12 field at bytes 14-16; `RX_POWER_ALARM` only ever read 2 of the 3) and repointing
`ff_rx_power_alarm_row` at `get_FF_POWER_ALARM_0_data`.

## FireFly Register Map (from the 2026-09-12 CERNB/25G4/25G12 audit)
Found while fixing the 25G4 `LOS_ALARM`/`CDR_LOL_ALARM` address/size bug (see the PR for `fix/ff-register-sizes`). These were deliberately deferred rather than folded into that fix — two
are structural and one is an open question needing hardware knowledge. (Item 20, a duplicate-entry
cleanup, was resolved on 2026-09-26 and has moved to **Resolved** above.)

### 18. FireFly Tx/Rx Sub-Device Register Collision (CERNB, 25G12)

CERNB and 25G12 optical modules are each wired as two separate I2C sub-devices — Tx at address 0x50, Rx at
0x54 (`LocalTasks.c`) — but `MonUtils.c`'s `FireflyType()` assigns both sides the same device-type bit, so
every register-table entry in `MON_I2C_rev{2,3}.yml` is polled identically on both. Most entries really are
identical on both sides (`STATUS_REG`, `TEMPERATURE_ALARM`, `VCC3V3_ALARM`, `TEMPERATURE`, `VCC3V3` —
confirmed against all three datasheets), but two are not:

- `LOS_ALARM` for CERNB: correct on the Rx map (addr 7-8) but reads *reserved* memory on the Tx map — the
  real Tx-side laser-fault register (addr 9-10) has no entry of its own for CERNB at all, so CERNB's
  Tx-side fault alarm is never read by any entry today.
- `TX_FAULT_ALARM` / `POWER_ALARM_0` for 25G12: each is correctly addressed for one
  side (Tx fault at addr 9, Rx power at addr 14) but polled on both, so `ff_tx_fault_alarm` shows garbage
  on Rx devices and `ff_rx_power_alarm`/`ff_power_alarm_status` show garbage on Tx devices.

`ff_optpow` (`FireflyCommands.c`, ~line 851) already has an `isTx = strstr(name,"Tx")` guard for exactly
this kind of split — the likely pattern to extend, or a device-type split (e.g.
`DEVICE_25G12_TX`/`DEVICE_25G12_RX`), done carefully so as not to break the entries that are genuinely
shared.

**Action:** Design a Tx/Rx-aware register mechanism (display-side filter or device-type split) and add the
missing CERNB Tx-side fault-alarm entry.

### 19. 25G12 `OPT_POWER_CH1`..`CH12` Channel-Number Order (needs schematic review)

Firmware's channel numbering runs opposite to the datasheet's Rx0..Rx11 address order (CH1→datasheet's
Rx11, CH12→Rx00), while the 25G4 table runs forward (CH1→Rx1). Every individual address is itself
correctly sized/paged — nothing is misread — so this may be an intentional physical-fiber-numbering
convention rather than a bug.

**Action:** Confirm against the board schematic/pinout whether "channel 1" as displayed by `ff_optpow` is
meant to match the physical fiber labeled 1; fix the YAML ordering if not.
