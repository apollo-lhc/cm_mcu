# I2C Rare Lockup / PERIPHERAL_BUSY Analysis

**Branch:** `feature/i2c_speedup`
**Soak-test baseline firmware:** `06f23fb` (`v0.99.22-7-g06f23fb`, "claude.md error fix", 2026-05-30) —
this is the build the overnight soak ran on. Return to it with `git checkout 06f23fb` to reproduce the
baseline behavior before the item-2/item-3 fixes below were applied.  
**Observed symptom:** Occasional `SMBUS_PERIPHERAL_BUSY` returns from `SMBusMasterXxx` initiation
functions, approximately every 13 minutes (high variance) despite hundreds of I2C transactions per
second.

---

## Background: notification-based I2C

The branch switched from polling (`SMBusStatusGet` spin loop) to FreeRTOS task notifications.  The
pattern in `I2CCommunication.c`:

```c
i2c_arm_notify_slot(device);        // stores current task handle in TaskNotifySMBus[device]
r = SMBusMasterXxx(...);            // initiates hw transfer; SMBUS_PERIPHERAL_BUSY if hw busy
if (r == SMBUS_OK) {
    i2c_wait_for_transfer(device);  // ulTaskNotifyTake, 250 ms FreeRTOS timeout
    r = *eStatus[device];           // ISR wrote final status here
} else {
    TaskNotifySMBus[device] = NULL;
}
```

The ISR (`SMBusMasterIntHandlerCore`, `InterruptHandlers.c:194`) calls `SMBusMasterIntProcess`,
then — if `FLAG_TRANSFER_IN_PROGRESS` is cleared **and** `TaskNotifySMBus[device] != NULL` —
calls `vTaskNotifyGiveFromISR` and nulls the slot.

`SMBUS_PERIPHERAL_BUSY` is returned by every `SMBusMasterXxx` initiation function when
`MAP_I2CMasterBusy()` (hardware BUSY bit) returns true at function entry.

---

## Root cause candidates

### 1. Hardware-timeout ISR notifies task before auto-STOP completes (primary suspect)

**Where:** `SMBusMasterIntProcess`, `smbus.c:2312–2331`

```c
if (ui32IntStatus & I2C_MASTER_INT_TIMEOUT) {
    MAP_I2CMasterIntClearEx(..., I2C_MASTER_INT_TIMEOUT | I2C_MASTER_INT_DATA);
    HWREGBITB(&psSMBus->ui16Flags, FLAG_TRANSFER_IN_PROGRESS) = 0;
    return SMBUS_TIMEOUT;
    // comment: "peripheral will automatically issue a stop, just clear and return"
}
```

**Race:**
1. SMBus 35 ms hardware timeout fires (slave stretching SCL, or bus anomaly — consistent
   with HL-LHC environment).
2. ISR clears `FLAG_TRANSFER_IN_PROGRESS`, returns `SMBUS_TIMEOUT`.
3. `SMBusMasterIntHandlerCore` sends task notification; `portYIELD_FROM_ISR` context-switches
   to the waiting task within ~1–5 µs (80 MHz CPU).
4. Hardware is still issuing its automatic STOP on the bus (~2.5–20 µs at 400–100 kHz).
5. Task calls the next `apollo_i2c_ctl_xxx` → `SMBusMasterXxx` → `MAP_I2CMasterBusy()` = **true**
   → **`SMBUS_PERIPHERAL_BUSY`**.

**Why rare:** the 35 ms hardware timeout only fires when a slave holds SCL low past the SMBus
limit; the context-switch window vs. STOP duration is only a few microseconds.

**Fix direction:** after the timeout ISR, spin-wait on `MAP_I2CMasterBusy()` before notifying,
or add a brief guard in `i2c_wait_for_transfer` / at the start of each `apollo_i2c_ctl_xxx`.

---

### 2. `SMBUS_STATE_IDLE` conditional flag clear creates a dead end for write transactions

**Where:** `SMBusMasterIntProcess`, `smbus.c:2433–2451`

```c
case SMBUS_STATE_IDLE:
    if (!MAP_I2CMasterBusy(psSMBus->ui32I2CBase))   // conditional — unlike every other state
        HWREGBITB(&psSMBus->ui16Flags, FLAG_TRANSFER_IN_PROGRESS) = 0;
    break;
```

Write-only transactions end in `WRITE_FINAL` → `BURST_SEND_FINISH` → state set to `IDLE`.  The
subsequent DATA ISR enters this case.  All other terminal states (`READ_WAIT`, `READ_ERROR_STOP`)
clear `FLAG_TRANSFER_IN_PROGRESS` unconditionally.

If the DATA interrupt fires while the hardware BUSY bit is still asserted (possible if the
interrupt assertion and BUSY-clear are not perfectly synchronous), the flag stays set and **no
further ISR is generated**.  The FreeRTOS 250 ms timeout then fires:

1. `TaskNotifySMBus[device] = NULL` (timeout handler).
2. Caller starts next transaction: `i2c_arm_notify_slot(device)` sets a new non-NULL handle.
3. Late ISR from the previous stuck transaction fires in this window → sees non-NULL handle →
   sends **spurious notification** → nulls the slot.
4. The new `SMBusMasterXxx` succeeds (hardware is now idle).
5. `ulTaskNotifyTake` returns immediately on the spurious count without waiting for the new ISR.
6. Task reads stale `*eStatus`, proceeds to another transaction.
7. New transaction's ISR fires, sees NULL handle, does not notify.
8. Yet another transaction starts: hardware still in previous transfer → **`SMBUS_PERIPHERAL_BUSY`**.

---

### 3. NACK error recovery overwrites error status with `SMBUS_OK` (latent correctness bug)

**Where:** `SMBusMasterIntProcess`, `smbus.c:2369–2421`

On address or data NACK with bus still busy, the ISR issues `BURST_SEND_ERROR_STOP` and returns
the NACK error **without** clearing `FLAG_TRANSFER_IN_PROGRESS`.  A second ISR fires (IDLE state
after STOP), clears the flag, and returns `SMBUS_OK`.  `SMBusMasterIntHandlerCore` overwrites
`*status` with `SMBUS_OK`; the task sees a successful transaction despite the NACK.

Not a direct cause of PERIPHERAL_BUSY, but masks real errors.

---

### 4. Double-logging / missing early return on initiation failure

**Where:** all four `apollo_i2c_ctl_xxx` functions in `I2CCommunication.c`

When `SMBusMasterXxx` returns a non-OK status (initiation failed), the `else` branch logs the
error but then falls through to a second `if (r != SMBUS_OK)` block that logs again.  Adding
`return r` in the `else` branch eliminates the duplicate log and makes the early exit explicit.
`apollo_pmbus_rw` already handles this correctly.

---

## Summary table

| # | Location | Mechanism | Direct cause of PERIPHERAL_BUSY? |
|---|----------|-----------|----------------------------------|
| 1 | `smbus.c:2312` timeout ISR | task resumes before auto-STOP finishes | **Yes — primary** |
| 2 | `smbus.c:2440` IDLE case | stuck flag → FreeRTOS timeout → spurious late notification | Yes — secondary |
| 3 | `smbus.c:2369` NACK recovery | error status overwritten with OK | No (correctness bug) |
| 4 | `I2CCommunication.c` else branches | double log, no early return | No — **fixed in `c8f8c45`** (early `return r`) |

---

## Overnight run analysis (2026-05-29 14:30 → 05-30 08:26, ~17.9 h)

Captured on commit `c8f8c45` ("chase rare errors in i2c transactions"). **Important:** the board
was running a *dirty/uncommitted* build of that same code, so the log already includes the
TCA9548A mux-reset mitigation and the early-`return r` fix. The overnight log is therefore an
unintended A/B test of the mux-reset.

### Frequency

- 77 `PERIPHERAL_BUSY` events / 17.9 h ⇒ **mean spacing 14.2 min** (matches the "~13 min" estimate).
- Inter-arrival stdev (13.1 min) ≈ mean (14.2 min) ⇒ **memoryless / Poisson** — sporadic random
  event, not periodic.
- First half 34 vs second half 43, ~4/hr every hour ⇒ **stationary, no overnight degradation**
  (the bus is not slowly wedging).

### Breakdown by instance / op / slot

| Instance | Bus (`dev`) | Events | Failing op | Dominant slot (`ps`) |
|----------|-------------|--------|------------|----------------------|
| CLK   | 2 | 40 (52%) | **mux write** (single-byte W) | ps=3 (33/40) |
| FF_F2 | 3/4 | 21 (27%) | **read** CHANNEL_DISABLE | ps=1 (15) |
| FF_F1 | 3/4 | 16 (21%) | **read** CHANNEL_DISABLE | ps=7 (10), ps=9 (6) |

`ps` is the **loop index** into the instance's device table (`MonitorTaskI2C.c` lines 105/143/160),
**not** the I2C slave address. The failing transaction is always the *first one issued right after
the previous one completed*: CLK's mux-select (line 105) follows the prior device's mux-clear
(line 166); FF's CHANNEL_DISABLE is the first read after the mux+page write→read turnaround. Both
are classic STOP-race positions.

### Why this points at the on-chip master FSM (hypothesis b), not the external mux (a)

1. Each instance is one task on its own I2C peripheral, holding that bus's mutex for the whole
   sweep ⇒ no cross-task contention. `PERIPHERAL_BUSY` is `MAP_I2CMasterBusy()` at the *top* of
   `SMBusMasterXxx`, **before any bus access** (`SMBUS_BUS_BUSY`/`ADDR_ACK` come later). So it is
   the on-chip master not yet idle from the *previous* transaction — the auto-STOP / state-machine
   race.
2. **The mux-reset mitigation was live and did not help** — rate stayed flat (~4/hr) with no decay.
   A wedged TCA9548A would have been cured or changed signature by the reset.
3. **Failures are isolated, not runs.** A stuck mux/bus would fail consecutive 250 ms sweeps until
   reset; instead we see single events minutes apart, each followed by clean sweeps ⇒ the bus was
   never staying wedged.
4. The mux-reset resets the external chip, but `PERIPHERAL_BUSY` originates on the TM4C master
   before it touches the mux ⇒ likely a no-op for this status (keep it only as a `BUS_BUSY`
   fallback).

**Caveat the log cannot resolve:** we can't *prove* the reset wasn't silently rescuing a real wedge
each time (turning a would-be hang into one logged event). The diagnostic below settles it.

### Diagnostic plan (to confirm a vs b on the next run)

Instrument `I2CCommunication.c`:
- **Bounded idle-wait** folded into `i2c_arm_notify_slot` (the choke point all wrappers + 
  `apollo_pmbus_rw` use before initiating): `while (MAP_I2CMasterBusy(base) && us < 250) DELAY_US(1)`.
  If errors vanish across **all** instances/slots ⇒ confirmed (b); demote mux-reset to a fallback.
  If a specific `ps` still fails ⇒ that physical device is marginal (a).
- **Capture raw `MCS` at the failure site, before the mux reset toggles:** log `BUSY` vs `BUSBSY`
  (`HWREG(base + I2C_O_MCS)`, bits `I2C_MCS_BUSY` / `I2C_MCS_BUSBSY` from `inc/hw_i2c.h`).
  `BUSY` only ⇒ on-chip ⇒ (b); `BUSBSY` ⇒ line held externally ⇒ (a).
- **Close the logging gap:** the read path (`I2CCommunication.c:121`) logs neither bus nor slave
  address; add `address` so failures map to a physical port, not just a `ps` slot index.

---

## Companion anomaly: a `log_debug` printed once at `FTL` level (silent memory corruption)

Observed in the same overnight log, exactly once:

```
202605291910 MONI2C FTL MonitorTaskI2C.c:125:FF_F2: command OPT_POWER_CH7 not for device 8 (mask: 8 this: 4)
```

The source at that line is a **`log_debug`** call (level `LOG_DEBUG == 4`), yet it printed and was
tagged `FTL` (`LOG_FATAL == 0`). Debug is normally suppressed for `MONI2C`.

### Why this is proof of a flipped value, not a config change

In `common/log.c`:
- gate (line 326): `if (!L.quiet && level <= L.level[facility])`
- tag (line 200): `level_strings[ev->level]`, with `level_strings[0] == "FTL"`

For the line to **print and read `FTL`**, `level` had to be `0` both at the gate and in `ApolloLog`
(`ev.level` is set `= level` at line 317, before both uses). `0 <= L.level[fac]` is true for any
threshold — which is exactly why a normally-suppressed debug line printed. So a single transient
corruption flipped the `level` value **4 → 0** inside `log_log`'s stack frame. Logging merely
*surfaced* a stray write into the calling task's stack frame; it is not a logging bug.

### Ruled out
- **Not a log-config change**: no `log_set_lock` is ever installed, no level change.
- **Not interrupt-priority misconfig**: all I2C ISRs are at `configKERNEL_INTERRUPT_PRIORITY`
  (`cm_mcu.c:152–178`), same as ADC/UART → legal for `...FromISR`.
- **Not classic task-stack overflow**: `configCHECK_FOR_STACK_OVERFLOW=2` is on and high-water shows
  >100 words free on every MonitorI2C task. (But high-water/end-canary do **not** catch a *localized*
  OOB write that lands inside the still-valid stack — so this does not exonerate a stray write.)

### Likely cause and relation to PERIPHERAL_BUSY
The `FTL` flip needs a write into the **task's own stack frame** (a stack local). That points away
from the ISR (context is HW-saved, priorities correct) and toward a **localized OOB write in the
MonitorI2C task context**, most economically fed by the *same* not-airtight completion handshake
(#2/#3): a spurious/late notification or NACK-overwritten-as-OK lets `ulTaskNotifyTake` return with
**stale/garbage data**, which is then used as an index/size. Amplifiers in the path:
- `devtype = 31 - __builtin_clz(devtype_mask)` (`MonitorTaskI2C.c:119`): if `devtype_mask == 0`,
  `__builtin_clz(0)` is **undefined** → huge `devtype` → wild `commands[c].page[devtype]` /
  `command[devtype]` indexing.
- data-derived indices into `storeData(..., device)` etc.

Contributing latent bug (parallel, not the flip itself): **`log_log` is not thread-safe** — no lock
installed, so every task writes the shared global ring buffer `b`/`log_buffer` with no mutual
exclusion. Benign under the old `vTaskDelay` cadence; genuinely concurrent now. Corrupts globals
(saved text), not `ev.level`, so it does not explain the `FTL` flip — but remove it to de-noise.

**Verdict:** treat PERIPHERAL_BUSY and the `FTL`/corruption as the **same family, different
severity** — both new since dropping the polling wait, both rooted in the ISR/notification handshake.
The busy error is benign; the corruption is the dangerous cousin.

### Investigation ideas
1. **`-fstack-protector-strong`** on `MonitorTaskI2C.c` + `MonUtils.c` to trap a localized stack
   overrun at function return; or a J-Link **DWT data watchpoint** on the target once reproducible.
2. **Paint the MSP/system (startup) stack** and scan low-water — `configCHECK_FOR_STACK_OVERFLOW`
   does not cover handler-mode MSP. (Ranked lower: MSP overflow would hit globals, not `ev.level`.)
3. **Install `log_set_lock`** (mutex / critical section) to remove the unprotected shared-buffer
   corruptor as a confound.
4. **Guard the UB**: `if (devtype_mask == 0) continue;` before `__builtin_clz`, and bounds-check
   `devtype` against the type-table size. Verify `FireflyType*/ClockType` cannot transiently return 0.
5. **Test the shared root**: apply the #2/#3 handshake hardening; if the `FTL`/corruption anomalies
   vanish together with reduced PERIPHERAL_BUSY, the common cause is confirmed.
6. **Correlate with more data**: log a monotonic counter + instance on every busy event and every
   anomaly; check whether corruption events cluster near busy events on the same bus. (The single
   `FTL`, 19:10 FF_F2, sits among FF_F2 busy errors at 18:54 / 19:20 — suggestive, not conclusive.)

---

## Semaphore / direct-hardware audit (cm_mcu only) and fixes applied

Audit of every I2C call site against the per-bus mutex discipline
(bus1=`i2c1_sem`/dcdc, bus2/clk, bus3/ff_f2, bus4/ff_f1, bus5/fpga). No task-context code pokes the
I2C hardware registers directly — all `HWREG`/`MAP_I2C`/`ROM_I2C` touches are in the ISR or one-time
init before the scheduler. Three findings:

1. **Raw CLI commands bypass the semaphore — by design.** `commands/I2CCommands.c`
   (`i2cr`/`i2crr`/`i2cw`/`i2cwr`/`i2c_scan`) take no mutex so they can be composed under a manually
   held lock via `sem_ctl <bus> take/release`. Verified this still works under the notification
   driver: the UART stream buffer and the I2C `ulTaskNotifyTake` share task-notification index 0 but
   never overlap (the stream buffer only notifies a task parked in `xStreamBufferReceive`). Caveat +
   robust fix (own notification index) documented in `projects/cm_mcu/README.md`. **Left as-is.**

2. **`snapdump` leaked `i2c1_sem` on three error returns** (`LocalTasks.c`). **FIXED** — converted to a
   single `cleanup:` label that always releases the mutex.

3. **`MonitorTask` (dcdc bus 1, fpga bus 5) and `snapdump` used the old polling completion**
   (`SMBusStatusGet` + `vTaskDelayUntil`) and direct `SMBusMaster*` calls, bypassing the `apollo_*`
   wrappers and the notification handshake. **FIXED** — both now go through `apollo_i2c_ctl_*`:
   - Added `apollo_i2c_ctl_block_r()` (variable-length SMBus block read via the notification
     handshake) and exposed `smbus_get_device_index()` in `I2CCommunication.{c,h}`.
   - `MonitorTask` derives its bus index from `args->smbus` and uses `apollo_i2c_ctl_w` /
     `apollo_i2c_ctl_reg_w` / `apollo_i2c_ctl_reg_r`. All `SMBusStatusGet` poll loops removed.
   - Behavior note: the old code distinguished initiation-failure (`continue`) from completion-error
     (`break`) on register reads; these merge into one `break`-on-error path now. Wire transactions
     are unchanged (no PEC was used).

Builds clean for REV1/REV2/REV3, no warnings. Applied on top of soak baseline `06f23fb`.

---

## Second corruption event (soak on `06f23fb`, 2026-05-30 10:55)

```
202605301055 MONI2C DBG MonitorTaskI2C.c:158:FF_F2: dev08<<reg OPT_POWER_CH4 (pg 0 add 0x28), value 0x4
```

`MonitorTaskI2C.c:158` is the success-path `log_debug`. Default `L.level[MONI2C]=LOG_INFO(3)`
(`cm_mcu.c:237`); no facility is at DEBUG, so this is anomalous (unless `loglevel` was used).

**Decode:** tag is `DBG` ⇒ `ev.level==4` (correct, *not* corrupted). For the gate
`level <= L.level[fac]` (`log.c:326`) to pass, the `level` used at the gate was transiently below
threshold while `ev.level` (copied at `log.c:317`, used for the tag) stayed 4. ⇒ **same family as the
FTL event** — a clobber of the log-level value in `log_log`'s frame — caught *after* `ev.level=level`
this time instead of before. Same victim, transient, one-off.

**Key result: the clz guard is exonerated.** `MonitorTaskI2C.c:119–124` skips `devtype_mask==0` and
is in `06f23fb`, yet corruption recurred. ⇒ `__builtin_clz(0)` is not the source. Suspicion shifts to
the still-open items: smbus.c handshake races (#2/#3), the **unprotected `log_log`** (no
`log_set_lock`; shared `b`/`log_buffer`/`L` written with no mutex), and the unguarded store below.

**Audit finding — unbounded `set_*_data`:** every firefly store is
`void set_FF_Fx_..._data(uint16_t data, int which){ ..._data[which]=data; }` with **no bounds check**
(`MonI2C_addresses.c`). Arrays are sized `[NFIREFLIES_Fx]` and `which==device<n_devices`, so in-bounds
today (`dev08` is fine for size 10). But it is a bare `.bss` store that fails *silently* the moment
`which` is ever wrong — and `L` (log levels), `log_buffer`, `b` are file-scope statics adjacent in
`.bss`. Exactly the profile of a data-dependent wild store.

**Resolved (user, 2026-05-30):** ONE line, none in the following ~30 min, `loglevel` NOT changed.
⇒ the threshold global was **not** persistently corrupted; this was a **transient, one-off,
self-healing clobber of the `level` value in `log_log`'s frame**, same victim as the FTL event.
- **SEU considered, ranked down:** HL-LHC firmware, so a one-off self-healing bit flip *looks* like a
  single-event upset — but 2 events in ~16 h is far above the bench cosmic-ray SEU rate for one TM4C
  (~weeks–months/upset). Treat as a firmware defect unless the board is in a beam/source.
- **`set_*_data` downgraded as a source:** arrays, getters, setters all come from one YAML pass, so
  `which == device < n_devices == array length` by construction — no size mismatch. Keep a bounds
  check only as a **tripwire** for a *corrupted* index, not as a presumed fix.

**Decisive next steps (re-ordered for a transient stack/register victim):**
1. `-fstack-protector-strong` on `MonitorTaskI2C.c`, `MonUtils.c`, `log.c` — best catch if the source
   is a localized stack overrun; traps at function return and names the frame.
2. J-Link DWT data watchpoint on `&L.level[LOG_MONI2C]` as a *filter* — catches a stray write to that
   global (even self-healing). If it never fires, the corruption is register/stack-local.
3. `set_*_data`/`get_*_data` bounds-check tripwire — logs `which` + caller when out of range.
4. Install `log_set_lock`; apply smbus.c #2/#3 hardening; re-soak.

---

## Session update (2026-05-30/31): root-cause #1 confirmed-fixed; corruption model + suspect device

### PERIPHERAL_BUSY (root cause #1) — fixed by the bounded idle-wait, confirmed
`fddacb4` added a bounded spin in `i2c_arm_notify_slot` (`while (MAP_I2CMasterBusy() && us<250) DELAY_US(1)`).
This is the **real fix**, not diagnostics: it converts the auto-STOP/FSM race into a bounded wait.
Turning that wait's `log_debug` into `log_warn` showed it firing constantly (`dev N waited M us`), with a
clear per-bus signature: **FF_F2 (dev 3) routinely ~76–84 µs**, CLK (dev 2) ~9 µs, FF_F1 (dev 4) mixed —
i.e. a slow/clock-stretching device on FF_F2.

**One leak-through remained** (1 in ~17.5 h): `dev 3 reg read PERIPHERAL_BUSY MCS=0x41` with **no preceding
`waited` line** (`us==0`). So `BUSY` read clear when the wait sampled it, then `BUSY` was set ~1 µs later at
`SMBusMasterI2CWriteRead`'s own check. The single-sample wait was fooled by a transient `BUSY` low during the
prior transaction's completion. **Fix:** spin on `(MCS & (BUSY|BUSBSY))`, not just `MAP_I2CMasterBusy()` (BUSY).
Note: on TM4C, **`BUSY=1` invalidates the other MCS status bits**, so the `BUSBSY=1` in `MCS=0x41` is
indeterminate — don't over-read it (I did at first).

### MSP overflow — RULED OUT
The boot/idle line `Stack canary now N` is **not** the SSP guard; it's a home-grown **MSP (system-stack)
high-water monitor** (`SystemStackWaterHighWaterMark()` in `cm_mcu.c`, paints 128 words with `0xdeadbeef`).
It sat at ~45/128 words free, stable, over 17.5 h ⇒ the handler/ISR stack is **not** overflowing. Removes the
one stack region `configCHECK_FOR_STACK_OVERFLOW` can't see. Task stacks also healthy/stable (~138 free).

### The corruption: suspect device + mechanism model
**All three anomalies are FF_F2, device index 8** — FTL ("not for **device 8**"), DBG ("**dev08** OPT_POWER_CH4"),
and the PERIPHERAL_BUSY leak (**ps=8**). On REV3, `ff_moni2c_addrs_f2[8]` = **"F2_5 4 XCVR"** (mux `0x70` bit 2,
addr `0x50`), present, serial `B0425040011201`. (REV2 index 8 = "F2_6 4 XCVR".) This 4-channel XCVR is the
clock-stretcher behind the long FF_F2 waits and the prime hardware suspect.

**Model (well-supported, 2 data points):** `log_log`'s `level` parameter (the spilled-`r0` stack slot at a
fixed frame offset) is overwritten **after** the `ev.level = level` copy (`log.c:313`) but **before** the gate
(`log.c:326`). The **DBG event proves the value arrives correct** (`ev.level==4` ⇒ tag "DBG") and is clobbered
*inside* the function. Same 4-byte slot hit **twice** ⇒ not a random spray but a **deterministic OOB store** in
the repeatable `FF_F2 monitor → log_debug → log_log` path (same task, same stack layout). A single OOB store
(`a[bad_i]=v` / wild pointer) does **not** cross the SSP canary (which only catches contiguous overruns at
return) — consistent with the canary never tripping.

### Heisenbug warning
Corruption appeared on the *quieter* builds and **vanished** on the stack-protector + noisy-`log_warn` build
(17.5 h clean). Neither change fixes logic ⇒ likely **timing-masked, not fixed**. To catch it: go *quieter*
(idle-wait back to `log_debug`), keep a *minimal* trap. Don't pile on diagnostics.

### Catch mechanism in place (committed separately)
- `common/log.c`: shadow check — `volatile int level_chk = level;` then `if (level != level_chk) { GET_LR(); bkpt #0; }`
  right before the gate. Catches the **effect** (slot clobbered) and halts in the offending frame.
- `-fstack-protector-strong` on `MonitorTaskI2C.o`/`MonUtils.o`/`log.o` (catches it only if it's a contiguous overrun).
- `cm_mcu.c`: `__stack_chk_fail` prints `LR` (Thumb-bit cleared) for `addr2line`, then `configASSERT` (freeze in DEBUG).
- At a `bkpt` halt, first grab `p &level`, `p/x level` (the overwriting value often fingerprints the source) and `bt`
  (expect the FF_F2 device-8 read/error path).

### Open / next
- The bug is **not yet caught or fixed** — best evidence is "improved/masked." Need a soak at failing-timing
  (quiet) with the shadow trap, ideally catching the `bkpt`.
- Hardening still pending: idle-wait → `(BUSY|BUSBSY)`; the item-2/#3 + smbus.c #2/#3 changes are on a separate
  uncommitted track.

---

## Mechanism refined (2026-05-31): use-after-return on a stack RX buffer (not an OOB store)

Reading `MonitorTaskI2C.c` + `I2CCommunication.c` end-to-end overturned the earlier "deterministic OOB
store in MonitorTaskI2C" guess. There is **no stack-smashing write in MonitorTaskI2C's own code**
(its only writes are `storeData()` → `.bss`, in-bounds). The real defect is in the I2C wrapper layer:

- `apollo_i2c_ctl_reg_r` declares **`uint8_t data[MAX_BYTES]` on its stack** and hands `&data` to the
  TI SMBus driver (`pui8RxBuffer`). The driver's ISR writes received bytes into it.
- It's **spatially safe** — the driver bounds-checks `ui8RxIndex >= ui8RxSize` and `nbytes ≤ MAX_BYTES`.
- It's **temporally unsafe**: if the task is notified/returns *before the transfer truly completes*
  (the early/cross-wake that also yields the PERIPHERAL_BUSY leak), `apollo_i2c_ctl_reg_r` returns, its
  frame is freed, `MonitorTaskI2C` immediately calls `log_debug → log_log` which **reuses that stack
  region**, and the **late completion ISR writes a received byte into the dead `data[]` → onto
  `log_log`'s `level`** (in-bounds index, stale pointer). Classic "return a pointer to a local, write
  through it later."

This explains everything the OOB-store guess strained to: same slot every time (fixed `reg_r → log`
call structure → fixed frame overlap), canary-silent (in-bounds single byte), value = an I2C data byte
(small → visible "printed when suppressed"; large → silent), device-8-triggered (its clock-stretch
creates the overlap), self-healing (next `log_log` re-spills the correct `level`).

**Device 8 (F2_5 XCVR) is the trigger, not the cause.** The cause is the firmware lifetime bug; any
slow/marginal slave would trip it. Masking device 8 would hide the symptom and leave the landmine.

### Fixes
- **Memory-safety net (implementing now):** per-bus **static** RX/TX scratch buffers in
  `I2CCommunication.c`, passed to `SMBusMasterI2C*` instead of stack locals. Stays in our code — `tSMBus`
  is **TI vendor code** (`common/smbus.h`); its `pui8RxBuffer`/`pui8TxBuffer` are already
  *caller-provided pointers*, so we just make ours persistent `.bss`, indexed by bus, safe under the
  per-bus semaphore. A late straggler then lands in `.bss`, not a live frame.
- **Correctness fix (later):** STOP-interrupt completion (`I2C_MASTER_INT_STOP`) and/or a robust
  `BUSY|BUSBSY` idle-wait — removes the early-wake race itself (also fixes stale-data and PERIPHERAL_BUSY).
- **Landed (independent):** `page[devtype]`/`command[devtype]` **OOB read** guard in `MonitorTaskI2C.c`
  (`DEVICE_NONE`=0x80 → devtype 7 vs `[4]` arrays; `if (devtype >= sizeof(args->commands->page)) continue;`).
  A device-8 agitator (garbage register on a misread type), not the smash.

### Caveats
The early/cross-wake step (ISR giving B's notification while A trails) is consistent with the data but
not yet proven against an smbus.c ISR-sequence trace. The static buffer is a safety net regardless; the
shadow-`bkpt` in `log.c` should still be used to catch a live event and confirm the value/offset.

---

## Trigger generalizes off device 8 → the 4-ch XCVR Firefly *model* (2026-05-31)

A PERIPHERAL_BUSY leak appeared on a **different bus and device**: `2026-05-31 17:07 dev 4 (FF_F1)
reg read PERIPHERAL_BUSY MCS=0x41`, `FF_F1: RX_POWER_ALARM ... (ps=9)` — same `MCS=0x41`, no preceding
`waited` line (`us==0`), i.e. the same single-sample idle-wait fooled. Benign (no corruption), stacks/
heap fine, uptime 6.5 h. This was on the **old code** (no buffer/wait fix).

FF_F1 idx 9 (REV3) = **F1_6 "4 XCVR"** (mux 0x71 bit 2, addr 0x50). Its ID register reads
`B0425040011201` — the same value as FF_F2 idx 8 (**F2_5**). **`B0425040011201` is the part MODEL
number** (read from the ID reg), *not* a serial — so two units reading it identically is expected.

**Conclusion:** the trigger is the **4-channel XCVR Firefly model `B0425040011201`**, whose ~80 µs
clock-stretching trips the early-wake race wherever a unit of that model sits (F2_5 on bus 3, F1_6 on
bus 4, …). It's a device-**model** behavior **across buses**, not "device 8." Implications:
- "Swap the bad device" is off the table; the fix must be firmware-side and bus-agnostic — which is what
  the per-bus static buffers (+ robust `BUSY|BUSBSY` wait / STOP interrupt) are.
- The visible *corruption* has only been seen on F2_5 so far, but F1_6 has the identical race, so it can
  smash on FF_F1 too (just with an invisible byte value, or not-yet-observed). The buffer fix covers all
  six buses uniformly.
- Still unverified (would need the Samtec datasheet for P/N `B0425040011201`): *why* this model
  clock-stretches and whether it's spec'd or marginal. Does not gate the firmware fix.
