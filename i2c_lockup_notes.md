# I2C Rare Lockup / PERIPHERAL_BUSY Analysis

**Branch:** `hotfix/stale_temp_alarm`  
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
