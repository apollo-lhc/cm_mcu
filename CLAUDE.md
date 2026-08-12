# CLAUDE.md — Apollo Command Module MCU Firmware

Bare-metal FreeRTOS firmware for the **Apollo command module**, the ATCA blade destined for the
HL-LHC. It handles power sequencing, temperature/voltage monitoring, I2C/SMBus communication, and
FPGA/Firefly control for high-energy-physics detector readout.

- **MCU**: TI Tiva TM4C1290NCPDT, ARM Cortex-M4F with FPU, 40 MHz
- **Revisions**: REV1, REV2, REV3 (default REV3) — exactly one may be defined at build time
- **Compiler**: `arm-none-eabi-gcc` 13.2.Rel1 from ARM — **not** the distribution package, which is
  usually too old. `clang` (LLVM Embedded Toolchain for Arm) is also supported and built in CI.
- **Debugger**: Segger J-LINK

The revision drives the pin maps (`common/pinout_rev*.c`) and a good deal of conditional code.
REV1 differs from REV2/REV3 in: the FPGA I2C bus (I2C6 vs I2C5), UART assignments, ADC channel-to-
signal mapping, and Firefly/clock/DCDC device counts. REV2 and REV3 differ mainly in Firefly part
populations and clock-chip identities. Assume nothing is shared until you have checked the `#if`s.

`ECN001` is a hardware change note affecting REV1 power sequencing (`PowerSupplyTask.c`). It is
defined by default; `make NO_ECN001=1` builds the pre-ECN variant, and releases ship both.

## Detailed documentation

This file is deliberately short — read the linked file before working in that area.

| Topic | File |
| ----- | ---- |
| Full source tree, file by file | [`repo_layout.md`](repo_layout.md) |
| cm_mcu internals: boot sequence, task table, LED states, I2C transaction/semaphore contract, EEPROM layout, logging, temperature alarms, ProgCom UART protocol | [`projects/cm_mcu/README.md`](projects/cm_mcu/README.md) |
| prod_test: architecture, I2C bus map, CLI table, `prodtest1` sequence | [`projects/prod_test/README.md`](projects/prod_test/README.md) |
| MCU peripheral assignments (UARTs, timers, pins) | [`mcu_peripherals.md`](mcu_peripherals.md) |
| I2C lockup investigation and residual races | [`i2c_lockup_notes.md`](i2c_lockup_notes.md) |

## Build

On macOS use `sysctl -n hw.ncpu` in place of `nproc`.

```bash
git submodule update --init --recursive   # FreeRTOS, first time only
make -j $(nproc)              # standard build -> projects/cm_mcu/cm_mcu.axf
make -j $(nproc) DEBUG=1      # debug build (symbols; asserts spin instead of resetting)
make -j $(nproc) VERBOSE=1    # show full command lines
make REV1=1                   # or REV2=1, REV3=1
make COMPILER=clang REV3=1    # clang build, as run in CI
make check-for-pr             # format check + full build matrix (runs build_all.sh)
make release                  # release binaries + tarball
# also: all (default), clean, format, format-apply
```

**Always `make clean` when switching revision, compiler, or between DEBUG and non-DEBUG.** The
Makefile does not track flag changes, so a stale object tree silently links a binary built for the
previous configuration — with no warning. The top-level Makefile defaults to `REV3=1`, but the
per-project ones do not, so `make clean` run from inside `projects/cm_mcu` fails with
`"No Revision defined"` unless you pass `REVn=1`.

## Repository layout

`projects/cm_mcu/` is the primary runtime firmware (`cm_mcu.c` holds `main`; `commands/` holds the
CLI handlers). `projects/prod_test/` is the production test firmware (CLI + ADC + I2C slave only);
`projects/` also has `boot_loader`, `blinky`, `i2c-sensors`, `uart_echo`. `common/` is shared code
(utils, log, printf, UART, SMBus, per-revision pinout), `inc/` + `driverlib/` are TI TivaWare,
`FreeRTOS-Kernel/` is a submodule, `sm_cm_config/` holds board config and the codegen scripts.

`projects/cm_mcu/Tasks.h` is the central header: task prototypes, queue/semaphore externs, and most
inter-task definitions live there. Full tree: [`repo_layout.md`](repo_layout.md).

## Architecture essentials

**Tasks.** All significant work happens in dedicated FreeRTOS tasks that communicate via queues:
`EEPROMTask` (EEPROM gatekeeper), `InitTask` (one-shot startup), `GenericAlarmTask` (temperature and
voltage alarm state machines, 50 ms), `CommandLineTask` (UART CLI), `ProgComTask` (programmatic UART7
interface, REV2/3), `MonitorTask`/`MonitorTaskI2C` (SMBus/I2C polling), `PowerSupplyTask`,
`ADCMonitorTask`, `ZynqMonTask`, `WatchdogTask`, `LedTask`, `I2CSlaveTask`. Full table with source
files in the cm_mcu README.

Higher priority number wins. The safety-critical tasks (`PowerSupplyTask`, `I2CSlaveTask`,
`InitTask`, `ZynqMonTask`, `GenericAlarmTask`) run at `tskIDLE_PRIORITY + 4`; monitoring, CLI and
ProgCom run at `+3`; `LedTask` at `+1`. Most tasks are a short init block followed by a
`vTaskDelayUntil` loop.

**Power state machine.** `PowerSupplyTask` walks
`POWER_FAILURE → POWER_INIT → POWER_DOWN → POWER_OFF → POWER_L1ON … POWER_L6ON → POWER_ON`
(the `X_MACRO_PS_SYSTEM_STATES` list in `Tasks.h`). A `TEMP_ALARM` forces a transition to
`POWER_OFF`.

**I2C buses.** Fixed assignment, `cm_mcu` and `prod_test` alike:

| Bus | Role |
| --- | ---- |
| I2C0 | I2C slave, address `0x40` (external master / IPMC) |
| I2C1 | DCDC power supplies (PMBus) |
| I2C2 | Clock synthesizers (Si5395) |
| I2C3 | F2 Firefly optics (`I2C_DEVICE_F2`) |
| I2C4 | F1 Firefly optics (`I2C_DEVICE_F1`) |
| I2C5 | FPGA monitoring — **REV2/REV3** |
| I2C6 | FPGA monitoring — **REV1** |

**I2C/SMBus.** Interrupt/notification-based, not polling. A caller arms a per-bus completion slot,
starts the transfer, then blocks on `ulTaskNotifyTake` with a 250 ms timeout. Each bus (1–6) has its
own FreeRTOS mutex (`i2c1_sem` … `i2c6_sem`) which **the caller must hold for the whole transaction
sequence** — mux select, page select and data are separate transactions sharing per-bus `.bss` scratch
buffers. Release with the guarded give used everywhere in the tree:

```c
if (xSemaphoreGetMutexHolder(sem) == xTaskGetCurrentTaskHandle())
  xSemaphoreGive(sem);
```

The raw `i2c*` CLI commands deliberately skip the mutex so a user can compose a sequence under
`sem_ctl <bus> take`/`release`. Details, rationale and the ownership contract: cm_mcu README.

**EEPROM.** Never touch the internal EEPROM directly from a task — go through the `EEPROMTask`
gatekeeper (`write_eeprom()`, `read_eeprom_single()`, `read_eeprom_multi()`). Block 1 is password
protected; block 6 (temperature alarms) is not. Layout and the raw/ISR-only variants: cm_mcu README.
`prod_test` has no gatekeeper and does not use this pattern.

**CLI handlers.** Both projects share the signature `BaseType_t my_command(int argc, char **argv,
char *m)` — `argv[0]` is the command name, output is written to `m` with `snprintf`, and the command
is registered in the static `commands[]` array in `CommandLineTask.c`. The return conventions differ:

| Project | Return type | Values | `SCRATCH_SIZE` |
| ------- | ----------- | ------ | -------------- |
| `cm_mcu` | `BaseType_t` | `pdFALSE` done, `pdTRUE` more output pending | 1024 |
| `prod_test` | `cli_status_t` | `CLI_OK`, `CLI_MORE`, `CLI_ERROR` | 512 |

`prod_test` registrations also declare `num_args` (exact count required, or `-1` for any); the
dispatcher enforces it before calling the handler.

## Generated sources — do not hand-edit

Two address tables are generated from the YAML in `sm_cm_config/data` by scripts in
`sm_cm_config/src`, with rules in `projects/cm_mcu/Makefile`:

| Generated | From | Script |
| --------- | ---- | ------ |
| `ZynqMon_addresses.c` / `.h` | `PL_MEM*.yml` | `mcu_generate.py` |
| `MonI2C_addresses.c` | `MON_I2C*.yml` | `mon_generate.py` |

`make` regenerates them whenever the YAML is newer, so edits to the `.c`/`.h` are silently lost.
Change the YAML instead, and commit the regenerated output alongside it. The `release` target
additionally runs `xml_generate.py` to produce the Zynq-side XML.

## Debugging

- GDB: use the provided `cm_mcu.gdbinit`, connecting via Segger J-LINK EDU.
- Serial terminal: send `\n` on Enter; firmware prints `\r\n` for broad compatibility.
- Runtime introspection from the CLI: `mem` (heap free / min-ever-free), `taskstats`, `taskinfo`,
  `stack_usage`, `log`, `errorlog`, `semaphore`.

Three runtime safety nets exist; read their reports rather than working around them.
`configCHECK_FOR_STACK_OVERFLOW` is `2`, so an overflow hits `vApplicationStackOverflowHook()`
(halts under `DEBUG`, resets otherwise). `-fstack-protector-strong` is on for `MonitorTaskI2C.c`,
`MonUtils.c` and `log.c` only (see `projects/cm_mcu/Makefile`); `__stack_chk_fail()` records the
caller's LR, decodable with `arm-none-eabi-addr2line`. Heap exhaustion calls
`vApplicationMallocFailedHook()`, so a task that fails to start is loud rather than silent.

## CI and code quality

- GitHub Actions on pull requests (`.github/workflows/c-cpp.yml`) builds nine configurations:
  gcc × {REV1, REV2, REV3} × {DEBUG, release}, plus clang × three revisions (release only). Non-debug
  builds carry `-Werror`.
- Formatting is enforced separately by `clang-format` **version 17** (`DoozyX/clang-format-lint-action`).
  Matching that exact version locally matters; see the rules below.
- `release.yml` builds gcc-only release artifacts per revision, with and without `NO_ECN001=1`.
- `build_all.sh` is the local equivalent of the build matrix (six configs: three revisions × two
  compilers) and is what `make check-for-pr` runs.
- **There is no unit test suite.** Verification is builds plus hardware integration testing — don't
  go looking for tests, and don't invent a test harness unless asked.

## Common issues

| Symptom | Cause | Fix |
| ------- | ----- | --- |
| FreeRTOS build errors | Submodule not initialized | `git submodule update --init --recursive --remote` |
| Linker errors, wrong-revision behavior | Stale objects after a revision/compiler/DEBUG switch | `make clean` (or `rm -rf projects/*/gcc projects/*/clang`), rebuild |
| Unsupported compiler flags | Wrong compiler version | Install `arm-none-eabi-gcc` 13.2.Rel1 from the ARM website |
| Task silently never starts | FreeRTOS heap exhausted (`configTOTAL_HEAP_SIZE`, 25 KB) | Check `mem` on the CLI; shrink a task stack or raise the heap |
| `SMBUS_PERIPHERAL_BUSY` returns | I2C master FSM race | Already mitigated by the bounded idle-wait; background in [`i2c_lockup_notes.md`](i2c_lockup_notes.md) |
| Temperature alarm re-fires after a power cycle | Stale `pm_values` | Already fixed via the `-999.f` sentinel + backdated `updateTick`; don't undo either half |
| Edits to `ZynqMon_addresses.*` / `MonI2C_addresses.c` disappear | They are generated from YAML | Edit `sm_cm_config/data/*.yml` instead |

## Rules when working in this repo

- **Do not offer to commit.** The maintainer commits their own code.
- **Do not fix or fret about formatting.** `clang-format` is version-sensitive and the maintainer
  handles it separately. Do not run `format-apply`, and do not block on format errors.
- **Stay narrowly on the task at hand.** Do not expand scope beyond what was asked.
- Check which hardware revision you are targeting before changing anything, and build `DEBUG=1`
  during development.
- This is bare-metal firmware: memory is constrained, heap allocation after startup is to be avoided,
  and task stack sizes are tight. Prefer the ROM-based TivaWare (`ROM_`/`MAP_`) drivers.
- Thread safety means FreeRTOS primitives — mutexes, queues, task notifications. Changes may land in
  time-critical interrupt handlers, which must use the `…FromISR` API variants and must not log:
  `log_*` is task-context only, and its lock is a non-blocking 0-tick acquire so logging never stalls
  a caller.
- Use sized integer types (`uint32_t`, `int32_t`, …) rather than bare `int` for register-width values.
- Float↔`uint32_t` conversion for EEPROM: `memcpy(&u32, &f, sizeof(float))`, never union type-punning.
- When adding a CLI subcommand, update the help string in `CommandLineTask.c` too.

## Related resources

[TM4C1290NCPDT datasheet](https://www.ti.com/product/TM4C1290NCPDT) ·
[FreeRTOS docs](https://freertos.org) ·
[TivaWare driver library](https://www.ti.com/tool/SW-TM4C) ·
[Segger J-LINK](https://www.segger.com)
