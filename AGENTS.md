# AGENTS.md — Apollo Command Module MCU Firmware

Bare-metal firmware for **TI Tiva TM4C1290NCPDT** (ARM Cortex-M4F, 40 MHz). Manages power sequencing, temperature/voltage monitoring, I2C, FPGA/Firefly control for HL-LHC detector readout.

## Critical Constraints

**Don'ts:** Never commit or format code — user handles both.

**Hardware Revisions:** Exactly one of `REV1`, `REV2`, `REV3` must be defined (default `REV3=1`). Drives pin maps, I2C bus assignments (I2C6 for FPGA on REV1, I2C5 on REV2/3), UART, ADC mappings, Firefly/clock counts.

**Build:**
```bash
make -j $(nproc) DEBUG=1 REV3=1   # Debug (symbols, no -Werror)
make -j $(nproc) REV3=1           # Release (-Werror)
make clean                         # Required when switching
```
MacOS: use `sysctl -n hw.ncpu` instead of `nproc`. GCC arm-none-eabi **13.2.Rel1** required. Clang for static analysis only.

## I2C/SMBus Architecture

**Notification-based (interrupt-driven), not polling:**
```c
i2c_arm_notify_slot(device); r = SMBusMasterXxx(...);
if (r == SMBUS_OK) { i2c_wait_for_transfer(device); r = *eStatus[device]; }
else { TaskNotifySMBus[device] = NULL; }
```
Static scratch buffers (`i2c_txbuf`/`i2c_rxbuf` in `I2CCommunication.c`) prevent use-after-return. Per-bus mutexes (`i2c1_sem`...`i2c6_sem`) required for transactions. CLI commands (`i2cr`, `i2cw`, `i2c_scan`) skip semaphores; use `sem_ctl <bus> take/release` for manual locking.

## EEPROM

**Use gatekeeper API only:**
```c
write_eeprom(data, addr); read_eeprom_single(addr); read_eeprom_multi(addr);
write_eeprom_raw(data, addr); read_eeprom_raw(addr);  // ISR-safe only
```
**Layout (6 KB):** Block 0 free; Block 1 (0x040-0x07F) board identity (password 0x12345678); Blocks 2-5 error log; Block 6 temp thresholds; Blocks 7+ available. Block 1 requires `EPRM_UNLOCK_BLOCK`/`EPRM_LOCK_BLOCK`.

**Float conversion:** Use `memcpy(&u32, &f, sizeof(float))` — no union type-punning.

## Logging

```c
log_set_lock(vGiveOrTakeSemaphore, log_sem);  // Call in main() first
```
Acquire is non-blocking. Task-context only (no ISR logging).

## Temperature Alarms

| Device | Default | EEPROM |
|--------|---------|--------|
| Firefly | 55°C | 0x180 |
| DCDC | 70°C | 0x184 |
| TM4C | 70°C | 0x188 |
| FPGA | 81°C | 0x18C |

Defaults used if EEPROM reads 0xFFFFFFFF. On power leaving `POWER_ON`, `MonitorTask` clears `pm_values[]` to `-999.f` and backdates `updateTick` 60s.

## CLI Commands

```c
BaseType_t my_command(int argc, char **argv, char *m);  // argv[0] = name
```
SCRATCH_SIZE: 1024 (`cm_mcu`), 512 (`prod_test`). Returns: `pdFALSE`/`pdTRUE` (cm_mcu), `CLI_OK`/`CLI_MORE`/`CLI_ERROR` (prod_test).

## FreeRTOS Tasks

```c
void Task(void *params) {
  TickType_t xLastWakeTime = xTaskGetTickCount();
  for (;;) { /* work */ vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(period_ms)); }
}
```
Priorities: 4 = critical (PowerSupply, I2CSlave, Init, ZynqMon, Alarm), 3 = monitoring.

**Main Tasks:** InitTask (one-shot), EEPROMTask (queue), GenericAlarmTask (50ms), CommandLineTask (event), PowerSupplyTask (event), MonitorTask (10ms), MonitorTaskI2C (10ms), ADCMonitorTask (25ms), ZynqMonTask (25ms), I2CSlaveTask (event), LedTask (250ms), WatchdogTask (periodic).

**I2C Buses:** I2C0=Slave(0x40), I2C1=DCDC, I2C2=Clocks, I2C3=F2 Firefly, I2C4=F1 Firefly, I2C5=FPGA(REV2/3), I2C6=FPGA(REV1).

**Power FSM:** `POWER_FAILURE → POWER_INIT → POWER_DOWN → POWER_OFF → POWER_L1ON → ... → POWER_L6ON → POWER_ON`. TEMP_ALARM → POWER_OFF.

## Generated Files

From YAML in `sm_cm_config/data`: `ZynqMon_addresses.c/.h` (PL_MEM_CM_rev*.yml), `MonI2C_addresses.c` (MON_I2C*.yml). Make regenerates; commit outputs on YAML changes.

## Testing

**Pre-PR:**
```bash
git submodule update --init --recursive && make clean
make -j DEBUG=1 REV1=1 && make clean && make -j DEBUG=1 REV2=1 && make clean
make -j DEBUG=1 REV3=1 && make format
```
**Full matrix:** `./build_all.sh` (all revisions, GCC+Clang, format). CI: GitHub Actions builds all revisions (GCC debug/release, Clang), clang-format lint, release binaries on tags.

## Common Issues

| Symptom | Solution |
|---------|----------|
| FreeRTOS build errors | `git submodule update --init --recursive --remote` |
| Linker errors / runtime issues | `make clean`, rebuild consistently |
| Compilation errors | Install GCC arm-none-eabi 13.2.Rel1 |
| Windows incremental build fails | Use WSL or `make clean` between builds |
| `SMBUS_PERIPHERAL_BUSY` | See `i2c_lockup_notes.md` |
| Temp alarm re-triggers after power cycle | Stale `pm_values` invalidated via sentinel + backdated tick |

## Code Style

`uint32_t`/`int32_t` for register-width values. clang-format v17, clang-tidy. No heap — static buffers only. FreeRTOS primitives for thread safety. ISRs use `FromISR` variants.

## Key Files

```
projects/cm_mcu/: cm_mcu.c, Tasks.h, FreeRTOSConfig.h, InterruptHandlers.c,
  startup_gcc.c, CommandLineTask.c, commands/*, MonitorTask*.c,
  I2CCommunication.c, EEPROMTask.c, PowerSupplyTask.c, AlarmUtilities.c,
  FireflyUtils.c, ZynqMonTask.c, LedTask.c
common/: LocalUart.c/h, smbus.c/h, smbus_helper.h, utils.c/h, log.c/h,
  pinout_rev*.c
```

## Debugging

**GDB:** Segger J-LINK EDU, use `cm_mcu.gdbinit`. **Serial:** Terminal sends `\n`, code prints `\r\n`. **Stack overflow:** `configCHECK_FOR_STACK_OVERFLOW=2`, triggers hook (halt DEBUG, reset release). **Stack smashing:** `-fstack-protector-strong` on `MonitorTaskI2C.c`, `MonUtils.c`, `log.c`; `__stack_chk_fail()` decodes LR via `arm-none-eabi-addr2line`.

## Commands

`make check-and-reinit-submodules` (auto on `make all`), `make -j $(nproc) DEBUG=1` (default REV3), `make -j $(nproc) REV3=1` (release), `make clean`, `make format`, `make format-apply`, `make check-for-pr`, `./build_all.sh`, `make release`.
