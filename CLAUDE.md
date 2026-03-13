# CLAUDE.md — Apollo Command Module MCU Firmware

## Project Overview

This repository contains bare-metal firmware for the **Apollo command module**, targeting the **TI Tiva TM4C1290NCPDT** (Cortex-M4F, 80 MHz). The firmware manages power sequencing, temperature/voltage monitoring, I2C communication, and FPGA/Firefly control for high-energy physics detector readout systems.

## Build

```bash
# Standard build (produces projects/cm_mcu/cm_mcu.axf)
make -j $(nproc)

# Debug build (with symbols)
make -j $(nproc) DEBUG=1

# Verbose output
make -j $(nproc) VERBOSE=1
```

**Do not mix debug and non-debug builds.** Run `make clean` if switching.

**Compiler required:** `arm-none-eabi-gcc` 13.2.Rel1 (not the distribution-provided version — download from ARM).

**Submodule init (if build fails):**
```bash
git submodule update --remote --recursive
```

## Project Structure

```
projects/cm_mcu/          # Main firmware (primary target)
  AlarmUtilities.c/h      # Temperature and voltage alarm logic
  InitTask.c              # One-shot startup initialization task
  EEPROMTask.c            # EEPROM gatekeeper task
  commands/               # CLI command handler implementations
    SensorControl.c       # talarm, sensor CLI commands
    BoardCommands.c       # Board ID, EEPROM, GPIO CLI commands
    EEPROMCommands.c      # Low-level EEPROM read/write commands
common/                   # Shared utilities across projects
  utils.c/h               # EEPROM helpers, error buffer, float utilities
  log.c/h                 # Logging macros
driverlib/                # TI Tivaware driver library source
inc/                      # TI Tivaware driver library headers
FreeRTOS-Kernel/          # FreeRTOS (git submodule)
```

## Architecture

### FreeRTOS Tasks

The firmware uses a FreeRTOS multi-task architecture. All significant work happens in dedicated tasks communicating via queues.

**Key tasks:**
- `EEPROMTask` — gatekeeper for all EEPROM access (queue-based, thread-safe)
- `InitTask` — runs once at startup to load board config and initialize peripherals
- `GenericAlarmTask` — state machine for temperature/voltage alarms (runs every 50 ms)
- `CommandLineTask` — UART CLI interface (two instances)
- `MonitorTask` / `MonitorI2CTask` — periodic I2C polling for power supplies and FPGAs
- `FireflyTask` — SamTec Firefly transceiver monitoring
- `PowerSupplyTask` — power supply sequencing and TEMP_ALARM response

### EEPROM Access Pattern

**Always use the gatekeeper API** — never write to EEPROM directly from tasks:

```c
// Queue-based (safe from any task):
write_eeprom(uint32_t data, uint32_t addr);
uint32_t read_eeprom_single(uint32_t addr);
uint64_t read_eeprom_multi(uint32_t addr);

// Raw/direct (ISR-safe only, bypass queue):
write_eeprom_raw(uint32_t data, uint32_t addr);
uint32_t read_eeprom_raw(uint32_t addr);
```

For block password operations, use `EPRMMessage()` + `xQueueSendToBack(xEPRMQueue_in, ...)` directly (see `BoardCommands.c`).

### EEPROM Layout (Internal EEPROM, 6 KB total)

```
Block 0:  0x000–0x03F   (free)
Block 1:  0x040–0x07F   Apollo board identity (password 0x12345678)
  0x040 ADDR_ID          board_id (upper 16b) + revision (lower 16b)
  0x044 ADDR_FF          Firefly USER config mask
  0x048 ADDR_PS          Power supply ignore mask
Blocks 2–5: 0x080–0x17F  Error log ring buffer (EBUF_MINBLK=2, EBUF_MAXBLK=5)
Block 6:  0x180–0x1BF   Temperature alarm thresholds (runtime-configurable)
  0x180 ADDR_TEMP_FF     Firefly alarm temp (float, 4 bytes)
  0x184 ADDR_TEMP_DCDC   DCDC alarm temp (float, 4 bytes)
  0x188 ADDR_TEMP_TM4C   TM4C alarm temp (float, 4 bytes)
  0x18C ADDR_TEMP_FPGA   FPGA alarm temp (float, 4 bytes)
Blocks 7+: 0x1C0+        Available
```

Uninitialized EEPROM words read as `0xFFFFFFFF`. Code that loads EEPROM config must check for this sentinel and fall back to compile-time defaults.

### CLI Command Pattern

Command handlers follow this signature:

```c
BaseType_t my_command(int argc, char **argv, char *m);
```

- `argv[0]` is the command name, `argv[1..argc-1]` are arguments
- Write output to `m` (buffer size `SCRATCH_SIZE` = 1024 bytes) using `snprintf`
- Return `pdFALSE` on completion (commands are non-blocking)
- Registered in `CommandLineTask.c` in the static `commands[]` array

## Temperature Alarm Thresholds

Defaults (used if EEPROM is uninitialized):

| Device | Default | EEPROM Address |
|--------|---------|---------------|
| Firefly | 55°C | 0x180 |
| DCDC | 70°C | 0x184 |
| TM4C MCU | 70°C | 0x188 |
| FPGA | 81°C | 0x18C |

Alarm tolerance: +5°C above threshold triggers power-off (shutdown).

**CLI:**
```
talarm status                    # show current thresholds and alarm status
talarm settemp [ff|fpga|dcdc|tm4c] <temp>   # set threshold (persists to EEPROM)
talarm resettemp [ff|fpga|dcdc|tm4c|all]    # reset to compile-time defaults
```

## CI / Code Quality

- GitHub Actions builds with `-Werror` (warnings are errors)
- Code formatting: `clang-format` (see `.clang-format`)
- Static analysis: `clang-tidy` (see `.clang-tidy`)
- Target MCU: `REV2` or `REV3` (define at build time via Makefile)

## Development Notes for AI Assistants

- Integers: use `uint32_t`, `int32_t`, etc. — not bare `int` for register-width values
- Float-to-uint32 conversion for EEPROM: use `memcpy(&u32, &f, sizeof(float))` — not union type-punning
- All EEPROM writes from tasks must use `write_eeprom()` (queue-based), never direct driver calls
- Block 1 EEPROM is password-protected — requires `EPRM_UNLOCK_BLOCK` / `EPRM_LOCK_BLOCK` messages around writes
- Block 6 (temperature alarms) is unprotected — direct `write_eeprom()` calls are sufficient
- `Tasks.h` is the central header; most inter-task definitions live there
- When adding a new CLI subcommand, update the help string in `CommandLineTask.c`
