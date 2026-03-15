# CLAUDE.md — Apollo Command Module MCU Firmware

## Project Overview

This repository contains bare-metal firmware for the **Apollo command module**, targeting the **TI Tiva TM4C1290NCPDT** (ARM Cortex-M4F with FPU, 80 MHz). The firmware manages power sequencing, temperature/voltage monitoring, I2C communication, and FPGA/Firefly control for high-energy physics detector readout systems.

## Hardware

- **MCU**: TM4C1290NCPDT (Texas Instruments Tiva C Series)
- **Architecture**: ARM Cortex-M4F with FPU
- **Hardware Revisions**: REV1, REV2, REV3 (default: REV3)
- **Programmer/Debugger**: Segger J-LINK EDU (with JTAG adapter for Xilinx headers)

## Development Environment

### Required Tools

- **Compiler**: `arm-none-eabi-gcc` 13.2.Rel1 — download from ARM, do **not** use the distribution-provided version (usually too old)
- **Build System**: Make
- **Version Control**: Git (with submodule support)
- **Optional IDE**: Eclipse with GNU MCU Eclipse plugin

### Platform Support

- **Fully Supported**: Linux (RHEL7/SC7), macOS
- **Experimental**: Windows via WSL (recommended) or Cygwin (requires `make clean` between builds)

## Build

### Initial Setup

```bash
# Initialize and update submodules (FreeRTOS)
git submodule update --init --recursive
```

### Build Commands

```bash
# Standard build (produces projects/cm_mcu/cm_mcu.axf)
make -j $(nproc)

# Debug build (with symbols)
make -j $(nproc) DEBUG=1

# Verbose output
make -j $(nproc) VERBOSE=1

# Build specific hardware revision
make REV1=1   # or REV2=1, REV3=1

# Check formatting + build all configurations (run before PR)
make check-for-pr

# Create release package
make release
```

**Do not mix debug and non-debug builds.** Run `make clean` if switching.

### Build Targets

| Target | Description |
| ------ | ----------- |
| `all` (default) | Build all projects |
| `clean` | Remove all build artifacts |
| `format` | Check formatting of modified files |
| `format-apply` | Auto-format modified files |
| `check-for-pr` | Format check + build all configurations |
| `release` | Build release binaries and create tarball |

### Hardware Revisions

Only **one** revision can be defined at build time — the Makefile enforces this:

- `REV1=1` — Revision 1
- `REV2=1` — Revision 2
- `REV3=1` — Revision 3 (default if none specified)

## Project Structure

```
apollo_cm_mcu/
├── projects/
│   ├── cm_mcu/                  # PRIMARY: command module runtime firmware
│   │   ├── cm_mcu.c             # Main entry point, task creation
│   │   ├── cm_mcu.ld            # Linker script
│   │   ├── Tasks.h              # Central header: all task/queue/semaphore externs
│   │   ├── FreeRTOSConfig.h     # FreeRTOS tuning (tick rate, stack sizes, etc.)
│   │   │
│   │   ├── InitTask.c           # One-shot startup: loads EEPROM config, init peripherals
│   │   ├── EEPROMTask.c         # EEPROM gatekeeper task (queue-based, thread-safe)
│   │   ├── CommandLineTask.c/h  # UART CLI interface (two instances); registers all commands
│   │   ├── GenericAlarmTask.c   # Alarm state machine (runs every 50 ms)
│   │   ├── AlarmUtilities.c/h   # Temperature and voltage alarm logic
│   │   ├── PowerSupplyTask.c    # Power sequencing and TEMP_ALARM response
│   │   ├── MonitorTask.c/h      # SMBus/PMBus polling of power supplies
│   │   ├── MonitorTaskI2C.c/h   # I2C polling of FPGAs
│   │   ├── MonI2C_addresses.c/h # I2C address tables for MonitorTaskI2C
│   │   ├── MonUtils.c/h         # Shared monitor utilities (value decoding, etc.)
│   │   ├── FireflyUtils.c/h     # SamTec Firefly transceiver register access
│   │   ├── ZynqMonTask.c        # Zynq FPGA monitoring task
│   │   ├── ZynqMon_addresses.c/h# Zynq I2C register address tables
│   │   ├── ADCMonitorTask.c     # On-chip ADC monitoring (TM4C internal sensors)
│   │   ├── I2CCommunication.c/h # Low-level I2C transaction abstraction
│   │   ├── I2CSlaveTask.c/h     # I2C slave interface (responds to external master)
│   │   ├── clocksynth.c/h       # Clock synthesizer (Si5395) control
│   │   ├── LedTask.c            # Status LED blink patterns
│   │   ├── WatchdogTask.c       # Hardware watchdog refresh task
│   │   ├── InterruptHandlers.c/h# ISR implementations
│   │   ├── Semaphore.c/h        # I2C bus semaphore helpers
│   │   ├── LocalTasks.c         # Miscellaneous local task utilities
│   │   ├── startup_gcc.c        # GCC startup / vector table
│   │   ├── startup_clang.c      # Clang startup / vector table
│   │   └── commands/            # CLI command handler implementations
│   │       ├── BoardCommands.c/h    # Board ID, EEPROM identity, GPIO commands
│   │       ├── BufferCommands.c/h   # Error log ring buffer commands
│   │       ├── EEPROMCommands.c/h   # Low-level EEPROM read/write commands
│   │       ├── I2CCommands.c/h      # I2C bus scan and raw access commands
│   │       ├── SensorControl.c/h    # talarm, sensor monitoring commands
│   │       ├── SoftwareCommands.c/h # Software version, reset, watchdog commands
│   │       └── parameters.h         # Shared CLI parameter definitions
│   │
│   ├── prod_test/               # SECONDARY: production test firmware
│   │   ├── prod_test.c/h        # Main entry point and test framework
│   │   ├── prod_test.ld         # Linker script
│   │   ├── CommandLineTask.c    # CLI task (prod_test-specific command set)
│   │   ├── commands.c/h         # Top-level command dispatch
│   │   ├── ADCMonitorTask.c/h   # ADC verification tests
│   │   ├── I2CCommunication.c/h # I2C abstraction (shared with cm_mcu)
│   │   ├── I2CSlaveTask.c/h     # I2C slave interface
│   │   ├── InterruptHandlers.c/h# ISRs
│   │   ├── ClockI2CCommands.c/h # Clock synthesizer I2C test commands
│   │   ├── EEPROMI2CCommands.c/h# EEPROM I2C test commands
│   │   ├── FPGAI2CCommands.c/h  # FPGA I2C test commands
│   │   ├── FireflyI2CCommands.c/h# Firefly I2C test commands
│   │   └── PowerI2CCommands.c/h # Power supply I2C test commands
│   │
│   ├── boot_loader/             # Bootloader
│   ├── blinky/                  # LED blink sanity test
│   ├── i2c-sensors/             # I2C sensor demos
│   └── uart_echo/               # UART loopback test
│
├── common/                      # Shared utilities across all projects
│   ├── utils.c/h                # EEPROM helpers, error buffer, float utilities
│   └── log.c/h                  # Logging macros
├── inc/                         # TI Tivaware driver library headers
├── driverlib/                   # TI Tivaware driver library source
├── FreeRTOS-Kernel/             # FreeRTOS (git submodule)
├── sm_cm_config/                # Board configuration files
└── makedefs                     # Common Makefile variable definitions
```

## Architecture

### cm_mcu: FreeRTOS Tasks

The firmware uses a FreeRTOS multi-task architecture. All significant work happens in dedicated tasks communicating via queues.

**Key tasks:**
- `EEPROMTask` — gatekeeper for all EEPROM access (queue-based, thread-safe)
- `InitTask` — runs once at startup to load board config and initialize peripherals
- `GenericAlarmTask` — state machine for temperature/voltage alarms (runs every 50 ms)
- `CommandLineTask` — UART CLI interface (two instances: front panel + Zynq UART)
- `MonitorTask` / `MonitorI2CTask` — periodic SMBus/I2C polling for power supplies and FPGAs
- `FireflyTask` — SamTec Firefly transceiver monitoring
- `PowerSupplyTask` — power supply sequencing and TEMP_ALARM response
- `ADCMonitorTask` — on-chip ADC monitoring (internal TM4C temperature, voltages)
- `ZynqMonTask` — Zynq FPGA monitoring via I2C
- `WatchdogTask` — hardware watchdog refresh
- `LedTask` — status LED blink patterns
- `I2CSlaveTask` — responds to commands from an external I2C master

### cm_mcu: EEPROM Access Pattern

> **This pattern applies only to cm_mcu**, which has a dedicated `EEPROMTask` gatekeeper. The `prod_test` project does not use this pattern.

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

### cm_mcu: EEPROM Layout (Internal EEPROM, 6 KB total)

```
Block 0:  0x000–0x03F   (free)
Block 1:  0x040–0x07F   Apollo board identity (password 0x12345678)
  0x040 ADDR_ID          board_id (upper 16b) + revision (lower 16b)
  0x044 ADDR_FF          Firefly USER config mask
  0x048 ADDR_PS          Power supply ignore mask
Blocks 2–5: 0x080–0x17F  Error log ring buffer (EBUF_MINBLK=2, EBUF_MAXBLK=5)
Block 6:  0x180–0x1BF   Temperature alarm thresholds (runtime-configurable)
  0x180 ADDR_TEMP_FF     Firefly alarm temp (int16_t degrees C in lower 16b of 32b word; upper 16b reserved)
  0x184 ADDR_TEMP_DCDC   DCDC alarm temp (int16_t degrees C in lower 16b of 32b word; upper 16b reserved)
  0x188 ADDR_TEMP_TM4C   TM4C alarm temp (int16_t degrees C in lower 16b of 32b word; upper 16b reserved)
  0x18C ADDR_TEMP_FPGA   FPGA alarm temp (int16_t degrees C in lower 16b of 32b word; upper 16b reserved)
Blocks 7+: 0x1C0+        Available
```

Uninitialized EEPROM words read as `0xFFFFFFFF`. Code that loads EEPROM config must check for this sentinel and fall back to compile-time defaults.

### prod_test: Architecture

`prod_test` is a minimal FreeRTOS application — no alarm, EEPROM, or monitor tasks. It has three tasks:

- `vCommandLineTask` — UART CLI on UART0 (Zynq UART), single instance
- `ADCMonitorTask` — continuously samples on-chip ADC; values read by `adc` CLI command
- `I2CSlaveTask` — responds to an external I2C master on I2C0 (slave address `0x40`)

**I2C bus assignments** (initialized in `SystemInit()`):

| Bus | Role |
| --- | ---- |
| I2C0 | I2C slave (responds to external master) |
| I2C1 | SMBus master → DCDC power supplies |
| I2C2 | SMBus master → clock synthesizers |
| I2C3 | SMBus master → F2 Firefly optics |
| I2C4 | SMBus master → F1 Firefly optics |
| I2C5 | SMBus master → F1 Firefly optics (additional) |

**prod_test CLI commands:**

| Command | Description |
| ------- | ----------- |
| `adc` | Display ADC voltage/current measurements with pass/fail |
| `poweron <level>` | Enable power supplies up to the given priority level |
| `poweroff` | Disable all power supplies |
| `dcdci2ctest` | Test I2C communication to DCDC converters |
| `dcdcpowertest` | Test DCDC power-on and voltage levels |
| `clocki2ctest` | Test I2C to clock synthesizers |
| `ffi2ctest [mask]` | Test I2C to Firefly optics (optional channel mask) |
| `eepromi2ctest` | Test I2C to external EEPROM |
| `fpgai2ctest` | Test I2C to FPGAs |
| `initclockreg` | Initialize clock synthesizer IO expanders |
| `initffreg` | Initialize Firefly IO expanders |
| `prodtest1 [mask]` | Run full first-step production test sequence (see below) |
| `version` | Display firmware version and build time |
| `heap` | Show free and minimum-ever-free heap |
| `bootloader` | Jump to ROM bootloader |
| `restart` | Reset the MCU |

**`prodtest1` sequence** (stops on first failure):

1. DCDC I2C connectivity check
2. DCDC power-on and voltage verification
3. Initialize clock IO expanders
4. Clock I2C test
5. FPGA I2C test
6. Initialize Firefly IO expanders
7. Firefly I2C test
8. External EEPROM I2C test

### CLI Command Pattern

Command handlers in **both projects** share this signature:

```c
BaseType_t my_command(int argc, char **argv, char *m);
```

- `argv[0]` is the command name, `argv[1..argc-1]` are arguments
- Write output to `m` using `snprintf`
- Registered in `CommandLineTask.c` in the static `commands[]` array

**Return values differ between the two projects:**

| Project | Return type | Values |
| ------- | ----------- | ------ |
| `cm_mcu` | `BaseType_t` | `pdFALSE` = done, `pdTRUE` = more output pending |
| `prod_test` | `cli_status_t` | `CLI_OK` = done, `CLI_MORE` = more output pending, `CLI_ERROR` = failed |

**`SCRATCH_SIZE`** (output buffer): **1024** bytes in `cm_mcu`, **512** bytes in `prod_test`.

In `prod_test`, each command registration also declares `num_args` (the exact number of required arguments, or `-1` to accept any). The dispatcher enforces this before calling the handler.

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
alm status                                 # show current thresholds and alarm status
alm settemp [ff|fpga|dcdc|tm4c] <temp>     # set threshold (persists to EEPROM)
alm resettemp [ff|fpga|dcdc|tm4c|all]      # reset to compile-time defaults
```

## Debugging

### GDB

A `.gdbinit` file is provided: `cm_mcu.gdbinit`. Connect via Segger J-LINK EDU.

### Serial Terminal

- Terminal should send `\n` on Enter
- Code should print `\r\n` for broad compatibility

## CI / Code Quality

- GitHub Actions runs on pull requests: format check + build verification with `-Werror`
- Multi-revision builds (REV1, REV2, REV3) — release binaries published on GitHub releases
- Code formatting: `clang-format` (see `.clang-format`) — run `make format-apply` before committing
- Static analysis: `clang-tidy` (see `.clang-tidy`)

## Common Issues

| Symptom | Cause | Solution |
| ------- | ----- | -------- |
| FreeRTOS build errors | Submodule not initialized | `git submodule update --init --recursive --remote` |
| Linker errors / weird runtime behavior | Mixed debug/release build | `make clean`, then rebuild consistently |
| Compilation errors / unsupported flags | Wrong compiler version | Install `arm-none-eabi-gcc` 13.2.Rel1 from ARM website |
| Incremental builds fail on Windows | Cygwin limitation | Use WSL, or run `make clean` between each build |

## When Working on This Project

### Before Making Changes

1. Check which hardware revision you're targeting
2. Ensure FreeRTOS submodule is up to date: `git submodule update --init --recursive --remote`
3. Build with `DEBUG=1` for development
4. Check code formatting: `make format`

### Before Committing

1. Run `make format-apply` to auto-format code
2. Run `make check-for-pr` to verify all builds pass
3. Ensure no new warnings (CI treats warnings as errors)
4. Test on actual hardware if possible

### Key Considerations

- This is bare-metal firmware with FreeRTOS — not a hosted OS
- Memory is constrained: avoid heap allocation, be mindful of task stack sizes
- ROM-based TivaWare drivers are preferred (already in MCU ROM)
- Thread safety requires FreeRTOS primitives (mutexes, queues, etc.)
- Changes may affect time-critical interrupt handlers

## Development Notes for AI Assistants

- Integers: use `uint32_t`, `int32_t`, etc. — not bare `int` for register-width values
- Float-to-uint32 conversion for EEPROM: use `memcpy(&u32, &f, sizeof(float))` — not union type-punning
- All EEPROM writes from tasks must use `write_eeprom()` (queue-based), never direct driver calls
- Block 1 EEPROM is password-protected — requires `EPRM_UNLOCK_BLOCK` / `EPRM_LOCK_BLOCK` messages around writes
- Block 6 (temperature alarms) is unprotected — direct `write_eeprom()` calls are sufficient
- `Tasks.h` is the central header; most inter-task definitions live there
- When adding a new CLI subcommand, update the help string in `CommandLineTask.c`

## Related Resources

- [TM4C1290NCPDT Datasheet](https://www.ti.com/product/TM4C1290NCPDT)
- [FreeRTOS Documentation](https://freertos.org)
- [TivaWare Peripheral Driver Library](https://www.ti.com/tool/SW-TM4C)
- [Segger J-LINK](https://www.segger.com)
