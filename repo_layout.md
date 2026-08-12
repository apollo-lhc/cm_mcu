# Repository layout

Full source tree, with a one-line description per file. Summarized in
[`CLAUDE.md`](CLAUDE.md); per-project detail lives in each project's `README.md`.

## Flash memory map

The bootloader and the application are separate images that must agree on where one ends and the
other begins:

| Region | Owner | Set in |
| ------ | ----- | ------ |
| `0x00000000 –` | bootloader image | load address in `projects/boot_loader/bl_link.ld` |
| `0x00004000 – 0x000FFFFF` | application | `FLASH ORIGIN` in `projects/cm_mcu/cm_mcu.ld` and `projects/prod_test/prod_test.ld` |

The `0x4000` boundary appears in **three** independent places — the two application linker scripts
and `APP_START_ADDRESS` in `boot_loader/bl_config.h`. Nothing cross-checks them, and nothing caps the
bootloader's size: `bl_link.ld` has no `MEMORY` block, so a bootloader that grows past `0x4000` will
link happily and silently overlap the application. Change one, change all three.

Note that `bl_link.ld` links the bootloader to **execute from SRAM** (`.text` VMA `0x20000000`) while
loading it from flash `0x00000000` — that is TI's stock arrangement, not an Apollo modification, and
it is why the bootloader's addresses look nothing like the application's.

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
│   │   ├── ProgComTask.c/h      # Programmatic (non-CLI) UART7 command interface, REV2/3 only
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
│   │       ├── AlarmCommands.c/h    # Temperature/voltage alarm thresholds
│   │       ├── BoardCommands.c/h    # Board ID, EEPROM identity, GPIO commands
│   │       ├── BufferCommands.c/h   # Error log ring buffer commands
│   │       ├── ClockCommands.c/h    # Clock synthesizer commands
│   │       ├── EEPROMCommands.c/h   # Low-level EEPROM read/write commands
│   │       ├── FPGACommands.c/h     # FPGA monitoring commands
│   │       ├── FireflyCommands.c/h  # Firefly register access and CDR/status commands
│   │       ├── I2CCommands.c/h      # I2C bus scan and raw access commands
│   │       ├── PowerCommands.c/h    # Power supply control and PMBus register access
│   │       ├── SensorControl.c/h    # Sensor monitoring commands
│   │       ├── SoftwareCommands.c/h # Version, reset, watchdog, heap, sem_ctl commands
│   │       ├── ZynqCommands.c/h     # ZynqMon commands
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
│   ├── boot_loader/             # Serial bootloader (TI stock `bl_*` + Apollo hooks)
│   │   ├── bl_config.h          # THE config file: UART update, 115200 fixed, app @ 0x4000
│   │   ├── bl_main.c            # Bootloader main loop and command dispatch
│   │   ├── bl_userhooks.c/h     # Apollo customization: RGB LED, mini-CLI (h/b/r/f), update check
│   │   ├── rl_config.h          # microrl config for the bootloader's mini-CLI
│   │   ├── bl_check.c/h         # Decides whether to enter the bootloader or the app
│   │   ├── bl_packet.c/h        # Serial packet framing / ACK-NAK
│   │   ├── bl_uart.c/h          # UART transport
│   │   ├── bl_autobaud.c        # Autobaud (unused: UART_FIXED_BAUDRATE is set)
│   │   ├── bl_flash.c/h         # Flash erase/program primitives
│   │   ├── bl_crc32.c/h         # Image CRC
│   │   ├── bl_commands.h        # Wire protocol command codes (COMMAND_PING, …)
│   │   ├── bl_hooks.h           # Hook declarations TI calls into
│   │   ├── bl_crystal.h         # Crystal frequency validation
│   │   ├── bl_startup_gcc.S     # Bootloader vector table / reset (GCC)
│   │   ├── bl_startup_clang.S   # Bootloader vector table / reset (clang)
│   │   └── bl_link.ld           # Linker script: loads at flash 0x0, executes from SRAM
│   │
│   ├── blinky/                  # LED blink sanity test (blinky.c + startup)
│   ├── i2c-sensors/             # I2C sensor demo (i2c-sensors.c + startup)
│   └── uart_echo/               # UART loopback test (uart_echo.c + startup)
│
├── common/                      # Shared utilities across all projects
│   ├── utils.c/h                # EEPROM helpers, error buffer, float utilities
│   ├── log.c/h                  # Logging macros (modified rxi/log.c)
│   ├── printf.c/h               # Embedded printf implementation
│   ├── LocalUart.c/h            # UART init and blocking print helpers
│   ├── microrl.c/h              # CLI line editor / history
│   ├── smbus.c/h                # TI SMBus state machine
│   ├── smbus_helper.c/h         # SMBus error strings, NACK classification
│   ├── smbus_units.c/h          # LINEAR11 / LINEAR16 decoding
│   ├── power_ctl.c/h            # Power supply enable/status GPIO control
│   ├── softuart.c/h             # Soft UART used by ZynqMonTask
│   ├── i2c_reg.c/h              # Register-level I2C helpers
│   ├── pinsel.c/h, pinout.h     # Pin selection helpers
│   ├── pinout_rev1.c            # TI PinMux-generated pin configuration, REV1
│   ├── pinout_rev2.c            # TI PinMux-generated pin configuration, REV2/REV3
│   └── gpio_pins_rev*.def       # X-macro GPIO pin tables per revision
├── inc/                         # TI Tivaware driver library headers
├── driverlib/                   # TI Tivaware driver library source
├── FreeRTOS-Kernel/             # FreeRTOS (git submodule)
├── sm_cm_config/                # Board configuration files
└── makedefs                     # Common Makefile variable definitions
```
